#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, Int32
import math
from enum import Enum
import collections

# 定义系统状态机
class State(Enum):
    IDLE = 0
    UP_1_PREPARE = 10
    UP_2_LIFT = 11
    UP_3_FRONT_DOCK = 12
    UP_4_RETRACT_FRONT = 13
    UP_5_FRONT_LAND = 14
    UP_6_SIDE_DOCK_RETRACT_REAR = 15
    UP_7_REAR_LAND = 16
    UP_8_RECOVER = 17

    DOWN_1_PREPARE = 20
    DOWN_2_FRONT_HOVER_LAND = 21
    DOWN_3_REAR_HOVER_LAND = 22
    DOWN_4_RECOVERY = 23


class Direction(Enum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2
    BACKWARD = 3

class SuspensionController(Node):
    def __init__(self):
        super().__init__('suspension_controller')

        # --- 参数配置 ---
        self.H_LIFT_LOW = 205.0   # 台阶高度1
        self.H_LIFT_HIGH = 400.0  # 台阶高度2
        self.H_INIT = 30.0        # 初始/常规运动姿态高度
        self.CREEP_SPEED = 0.2   # 安全蠕行速度 (m/s)
        self.HEIGHT_TOLERANCE = 20.0

        self.control_by_sbus = False

        # --- 状态变量 ---
        self.current_state = State.IDLE
        self.target_height = 0.0  # 当前识别到的台阶目标高度
        self.current_direction = Direction.FORWARD
        self.is_reversing = False

        self.YAW_KP = 0.025
        self.YAW_KD = 0.04
        self.YAW_TOLERANCE = 0.05
        self.ANGULAR_Z_COMPENSATION = 0.152
        self.MAX_ANGULAR_VEL = 0.5
        self.YAW_DEBUG_INTERVAL = 0.2
        self.imu_yaw_raw = 0.0
        self.imu_yaw_unwrapped = 0.0
        self.relative_yaw = 0.0
        self.relative_yaw_rate = 0.0
        self.yaw_offset = 0.0
        self.last_imu_yaw_raw = None
        self.last_relative_yaw = None
        self.last_imu_time_sec = None
        self.last_yaw_debug_time_sec = 0.0
        self.target_relative_yaw = 0.0
        self.yaw_correction_enabled = True
        self.has_imu_yaw = False
        self.imu_zero_locked = False

        # 物理层状态
        self.raw_cmd_vel = Twist()
        self.distances_raw = [0.0] * 8
        self.pe_switches_raw = [0] * 4
        self.wheel_heights_current = [0.0] * 4  # [0, 1, 2, 3]

        # 滤波后状态
        self.distance_filtered = [0.0] * 8
        self.pe_switches_filtered = [0] * 4

        # 输出控制状态
        self.wheel_heights_target = [self.H_INIT] * 4
        self.chassis_cmd_vel = Twist()
        self.v_distances_idx = [0.0] * 6

        # 虚拟层状态 (根据方向映射后的 0, 1, 2, 3)
        self.v_wheels_idx = [0, 1, 2, 3]
        self.v_pe_idx = [0, 1, 2, 3]

        # --- 滤波器初始化 ---
        self.distance_buffers = [collections.deque(maxlen=5) for _ in range(8)]
        self.pe_debounce_counters = [0] * 4  # 用于 10ms 防抖 (主频 100Hz 下 1 帧 = 10ms)
        self.pe_last_states = [0] * 4

        # 初始化高度锁存标志位
        self._height_latched = False

        # 状态机通用防抖计数器字典
        # key 为逻辑判断的唯一标识(字符串)，value 为连续触发的次数
        self._stable_counters = collections.defaultdict(int)

        # --- ROS 2 接口 ---
        self.sub_direction = self.create_subscription(Int32, 'direction', self.direction_cb, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.sub_sensor_dist = self.create_subscription(Float32MultiArray, 'sensor_distances', self.dist_cb, 10)
        self.sub_r0x0121 = self.create_subscription(Float32MultiArray, 'r0x0121', self.hw_status_cb, 10)
        self.pub_action = self.create_publisher(Float32MultiArray, 't0x0111_action', 10)
        self.pub_chassis_vel = self.create_publisher(Twist, 'cmd_vel_chassis', 10)
        self.pub_state = self.create_publisher(Int32, 'current_state', 10)
        # 控制主循环 (100Hz)
        self.delay_timer = self.create_timer(0.2, self.start_control_loop)
        self.get_logger().info("Step Climber Node Initialized.")
        self.get_logger().info(
            f"Suspension node code: {__file__} | "
            "yaw correction runs on MCU; cmd_vel.angular.z/action[6] is target yaw in deg"
        )

    def start_control_loop(self):
        self.delay_timer.cancel()
        self.timer = self.create_timer(0.01, self.control_loop)

    def _is_stable(self, condition, key, threshold=5):
        """
        通用状态防抖判断函数
        :param condition: 当前帧的布尔条件
        :param key: 计数器的唯一标识符
        :param threshold: 需要连续满足的次数，默认 5 次 (主频 100Hz 下 = 50ms)
        :return: bool，是否稳定满足
        """
        if condition:
            # 只有还没达到阈值时才累加，防止长时间停留在某状态导致整数溢出
            if self._stable_counters[key] < threshold:
                self._stable_counters[key] += 1

            # 达到阈值后，只要 condition 依然为真，就一直返回 True
            if self._stable_counters[key] >= threshold:
                return True
        else:
            # 只要有一次不满足（即使之前满足了），立刻清零
            self._stable_counters[key] = 0

        return False


    def direction_cb(self, msg):
        if self.current_state != State.IDLE:
            return
        if self.control_by_sbus:
            return
        if msg.data == 0:
            self.current_direction = Direction.FORWARD
        elif msg.data == -1:
            self.current_direction = Direction.LEFT
        elif msg.data == 1:
            self.current_direction = Direction.RIGHT

    def imu_cb(self, msg):
        self.imu_yaw_raw = math.radians(float(msg.yaw))
        if self.last_imu_yaw_raw is None:
            self.imu_yaw_unwrapped = self.imu_yaw_raw
        else:
            delta_yaw = self.normalize_angle(self.imu_yaw_raw - self.last_imu_yaw_raw)
            self.imu_yaw_unwrapped += delta_yaw
        self.last_imu_yaw_raw = self.imu_yaw_raw

        if not self.imu_zero_locked:
            self.yaw_offset = self.imu_yaw_unwrapped
            self.last_relative_yaw = 0.0
            self.has_imu_yaw = True
            self.imu_zero_locked = True
            self.target_relative_yaw = 0.0
            self.get_logger().info(
                "Yaw relative zero locked from first valid IMU angle: "
                f"offset={math.degrees(self.yaw_offset):.2f} deg"
            )
        relative_yaw = self.imu_yaw_unwrapped - self.yaw_offset

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.last_relative_yaw is not None and self.last_imu_time_sec is not None:
            dt = stamp_sec - self.last_imu_time_sec
            if dt > 1e-4:
                delta_yaw = relative_yaw - self.last_relative_yaw
                self.relative_yaw_rate = delta_yaw / dt
            else:
                self.relative_yaw_rate = 0.0
        else:
            self.relative_yaw_rate = 0.0

        self.relative_yaw = relative_yaw
        self.last_relative_yaw = relative_yaw
        self.last_imu_time_sec = stamp_sec

    def target_yaw_cb(self, msg):
        target_yaw_deg = float(msg.data)
        self.target_relative_yaw = math.radians(target_yaw_deg)
        self.get_logger().info(f"Target relative yaw updated to {target_yaw_deg:.2f} deg")

    def yaw_correction(self):
        if not self.yaw_correction_enabled or not self.has_imu_yaw:
            return 0.0

        yaw_error = self.target_relative_yaw - self.relative_yaw
        angular_correction = self.YAW_KP * yaw_error - self.YAW_KD * self.relative_yaw_rate

        if abs(yaw_error) < self.YAW_TOLERANCE:
            angular_correction = 0.0

        angular_correction = max(
            -self.MAX_ANGULAR_VEL,
            min(self.MAX_ANGULAR_VEL, angular_correction),
        )
        self.chassis_cmd_vel.angular.z = angular_correction
        self._log_yaw_debug(yaw_error, angular_correction)
        return angular_correction

    def compensate_angular_z(self, angular_z):
        if abs(angular_z) < 1e-6:
            return 0.0
        return angular_z + math.copysign(self.ANGULAR_Z_COMPENSATION, angular_z)

    def clamp_angular_z(self, angular_z):
        return max(-self.MAX_ANGULAR_VEL, min(self.MAX_ANGULAR_VEL, angular_z))

    def _log_yaw_debug(self, yaw_error, angular_correction):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_yaw_debug_time_sec < self.YAW_DEBUG_INTERVAL:
            return

        self.last_yaw_debug_time_sec = now_sec
        self.get_logger().info(
            "yaw_dbg "
            f"imu_raw={math.degrees(self.imu_yaw_raw):.1f}deg "
            f"rel={math.degrees(self.relative_yaw):.1f}deg "
            f"target={math.degrees(self.target_relative_yaw):.1f}deg "
            f"err={math.degrees(yaw_error):.1f}deg "
            f"rate={math.degrees(self.relative_yaw_rate):.1f}deg/s "
            f"cmd_z={angular_correction:.3f}"
        )


    def cmd_vel_cb(self, msg):
        self.raw_cmd_vel = msg

    def dist_cb(self, msg):
        if len(msg.data) >= 8:
            for i in range(8):
                self.distance_buffers[i].append(msg.data[i])
                self.distance_filtered[i] = sum(self.distance_buffers[i]) / len(self.distance_buffers[i])

    def hw_status_cb(self, msg):
        # r0x0121: [PE_0, PE_1, PE_2, PE_3, H_0, H_1, H_2, H_3]
        if len(msg.data) >= 12:
            for i in range(4):
                current_pe = int(msg.data[i])
                # 10ms (1 frame) 防抖逻辑
                if current_pe != self.pe_last_states[i]:
                    self.pe_debounce_counters[i] += 1
                    if self.pe_debounce_counters[i] >= 2: # 连续1次跳变(10ms)确认
                        self.pe_switches_filtered[i] = current_pe
                        self.pe_last_states[i] = current_pe
                        self.pe_debounce_counters[i] = 0
                else:
                    self.pe_debounce_counters[i] = 0

            for i in range(4):
                self.wheel_heights_current[i] = msg.data[4 + i]

            if self.current_state == State.IDLE and self.control_by_sbus:
                if msg.data[11] > 0:
                    self.current_direction = Direction.RIGHT
                elif msg.data[11] < 0:
                    self.current_direction = Direction.LEFT
                else:
                    self.current_direction = Direction.FORWARD
                self.raw_cmd_vel.linear.x = msg.data[8]
                self.raw_cmd_vel.linear.y = msg.data[9]
                self.raw_cmd_vel.angular.z = msg.data[10]

    # ================= 核心映射与控制 =================
    def update_virtual_mapping(self):
        """根据行驶方向，将物理轮/传感器映射为虚拟的 0, 1, 2, 3"""
        if self.current_direction == Direction.FORWARD:
            self.v_wheels_idx = [2, 1, 0, 3] # [0, 1, 2, 3]
            self.v_pe_idx = [0, 1, 3, 2]
            self.v_distances_idx = [0, 1, 5, 4]
        elif self.current_direction == Direction.LEFT:
            self.v_wheels_idx = [0, 2, 3, 1]
            self.v_pe_idx = [1, 2, 0, 3]
            self.v_distances_idx = [2, 3, 7, 6]
        elif self.current_direction == Direction.RIGHT:
            self.v_wheels_idx = [1, 3, 2, 0]
            self.v_pe_idx = [3, 0, 2, 1]
            self.v_distances_idx = [6, 7, 3, 2]

    def check_height_reached(self, virtual_indices, target_h):
        """高度闭环验证"""
        for v_idx in virtual_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            curr_h = self.wheel_heights_current[phys_idx]
            if abs(curr_h - target_h) > self.HEIGHT_TOLERANCE:
                return False
        return True

    def control_loop(self):
        self.update_virtual_mapping()

        v_0, v_1, v_2, v_3 = 0, 1, 2, 3

        self.execute_state_machine(v_0, v_1, v_2, v_3)
        target_yaw_deg = self.raw_cmd_vel.angular.z
        self.chassis_cmd_vel.angular.z = target_yaw_deg

        msg = []
        msg.extend(self.wheel_heights_target)
        msg.extend([self.chassis_cmd_vel.linear.x, self.chassis_cmd_vel.linear.y, target_yaw_deg])
        ros_msg = Float32MultiArray()
        ros_msg.data = msg
        self.pub_action.publish(ros_msg)
        self.pub_chassis_vel.publish(self.chassis_cmd_vel)

        state_msg = Int32(data=self.current_state.value)
        self.pub_state.publish(state_msg)

    def execute_state_machine(self, v_0, v_1, v_2, v_3):
        """核心状态机 (全面加入 50ms 逻辑防抖)"""
        state = self.current_state
        prev_state = self.current_state
        self.chassis_cmd_vel.linear.x = self.raw_cmd_vel.linear.x
        self.chassis_cmd_vel.linear.y = self.raw_cmd_vel.linear.y
        self.chassis_cmd_vel.angular.z = self.raw_cmd_vel.angular.z

        if state == State.IDLE:



            cond_up = self._get_v_distance(1) < 200
            cond_down = self._get_v_distance(0) > 200

            if self._is_stable(cond_up, 'idle_to_up'):
                self.current_state = State.UP_1_PREPARE
            elif self._is_stable(cond_down, 'idle_to_down'):
                self.current_state = State.DOWN_1_PREPARE

        # ================= 上台阶逻辑 =================
        elif state == State.UP_1_PREPARE:
            self.target_height = self.H_LIFT_LOW
            self.wheel_heights_target = [self.target_height] * 4
            cond_height = self.check_height_reached([v_0, v_1, v_2, v_3], self.target_height)
            if self._is_stable(cond_height, 'up1_height', threshold=2):
                self.current_state = State.UP_2_LIFT

        elif state == State.UP_2_LIFT:
            self._stop_chassis()
            cond_high_dist = self._get_v_distance(1) < 200

            # 分支 1：确实是高台阶，继续升
            if self._is_stable(cond_high_dist, 'up2_high_dist'):
                self.target_height = self.H_LIFT_HIGH
                self.wheel_heights_target = [self.target_height] * 4
                cond_height = self.check_height_reached([v_0, v_1, v_2, v_3], self.target_height)
                if self._is_stable(cond_height, 'up2_height', threshold=2):
                    self.current_state = State.UP_3_FRONT_DOCK
            # 分支 2：并非高台阶，防抖确认后直接进入搭接阶段
            elif self._is_stable(not cond_high_dist, 'up2_low_dist'):
                self.current_state = State.UP_3_FRONT_DOCK

        elif state == State.UP_3_FRONT_DOCK:
            self._creep_forward()
            cond_dist = self._get_v_distance(0) < 80.0
            if self._is_stable(cond_dist, 'up3_dist'):
                self.current_state = State.UP_4_RETRACT_FRONT

        elif state == State.UP_4_RETRACT_FRONT:
            self._stop_chassis()
            self._set_v_wheel_height([v_0, v_1], 5.0)
            cond_height = self.check_height_reached([v_0, v_1], 5.0)
            if self._is_stable(cond_height, 'up4_height', threshold=2):
                self.current_state = State.UP_5_FRONT_LAND

        elif state == State.UP_5_FRONT_LAND:
            self._creep_forward()

            cond_pe = (self._get_v_pe(v_0) == 1 )

            if self._is_stable(cond_pe, 'up5_pe'):
                self._stop_chassis()
                self._set_v_wheel_height([v_0, v_1], 35.0)
                self._set_v_wheel_height([v_2, v_3], self.target_height + 3.0)

                cond_height = self.check_height_reached([v_2, v_3], self.target_height + 3.0)
                if self._is_stable(cond_height, 'up5_height', threshold=2):
                    self.current_state = State.UP_6_SIDE_DOCK_RETRACT_REAR

        elif state == State.UP_6_SIDE_DOCK_RETRACT_REAR:
            self._creep_forward()
            cond_pe = (self._get_v_pe(v_2) == 1)
            if self._is_stable(cond_pe, 'up6_pe', threshold=20):
                self._stop_chassis()
                self._set_v_wheel_height([v_2, v_3], 0.0)

                cond_height = self.check_height_reached([v_2, v_3], 0.0)
                if self._is_stable(cond_height, 'up6_height', threshold=2):
                    self.current_state = State.UP_7_REAR_LAND

        elif state == State.UP_7_REAR_LAND:
            self._creep_forward()
            cond_pe = (self._get_v_pe(v_2) == 1 and self._get_v_pe(v_3) == 1)

            if self._is_stable(cond_pe, 'up7_pe'):
                self._set_v_wheel_height([v_2, v_3], 3.0)
                cond_height = self.check_height_reached([v_2, v_3], 3.0)
                if self._is_stable(cond_height, 'up7_height', threshold=2):
                    self.current_state = State.UP_8_RECOVER

        elif state == State.UP_8_RECOVER:
            self._stop_chassis()
            self.wheel_heights_target = [self.H_INIT] * 4
            cond_height = self.check_height_reached([v_0, v_1, v_2, v_3], self.H_INIT)
            if self._is_stable(cond_height, 'up8_height', threshold=2):
                self.get_logger().info("Up step sequence complete.")
                self.current_state = State.IDLE

        # ================= 下台阶逻辑 =================
        elif state == State.DOWN_1_PREPARE:
            cond_pe = self._get_v_pe(v_0) == 0

            if self._is_stable(cond_pe, 'down1_pe'):
                self._stop_chassis()

                if not self._height_latched:
                    dist = self._get_v_distance(0)
                    if dist > 380:
                        self.target_height = self.H_LIFT_HIGH
                    elif dist > 180:
                        self.target_height = self.H_LIFT_LOW
                    else:
                        self.target_height = self.H_LIFT_LOW
                    self._height_latched = True

                self._set_v_wheel_height([v_0, v_1], self.target_height + 30.0)

                cond_height = self.check_height_reached([v_0, v_1], self.target_height + 10.0)
                if self._is_stable(cond_height, 'down1_height', threshold=2):
                    self._height_latched = False
                    self.current_state = State.DOWN_2_FRONT_HOVER_LAND
            else:
                self._creep_forward()

        elif state == State.DOWN_2_FRONT_HOVER_LAND:
            self._creep_forward()
            cond_pe = self._get_v_pe(v_3) == 0

            if self._is_stable(cond_pe, 'down2_pe'):
                self._stop_chassis()
                self._set_v_wheel_height([v_2, v_3], self.target_height + 10.0)

                cond_height = self.check_height_reached([v_2, v_3], self.target_height)
                if self._is_stable(cond_height, 'down2_height', threshold=2):
                    self.current_state = State.DOWN_3_REAR_HOVER_LAND

        elif state == State.DOWN_3_REAR_HOVER_LAND:
            self._creep_forward()
            cond_dist = self._get_v_distance(3) > 200.0

            if self._is_stable(cond_dist, 'down3_dist'):
                self.wheel_heights_target = [self.H_INIT] * 4
                self.current_state = State.DOWN_4_RECOVERY

        elif state == State.DOWN_4_RECOVERY:
            self._creep_forward()
            cond_height = self.check_height_reached([v_0, v_1, v_2, v_3], self.H_INIT)

            if self._is_stable(cond_height, 'down4_height', threshold=2):
                self.get_logger().info("Down step sequence complete.")
                self.current_state = State.IDLE

        if self.current_state != prev_state:
            self._stable_counters.clear()

    def _stop_chassis(self):
        """强制速度为0"""
        self.chassis_cmd_vel.linear.x = 0.0
        self.chassis_cmd_vel.linear.y = 0.0
        self.chassis_cmd_vel.angular.z = 0.0

    def _creep_forward(self):
        """钳位安全速度"""
        if self.current_direction == Direction.FORWARD:
            self.chassis_cmd_vel.linear.x = self.CREEP_SPEED
        elif self.current_direction == Direction.LEFT:
            self.chassis_cmd_vel.linear.y = self.CREEP_SPEED
        elif self.current_direction == Direction.RIGHT:
            self.chassis_cmd_vel.linear.y = -self.CREEP_SPEED

    def _set_v_wheel_height(self, v_indices, height):
        """设置虚拟轮的物理高度"""
        for v_idx in v_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            self.wheel_heights_target[phys_idx] = float(height)

    def _get_v_pe(self, v_idx):
        """获取虚拟轮对应的光电开关状态"""
        phys_pe_idx = self.v_pe_idx[v_idx]
        return self.pe_switches_filtered[phys_pe_idx]

    def _get_v_distance(self, v_idx):
        """获取虚拟轮对应的距离传感器值"""
        phys_dist_idx = self.v_distances_idx[v_idx]
        return self.distance_filtered[phys_dist_idx]

    def normalize_angle(self, angle):
        """将角度标准化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SuspensionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
