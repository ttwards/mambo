/*
 * ares_r2_tool —— 新机器人对接器(connector/spear)下位机
 *
 * 外部控制复用 ARES dual_protocol 的 SYNC 帧承载(参考 app/relay_comm)，走 USB bulk。
 * "帧头" 即 SYNC ID = 动作对象；payload 首个 uint32 为动作 enum，其后 4 个预留 float32(=0)。
 *
 *   指令帧 (上位机 -> 本板)  head=0x5A5A, ID=接收对象, data={action(u32), f32×4=0}
 *   反馈帧 (本板 -> 上位机)  head=0x5A5A, ID=发送对象, data={action(u32), f32×4=0}
 *
 * 动作对象 ID: arm rx=0x0211 tx=0x0221, connector/spear rx=0x0212 tx=0x0222。
 * connector 动作 enum: 1=prepare 2=grasp 3=dock_extend 4=dock_release。
 * arm 动作 enum: 1=grasp 2=store_to_body 3=store_on_arm 4=get_body 5=place_mid 6=place_high。
 * payload 的 4 个 float 为通用参数 reserved[4]；prepare 用 reserved[0]=length(m) 拟合出 wye 角度。
 * 动作异步执行，完成后回发反馈帧(action 字段回显刚完成的动作)。
 */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/linear_actuator.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

#include "device.h"

LOG_MODULE_REGISTER(ares_r2_tool, LOG_LEVEL_INF);

/* ---- 动作对象(SYNC ID, 即"帧头")与动作 enum ---- */
#define SYNC_ID_ARM_RX   0x0211
#define SYNC_ID_ARM_TX   0x0221
#define SYNC_ID_SPEAR_RX 0x0212
#define SYNC_ID_SPEAR_TX 0x0222

enum spear_cmd {
	SPEAR_CMD_PREPARE = 1,      /* 张开夹爪、解锁机构、移动到待抓取位姿 */
	SPEAR_CMD_GRASP = 2,        /* 电缸夹爪闭合抓取矛头 */
	SPEAR_CMD_DOCK_EXTEND = 3,  /* 转动 roll，将矛头伸出到准备对接位置 */
	SPEAR_CMD_DOCK_RELEASE = 4, /* 张开电缸夹爪并收回，不由指令触发 */
};

enum arm_cmd {
	ARM_CMD_GRASP = 1,         /* 抓取 KFS */
	ARM_CMD_STORE_TO_BODY = 2, /* 转存到车体储存位，再回空闲位 */
	ARM_CMD_STORE_ON_ARM = 3,  /* KFS 暂持在机械臂上 */
	ARM_CMD_GET_BODY = 4,      /* 从车体取回 */
	ARM_CMD_PLACE_MID = 5,     /* 放置到中位 */
	ARM_CMD_PLACE_HIGH = 6,    /* 放置到高位 */
};

/* SYNC payload: 动作 enum + 4 个预留 float32(=0)，共 20 字节 */
struct spear_frame {
	uint32_t action;
	float reserved[4];
};

/* ---- 连接器动作角度(可标定) ---- */
float connector_wye_center_angle = -770.0f;
float connector_pitch_prepare_angle = 0.0f;
float connector_roll_zero_angle = 0.0f;
float connector_roll_prepare_angle = 90.0f;

#define CONNECTOR_GRIPPER_GRASP_POS 1400U
#define CONNECTOR_GRIPPER_OPEN_POS  2000U

/* prepare 长度(m)→wye 角度(deg) 线性拟合标定点(中位角复用 connector_wye_center_angle) */
float connector_wye_center_length = 0.2975f; /* 中位角对应长度 */
float connector_wye_zero_length = 0.067f;    /* 0° 对应长度 */

/* ---- 机械臂动作角度 ---- */
float yaw_zero_angle = 0.0f;
float yaw_task_angle = 350.0f;
float lift_down_angle = 0.0f;
float lift_half_angle = 1050.0f;
float lift_up_angle = 2100.0f;

/* ---- wye 电流监控 ---- */
/* connector_wye_current: wye(M2006) 电流反馈(以力矩形式给出，∝电流)，由 spear_dock_release
 * 内的循环写入。connector_wye_current_threshold 为对接到位的电流阈值。 */
float connector_wye_current = 0.0f;
float connector_wye_current_threshold = -0.3f;

/* ---- 通信对象 ---- */
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

DUAL_PROPOSE_PROTOCOL_DEFINE(tool_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static struct spear_frame spear_cmd_rx;  /* connector 指令帧 payload */
static struct spear_frame spear_done_tx; /* connector 反馈帧 payload */
static sync_table_t *done_pack;

static enum spear_cmd pending_spear_cmd;
static float pending_spear_args[4]; /* 指令帧 4 个预留参数的快照 */
K_SEM_DEFINE(spear_start_sem, 0, 1);

static struct spear_frame arm_cmd_rx;  /* arm 指令帧 payload */
static struct spear_frame arm_done_tx; /* arm 反馈帧 payload */
static sync_table_t *arm_done_pack;

static enum arm_cmd pending_arm_cmd;
static uint32_t arm_grasp_count;
static bool arm_payload_on_arm;
K_SEM_DEFINE(arm_start_sem, 0, 1);

/* ---- 心跳 LED ---- */
void led_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (!gpio_is_ready_dt(&led)) {
		return;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(1000);
	}
}

K_THREAD_DEFINE(led_toggle, 512, led_thread, NULL, NULL, NULL, 7, 0, 0);

/* 五次多项式插值轨迹(connector pitch/roll 均为 DM PV) */
void move_traj(const struct device *motor, float start_angle, float end_angle, uint32_t duration_ms)
{
	float t;
	float s;
	float ds;
	float angle;
	float rpm;
	uint32_t start_time = k_uptime_get_32();

	do {
		t = (k_uptime_get_32() - start_time) / (float)duration_ms;

		if (t > 1.0f) {
			t = 1.0f;
		}

		s = t * t * t * (10.0f - 15.0f * t + 6.0f * t * t);
		ds = 30.0f * t * t * (1.0f - t) * (1.0f - t);
		angle = start_angle + (end_angle - start_angle) * s;
		rpm = (end_angle - start_angle) * ds * 1000.0f / duration_ms / 6.0f;

		if (motor == arm_pitch_motor) {
			motor_set_mit(motor, rpm, angle, 0.0f);
		} else if (motor == connector_wye_motor) {
			motor_set(motor, &(motor_status_t){.angle = angle, .mode = ML_ANGLE});
		} else {
			motor_set(motor,
				  &(motor_status_t){
					  .rpm = 200.0f,
					  .angle = angle,
					  .mode = PV,
				  });
		}

		k_msleep(20);
	} while (t < 1.0f);

	if (motor == arm_pitch_motor) {
		motor_set_mit(motor, 0.0f, end_angle, 0.0f);
	} else if (motor == connector_wye_motor) {
		motor_set(motor, &(motor_status_t){.angle = end_angle, .mode = ML_ANGLE});
	} else {
		motor_set(motor,
			  &(motor_status_t){.rpm = 200.0f, .angle = end_angle, .mode = PV});
	}
	k_msleep(100);
}

/* 上电一次性归零：pitch/roll 回 0，wye 顶死寻零后移动到中心位 */
void connector_init(void)
{	
	motor_set(connector_pitch_motor, &(motor_status_t){.rpm = 200.0f, .angle = 0.0f, .mode = PV});
	motor_set(connector_roll_motor, &(motor_status_t){.rpm = 200.0f, .angle = 0.0f, .mode = PV});
	k_msleep(500);

	motor_set(connector_wye_motor, &(motor_status_t){.rpm = 200.0f, .mode = ML_SPEED});
	k_msleep(1500);
	motor_set(connector_wye_motor, &(motor_status_t){.rpm = 0.0f, .mode = ML_SPEED});
	k_msleep(1000);
	motor_control(connector_wye_motor, SET_ZERO);
	k_msleep(1000);
	move_traj(connector_wye_motor, 0.0f, connector_wye_center_angle, 500);
	k_msleep(1000);
}

/* 使能连接器电机(就绪时配置模式并上电) */
static void enable_connector(void)
{
	if (!device_is_ready(connector_pitch_motor)) {
		LOG_ERR("connector pitch motor is not ready");
		while (1) {
			k_msleep(1000);
		}
	}

	if (!device_is_ready(connector_roll_motor)) {
		LOG_ERR("connector roll motor is not ready");
		while (1) {
			k_msleep(1000);
		}
	}

	if (!device_is_ready(connector_wye_motor)) {
		LOG_ERR("connector wye motor is not ready");
		while (1) {
			k_msleep(1000);
		}
	}
	if (device_is_ready(connector_pitch_motor)) {
		motor_set_mode(connector_pitch_motor, PV);
		k_msleep(100);
		motor_control(connector_pitch_motor, ENABLE_MOTOR);
		k_msleep(100);
	}
	if (device_is_ready(connector_roll_motor)) {
		motor_set_mode(connector_roll_motor, PV);
		k_msleep(100);
		motor_control(connector_roll_motor, ENABLE_MOTOR);
		k_msleep(100);
	}
	if (device_is_ready(connector_wye_motor)) {
		motor_set(connector_wye_motor, &(motor_status_t){.rpm = 0.0f, .mode = ML_SPEED});
		k_msleep(100);
		motor_control(connector_wye_motor, ENABLE_MOTOR);
		k_msleep(100);
	}
}

static void enable_connector_gripper(void)
{
	int ret;

	if (!device_is_ready(connector_gripper)) {
		LOG_ERR("connector gripper is not ready");
		while (1) {
			k_msleep(1000);
		}
	}

	ret = la_clear_fault(connector_gripper);
	if (ret < 0) {
		LOG_WRN("connector gripper clear fault failed: %d", ret);
	}

	ret = la_enable(connector_gripper);
	if (ret < 0) {
		LOG_WRN("connector gripper enable failed: %d", ret);
	}
}

static void connector_gripper_set(uint16_t position)
{
	int ret = la_set_position(connector_gripper, position);

	if (ret < 0) {
		LOG_ERR("connector gripper set %u failed: %d", position, ret);
	}
}

static void connector_gripper_open(void)
{
	connector_gripper_set(CONNECTOR_GRIPPER_OPEN_POS);
}

static void connector_gripper_grasp(void)
{
	connector_gripper_set(CONNECTOR_GRIPPER_GRASP_POS);
}

/* prepare 长度(m)→wye 角度(deg)：zero_length→0°, center_length→connector_wye_center_angle */
static float prepare_length_to_angle(float length_m)
{
	return connector_wye_center_angle /
	       (connector_wye_center_length - connector_wye_zero_length) *
	       (length_m - connector_wye_zero_length);
}

static void spear_prepare_to_length(float length_m)
{
	motor_status_t status;

	connector_gripper_open();

	if (motor_get(connector_roll_motor, &status) == 0) {
		move_traj(connector_roll_motor, status.angle, connector_roll_prepare_angle, 500);
	}
	if (motor_get(connector_wye_motor, &status) == 0) {
		move_traj(connector_wye_motor, status.sum_angle, prepare_length_to_angle(length_m), 500);
	}
	k_msleep(1000);
}

/* 行为1：初始化操作，移动到待抓取位姿(wye 角度由 args[0]=length 换算) */
void spear_prepare(void)
{
	spear_prepare_to_length(pending_spear_args[0]);
}

/* 行为2：单一夹取动作，电缸位置 1400。 */
void spear_grasp(void)
{
	connector_gripper_grasp();
	k_msleep(500);
}

void spear_dock_release(void); /* 前置声明：dock_extend 末尾调用 */

/* 行为3：转动 roll 将矛头伸出到准备对接位置，随后进入基于电流的对接释放。 */
void spear_dock_extend(void)
{
	motor_status_t status;

	if (motor_get(connector_roll_motor, &status) == 0) {
		move_traj(connector_roll_motor, status.angle, connector_roll_zero_angle, 200);
	}
	k_msleep(1000);

	spear_dock_release();
}

/* 行为4：对接——朝对接方向旋转 wye；监控 wye 电流(力矩)，先到达对接阈值(≤ 阈值)、之后再
 * 回到 0 即判定对接到位，张开电缸夹爪并停止。由 spear_dock_extend 末尾调用。
 */
void spear_dock_release(void)
{
	motor_status_t status;
	bool reached = false;

	motor_set_angle(connector_wye_motor, 1.5f * connector_wye_center_angle);
	k_msleep(1000);

	/* 监控 wye 电流：先到达对接阈值(≤ -0.5)，之后再回到 0，才触发松开 */
	while (1) {
		if (motor_get(connector_wye_motor, &status) == 0) {
			connector_wye_current = status.torque;
		}

		if (!reached) {
			if (connector_wye_current <= connector_wye_current_threshold) {
				reached = true; /* 已到达对接电流阈值 */
			}
		} else if (connector_wye_current >= -0.1f) {
			motor_set_angle(connector_wye_motor, connector_wye_center_angle); // 回到中心位(演示用)
			k_msleep(100);
			connector_gripper_open();
			break; /* 到达阈值后电流回到 0 → 触发松开 */
		}
		k_msleep(10);
	}
}

/* ---- arm 动作(按协议预留空函数，逻辑待补全) ---- */

/* 上电一次性归零 */
void arm_init(void)
{
	motor_status_t status;

	if (motor_get(arm_yaw_motor, &status) == 0) {
		move_traj(arm_yaw_motor, status.angle, yaw_zero_angle, 500);
	}
	motor_set(arm_zed_motor, &(motor_status_t){.rpm = 200.0f, .angle = lift_down_angle, .mode = PV});
	// if (motor_get(arm_pitch_motor, &status) == 0) {
	// 	move_traj(arm_pitch_motor, status.angle, 0.0f, 1000);
	// }
	motor_set_mit(arm_pitch_motor, 0.0f, 0.0f, 0.0f);
}

/* 使能机械臂电机 */
void enable_arm(void)
{
	if (!device_is_ready(arm_yaw_motor)) {
		LOG_ERR("arm yaw motor is not ready");
		while (1) {
			k_msleep(1000);
		}
	}

	if (!device_is_ready(arm_zed_motor)) {
		LOG_ERR("arm zed motor is not ready");
		while (1) {
			k_msleep(1000);
		}
	}

	if (!device_is_ready(arm_pitch_motor)) {
		LOG_ERR("arm pitch motor is not ready");
		while (1) {
			k_msleep(1000);
		}
	}

	motor_control(arm_yaw_motor, ENABLE_MOTOR);
	k_msleep(100);
	motor_control(arm_zed_motor, ENABLE_MOTOR);
	k_msleep(100);
	motor_control(arm_pitch_motor, ENABLE_MOTOR);
	k_msleep(100);

	motor_set_mode(arm_yaw_motor, PV);
	k_msleep(100);
	motor_set_mode(arm_zed_motor, PV);
	k_msleep(100);
	motor_set_mode(arm_pitch_motor, MIT);
	k_msleep(100);
}

/* 抓取 KFS */
void arm_grasp(void)
{
	move_traj(arm_zed_motor, lift_down_angle, lift_half_angle, 500);
	move_traj(arm_pitch_motor, 0.0f, 90.0f, 1000);
	/* 吸盘吸操作 */
	move_traj(arm_pitch_motor, 90.0f, 0.0f, 1000);
}

/* 转存到车体储存位，再回空闲位 */
void arm_store_to_body(void)
{
	move_traj(arm_zed_motor, lift_half_angle, lift_up_angle, 500);
	move_traj(arm_yaw_motor, yaw_zero_angle, yaw_task_angle, 200);
	move_traj(arm_zed_motor, lift_up_angle, lift_half_angle, 500);
	k_msleep(1000);
	/* 吸盘放操作 */
	move_traj(arm_zed_motor, lift_half_angle, lift_up_angle, 500);
	move_traj(arm_yaw_motor, yaw_task_angle, yaw_zero_angle, 200);
	move_traj(arm_zed_motor, lift_up_angle, lift_down_angle, 1000);
}

/* KFS 暂持在机械臂上 */
void arm_store_on_arm(void)
{
	/* 抓取完成后保持当前姿态和吸盘状态，作为暂存在机械臂上的状态。 */
	arm_payload_on_arm = true;
}

/* 丢掉当前暂存在手臂上的 KFS。气动/吸盘未接入，此处仅保留释放动作占位。 */
static void arm_drop_payload_on_arm(void)
{
	if (!arm_payload_on_arm) {
		return;
	}

	/* 吸盘放操作 */
	arm_payload_on_arm = false;
	k_msleep(100);
}

static void arm_auto_store_after_grasp(void)
{
	arm_grasp_count++;

	if (arm_grasp_count == 1U) {
		arm_store_to_body();
		arm_payload_on_arm = false;
		return;
	}

	if (arm_grasp_count >= 3U) {
		arm_drop_payload_on_arm();
	}

	arm_store_on_arm();
}

/* 从车体取回 */
void arm_get_body(void)
{
	move_traj(arm_zed_motor, lift_half_angle, lift_up_angle, 500);
	move_traj(arm_yaw_motor, yaw_zero_angle, yaw_task_angle, 200);
	move_traj(arm_zed_motor, lift_up_angle, lift_half_angle, 500);
	k_msleep(1000);
	/* 吸盘抓操作 */
	move_traj(arm_zed_motor, lift_half_angle, lift_up_angle, 500);
	move_traj(arm_yaw_motor, yaw_task_angle, yaw_zero_angle, 200);
	move_traj(arm_zed_motor, lift_up_angle, lift_half_angle, 1000);
}

/* 放置到中位 */
void arm_place_mid(void)
{
	/* 具体动作待补全，此处仅保留函数 */
}

/* 放置到高位 */
void arm_place_high(void)
{
	/* 具体动作待补全，此处仅保留函数 */
}

/* 反馈帧：action 字段回显刚完成的动作，4 个预留 float32 = 0 */
static void spear_send_done(enum spear_cmd cmd)
{
	spear_done_tx.action = (uint32_t)cmd;
	spear_done_tx.reserved[0] = 0.0f;
	spear_done_tx.reserved[1] = 0.0f;
	spear_done_tx.reserved[2] = 0.0f;
	spear_done_tx.reserved[3] = 0.0f;

	if (dual_sync_flush(&tool_protocol, done_pack) != 0) {
		LOG_WRN("failed to send done frame");
	}
}

void spear_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_sem_take(&spear_start_sem, K_FOREVER);

		enum spear_cmd cmd = pending_spear_cmd;

		switch (cmd) {
		case SPEAR_CMD_PREPARE:
			spear_prepare();
			break;
		case SPEAR_CMD_GRASP:
			spear_grasp();
			break;
		case SPEAR_CMD_DOCK_EXTEND:
			spear_dock_extend();
			break;
		default:
			break;
		}

		spear_send_done(cmd);
	}
}

K_THREAD_DEFINE(spear, 2048, spear_thread, NULL, NULL, NULL, 6, 0, 0);

/* 指令帧回调：取出 payload 中的 action enum，置位待办指令并唤醒执行线程 */
static void spear_cmd_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	enum spear_cmd cmd = (enum spear_cmd)spear_cmd_rx.action;

	if (cmd != SPEAR_CMD_PREPARE && cmd != SPEAR_CMD_GRASP && cmd != SPEAR_CMD_DOCK_EXTEND) {
		LOG_WRN("unsupported spear action: %u", spear_cmd_rx.action);
		return;
	}

	for (int i = 0; i < 4; i++) {
		pending_spear_args[i] = spear_cmd_rx.reserved[i]; /* 快照通用参数 */
	}

	pending_spear_cmd = cmd;
	k_sem_give(&spear_start_sem);
}

/* arm 反馈帧：action 字段回显刚完成的动作，4 个预留 float32 = 0 */
static void arm_send_done(enum arm_cmd cmd)
{
	arm_done_tx.action = (uint32_t)cmd;
	arm_done_tx.reserved[0] = 0.0f;
	arm_done_tx.reserved[1] = 0.0f;
	arm_done_tx.reserved[2] = 0.0f;
	arm_done_tx.reserved[3] = 0.0f;

	if (dual_sync_flush(&tool_protocol, arm_done_pack) != 0) {
		LOG_WRN("failed to send arm done frame");
	}
}

void arm_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_sem_take(&arm_start_sem, K_FOREVER);

		enum arm_cmd cmd = pending_arm_cmd;

		switch (cmd) {
		case ARM_CMD_GRASP:
			arm_grasp();
			arm_send_done(cmd);
			arm_auto_store_after_grasp();
			continue;
		case ARM_CMD_STORE_TO_BODY:
			arm_store_to_body();
			arm_payload_on_arm = false;
			break;
		case ARM_CMD_STORE_ON_ARM:
			arm_store_on_arm();
			break;
		case ARM_CMD_GET_BODY:
			arm_get_body();
			break;
		case ARM_CMD_PLACE_MID:
			arm_place_mid();
			break;
		case ARM_CMD_PLACE_HIGH:
			arm_place_high();
			break;
		default:
			break;
		}

		arm_send_done(cmd);
	}
}

K_THREAD_DEFINE(arm, 2048, arm_thread, NULL, NULL, NULL, 6, 0, 0);

/* arm 指令帧回调：取出 payload 中的 action enum，置位待办指令并唤醒执行线程 */
static void arm_cmd_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	enum arm_cmd cmd = (enum arm_cmd)arm_cmd_rx.action;

	if (cmd < ARM_CMD_GRASP || cmd > ARM_CMD_PLACE_HIGH) {
		LOG_WRN("unsupported arm action: %u", arm_cmd_rx.action);
		return;
	}

	pending_arm_cmd = cmd;
	k_sem_give(&arm_start_sem);
}

void init(void)
{
	LOG_INF("ares_r2_tool connector start");

	/* USB 指令通道(dual_protocol SYNC) */
	if (ares_bind_interface(&usb_bulk_interface, &tool_protocol) != 0) {
		LOG_ERR("failed to initialize ARES USB interface");
	}

	dual_sync_add(&tool_protocol, SYNC_ID_SPEAR_RX, (uint8_t *)&spear_cmd_rx,
		      sizeof(spear_cmd_rx), spear_cmd_cb);
	done_pack = dual_sync_add(&tool_protocol, SYNC_ID_SPEAR_TX, (uint8_t *)&spear_done_tx,
				  sizeof(spear_done_tx), NULL);

	dual_sync_add(&tool_protocol, SYNC_ID_ARM_RX, (uint8_t *)&arm_cmd_rx,
		      sizeof(arm_cmd_rx), arm_cmd_cb);
	arm_done_pack = dual_sync_add(&tool_protocol, SYNC_ID_ARM_TX, (uint8_t *)&arm_done_tx,
				      sizeof(arm_done_tx), NULL);

	enable_connector();
	enable_connector_gripper();
	enable_arm();
	connector_init();
	arm_init();
	spear_prepare_to_length(connector_wye_center_length);
}

int main(void)
{
	init();

	while (1) {
		k_msleep(1000);
	}
}
