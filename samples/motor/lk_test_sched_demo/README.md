# LK Motor Functional Test Demo

完整的功能测试 demo，用于验证 `lk,test-motor` 驱动的所有功能。

## 测试流程

### 阶段 1: 初始化
```
1. 检查设备是否就绪
2. 等待启动稳定（200ms）
```

### 阶段 2: 命令功能测试
```
1. Clear Error (0x9B)
   - 验证清除错误帧发送
   - 预期：成功发送

2. Set Zero (0x95)
   - 设置当前位置为零点
   - 预期：成功发送

观察点：
- 所有命令应立即返回（异步提交到 scheduler）
- one-shot 命令会追踪回复帧并自动重试
- 无阻塞等待
- 检查日志中的成功/失败消息
```

### 阶段 3: 控制模式测试
```
1. SPEED 模式 (0xA2)
   - 目标转速：600 RPM
   - 转矩限制：2 Nm
   - 运行时间：1 秒
   - 观察：速度、转矩、温度

2. TORQUE 模式 (0xA1)
   - 目标转矩：1 Nm
   - 运行时间：1 秒
   - 观察：转矩变化、温度

3. ANGLE 模式（切换回 0xA4）
   - 目标位置：0°
   - 速度限制：800 rpm
   - 运行时间：1 秒
   - 观察：位置变化

观察点：
- 每个模式应成功切换
- 运行时实时状态更新
- 模式切换应平滑
```

### 阶段 4: 启停循环测试
```
1. Disable Motor (0x80)
   - 等待：500ms

2. Enable Motor (0x88)
   - 等待：500ms

3. 重复 3 次

观察点：
- 每次禁用/使能都应成功
- 每次循环后检查电机状态
- 验证重连机制是否正常
```

### 阶段 5: 主测试循环
```
1. 启用电机
   - 预期：立即返回

2. 位置控制循环
   - 目标：-180° ~ +180°
   - 步长：45°
   - 速度限制：800 rpm
   - 控制周期：100ms

3. 持续运行
   - 每 10 次循环输出状态
   - 到达边界自动换向
   - 无限循环

观察点：
- 电机应稳定运行
- 位置应平滑跟随目标
- 边界自动转向
- 无阻塞发送
```

## 调试日志格式

### 状态日志
```
motor state: mode=4 angle=45.00° rpm=600.00 torque=1.000Nm temp=35.0°C 
limits: speed=800.000rpm torque=2.000Nm
```
说明：
- mode: 当前控制模式（3=ANGLE, 4=SPEED, 5=TORQUE）
- angle: 当前角度（度）
- rpm: 当前转速（RPM）
- torque: 当前转矩
- temp: 电机温度（°C）
- speed_limit: 当前速度限制（rpm）
- torque_limit: 当前转矩限制（Nm）

### 命令日志
```
--- Testing motor commands ---
1. Clear error
   Clear error sent (async)
2. Set zero position
   Set zero sent (async)
--- Command testing completed ---
```

### 控制模式日志
```
--- Testing control modes ---
1. Testing SPEED mode
   SPEED mode set successfully
2. Testing TORQUE mode
   TORQUE mode set successfully
3. Switching back to ANGLE mode
   ANGLE mode set successfully
--- Control mode testing completed ---
```

### 启停循环日志
```
--- Testing enable/disable cycle ---
Cycle 1: Disable motor
Cycle 1: Enable motor
motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=35.0°C ...

Cycle 2: Disable motor
Cycle 2: Enable motor
motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=35.0°C ...

Cycle 3: Disable motor
Cycle 3: Enable motor
motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=35.0°C ...
--- Enable/disable cycle completed ---
```

### 主循环日志
```
Loop 10: angle=135.00° direction=+1.0
motor state: mode=3 angle=135.00° rpm=600.00 torque=1.200Nm temp=36.0°C 
limits: speed=800.000rpm torque=2.000Nm

Loop 20: angle=180.00° direction=-1.0
motor state: mode=3 angle=180.00° rpm=600.00 torque=1.150Nm temp=36.0°C 
limits: speed=800.000rpm torque=2.000Nm

Reached +180°, changing direction to -1

Loop 30: angle=-45.00° direction=+1.0
motor state: mode=3 angle=-45.00° rpm=600.00 torque=1.100Nm temp=36.0°C 
limits: speed=800.000rpm torque=2.000Nm
```

## 故障排查指南

### 启动失败
```
[INF] lk_motor not ready
```
原因：
- CAN 设备未初始化
- 电机硬件未连接

解决：
- 检查 CAN 设备是否就绪
- 确认电机供电和 CAN 连线

### 命令发送失败
```
[ERR] motor_set failed at angle=45.00°: -2
[ERR] Clear error failed: -22
```
原因：
- CAN 总线繁忙
- 电机离线

解决：
- 检查 CAN 总线状态
- 观察驱动日志中的 offline 消息
- 等待自动重连

### 状态无更新
```
[INF] motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=0.0°C 
```
原因：
- 电机未使能
- CAN 通信问题

解决：
- 确认已发送 ENABLE 命令
- 检查 CAN 接收日志
- 观察 trace_lifecycle 帧级日志

### 离线/重连问题
```
[WRN] m_lk_test_0 offline (no feedback for 80 ms)
[INF] m_lk_test_0 online (feedback restored)
[INF] m_lk_test_0 re-synced PID after reconnect
```
这是正常的重连流程。如果频繁离线：
- 检查 CAN 连接质量
- 检查电机供电稳定性
- 调整控制频率（降低到 100Hz）

### 转矩/速度异常
```
[INF] motor state: mode=3 angle=90.00° rpm=600.00 torque=5.000Nm temp=36.0°C
[INF] motor state: mode=3 angle=90.00° rpm=600.00 torque=-2.000Nm temp=36.0°C
```
原因：
- PID 参数未正确同步
- 电机负载过重

解决：
- 检查 PID 参数是否正确
- 重新启动测试（会自动重发 PID）
- 观察温度是否过高

## 启用详细日志

### 方式 1: 启用 trace_lifecycle（推荐）
在 devicetree 中添加：
```dts
lk_motor: lk_motor {
    compatible = "lk,test-motor";
    id = <1>;
    trace_lifecycle;  /* 添加这一行 */
    ...
};
```
会输出所有 CAN 帧：
```
[INF] m_lk_test_0 rx id=0x141 cmd=0x9C data=17 00 00 2D 0A 00 1F 40 00
[INF] m_lk_test_0 tx id=0x141 cmd=0xA4 data=A4 00 1F 40 00 0A 00 5A 1F 40
```

### 方式 2: 启用驱动级调试日志
在 `prj.conf` 中设置：
```kconfig
CONFIG_MOTOR_LOG_LEVEL=3  # LOG_LEVEL_DBG
```

### 方式 3: 启用 scheduler 调试
需要修改 `drivers/motor/common/` 并重新编译：
```c
#define MOTOR_CAN_SCHED_DEBUG 1
```

## 预期输出

正常情况下，终端输出类似：

```
╔═══════════════════════════════════════╗
   LK Motor Functional Test Demo
   Device: m_lk_test_0
╚═══════════════════════════════════════╝

--- Testing motor commands ---
1. Clear error
   Clear error sent (async)
2. Set zero position
   Set zero sent (async)
--- Command testing completed ---

--- Testing control modes ---
1. Testing SPEED mode
   SPEED mode set successfully
motor state: mode=4 angle=0.00° rpm=600.00 torque=1.000Nm temp=35.0°C 
   limits: speed=800.000rpm torque=2.000Nm

2. Testing TORQUE mode
   TORQUE mode set successfully
motor state: mode=5 angle=0.00° rpm=0.00 torque=1.000Nm temp=35.0°C 
   limits: speed=800.000rpm torque=2.000Nm

3. Switching back to ANGLE mode
   ANGLE mode set successfully
motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=35.0°C 
   limits: speed=800.000rpm torque=2.000Nm

--- Control mode testing completed ---

--- Testing enable/disable cycle ---
Cycle 1: Disable motor
Cycle 1: Enable motor
motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=35.0°C 
   limits: speed=800.000rpm torque=2.000Nm

Cycle 2: Disable motor
Cycle 2: Enable motor
motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=35.0°C 
   limits: speed=800.000rpm torque=2.000Nm

Cycle 3: Disable motor
Cycle 3: Enable motor
motor state: mode=3 angle=0.00° rpm=0.00 torque=0.000Nm temp=35.0°C 
   limits: speed=800.000rpm torque=2.000Nm

--- Enable/disable cycle completed ---

╔═══════════════════════════════════════╗
   Main Test Loop: Continuous Angle Control
╚═══════════════════════════════════════╝

Loop 10: angle=45.00° direction=+1.0
motor state: mode=3 angle=45.00° rpm=600.00 torque=1.200Nm temp=36.0°C 
   limits: speed=800.000rpm torque=2.000Nm

Loop 20: angle=90.00° direction=+1.0
motor state: mode=3 angle=90.00° rpm=600.00 torque=1.150Nm temp=36.0°C 
   limits: speed=800.000rpm torque=2.000Nm

Loop 30: angle=135.00° direction=+1.0
motor state: mode=3 angle=135.00° rpm=600.00 torque=1.100Nm temp=36.0°C 
   limits: speed=800.000rpm torque=2.000Nm

Loop 40: angle=180.00° direction=-1.0
Reached +180°, changing direction to -1
motor state: mode=3 angle=180.00° rpm=600.00 torque=1.150Nm temp=36.0°C 
   limits: speed=800.000rpm torque=2.000Nm
```

## 快速测试

如果只想快速测试电机基本功能，可以：

1. 只运行启动部分：
   ```c
   motor_control(lk_motor, ENABLE_MOTOR);
   while (1) {
       motor_status_t target = {
           .mode = ML_ANGLE,
           .angle = 90.0f,
           .speed_limit = {800.0f, 800.0f},
       };
       motor_set(lk_motor, &target);
       log_status();
       k_msleep(500);
   }
   ```

2. 启用 trace_lifecycle 查看 CAN 帧：
   ```c
   // 在 app.overlay 中添加
   lk_motor: lk_motor {
       compatible = "lk,test-motor";
       id = <1>;
       trace_lifecycle;  // 添加这一行
   };
   ```

## 关键测试点

1. ✅ **异步发送**: 所有命令立即返回，无阻塞
2. ✅ **命令覆盖**: 测试所有支持的命令
3. ✅ **模式切换**: 验证 3 种控制模式
4. ✅ **启停循环**: 验证重连机制
5. ✅ **实时控制**: 验证持续位置控制
6. ✅ **详细日志**: 方便调试和问题诊断

## 工程参考

- 参考完整 demo：`samples/motor/lk_test_sched_demo/`
- 驱动文档：`drivers/motor/lk_test/README.md`
- 协议手册：`drivers/motor/lk_test/20260326142653f.pdf`
- CAN 调度器：`drivers/motor/common/motor_can_sched.h`
