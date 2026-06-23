# LK Test Driver for MF9025E

`lk,test-motor` 是按 LK MF9025E V2.0 协议实现的电机驱动，支持多圈位置、速度、转矩控制模式。

## 协议概述

- **CAN 标准**: CAN 2.0 标准帧，8 字节数据，小端序
- **波特率**: 1 Mbps
- **数据频率**: 5 Mbps（需配置 RS485 引脚）
- **电机 ID**: 1~32
- **CAN ID**: 命令帧 = 0x140 + ID，回复帧 = 0x140 + ID
- **通信**: 主机发送命令，电机 1:1 应答

## 命令帧格式

| 命令 | DATA[0] | 功能 | DATA 布局 |
|------|---------|------|-----------|
| 读状态 | 0x9C | 读取电机状态 | 1字节 |
| 电机关闭 | 0x80 | 关闭电机 | 1字节 |
| 电机刹车 | 0x81 | 刹车（未实现） | 1字节 |
| 电机运行 | 0x88 | 使能电机 | 1字节 |
| 设置零点 | 0x95 | 设置当前位置为零位 | 1字节 |
| 转矩闭环 | 0xA1 | 转矩控制 | [0]=0xA1, [4:5]=电流(0~2000) |
| 速度闭环 | 0xA2 | 速度控制 | [0]=0xA2, [2:3]=转矩限制, [4:7]=速度(0.01dps) |
| 多圈位置 | 0xA4 | 多圈位置控制 | [0]=0xA4, [2:3]=速度限制(dps), [4:7]=角度(0.01°) |
| 单圈位置 | 0xA5 | 单圈位置控制（未实现） | [0]=0xA5, [2:3]=速度限制, [4:5]=角度(0~65535) |
| 读错误 | 0x9A | 读取错误状态 | 1字节 |
| 清除错误 | 0x9B | 清除错误标志 | 1字节 |
| 修改PID | 0xC1 | 修改PID参数 | [0]=0xC1, [1]=子命令, [2:3]=Kp, [4:5]=Ki, [6:7]=Kd |

**PID 子命令**:
- 角度: 0x0A
- 速度: 0x0B
- 转矩: 0x0C

## 反馈帧格式

### 状态2帧 (0x9C/0xA1/0xA2/0xA4/0xA5 应答)
| 字节 | 内容 | 类型 | 缩放 |
|------|------|------|------|
| [0] | 命令回显 | uint8 | - |
| [1] | 温度 | int8 | 1°C/LSB |
| [2:3] | 转矩电流 | int16 | 反馈值/2048*16 = Nm |
| [4:5] | 速度 | int16 | 1 dps/LSB, /6 = RPM |
| [6:7] | 编码器位置 | uint16 | 0~65535 = 0~360° |

### 状态1帧 (0x9A/0x9B 应答)
| 字节 | 内容 |
|------|------|
| [0] | 命令回显 |
| [1] | 温度 |
| [2:6] | 保留 |
| [7] | 错误标志位 |

### 错误标志位定义
| bit | 含义 |
|-----|------|
| 0 | 电机堵转 |
| 1 | 过压 |
| 2 | 欠压 |
| 3 | MOS管过温 |
| 4 | 电机线圈过温 |
| 5 | MOS管传感器故障 |
| 6 | 编码器故障 |

## 驱动架构 (2层)

- `motor_lk_test.c` - facade + lifecycle + transport 层
  - 对接 Zephyr motor API (`motor_set/get/control`)
  - 状态管理：target, feedback, online, enabled
  - CAN 通信：scheduler、offline 检测、PID 同步
- `motor_lk_test_codec.c` - protocol codec 层
  - 帧打包与解析
  - 协议常量定义
  - 回复匹配逻辑

## 支持的控制模式

### ML_ANGLE (多圈位置)
- 命令：0xA4
- 输入：`angle` (角度), `speed_limit[0]` (速度限制)
- 内部：角度 ×100, 速度限制直接使用

### ML_SPEED (速度)
- 命令：0xA2
- 输入：`rpm` (转速), `torque_limit[0]` (力矩限制)
- 转换：rpm × 6 = dps, 力矩限制直接使用

### ML_TORQUE (力矩)
- 命令：0xA1
- 输入：`torque` (力矩)
- 转换：`torque / 3.0 * 2048`，限制在 [-2048, 2047]

## 命令支持

- `ENABLE_MOTOR` - 电机运行 (0x88)
- `DISABLE_MOTOR` - 电机关闭 (0x80)
- `SET_ZERO` - 设置零点 (0x95)
- `CLEAR_ERROR` - 清除错误 (0x9B)
- `CLEAR_PID` - 清空并重发 PID 参数

`CLEAR_PID` 会把驱动缓存的 PID 参数清零并重新下发，实机测试 demo 默认不执行该命令。

## 重连机制

- **离线检测**: 使能后由 watchdog 每 20ms 检查一次反馈时间，80ms 无反馈判定为 offline
- **自动重连**: offline 且仍处于 enabled 状态时，额外以 20Hz 发送 `ENABLE_MOTOR` (0x88) 帧
- **恢复判定**: 任意合法回复帧恢复后将电机标记为 online
- **PID 重同步**: 恢复 online 后自动重新同步所有 PID 参数，并继续更新周期控制帧

## 异步发送架构

### 设计理念
- **public API 异步返回**：`motor_control()` / `motor_set()` 提交到 scheduler 后立即返回
- **命令回复追踪**：one-shot 命令会追踪协议回复帧，但不会阻塞调用线程
- **重试保证可靠**：未收到回复的 one-shot 命令由 scheduler 自动重试（最多 2 次）
- **状态异步通知**：通过 RX 帧异步更新状态

### 发送流程
```c
motor_control(dev, ENABLE_MOTOR);  // 立即返回
    ↓ 发送到 scheduler
    ↓ 建立回复追踪 pending
    ↓ 电机通常在 0.25ms 内回复同 ID 帧
    ↓ 未收到匹配回复则自动重试（最多 2 次，2ms 超时）
```

### 回复匹配

单电机命令和回复使用相同 CAN ID：`0x140 + ID`。驱动用以下规则匹配回复：

- 普通命令：回复帧 `DATA[0]` 必须等于发送帧 `DATA[0]`
- PID 参数命令 `0xC1`：回复帧 `DATA[0]` 和 `DATA[1]` 都必须匹配
- 周期控制帧默认不启用 reply tracking，避免电机掉线时高频控制帧持续重试导致总线负载上升

### 重试机制
- 超时时长：2ms
- 最大重试次数：2 次
- 总超时窗口：约 4ms
- pending 会在实际 `can_send()` 前建立，可捕获很快返回的回复帧

### 性能优势
- **启停控制**：0ms 延迟（原 8-24ms）
- **所有命令**：立即返回，无阻塞
- **重试保护**：自动处理网络波动

## 设备树配置

```c
compatible = "lk,test-motor";
id = <4>;                    // 电机 ID，CAN ID = 0x144
can_channel = <&can1>;        // CAN 设备
freq = <200>;                // 控制频率 (Hz)
controllers = <&pid1 &pid2 &pid3>; // PID 控制器
capabilities = "angle", "speed", "torque"; // PID 模式
trace_lifecycle;           // 启用帧级日志
```

## PID 配置

每个 PID 控制器需在 devicetree 中声明：
```c
pid_controller {
    compatible = "pid,single";
    k_p = "50";    // 比例系数
    k_i = "1";     // 积分系数
    k_d = "15";    // 微分系数
};
```

## 测试示例

```c
static const struct device *motor = DEVICE_DT_GET(DT_PATH(motor, lk_motor));

motor_control(motor, ENABLE_MOTOR);
k_msleep(500);

motor_status_t target = {
    .mode = ML_ANGLE,
    .angle = 90.0f,
    .speed_limit = {800.0f, 800.0f},
};
motor_set(motor, &target);

motor_status_t status;
motor_get(motor, &status);
```

## 已知差异与限制

| 协议差异 | PDF 手册 | 驱动实现 | 说明 |
|---------|---------|----------|------|
| 置零命令 | 0x19 | 0x95 | 旧驱动版本，实测可用 |
| PID 命令 | 0x31 | 0xC1 | 旧驱动版本，实测可用 |
| 转矩范围 | 0~2000 | ±2048 | 不同固件版本差异 |
| 转矩反馈 | raw/2048*16 Nm | raw*16/4096 Nm | 差 2 倍，需实测验证 |

## 未实现功能

- 单圈位置控制 (0xA5)
- 电机刹车 (0x81)
- 读设备 ID (0x30)
- 波特率设置 (0x32)
- 参数持久化 (0x33)

## 工程参考

- 参考旧驱动：`drivers/motor/lk/`
- CAN 调度器：`drivers/motor/common/motor_can_sched.h`
- 电机控制 API：`include/zephyr/drivers/motor.h`
- 用户手册：[20260326142653f.pdf](20260326142653f.pdf)

## 测试入口

- 完整测试：`samples/motor/lk_test_sched_demo/`
- 构建命令：
  ```bash
  west build -b robomaster_board_c samples/motor/lk_test_sched_demo \
    -- -DZEPHYR_EXTRA_MODULES=$PWD
  ```
