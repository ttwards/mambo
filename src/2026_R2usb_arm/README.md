# 机械臂区域控制通信系统

## 架构

```
PC (USB)
 ↕ USB Bulk (dual_protocol)
2026_R2usb_arm (网关板: CAN电机 + IMU + 底盘 + 桥接)
 ↕ UART usart6 (dual_protocol)
relay_comm (继电器控制板: 继电器 + 电机角度采集)
```

## relay_comm — 继电器控制板

**位置：** `src/relay_comm/`

下位机，运行在 STM32 控制板上，负责 3 路继电器控制和电机角度采集。

### 硬件

| 功能 | 引脚 |
|------|------|
| 继电器 1 | PE9 |
| 继电器 2 | PE11 |
| 继电器 3 | PE13 |
| 通信 UART | usart6 |

### 协议

通过 ARES UART 双协议通信，无操作指令时静默（不发送）。

**接收帧 (0x0200, 上位机→本板) 8 float：**

| 索引 | 含义 |
|------|------|
| [0] | 操作指令枚举: 0=NONE, 1=RELAY_SET, 2=MOTOR_CTRL, 3=EMERGENCY_STOP |
| [1] | 对应数值 (RELAY_SET=位掩码, MOTOR_CTRL=目标角度) |
| [2..7] | 保留 |

**发送帧 (0x0100, 本板→上位机) 8 float：**

| 索引 | 含义 |
|------|------|
| [0] | 动作完成标志: DONE(0x4B0CD0F1) / PENDING(0x4E0DD0F1) |
| [1..4] | 4 路电机反馈角度 |
| [5..7] | 3 路继电器电平 (1.0=高, 0.0=低) |

### 脉冲机制

- 收到指令 → 发送 10 帧 DONE 标志 (100ms @ 100Hz) → 自动回到 PENDING
- 脉冲期间屏蔽新指令，防止完成标志重叠导致误触发
- DONE/PENDING 使用 uint32 magic number 映射为 float，避免串口噪声误判

---

## 2026_R2usb_arm — 机械臂区域控制网关

**位置：** `src/2026_R2usb_arm/`

上位网关板，基于 `2026_R2usb_imu` 扩展，保留全部原有功能并新增 relay 桥接。

### 原有功能

- 4 路 DM 悬架电机 (CAN, MIT 模式)
- 4 路 VESC 轮毂电机 (CAN, 速度模式)
- 4 路 LK 转向电机 (CAN, 角度/速度/扭矩模式)
- 4 路光电传感器 (GPIO 中断)
- 底盘控制 (CHASSIS_SRC_AUTO)
- USB Bulk 通信 (与 PC)

### 新增功能

- UART 桥接 relay_comm，双向转发
- 标志位转换：UART 侧 magic number ↔ USB 侧简化值 (-1.0/1.0)
- 统一日志：电机 + 继电器 数据合并输出

### USB 同步 ID

| ID | 方向 | 用途 | 数据量 |
|----|------|------|--------|
| 0x0101 | PC→MCU | 底盘速度指令 | 3 float |
| 0x0102 | PC→MCU | 悬架高度指令 | 4 float |
| 0x0103 | PC→MCU | relay 指令 (转发给 relay_comm) | 8 float |
| 0x0201 | MCU→PC | 底盘遥测 (传感器+高度+底盘) | 12 float |
| 0x0202 | MCU→PC | relay 状态 (角度+继电器) | 8 float |

### 标志位转换规则

| 链路 | DONE | PENDING |
|------|------|---------|
| UART (对 relay_comm) | `0x4B0CD0F1` as float | `0x4E0DD0F1` as float |
| USB (对 PC) | `-1.0f` | `1.0f` |

### USB 0x0202 上传数据格式 (8 float)

| 索引 | 含义 |
|------|------|
| [0] | 动作完成标志: -1.0f=DONE, 1.0f=PENDING |
| [1..4] | 4 路电机反馈角度 (当前用 DM 电机角度作为测试数据) |
| [5..7] | 3 路继电器电平 (1.0=高, 0.0=低) |

### USB 0x0103 下发数据格式 (8 float)

直接转发给 relay_comm，格式同 relay_comm 接收帧。

### 日志输出示例

每约 1 秒输出：
```
[INF] block: 0.0, 0.0, 0.0, 0.0
[INF] high: 30.0, 30.0, 30.0, 30.0
[INF] X=0.0 Y=0.0 Gyro=0.0
[INF] target_high: 30.00, 30.00, 30.00, 30.00
[INF] rx_count: chassis=100 height=100
[INF] relay: done=-1.0 angles=[30.0,30.0,30.0,30.0] R=[1,0,1] uart_rx=500 uart_tx=3 usb_rx=3 usb_tx=500
```

---

## 编译

```bash
# relay_comm
west build -b robomaster_board_c src/relay_comm

# 2026_R2usb_arm
west build -b robomaster_board_c src/2026_R2usb_arm
```
