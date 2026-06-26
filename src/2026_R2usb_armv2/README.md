# 2026_R2usb_armv2

机械臂区域底盘与悬挂控制包。

## 功能

- 4 路 DM 悬挂电机，MIT 模式
- 4 路 VESC 轮毂电机，速度模式
- 4 路 LK 转向电机，角度/速度/扭矩模式
- 4 路光电传感器，GPIO 中断
- USB Bulk 与 PC 通信
- SBUS 手动底盘控制
- SBUS 手动模式下两通道三档悬挂高度控制

## 控制来源

SBUS channel 5 用作控制来源选择：

| channel 5 | 底盘和悬挂来源 |
|-----------|----------------|
| > 0.3 | SBUS 手动控制 |
| <= 0.3 | USB `0x0111/0x0112` |

SBUS 手动控制时，底盘通道如下：

| SBUS channel | 用途 |
|--------------|------|
| channel 0 | 旋转 |
| channel 1 | 平移 Y |
| channel 3 | 平移 X |

channel 6 和 channel 7 分别控制两组悬挂高度：

| SBUS channel | 悬挂电机 |
|--------------|----------|
| channel 6 | 0, 1 |
| channel 7 | 2, 3 |

每个通道都是三档：

| 通道值 | 目标高度 |
|--------|----------|
| < -0.3 | -20 |
| -0.3 到 0.3 | 20 |
| > 0.3 | 200 |

## USB 同步 ID

| ID | 方向 | 用途 | 数据量 |
|----|------|------|--------|
| 0x0111 | PC -> MCU | 底盘速度指令 | 3 float |
| 0x0112 | PC -> MCU | 悬挂高度指令 | 4 float |
| 0x0121 | MCU -> PC | 遥测，传感器+高度+底盘 | 12 float |

## 编译

```bash
west build -b robomaster_board_c src/2026_R2usb_armv2
```
