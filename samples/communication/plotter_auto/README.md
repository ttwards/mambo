# AresPlot 自动采样示例

该示例展示 AresPlot 协议的自动初始化和周期采样流程。应用启动后会注册示例变量，并按配置周期通过 UART 向上位机发送采样数据。

## 功能

- 周期采样示例变量。
- 通过 UART 接口输出 AresPlot 数据帧。
- 使用 Kconfig 启用协议自动初始化。

## 硬件要求

- 支持 UART 的 Zephyr 开发板，例如 `robomaster_board_c`。
- 开发板 UART 与上位机串口连接。

## 构建和运行

```bash
west build -b robomaster_board_c samples/communication/plotter_auto

west flash
```

## 使用方法

1. 将开发板 UART 连接到上位机。
2. 使用 Chromium 内核浏览器打开 `https://captainkaz.github.io/web-serial-plotter/`。
3. 选择串口，波特率设置为 `921600`，协议选择 `AresPlot`。
4. 开发板启动后会自动发送示例变量的采样数据。
5. 如需按符号选择变量，可将 `build/zephyr/zephyr.elf` 导入上位机工具后重新开始采集。新的采集配置会覆盖当前监视列表。

## 监控的变量

- `sine_wave`: 正弦波值 (float)
- `counter`: 递增计数器 (int32_t)
- `random_value`: 随机值 (int32_t)
- `toggle_flag`: 布尔标志 (bool)

## 协议配置

该示例使用 `CONFIG_PLOTTER=y` 启用自动初始化。采样频率由 `CONFIG_ARESPLOT_FREQ` 配置。

## 代码结构

- `src/main.c`: 主应用程序
- `prj.conf`: 项目配置
