# AresPlot UART 示例

该示例展示 AresPlot 协议与 UART 接口的基本集成方式。应用会生成若干示例变量，并通过 AresPlot 协议发送给上位机。

## 功能

- 注册示例变量并通过 UART 输出。
- 支持上位机选择变量、设置采样周期和写入变量。
- 使用 Ares 接口抽象，协议层与传输层解耦。

## 硬件要求

- 支持 UART 的 Zephyr 开发板，例如 `robomaster_board_c`。
- 开发板 UART 与上位机串口连接。

## 构建和运行

```bash
west build -b robomaster_board_c samples/communication/plotter_demo

west flash
```

## 使用方法

1. 将开发板 UART 连接到上位机。
2. 使用 Chromium 内核浏览器打开 `https://captainkaz.github.io/web-serial-plotter/`。
3. 选择串口，波特率设置为 `921600`，协议选择 `AresPlot`。
4. 可将 `build/zephyr/zephyr.elf` 导入上位机工具，通过变量名选择监控对象并开始采集。新的采集配置会覆盖当前监视列表。

## 监控的变量

- `sine_wave`: 正弦波值 (float)
- `counter`: 递增计数器 (int32_t)
- `random_value`: 随机值 (int32_t)
- `toggle_flag`: 布尔标志 (bool)

## 协议配置

协议配置可以通过 Kconfig 选项调整：

- `CONFIG_ARESPLOT_MAX_VARS_TO_MONITOR`: 最大监控变量数量
- `CONFIG_ARESPLOT_SHARED_BUFFER_SIZE`: 缓冲区大小
- `CONFIG_ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS`: 默认采样周期

## 代码结构

- `src/main.c`: 主应用程序
- `prj.conf`: 项目配置
