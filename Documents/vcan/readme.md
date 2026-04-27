# Zephyr VCAN 双路 SPI 桥驱动说明

这个目录里的实现，目标是让 Host 板通过 `SPI` 访问另一块板子上的两路物理 CAN，并在 Host 侧把它们表现成标准的 Zephyr `can` 设备。

如果你要看“代码是怎么实现的、各层 API 分别干什么、为什么这样设计”，请直接读：

- [drivers/vcan/IMPLEMENTATION.md](/home/xiexiang/zephyrproject/zephyr_ws/drivers/vcan/IMPLEMENTATION.md)

当前实现已经收敛成“固定节拍轮询 + 本地发送短聚合唤醒 + 最小同步面”模型：

- Host 固定 `2000 Hz` 兜底轮询，也就是最慢每 `500 us` 进入一次服务周期
- 本地 `TX/bitrate` 更新会额外触发一次约 `50 us` 的短聚合窗口，尽量把 burst 打包进同一笔 SPI 同步
- Host/Slave 之间只同步三类信息：
  - CAN 帧本身
  - 帧所属 `channel`，也就是远端 `can1/can2`
  - 每个 channel 的 `bitrate`
- Slave 不再依赖 Host 显式下发 `mode/start/stop/recover`
- Slave 收到 `bitrate` 后在本地自己 `can_calc_timing()`，并按需自动启动对应 CAN 通道

这套模型的重点就是“简单、稳定、可控”：不追求极限低延迟，但把协议和时序收敛到最小集合，联调时更容易定位问题。

## 1. 当前方案一眼看懂

- Host 只有满足下面任一条件时，才会真的发起一次 SPI 同步帧：
  - 本地有待发送的 CAN 帧
  - 本地有待同步的 `bitrate`
  - Slave 的 `INT` 引脚为高，表示远端有 CAN 帧或状态要上报
- 每次 SPI 事务使用固定长度 `192` 字节同步帧
- 一次同步帧里最多同时搬运 `4` 个 CAN 帧
- 每个 CAN 条目里都带 `channel` 字段，所以可以区分 `channel 0 / channel 1`
- Slave 侧的 `INT` 是电平语义，不是边沿语义：
  - 只要本地 RX 队列非空，或者 CAN 状态有变化还没同步给 Host，`INT` 就一直保持为高
  - Host 下一次 `500 us` 轮询时就会把它们取走

## 2. 代码结构

### `spi_can_mfd.c`

父设备，负责：

- SPI 固定长度同步帧打包/解包
- 手动 CS 拉低/拉高
- `500 us` 轮询线程
- 检查是否需要发起 SPI 事务
- 维护每个通道的待发队列、待同步 `bitrate`
- 把 Slave 回来的 CAN 帧分发给对应子节点

### `spi_can_node.c`

子设备，负责实现标准 `can_driver_api`：

- `start/stop`
- `set_mode`
- `set_timing`
- `send`
- `add_rx_filter/remove_rx_filter`
- `get_state`

它本身不直接碰 SPI，而是把请求交给父设备。

当前约束：

- `set_mode()` 只接受 `CAN_MODE_NORMAL`
- `set_timing()` 只在本地换算并缓存 `bitrate`
- `send()` 如果虚拟 CAN 还没 `start()`，会自动补一次启动同步，Host 侧发帧更无感

### `spi_can_mfd.h`

父子设备之间共享的接口定义。

## 3. Host 侧真实工作流程

### 3.1 Host 发送 CAN

1. 上层调用 `can_send(can_v0/can_v1, ...)`
2. 如果对应虚拟通道还没启动，`spi_can_node_send()` 会先按当前 `bitrate` 自动补一次启动同步
3. `spi_can_node_send()` 把帧交给 `spi_can_mfd_send()`
4. 帧进入对应 channel 的 `k_msgq`
5. Host 父驱动会启动一次约 `50 us` 的短聚合窗口，然后唤醒 SPI 服务线程，不用死等到下一拍，也避免“一帧触发一笔 SPI 事务”
6. `spi_can_build_host_frame_locked()` 会从两个 channel 的队列里按轮询方式取帧，最多取 `4` 帧，写入本次 SPI 同步包

### 3.2 Host 接收远端 CAN

1. Slave 本地收到物理 CAN 后，把帧塞进自己的 RX 队列
2. 只要队列非空，Slave 就保持 `INT=1`
3. Host 的轮询线程在 `spi_can_should_transfer_locked()` 里看到 `INT` 为高，就发起 SPI 事务
4. `spi_can_parse_slave_frame_locked()` 解析 Slave 回包
5. 回包里的每个 CAN 条目都带 `channel`
6. Host 根据 `channel` 把帧转给 `can_v0` 或 `can_v1`
7. 子节点再按本地软件 filter 分发给 Zephyr 上层回调

### 3.3 Bitrate 同步和从机自启动

Host -> Slave 不再同步 `mode/start/stop/recover` 这些控制量，只保留每个 channel 的 `bitrate_mask + bitrate`：

- 哪一位被置位，就表示该 channel 这次带了有效 `bitrate`
- 同一拍里如果该 channel 还有待发 CAN 帧，Host 也会顺带把 `bitrate` 带上去

Slave 收到后会这样处理：

1. 先缓存这个 channel 的目标 `bitrate`
2. 如果通道还没启动，或者 `bitrate` 发生了变化，就在本地重新 `can_calc_timing(..., bitrate, 0)`
3. 本地 `can_set_timing()` 成功后自动 `can_start()`
4. 启动完成后在回包状态里带上 `STARTED` 标志位

这样 Host/Slave 间只需要传“帧 + 通道 + 波特率”，时序和时钟差异都留在 Slave 本地解决。

## 4. SPI 同步帧格式

Host -> Slave 和 Slave -> Host 都是固定 `192` 字节。

### Host -> Slave

- 头部：
  - `magic`
  - `version`
  - `seq`
  - `tx_count`
  - `bitrate_mask`
- 控制区：
  - 两个 channel 的 `bitrate`
- 数据区：
  - 最多 `4` 个 `tx_entries`

### Slave -> Host

- 头部：
  - `magic`
  - `version`
  - `seq`
  - `rx_count`
  - `more_rx_mask`
  - `state_mask`
- 状态区：
  - 两个 channel 的 `state/tx_err/rx_err/flags`
- 数据区：
  - 最多 `4` 个 `rx_entries`

`tx_entries` 和 `rx_entries` 的结构相同：

```c
struct spi_can_sync_can_entry {
	uint32_t id;
	uint8_t dlc;
	uint8_t flags;
	uint8_t channel;
	uint8_t reserved;
	uint8_t data[8];
};
```

其中 `channel` 就是双路区分的关键。

## 5. 设备树如何写

绑定文件：

- `dts/bindings/can/custom,spi-can-mfd.yaml`
- `dts/bindings/can/custom,spi-can-node.yaml`

典型 DTS 例子：

```dts
&spi2 {
	status = "okay";
	cs-gpios = <&gpiob 12 GPIO_ACTIVE_LOW>;

	virtual_can_host: spi_can@0 {
		compatible = "custom,spi-can-mfd";
		reg = <0>;
		spi-max-frequency = <10000000>;
		can-core-clock = <42000000>;
		int-gpios = <&gpiob 8 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
		status = "okay";

		can_v0: can_0 {
			compatible = "custom,spi-can-node";
			status = "okay";
			can-channel = <0>;
			bitrate = <1000000>;
			sample-point = <875>;
		};

		can_v1: can_1 {
			compatible = "custom,spi-can-node";
			status = "okay";
			can-channel = <1>;
			bitrate = <1000000>;
			sample-point = <875>;
		};
	};
};
```

说明：

- `int-gpios` 必须接 Slave -> Host 的提示线
- `can-core-clock` 仍然建议填写远端 CAN 控制器的核心时钟，本板当前是 `42000000`
  - Host 运行时如果有人调用 `set_timing()`，会用它把 timing 换算成 `bitrate`
- `host-int-gpios` 目前保留为兼容属性，但当前轮询式实现不依赖它
- 子节点的 `can-channel` 只能是 `0` 或 `1`
- 子节点 DTS 里的 `bitrate` 现在是最关键配置项，Slave 会按它在本地自适应算 timing

## 6. Demo 在哪里，怎么测

本仓库里已经配了两端 demo：

- Host: [src/vcan_host_demo](/home/xiexiang/zephyrproject/zephyr_ws/src/vcan_host_demo)
- Slave: [src/vcan_slave_demo](/home/xiexiang/zephyrproject/zephyr_ws/src/vcan_slave_demo)
- 双板联调说明: [src/vcan_dual_board_debug.md](/home/xiexiang/zephyrproject/zephyr_ws/src/vcan_dual_board_debug.md)

### Demo 映射关系

- `channel 0` <-> Slave 本地 `can1` <-> Host 侧 `can_v0`
- `channel 1` <-> Slave 本地 `can2` <-> Host 侧 `can_v1`

### Host demo 会持续跑 4 个用例

1. `Host can1 -> Slave can1 -> Host vcan0`
2. `Host vcan0 -> Slave can1 -> Host can1`
3. `Host can2 -> Slave can2 -> Host vcan1`
4. `Host vcan1 -> Slave can2 -> Host can2`

### 构建命令

```bash
source ../.venv/bin/activate

XDG_CACHE_HOME=$PWD/.cache CCACHE_TEMPDIR=$PWD/.ccache/tmp \
west build -d build_host -p always -b robomaster_board_c src/vcan_host_demo -- -DZEPHYR_EXTRA_MODULES=$PWD

XDG_CACHE_HOME=$PWD/.cache CCACHE_TEMPDIR=$PWD/.ccache/tmp \
west build -d build_slave -p always -b robomaster_board_c src/vcan_slave_demo -- -DZEPHYR_EXTRA_MODULES=$PWD
```

### 接线

- `PB12 <-> PB12`: SPI2 CS
- `PB13 <-> PB13`: SPI2 SCK
- `PB14 <-> PB14`: SPI2 MISO
- `PB15 <-> PB15`: SPI2 MOSI
- `PB8 Slave -> Host`: INT
- `CAN1_TX/RX <-> CAN1_TX/RX`
- `CAN2_TX/RX <-> CAN2_TX/RX`
- `GND <-> GND`

### 预期现象

Host 日志里会持续出现：

- `TEST PASS channel0(can1<->vcan0) host->remote`
- `TEST PASS channel0(can1<->vcan0) remote->host`
- `TEST PASS channel1(can2<->vcan1) host->remote`
- `TEST PASS channel1(can2<->vcan1) remote->host`

Slave 日志里会看到：

- `slave INT assert`
- `channel=0 ready dev=can1 bitrate=...`
- `channel=1 ready dev=can2 bitrate=...`
- `CAN RX dev=can1`
- `CAN RX dev=can2`
- `CAN TX enqueue channel=0`
- `CAN TX enqueue channel=1`

## 7. 目前这版实现的边界

- Host 每拍最多搬 `4` 帧，不会在一拍里无限 drain
- 如果瞬时流量大于 `4` 帧，剩余帧会留到后续 `500 us` 周期继续取
- Slave 从 SPI 收到的 host->CAN 帧会先进入每通道 TX 队列，再由 CAN TX 线程发送；这样 `bxCAN` mailbox 短时忙时不会因为 `can_send(..., K_NO_WAIT)` 直接丢帧
- `host-int-gpios` 没参与当前逻辑
- Host 侧 RX filter 目前是本地软件过滤，不是下发到 Slave 硬件过滤
- Slave demo 当前按 `bitrate` 自动起通道；如果以后要支持更复杂模式，再单独扩协议
- `vcan_host_demo` 的压测值先按上限自动推导：`1 MHz`、标准 `11-bit ID + 8-byte data` 理想模型约 `4504 fps/方向/通道`，但现有 VCAN 代码侧 `500 us * 4 frame / 2 channel` 更紧，理论上限是 `4000 fps/方向/通道`；当前 demo 默认按 `3000 fps/方向/通道` 运行。由于测试周期固定为 `2 ms`，Host 当前每拍固定发送 `6` 帧；Slave 侧则按每通道 `3000 fps` 的节拍平滑出帧，避免 SPI 批量下发时在物理 CAN 上形成明显 burst

## 8. 后面继续扩展时最值得注意的点

- 如果要追更低延迟，可以再考虑“INT 拉高后立即补一拍 SPI”，但先别破坏现在这个固定节拍模型
- 如果后面要扩成 CAN FD，就要重做同步帧尺寸和 entry 结构
- 如果 Slave 端也做成正式驱动而不是 demo，最好把当前应用层协议解析搬到独立模块里，Host/Slave 共用头文件
