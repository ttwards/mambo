# VCAN 双板联调说明

## 板子分工

- Host 板运行 [vcan_host_demo](/home/xiexiang/zephyrproject/zephyr_ws/src/vcan_host_demo)
- Slave 板运行 [vcan_slave_demo](/home/xiexiang/zephyrproject/zephyr_ws/src/vcan_slave_demo)

两块板子都使用 `robomaster_board_c`。

## 接线

- `PB12 <-> PB12`: SPI2 NSS / 手动 CS
- `PB13 <-> PB13`: SPI2 SCK
- `PB14 <-> PB14`: SPI2 MISO
- `PB15 <-> PB15`: SPI2 MOSI
- `PB8 Slave -> Host`: 数据/状态提示 INT
- `CAN1_RX/CAN1_TX`: 两块板子的 `can1` 互连
- `CAN2_RX/CAN2_TX`: 两块板子的 `can2` 互连
- `GND <-> GND`

## 构建

Host:

```bash
source ../.venv/bin/activate
XDG_CACHE_HOME=$PWD/.cache CCACHE_TEMPDIR=$PWD/.ccache/tmp \
west build -d build_host -p always -b robomaster_board_c src/vcan_host_demo -- -DZEPHYR_EXTRA_MODULES=$PWD
```

Slave:

```bash
XDG_CACHE_HOME=$PWD/.cache CCACHE_TEMPDIR=$PWD/.ccache/tmp \
west build -d build_slave -p always -b robomaster_board_c src/vcan_slave_demo -- -DZEPHYR_EXTRA_MODULES=$PWD
```

## 传输模型

- Host 侧固定每 `500 us` 检查一次。
- 只有“本地有待发 CAN 帧/控制命令”或“`PB8` 为高”时，Host 才真的做一次 SPI 同步事务。
- 每次同步事务最多搬运 `4` 个 CAN 帧，帧里带 `channel` 字段区分 `can1/can2`。
- Slave 侧只要本地 RX 队列非空或状态变更未上报，就持续把 `PB8` 拉高。
- Host demo 压测上限按当前代码自动推到 `4000 fps/方向/通道`；这低于双向均分后 `1 MHz CAN` 的理想值约 `4504 fps/方向/通道`，所以现阶段瓶颈在 VCAN 代码侧而不是 CAN 总线侧。

## 测试用例

1. `Host can1 -> Slave can1 -> Host vcan0`
2. `Host vcan0 -> Slave can1 -> Host can1`
3. `Host can2 -> Slave can2 -> Host vcan1`
4. `Host vcan1 -> Slave can2 -> Host can2`

## 预期日志关键词

Host:

- `TEST PASS channel0(can1<->vcan0) host->remote`
- `TEST PASS channel0(can1<->vcan0) remote->host`
- `TEST PASS channel1(can2<->vcan1) host->remote`
- `TEST PASS channel1(can2<->vcan1) remote->host`
- `summary`

Slave:

- `slave INT assert`
- `CAN RX dev=can1`
- `CAN RX dev=can2`
- `CAN TX enqueue channel=0`
- `CAN TX enqueue channel=1`

## 出问题时优先看什么

- Host 是否持续打印四条 `TEST PASS`
- Slave 的 `PB8` 是否会在有数据时拉高
- 如果失败，贴 `Host` 最近 20 行和 `Slave` 最近 20 行日志
