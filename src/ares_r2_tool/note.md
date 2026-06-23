# ARES R2 Tool 通信说明（connector 第一版）

`app/ares_r2_tool` 负责新机器人的对接器(connector/spear)。外部控制复用 ARES
`dual_protocol` 的 **SYNC 帧**承载（参考 `app/relay_comm`），走 USB bulk。

**“帧头” = SYNC ID = 动作对象**；**动作 enum** 放在 payload 首字段，其后是 4 个预留 float32。

## 帧格式（SYNC，小端，24 字节）

| 偏移 | 长度 | 字段 | 说明 |
| --- | ---: | --- | --- |
| 0 | 2 | `head` | SYNC 外层 magic，固定 `0x5A5A` |
| 2 | 2 | `id` | **帧头 = 动作对象 / 帧类型** |
| 4 | 4 | `action` | 动作 enum (uint32) |
| 8 | 16 | `reserved` | 4 个预留 `float32`，当前恒为 `0` |

FUNC/SYNC 帧末尾不附加 CRC（`DUAL_PROPOSE_PROTOCOL_DEFINE`）。

## 指令帧（上位机 -> 本板）

`id = 0x0204`（动作对象 = connector/spear）。`action` 取值：

| action | 名称 | 行为 |
| ---: | --- | --- |
| 1 | `prepare` | 初始化：张开夹爪/解锁机构(气动待接入) + pitch/roll/wye 移动到待抓取位姿 |
| 2 | `grasp` | 单一夹取动作；气动设备未接入，仅保留函数(空实现) |
| 3 | `dock_extend` | 转动 roll 将矛头伸出到准备对接位置（取消原 wye 往返动作）|

`reserved` 的 4 个 `float32` 当前填 `0`，后续可用于传参（如角度）。

> `action=4`(`dock_release`) 为释放气动夹爪并收回机械臂的动作，与 `grasp` 同为气动占位
> 函数；**本版本不由指令触发**（收到也会被忽略并告警）。

## 反馈帧 / 完成帧（本板 -> 上位机）

`id = 0x0204`，与指令帧**共用同一 ID**，表明这是对应机构(connector)的反馈，靠收/发方向区分。
动作执行线程跑完一个指令后回发一帧，`action` 字段回显刚完成的动作，`reserved` 4×`float32` = `0`。

指令是异步执行的：收到指令帧后立即唤醒 `spear` 线程执行电机动作，完成后才发反馈帧。

> 实现细节：指令帧与反馈帧在 sync 表中共用 ID 0x0204；`find_pack` 返回首个匹配项，
> 因此带回调的指令帧 pack 先注册(负责收帧)，反馈帧 pack 后注册(仅用于回发)。

## 设备 / 范围

- 电机、CAN、PID 配置与上一版一致（见 `app.overlay`、`prj.conf`、`device.h`）。
- 连接器电机(connector pitch/wye/roll)由 `enable_connector()` 就绪时使能，随后 `connector_init()` 归零。
- 本版本仅实现 connector；arm 仅在 `device.h` 保留设备实例，`main.c` 不涉及。

## 上位机对接

ROS2 侧(`app/ares_ws`)目前发送的是旧的 FUNC 帧(`0x0203/0x0204`)，与本版本的 SYNC
指令帧**不兼容**，需要后续改为发送上表的 SYNC 指令帧并(可选)接收完成帧：

- 指令帧：`5A 5A | 04 02 | <action u32 LE> | <16 字节 0>`
- 反馈帧：`5A 5A | 04 02 | <action u32 LE> | <16 字节 0>`（同 ID，方向相反）
