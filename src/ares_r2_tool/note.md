# ARES R2 Tool 通信说明（connector 第一版）

`app/ares_r2_tool` 负责新机器人的对接器(connector/spear)。外部控制复用 ARES
`dual_protocol` 的 **SYNC 帧**承载（参考 `app/relay_comm`），走 USB bulk。

**“帧头” = SYNC ID = 动作对象**；**动作 enum** 放在 payload 首字段，其后是 4 个预留 float32。

## ID 分配

| 对象 | 接收 ID | 发送 ID |
| --- | --- | --- |
| arm | `0x0211` | `0x0221` |
| connector/spear | `0x0212` | `0x0222` |

## 帧格式（SYNC，小端，24 字节）

| 偏移 | 长度 | 字段 | 说明 |
| --- | ---: | --- | --- |
| 0 | 2 | `head` | SYNC 外层 magic，固定 `0x5A5A` |
| 2 | 2 | `id` | **帧头 = 动作对象 / 帧类型** |
| 4 | 4 | `action` | 动作 enum (uint32) |
| 8 | 16 | `reserved` | 4 个预留 `float32`，当前恒为 `0` |

FUNC/SYNC 帧末尾不附加 CRC（`DUAL_PROPOSE_PROTOCOL_DEFINE`）。

## 指令帧（上位机 -> 本板）

connector/spear 接收 `id = 0x0212`。`action` 取值：

| action | 名称 | 行为 |
| ---: | --- | --- |
| 1 | `prepare` | 初始化：张开夹爪/解锁机构(气动待接入) + pitch/roll/wye 移动到待抓取位姿 |
| 2 | `grasp` | 单一夹取动作；气动设备未接入，仅保留函数(空实现) |
| 3 | `dock_extend` | 转动 roll 将矛头伸出到准备对接位置（取消原 wye 往返动作）|

`reserved` 的 4 个 `float32` 当前填 `0`，后续可用于传参（如角度）。

> `action=4`(`dock_release`) 为释放气动夹爪并收回机械臂的动作，与 `grasp` 同为气动占位
> 函数；**本版本不由指令触发**（收到也会被忽略并告警）。

## 反馈帧 / 完成帧（本板 -> 上位机）

connector/spear 发送 `id = 0x0222`。动作执行线程跑完一个指令后回发一帧，`action` 字段回显刚完成的动作，`reserved` 4×`float32` = `0`。

指令是异步执行的：收到指令帧后立即唤醒 `spear` 线程执行电机动作，完成后才发反馈帧。

> 当前 connector/spear 协议使用独立收发 ID：接收 `0x0212`，发送 `0x0222`。

## 设备 / 范围

- 电机、CAN、PID 配置与上一版一致（见 `app.overlay`、`prj.conf`、`device.h`）。
- 连接器电机(connector pitch/wye/roll)由 `enable_connector()` 就绪时使能，随后 `connector_init()` 归零，并自动执行一次 `prepare`，默认使用 `connector_wye_center_length` 作为长度参数。
- 机械臂收到 `arm_grasp` 后，抓取完成即回发成功反馈；后续存放由下位机自动继续执行：第 1 次存车体，第 2 次暂存在 arm 上，第 3 次及以后先丢掉 arm 上已有物，再把新抓取物暂存在 arm 上。

## 上位机对接

ROS2 侧(`app/ares_ws`)发送本版本的 SYNC 指令帧，并等待对应发送 ID、同 action 的完成反馈帧：

- 指令帧：`5A 5A | 12 02 | <action u32 LE> | <16 字节 0>`
- 反馈帧：`5A 5A | 22 02 | <action u32 LE> | <16 字节 0>`
