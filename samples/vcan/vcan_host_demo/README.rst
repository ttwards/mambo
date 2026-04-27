VCAN Host Demo
##############

这个示例跑在 Host 板上，验证 ``drivers/vcan`` 的双通道轮询同步逻辑。

设备映射：

- 本地 ``can1`` <-> 远端虚拟 ``can_v0``
- 本地 ``can2`` <-> 远端虚拟 ``can_v1``

测试内容：

1. ``can1 -> slave can1 -> can_v0``
2. ``can_v0 -> slave can1 -> can1``
3. ``can2 -> slave can2 -> can_v1``
4. ``can_v1 -> slave can2 -> can2``

接线约定：

- SPI2: ``PB12 NSS/CS``、``PB13 SCK``、``PB14 MISO``、``PB15 MOSI``
- Slave -> Host INT: ``PB8``
- 物理 CAN1 调试链路: 两块板子的 ``can1`` 互连
- 物理 CAN2 调试链路: 两块板子的 ``can2`` 互连

说明：

- Host 驱动内部固定每 ``500 us`` 兜底扫一次；本地待发 CAN/bitrate 更新还会触发一次约 ``50 us`` 的短聚合唤醒，避免一帧一笔 SPI 事务。
- 每次 SPI 同步帧最多打包 ``4`` 个 CAN 帧，帧内自带 ``channel`` 字段来区分 ``can1/can2``。
- Host demo 当前按上限压测：两路 ``1 MHz`` CAN 在 ``11-bit ID + 8-byte data`` 理想模型下约 ``4504 fps/方向/通道``，但现有 VCAN 代码侧 ``500 us * 4 frame`` 的瓶颈更紧，最终采用 ``4000 fps/方向/通道``。
- 当前简化方案不依赖 ``HOST_INT`` 反向提示线。
