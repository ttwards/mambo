VCAN Slave Demo
###############

这个示例跑在 Slave 板上，用应用层代码实现与 ``drivers/vcan`` 对齐的 SPI 双通道桥接协议。

功能：

- SPI2 从机，协议帧固定为 ``192`` 字节
- ``channel 0`` 映射到本地 ``can1``
- ``channel 1`` 映射到本地 ``can2``
- ``PB8`` 作为电平型 ``INT`` 输出，只要有待取 CAN/状态就保持拉高
- 后台每 ``500 us`` 刷新一次 ``INT``，与 Host 的 ``2 kHz`` 轮询周期对齐

说明：

- Slave 每次 SPI 响应最多返回 ``4`` 个 CAN 帧，超过部分继续留在本地队列里，并保持 ``INT`` 为高。
- 当前简化方案不依赖 Host -> Slave 的反向 ``HOST_INT`` 提示线。
- 如果 Host demo 日志里四条 ``TEST PASS`` 都能持续出现，说明双通道映射和批量传输都已经跑通。
