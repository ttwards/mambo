# VCAN 实现细节说明

这份文档面向继续维护这个项目的人，目标不是重复 README 里的“怎么用”，而是解释这套 VCAN 方案为什么这样设计、代码是怎么分层的、关键 Zephyr API 在这里分别承担什么职责。

## 1. 目标与边界

这个项目实现的是一套“Host 侧虚拟 CAN + Slave 侧物理 CAN”的双通道 SPI 桥。

最终效果是：

- Host 板上暴露出两个标准 Zephyr `can` 设备：`can_v0`、`can_v1`
- 这两个虚拟 CAN 设备背后并不直接连 CAN 控制器，而是通过 SPI 访问另一块板子上的 `can1`、`can2`
- 对 Host 上层应用来说，`can_v0/can_v1` 的使用方式尽量接近普通 `can` 设备

当前版本的约束也很明确：

- 只支持两路通道
- Host 和 Slave 之间同步固定长度 `192` 字节帧
- 每次同步最多搬运 `4` 个 CAN 帧
- Host 侧只支持 `CAN_MODE_NORMAL`
- 不实现完整远程 CAN 控制器管理协议，只同步：
  - CAN 帧
  - channel 编号
  - bitrate
  - 状态信息

这意味着它追求的是“简单、可控、够用”，而不是一开始就把 CAN 驱动所有能力都透传过去。

## 2. 目录与职责划分

### Host 侧驱动

- `drivers/vcan/spi_can_mfd.c`
  - 父设备
  - 负责 SPI 同步帧打包/解包
  - 维护每个通道的待发队列和状态缓存
  - 管理轮询线程、聚合定时器、手动 CS、INT 检查
  - 把 Slave 回包里的 CAN 帧分发给子设备

- `drivers/vcan/spi_can_node.c`
  - 子设备
  - 对上实现标准 `can_driver_api`
  - 对下调用 `spi_can_mfd_*()` 访问父设备
  - 维护软件 RX filter、本地状态缓存、启动状态

- `drivers/vcan/spi_can_mfd.h`
  - Host 父子设备之间共享的最小接口

### 设备树与构建配置

- `dts/bindings/can/custom,spi-can-mfd.yaml`
  - 定义父 SPI 桥节点属性

- `dts/bindings/can/custom,spi-can-node.yaml`
  - 定义子 CAN 通道节点属性

- `drivers/vcan/Kconfig`
  - 定义 RX filter 数量、工作线程栈、优先级、初始化优先级等

### Demo

- `src/vcan_host_demo`
  - Host 压测和联调示例

- `src/vcan_slave_demo`
  - Slave 端协议参考实现
  - 当前 Slave 不是正式驱动，而是应用层 demo

## 3. 为什么拆成“父设备 + 子设备”

这是这个项目最核心的接口设计。

### 父设备解决什么问题

SPI 总线、INT 引脚、同步协议、打包拆包、共享带宽，这些天然是“桥级别”的资源，不属于某一路 CAN。

所以父设备承担：

- 唯一 SPI owner
- 唯一的同步线程
- 唯一的 Host/Slave 协议编解码器
- 所有子通道的统一调度者

### 子设备解决什么问题

Zephyr 上层希望拿到的是标准 `can` 设备，而不是“一个 SPI 桥上的某个逻辑 channel”。

所以子设备承担：

- 暴露 `can_driver_api`
- 适配 Zephyr 的 CAN 生命周期
- 管理本地软件 filter
- 在上层看来像普通 CAN 控制器

这种设计的好处是：

- 上层代码不必知道 SPI 协议
- `can_send()`、`can_add_rx_filter()`、`can_get_state()` 这些接口还能继续用
- 桥接复杂度被关在父设备里

## 4. Host 侧关键数据结构

### `struct spi_can_mfd_config`

这是父设备的静态配置，来自设备树。

包含：

- `bus`
  - `struct spi_dt_spec`
  - 表示 SPI 总线参数和目标从设备

- `int_gpio`
  - Slave -> Host 的提示线

- `cs_gpio`
  - 手动 CS 引脚

- `can_core_clock`
  - 远端 CAN 控制器核心时钟
  - Host 侧 `set_timing()` 时要把 timing 重新换算回 bitrate，需要这个值

- `channels[]`
  - 对应子节点设备指针

### `struct spi_can_mfd_channel_data`

这是每个逻辑 CAN channel 的动态状态。

包含：

- `tx_queue`
  - 待发送到 Slave 的 CAN 帧队列

- `pending_bitrate`
  - 下次同步要送过去的 bitrate

- `bitrate_dirty`
  - 标记 bitrate 是否尚未同步到 Slave

- `cached_state` / `cached_err_cnt`
  - 最近一次从 Slave 拿到的状态快照

- `state_valid`
  - 状态缓存是否有效

### `struct spi_can_mfd_data`

这是父设备总状态。

关键字段：

- `tx_frame` / `rx_frame`
  - 一次固定长度 SPI 同步帧的发送/接收缓冲区

- `bus_lock`
  - 保护 SPI 同步事务和共享状态

- `service_sem`
  - 唤醒服务线程

- `service_timer`
  - 本地发送短聚合窗口

- `poll_thread`
  - 兜底轮询线程

- `service_timer_armed`
  - 防止重复启动聚合定时器

- `tx_seq`
  - Host 发给 Slave 的同步序号

- `rr_channel`
  - round-robin 起点，用来在多通道之间公平打包

### `struct spi_can_node_data`

子设备的动态状态。

包含：

- `common`
  - Zephyr CAN 驱动公共状态

- `filters[]`
  - 软件 RX filter 表

- `cached_state` / `cached_err_cnt`
  - 子设备视角下的当前状态

- `bitrate`
  - 当前配置的目标 bitrate

## 5. 同步协议为什么这样设计

当前协议故意只保留最小同步面。

### Host -> Slave

同步：

- `seq`
- `tx_count`
- `bitrate_mask`
- 两个 channel 的 bitrate
- 最多 `4` 个待发 CAN 帧

### Slave -> Host

同步：

- `seq`
- `rx_count`
- `more_rx_mask`
- `state_mask`
- 两个 channel 的状态
- 最多 `4` 个待上传 CAN 帧

### 为什么不直接同步完整 timing / mode / recover

因为这样会把 Host 和 Slave 的硬件差异绑死在协议层。

当前方案只同步 bitrate，由 Slave 本地完成：

1. `can_calc_timing()`
2. `can_set_timing()`
3. `can_start()`

这样协议更稳定，Slave 的具体 CAN 控制器差异也更容易被隔离。

## 6. Host 侧发送路径

上层调用：

1. `can_send(can_v0/can_v1, ...)`

在 `spi_can_node_send()` 中：

1. 检查 frame flags 和 DLC 合法性
2. 如果虚拟设备还没 start，先调用 `spi_can_node_start()`
3. 再调用 `spi_can_mfd_send()`

在 `spi_can_mfd_send()` 中：

1. 校验 channel
2. `k_msgq_put()` 把帧放入对应通道的 `tx_queue`
3. 调用 `spi_can_schedule_service()` 启动一次 `50 us` 聚合窗口

随后服务线程被唤醒：

1. `spi_can_poll_thread()` 被 `service_sem` 唤醒
2. 调用 `spi_can_service_once()`
3. `spi_can_service_locked()` 打包 SPI 帧
4. `spi_transceive()` 发起一次 SPI 同步
5. 成功后 `spi_can_commit_service_locked()` 从队列里真正弹出这次已经发走的帧

这里有一个重要设计点：

- “打包时先 peek，发送成功后再 get”

这样如果 SPI 事务失败，不会把消息从队列里误删。

## 7. Host 侧接收路径

SPI 回包在 `spi_can_parse_slave_frame_locked()` 中处理。

步骤：

1. 先检查 magic/version
2. 如需要尝试 `spi_can_try_align_sync_frame()` 做字节对齐修复
3. 解析每个 channel 的状态，并更新 `cached_state`
4. 如果该 channel 对应子设备有效，则调用 `spi_can_node_report_state()`
5. 遍历回包里的 `rx_entries`
6. 解出 `struct can_frame`
7. 根据 `channel` 找到对应子设备
8. 调用 `spi_can_node_handle_rx_frame()`

`spi_can_node_handle_rx_frame()` 再执行本地软件过滤：

1. 在持锁状态下扫描 `filters[]`
2. 用 `can_frame_matches_filter()` 判断命中
3. 退出锁后调用用户注册的 RX callback

这里“先收集 matches，再出锁回调”的设计是对的，因为：

- 避免用户回调重入修改 filter 表
- 减少锁持有时间

## 8. Slave 侧参考实现

当前 Slave 在 `src/vcan_slave_demo/src/main.c`，不是驱动，而是协议参考实现。

### Slave 的核心状态

每个 `slave_channel` 有：

- 本地物理 CAN 设备指针
- 本地 RX 队列
- 当前 timing / bitrate / started / state_dirty
- 最近一次状态和错误计数

### Slave 的接收物理 CAN 路径

物理 CAN 收到数据后：

1. `can_rx_cb()` 被驱动回调
2. 用 `k_msgq_put()` 放进本地 `rx_queue`
3. `update_int_line()` 把 INT 拉高

### Slave 的处理 Host 同步帧路径

在 `process_host_frame()` 中：

1. 校验 magic/version
2. 如存在 `bitrate_mask`，执行 `channel_apply_bitrate()`
3. bitrate 会触发 `channel_start_with_bitrate()`
4. Host 带来的待发 CAN 帧通过 `channel_send_can()` 发到本地物理 CAN

### Slave 的构造回包路径

在 `build_slave_response()` 中：

1. 填头部和 `seq`
2. 每个 channel 调用 `snapshot_state()` 写状态
3. round-robin 从两个 `rx_queue` 里取最多 `4` 帧
4. 如果还有剩余 RX，就设置 `more_rx_mask`

### Slave 的 INT 设计

INT 是电平语义，不是边沿语义。

只要以下任一条件成立就保持拉高：

- RX 队列非空
- `state_dirty` 为真

这样 Host 即使错过某个时刻，也能在下一拍轮询时继续看到 INT 为高。

## 9. 为什么 Host 用“轮询 + 聚合唤醒”

这是最近几轮任务里最关键的实现变化。

如果纯轮询：

- 本地刚入队的帧可能要白等下一拍

如果每次 `send()` 立刻同步：

- 又会退化成“一帧一笔 SPI 事务”，吞吐很差

当前方案折中为：

- 平时靠 `500 us` 兜底轮询
- 本地发送时触发一个 `50 us` 聚合窗口
- 聚合窗口结束后再唤醒服务线程

收益是：

- 不容易多等整整一拍
- 又能把 burst 尽量打进同一笔 SPI 帧

## 10. RTOS 并发模型为什么这样设计

这一节专门解释线程、锁和同步原语的设计意图。

如果只看 API 名称，很容易把它理解成“想到什么用什么”；实际上这里的选择和 SPI 桥的时序目标强相关。

### 10.1 Host 侧有哪些并发执行体

Host 侧至少有这几类执行上下文：

- 上层调用线程
  - 例如应用线程调用 `can_send()`、`can_get_state()`

- `spi_can_poll_thread`
  - 专门负责 SPI 服务
  - 在 `drivers/vcan/spi_can_mfd.c` 里被创建

- `service_timer` 的超时回调
  - 用来结束 `50 us` 聚合窗口

- 各类 CAN RX / state change 回调
  - 由 CAN 驱动在其自己的执行上下文里触发

这意味着 Host 不是单线程逻辑，而是一个典型的“多生产者 + 单总线消费者”模型：

- 多个线程都可能提出“我要发一帧”或“我要同步状态”
- 但真正访问 SPI 总线的动作必须串行

### 10.2 为什么 SPI 事务必须用 `k_mutex`

Host 父设备里最关键的资源是：

- `tx_frame`
- `rx_frame`
- `tx_seq`
- `rr_channel`
- SPI 总线本身

这些都属于“单次 SPI 事务的共享临界区”，所以代码用 `k_mutex` 包起来，而不是让多个调用者直接碰总线。

原因是：

1. SPI 同步帧是整包语义
   - 如果两个线程交错修改 `tx_frame`
   - 你拿到的就不是“帧 A + 帧 B”，而是损坏的混合包

2. SPI 是典型串行外设
   - 一次只能有一个事务在跑

3. `k_mutex` 适合线程上下文里的中等粒度临界区
   - 这里的临界区不是几条寄存器操作，而是一整次打包 + `spi_transceive()`
   - 这比自旋锁更合适

为什么不是 `k_spinlock`：

- `k_spinlock` 更适合极短临界区
- 这里会调用 `spi_transceive()`，属于可能耗时明显的操作
- 用 spinlock 会把 CPU 白白占住，而且不适合这种线程级串行化

### 10.3 为什么发送路径用 `k_msgq`，而不是“直接发 SPI”

`spi_can_mfd_send()` 并不直接做 SPI 事务，而是：

1. `k_msgq_put()` 入队
2. 调度服务线程

这里 `k_msgq` 的作用不是“缓存一下而已”，而是解决三个问题：

1. 解耦调用者节奏和总线节奏
   - 上层可能短时间 burst 连发很多帧
   - SPI 总线只能按桥协议一批一批送

2. 提供有界缓冲
   - 队列深度固定为 `SPI_CAN_SYNC_TX_QUEUE_DEPTH`
   - 不会在压力下无界分配内存

3. 让“多生产者 -> 单消费者”结构更清晰
   - 多个调用线程只负责入队
   - 单个服务线程负责统一取队列、打包、发 SPI

为什么不是 `k_fifo` / `k_pipe`：

- 这里的数据大小固定，都是 `struct can_frame`
- `k_msgq` 对固定大小元素最自然
- 不需要额外分配节点，也更适合受控内存场景

### 10.4 为什么是 `k_sem + k_timer + atomic` 组合

Host 为了实现“短聚合窗口”，用了三样东西：

- `k_timer`
  - 负责在 `50 us` 后触发

- `k_sem`
  - 负责真正唤醒 `spi_can_poll_thread`

- `atomic_t service_timer_armed`
  - 防止重复启动多个聚合定时器

这是一个很典型的 RTOS 设计：

- 定时器负责“什么时候该干活”
- 信号量负责“唤醒哪个工作线程”
- 原子变量负责“避免重复调度”

为什么不直接在 `k_timer` 回调里做 SPI：

- 定时器回调应该尽量短小
- SPI 打包和事务明显比“发一个唤醒信号”重
- 把重活放到服务线程里，系统行为更稳定

为什么不每次 `can_send()` 都 `k_sem_give()`：

- 那样会很快退化成“一帧一唤醒，一帧一事务”
- 吞吐会明显下降

`atomic_cas()` 的作用是：

- 第一个发送者启动聚合窗口
- 后续在窗口期内来的发送者不再重复启动定时器
- 这样一批 burst 会自然合并

### 10.5 为什么 `spi_can_poll_thread` 既能轮询又能被唤醒

这个线程不是纯轮询，也不是纯事件驱动，而是折中模型：

- 没有本地发送时，它每 `500 us` 醒一次，看 INT 和本地 dirty 状态
- 有本地发送时，它也能被 `service_sem` 提前唤醒

这样做是因为两类事件的时序特征不同：

- Host 本地发帧
  - 最好更快响应
  - 不想白等一整拍

- Slave 远端上报
  - 当前只有电平 INT
  - 固定节拍轮询就足够稳定

所以它不是通用 workqueue，而是“带轮询兜底的专用服务线程”。

### 10.6 为什么大量地方都用 `K_NO_WAIT`

这个项目里很多入队和回调路径都显式用 `K_NO_WAIT`，尤其在：

- Host/Slave 的 `k_msgq_put()`
- Slave 的 `can_send()`
- 定时器和回调相关路径

原因不是“偷懒不处理阻塞”，而是因为这些代码经常运行在时延敏感路径上：

- CAN RX callback
- 状态变化 callback
- 聚合窗口结束后的调度路径

在这些地方阻塞会带来两个问题：

1. 破坏时序
   - 延长回调执行时间
   - 放大上层不确定性

2. 增加上下文约束风险
   - 某些回调可能接近 ISR 语义
   - 阻塞行为会更危险

所以这里宁可：

- 失败就丢帧 / 返回错误
- 也不在敏感路径里睡眠等待

这是典型实时系统取舍：

- 优先保证系统节拍和可预测性
- 接受有界资源下的显式 backpressure

### 10.7 为什么 RX filter 和状态回调都“先出锁再回调用户”

`spi_can_node_handle_rx_frame()` 和 `spi_can_node_update_state()` 都采用同一个模式：

1. 先在锁内更新内部状态或收集匹配结果
2. 解锁
3. 再调用用户回调

这样做很重要，因为用户回调是不受驱动控制的，它可能：

- 再次调用 CAN API
- 增删 filter
- 触发别的同步对象
- 打日志甚至做更重的事

如果在锁内直接回调，容易出现：

- 死锁
- 锁递归
- 长时间占锁
- 上下文链条过长

所以这是一条很重要的 RTOS/驱动编写原则：

- 锁只保护驱动内部状态
- 不要在持锁时执行外部不可控代码

### 10.8 线程优先级为什么这样设

Host 侧服务线程的优先级来自 `CONFIG_VCAN_WORKQ_PRIORITY`，默认是 `0`。

在 Zephyr 里：

- 负优先级是 cooperative
- 非负优先级是 preemptive

所以默认 `0` 表示：

- 这个线程是普通可抢占线程
- 足够及时
- 但不会像 cooperative 线程那样长期霸占 CPU

这对 VCAN 很合适，因为：

- 它确实是带时序要求的后台线程
- 但不是系统里唯一最关键的实时任务

如果优先级太低：

- 聚合窗口结束后不能及时发 SPI
- 吞吐和时延会抖

如果优先级太高：

- 它会和其它业务线程、日志线程、设备线程争抢 CPU
- 反而可能放大系统级干扰

当前值属于比较稳妥的折中。

### 10.9 为什么 Host 用专用线程，而不是直接用系统 workqueue

也可以想象把 SPI 服务逻辑塞到系统 workqueue 里，但当前没有这么做，原因主要有三点：

1. 这条链路有固定节拍轮询需求
   - workqueue 更偏“被动接活”
   - 这里还要自己周期性检查 INT

2. 这条链路需要自己控制栈大小和优先级
   - 专用线程更直接

3. SPI 桥本身就是一个长期存在的服务循环
   - 用专用线程语义更清楚

### 10.10 Slave 为什么几乎没加锁

这是当前实现里一个必须说清楚的点。

Slave demo 里：

- 主循环在做 `spi_transceive()` 和协议解析
- `int_refresh_thread` 周期性更新 INT
- CAN RX / state change callback 会更新 `rx_queue`、`state_dirty`、`state`

严格来说这是有并发访问的。

它现在之所以还能工作，主要依赖于：

- 共享状态比较简单
  - 布尔标志、错误计数、状态枚举

- 真正的大对象传递走的是 `k_msgq`
  - 队列本身由内核保证并发安全

- 这个文件当前定位是 demo/reference implementation
  - 重点是把协议跑通
  - 不是完全 hardened 的生产级从机驱动

但要诚实地说：

- 如果以后要把 Slave 正式化
- 最应该补强的就是它的并发模型

尤其要考虑：

- `state/state_dirty/err_cnt` 的访问一致性
- `update_int_line()` 和队列状态判断之间的竞争窗口
- 主循环与回调之间更明确的同步边界

所以当前 Slave 是“锁轻量、实现直接、便于联调”的权衡，而不是最终形态。

### 10.11 从 RTOS 设计维度看 Host 方案

如果把 Host 侧拆开看，它其实不是“几个 API 凑起来”，而是完整的 RTOS 设计组合：

- 调度设计
  - 多个上层线程可以并发调用 `can_send()`
  - 但 SPI 事务只能有一个执行者
  - 所以代码形成了“多生产者 + 单消费者”的经典结构

- 同步设计
  - 用 `k_msgq` 承接生产者和消费者之间的数据流
  - 用 `k_sem` 唤醒消费者线程
  - 用 `k_mutex` 保护 SPI 临界区
  - 用 `atomic_t` 保护特别短的小状态切换

- 时序设计
  - `500 us` 轮询负责兜底
  - `50 us` 聚合窗口负责提升 burst 吞吐
  - `2 us` CS 延迟和 `200 us` re-arm 延迟负责照顾外设时序

- 数据所有权设计
  - 队列里的 `can_frame` 由队列持有
  - `tx_frame/rx_frame` 只在父设备服务路径里使用
  - 用户回调拿到的是独立副本，不直接碰驱动内部缓冲

- 过载处理设计
  - 队列满了就返回错误或丢弃
  - 单拍最多搬 `4` 帧
  - 用 `rr_channel` 保证双通道在过载下尽量公平

这一组设计合起来，目标只有一个：

- 不追求“绝不丢帧”的理想模型
- 而追求“在有限内存和明确节拍下，可预测地运行”

这就是 RTOS 设计和普通桌面程序思路最不一样的地方。桌面程序更常见的想法是“先存起来再说”；而这里必须先问：

- 谁在什么上下文里运行
- 能不能阻塞
- 最坏情况会不会拖垮整个周期

### 10.12 数据所有权为什么要讲得这么细

驱动里最容易出事故的，不是算法本身，而是谁拥有哪份数据、谁可以在什么时候改它。

这个项目里几种关键数据的所有权如下：

- `struct can_frame`
  - 在 `spi_can_mfd_send()` 里，被复制进 `k_msgq`
  - 进入队列后，就不再依赖调用者栈上的原始对象
  - 这样上层函数返回后，驱动仍然拿得到完整帧

- `tx_frame` / `rx_frame`
  - 它们是父设备内部的一次性 SPI 缓冲区
  - 只应该在 `bus_lock` 保护下读写
  - 这是典型的“共享工作缓冲区”

- `filters[]`
  - 归子设备 `spi_can_node_data` 管
  - 只能在持 `lock` 时修改或扫描
  - 但回调函数不能在锁内执行

- `cached_state` / `cached_err_cnt`
  - 它们是缓存，不是唯一事实来源
  - 真正来源还是最近一次 Slave 回包，或者 Slave 端本地 `can_get_state()`
  - 之所以需要缓存，是因为上层查询状态时，不可能每次都为它单独跑一笔 SPI 往返

为什么这件事很重要：

1. 如果没有清楚的数据所有权，锁会越加越多
2. 但锁多不等于安全，很多时候只是把竞争条件藏起来
3. 真正稳的做法是先定义“谁能改、在哪改、改完谁通知谁”

### 10.13 调度设计：为什么不是“收到事件就立刻做完”

RTOS 新手最常见的直觉是：

- 收到发送请求，立刻 SPI 发出去
- 收到定时器超时，立刻在回调里做同步
- 收到状态变化，立刻一路回调到底

这在简单 demo 里看起来很直观，但在真实并发系统里通常会把执行上下文搞乱。

这个项目反过来做：

1. 先把事件归类
   - 数据事件走 `k_msgq`
   - 唤醒事件走 `k_sem`
   - 时间事件走 `k_timer`

2. 再把重活集中到一个服务线程
   - 打包
   - SPI 事务
   - 解包
   - 提交已成功发送的帧

这样做的直接收益是：

- 复杂逻辑只在少数几个线程上下文里发生
- 回调和定时器保持很轻
- 更容易分析最坏时延

所以 `spi_can_poll_thread()` 本质上就是这个桥的“执行核心”，而不是简单的轮询线程。

### 10.14 时间参数为什么不是随便拍脑袋的

这几个时间参数都应该按“工程折中值”理解，而不是魔法数字：

- `SPI_CAN_SYNC_POLL_US = 500`
  - 作用：没有显式唤醒时，Host 也会定期检查本地 dirty 和远端 INT
  - 太大：远端上报延迟变大
  - 太小：空转 SPI 或 CPU 唤醒次数太多

- `SPI_CAN_KICK_COALESCE_US = 50`
  - 作用：给本地 burst 留一个很短的聚合窗口
  - 太大：发送时延上升
  - 太小：很难把多帧合并到同一拍

- `SPI_CAN_CS_DELAY_US = 2`
  - 作用：手动 CS 拉高/拉低前后给外设一点建立保持时间
  - 如果没有这个量，某些 SPI 从设备在边沿附近可能不稳定

- `SPI_CAN_REARM_DELAY_US = 200`
  - 作用：同步后给下一轮 re-arm 留余量
  - 这类值通常和板级连线、从设备状态机、SPI 从机实现有关

这说明 RTOS 里的“时间参数”不只是性能参数，很多时候它也是硬件接口参数。

### 10.15 过载时系统是怎么退化的

这个项目没有选择“过载时无限排队”，而是选择“有界退化”。

表现为：

- Host TX 队列深度固定
  - 超了就 `k_msgq_put(..., K_NO_WAIT)` 失败

- Slave RX 队列深度固定
  - 超了就记录日志并丢帧

- 每拍最多搬 `4` 帧
  - 就算某一瞬间积压很多，也只能分多拍清空

- `more_rx_mask`
  - 告诉 Host：“这拍没搬完，后面还有”

- `rr_channel`
  - 避免通道 0 一直吃满额度，通道 1 长期饿死

这就是典型 RTOS 里的 backpressure 设计。

所谓 backpressure，不是某个 API 名字，而是一种系统策略：

- 当下游处理不过来时
- 上游必须被限制、延迟、丢弃，或显式收到失败

如果没有 backpressure，系统表面上看似“从不报错”，实际上往往是在别的地方悄悄失控，比如：

- 内存越来越大
- 时延越来越不可预测
- 某一条通道长时间饿死

### 10.16 如果把 Slave 做成正式产品，RTOS 上最该补哪几块

当前 Slave demo 能跑通协议，但如果要做成长期维护版本，RTOS 方面建议优先补这几类能力：

1. 更清晰的数据同步边界
   - 给 `state`、`err_cnt`、`state_dirty` 加统一锁或原子语义

2. 更明确的事件归并
   - 把“状态更新”和“INT 维护”收敛到同一个工作上下文

3. 更强的错误恢复
   - 例如 SPI 长时间异常时如何重置从机状态
   - CAN 通道重配失败后如何上报和回退

4. 更完整的负载监控
   - 队列高水位
   - 丢帧计数
   - 平均每拍装载率

这些不是“锦上添花”，而是从 demo 走向量产时最先需要补齐的 RTOS 工程能力。

## 11. Zephyr API 在这里分别干什么

这部分是读代码时最容易混淆的地方。

### 设备与设备树

- `DEVICE_DT_GET(node)`
  - 从 DTS 节点拿设备对象

- `DEVICE_DT_INST_DEFINE(...)`
  - 注册父设备

- `CAN_DEVICE_DT_INST_DEFINE(...)`
  - 注册子 CAN 设备

- `SPI_DT_SPEC_INST_GET(...)`
  - 从设备树生成 `spi_dt_spec`

- `GPIO_DT_SPEC_INST_GET(...)`
  - 从设备树生成 `gpio_dt_spec`

### SPI

- `spi_is_ready_dt()`
  - 检查 SPI 设备和配置是否可用

- `spi_transceive()`
  - 一次同时发送和接收固定长度同步帧

### GPIO

- `gpio_is_ready_dt()`
  - 检查 GPIO 控制器是否可用

- `gpio_pin_configure_dt()`
  - 初始化 Host 侧 INT 输入或 CS 输出

- `gpio_pin_set_dt()` / `gpio_pin_set_raw()`
  - 控制 CS 或 Slave INT

- `gpio_pin_get_dt()`
  - Host 读取 Slave INT 电平

### CAN

- `can_send()`
  - Host 子设备把发送请求交给父设备
  - Slave 把同步帧里的数据真正发到物理 CAN

- `can_add_rx_filter()`
  - Slave 注册物理 CAN RX 回调
  - Host 本地普通 CAN 设备和虚拟 CAN 设备注册上层回调

- `can_set_state_change_callback()`
  - 监听状态变化

- `can_get_state()`
  - Slave 取本地物理 CAN 当前状态

- `can_calc_timing()`
  - Slave 按 bitrate 本地重新算 timing

- `can_set_timing()`
  - 应用 timing

- `can_start()` / `can_stop()`
  - 控制物理 CAN 通道启停

- `can_frame_matches_filter()`
  - Host 子设备做软件 RX filter

### 内核对象

- `k_msgq_init()`
  - 初始化每个通道的帧队列

- `k_msgq_put()`
  - 入队待发或待上传 CAN 帧

- `k_msgq_get()`
  - 出队

- `k_msgq_peek_at()`
  - 打包 SPI 帧时先看不删

- `k_mutex_lock()`
  - 保护父设备 SPI 事务和共享状态

- `k_sem_init()` / `k_sem_give()` / `k_sem_take()`
  - 做“事件通知”，而不是存放数据
  - 这里专门用来唤醒服务线程

- `k_timer_init()` / `k_timer_start()`
  - 实现短聚合窗口
  - 定时器回调本身不做重活，只负责触发后续线程执行

- `k_thread_create()`
  - 创建 Host 轮询线程和 Slave INT 刷新线程

- `k_busy_wait()`
  - CS 和 SPI re-arm 的微秒级延迟

- `atomic_cas()` / `atomic_clear()`
  - 保护“聚合定时器只启动一次”这个非常短的状态切换
  - 这里不需要 mutex 的完整重量级保护，只需要无锁原子位

### 字节序与工具宏

- `sys_put_le32()` / `sys_get_le32()`
  - 协议字段按 little-endian 编解码

- `BIT()`
  - 生成 mask

- `MIN()` / `ARRAY_SIZE()`
  - 限制循环范围

- `BUILD_ASSERT()`
  - 保证同步帧结构尺寸和协议定义一致

### 11.1 设备模型相关词汇，放到这个项目里是什么意思

**设备实例（device instance）**

Zephyr 里的很多驱动都不是“你自己 new 一个对象”，而是编译期根据 DTS 生成设备实例。这个项目里：

- `spi_can_mfd` 是一个父设备实例
- `spi_can_node` 是两个子设备实例

所以代码里很多宏看上去不像普通 C 初始化，是因为它们在参与 Zephyr 设备模型生成。

**父设备（parent device）**

父设备不是“上级领导”这个抽象词，而是：

- 持有 SPI 总线
- 持有同步协议
- 持有公共调度线程
- 为多个逻辑 CAN 通道提供共享服务

在本项目里就是 `drivers/vcan/spi_can_mfd.c` 注册出来的设备。

**子设备（child device）**

子设备是“挂在父桥上的逻辑 CAN 通道”。它的职责不是再去碰 SPI，而是：

- 对上实现标准 CAN API
- 对下转调父设备接口

在本项目里就是 `drivers/vcan/spi_can_node.c` 里的两个 `can_v*` 设备。

**静态配置（config）和动态数据（data）**

Zephyr 驱动常见拆法是：

- `xxx_config`
  - 编译期固定
  - 例如设备树来的 SPI/GPIO 参数、channel 编号

- `xxx_data`
  - 运行期变化
  - 例如队列、锁、缓存状态、线程对象

你在本项目里能直接看到：

- `struct spi_can_mfd_config` / `struct spi_can_mfd_data`
- `struct spi_can_node_config` / `struct spi_can_node_data`

这样拆的好处是：

- 只读配置和运行时状态边界清楚
- 多实例驱动更容易复用

**驱动 API（driver API）**

驱动 API 不是普通“函数集合”，而是 Zephyr 给某类设备规定的接口表。对 CAN 来说，上层期望的是：

- `start`
- `stop`
- `send`
- `add_rx_filter`
- `get_state`
- `set_timing`

`spi_can_node.c` 的意义，就是把“SPI 远端桥”包装成一套符合 `can` 子系统预期的驱动 API。

这就是适配器模式在 Zephyr 驱动里的落地形式。

### 11.2 并发与 RTOS 词汇，在本项目里的具体含义

**执行上下文（execution context）**

指“这段代码是在谁的上下文里跑”。同样是一个函数调用，不同上下文的限制可能完全不同。

本项目里的主要上下文有：

- 应用线程上下文
- Host 服务线程上下文
- 定时器回调上下文
- CAN 驱动回调上下文

为什么一定要先分上下文：

- 有的上下文允许阻塞
- 有的不适合做重活
- 有的上下文里再去拿锁会更危险

**线程（thread）**

线程是内核可调度的执行体。这里的关键不是“它会并发”，而是：

- Host 用专用线程承接 SPI 服务
- Slave 用刷新线程维护 INT 电平

这说明线程不是为了“显得高级”，而是为了给周期服务和重活一个稳定承载点。

**可抢占线程（preemptive）和协作线程（cooperative）**

Zephyr 里常见理解是：

- 可抢占线程
  - 更高优先级任务来了可以把它抢走

- 协作线程
  - 除非自己让出 CPU，否则不会被普通线程抢占

本项目默认把 Host 服务线程放在普通可抢占优先级，是因为它需要及时，但不能霸占系统。

**临界区（critical section）**

临界区指“这段代码必须以原子方式看待，不能被别的执行体同时破坏”。

在本项目里，最典型的临界区不是单个变量赋值，而是：

- 构造 `tx_frame`
- 发起一次完整 `spi_transceive()`
- 提交本次成功发走的队列项

这就是为什么锁保护的是“事务”，不是某一个字段。

**互斥锁（mutex）**

`k_mutex` 的作用不是单纯“防并发”，而是给线程级共享资源建立排他访问窗口。

本项目里它主要保护：

- SPI 总线事务
- 父设备共享缓冲区
- 子设备 filter/state 表

为什么这里用 mutex 而不是 spinlock：

- 因为临界区里可能包含耗时较长的 SPI 调用
- spinlock 更适合极短、不睡眠的临界区

**信号量（semaphore）**

这里的 `k_sem` 不是用来存数据，而是用来发“该你干活了”的通知。

本项目里：

- 生产者给 `service_sem`
- `spi_can_poll_thread()` 取 `service_sem`

它表达的是事件唤醒，而不是数据传递。

**消息队列（message queue）**

`k_msgq` 是固定大小元素的有界队列。本项目里它承担：

- Host 侧待下发 CAN 帧缓冲
- Slave 侧待上传 CAN 帧缓冲

为什么它特别适合这里：

- 元素大小固定，都是 `struct can_frame`
- 内存提前分配
- 行为容易预测

**定时器（timer）**

`k_timer` 的作用是“在未来某个时间点发一个轻量事件”，不是“把整个业务逻辑放进去”。

本项目里它负责结束 `50 us` 聚合窗口。它不直接做 SPI，是因为：

- 回调应尽量短
- SPI 事务属于重活
- 重活应收敛到服务线程

**原子变量（atomic）**

原子变量适合那种：

- 状态很小
- 更新很短
- 但多个上下文都可能同时碰

这里的 `service_timer_armed` 就是典型例子。它不需要完整互斥，只需要保证：

- 第一个调用者成功把 0 改成 1
- 后续调用者看到已经 armed，就不要重复启动定时器

**阻塞（blocking）**

阻塞不是坏事，但要看上下文。像 `k_mutex_lock(..., K_FOREVER)` 出现在普通线程路径里通常可以接受；而在 callback 或高时效路径里，大量阻塞就会把时序拖坏。

这个项目大量使用 `K_NO_WAIT`，就是明确在说：

- 宁可失败返回
- 也不在敏感路径里睡眠等待

### 11.3 协议、缓存和调度词汇，在本项目里怎么理解

**轮询（polling）**

轮询不是“低级方案”，而是按固定节拍主动检查状态。

本项目里 Host 每 `500 us` 会检查：

- 本地是否有 dirty 发送
- 远端 INT 是否拉高

轮询的好处是简单、稳定、容易分析最坏情况。

**事件驱动（event-driven）**

事件驱动是“有事再唤醒”。本项目里对应的是：

- 本地发送触发 `service_sem`
- 定时器结束触发 `service_sem`

**混合调度（hybrid scheduling）**

这个项目不是纯轮询，也不是纯事件驱动，而是混合式：

- 轮询保底
- 事件提前触发

这是一个很典型的工程折中，尤其适合“既要处理远端慢变化，又要兼顾本地突发发送”的桥接场景。

**dirty bit / dirty flag**

`bitrate_dirty`、`state_dirty` 这种名字里的 dirty，不是“脏数据”，而是：

- 本地状态已经变了
- 但还没和对端完成同步

所以 dirty flag 本质上是“待同步标记”。

**缓存状态（cached state）**

缓存不是重复保存一份没用的数据，而是把“最近一次已知状态”留在本地，避免每个查询都发远程事务。

比如 `cached_state` 的意义是：

- 上层随时能拿到最近快照
- 但它不保证和远端物理状态完全零延迟一致

这就是 RTOS/驱动里常见的 eventual consistency 思路。

**轮转公平（round-robin）**

`rr_channel` 的作用是：

- 这次从通道 0 开始打包
- 下次优先从通道 1 开始

这样双通道在共享 `4` 帧额度时更公平。

如果没有这个机制，容易出现一条热通道长期吃满窗口，另一条冷通道被饿住。

**打包/解包（pack / unpack）**

打包就是把驱动内部对象编码成协议字节流；解包则相反。

本项目里：

- `spi_can_pack_frame_entry()` 负责把 `struct can_frame` 写进协议项
- `spi_can_unpack_frame_entry()` 负责从协议项还原回来

这层抽象很重要，因为“内存里的 C 结构体”不等于“可稳定跨板传输的协议格式”。

**有界缓冲（bounded buffer）**

有界缓冲指容量固定、达到上限就必须处理失败的缓冲区。

本项目里的 `k_msgq` 就是有界缓冲。它的价值是：

- 内存不会无限增长
- 最坏情况能推导

代价则是：

- 过载时必须接受失败或丢帧

**反压（backpressure）**

反压是系统在忙不过来时，对上游施加限制的能力。

这里的体现包括：

- `K_NO_WAIT` 入队失败
- 单拍最多 `4` 帧
- 队列深度固定

这让系统在过载时是“可控退化”，而不是静悄悄地变成不可预测。

### 11.4 协议格式、字节序和内存布局词汇

**little-endian**

协议里多字节数值用 little-endian 存放，例如：

- `magic`
- `bitrate`

这就是为什么代码里显式使用 `sys_put_le32()` / `sys_get_le32()`。这样做的目的不是多写几行，而是避免把“CPU 本地字节序”误当成“协议字节序”。

**对齐（alignment）**

`__aligned(4)` 表示把缓冲区按 4 字节边界对齐。

这么做的常见原因有：

- 某些架构访问未对齐地址更慢
- 某些 DMA/SPI 实现更偏好对齐缓冲

所以它不是语法装饰，而是在为底层访问做准备。

**保留字段（reserved）**

协议里的 `reserved0`、`reserved1` 不是浪费，而是为了：

- 保持固定帧长
- 给未来扩展留空间
- 让结构布局更稳定

固定长度同步帧在 MCU 场景里很常见，因为解析简单、时延更可预测。

**固定长度帧**

当前同步帧固定为 `192` 字节。这样设计的好处是：

- Host/Slave 两端都不用做变长解析器
- SPI 事务长度恒定
- 更适合做结构尺寸校验和时延预算

代价是：

- 空载时也要传完整帧

这也是为什么当前方案偏向“简单稳定”，而不是“字节效率最优”。

**`BUILD_ASSERT`**

`BUILD_ASSERT(sizeof(...) == 192)` 的作用是在编译期就发现协议结构跑偏。

如果没有它，最危险的情况是：

- Host 和 Slave 都能编过
- 但一边结构体多了几个字节
- 上板后才出现非常难查的错位包问题

所以它是“把运行期灾难前移到编译期”的手段。

### 11.5 这些 Zephyr 宏和类型，实际在帮你做什么

**`DEVICE_DT_GET(node)`**

作用是从设备树节点拿到已经生成好的 `struct device *`。它常用于“我知道某个设备实例存在，现在想拿它的句柄”。

在这个项目语境里，可以把它理解成“把 DTS 里声明的设备，连接到 C 代码里”。

**`DEVICE_DT_INST_DEFINE(...)`**

这是“定义某个驱动实例”的宏。它会把：

- init 函数
- config
- data
- init level / priority

这些东西注册进 Zephyr 的设备模型。

父设备 `spi_can_mfd` 之所以能在启动时被创建，就是靠这一类宏。

**`CAN_DEVICE_DT_INST_DEFINE(...)`**

这是 CAN 子系统对应的设备定义宏，可以理解为“带 CAN 子系统语义的设备注册”。它让子设备既是普通 `struct device`，又能被 CAN 框架识别成一个 CAN 控制器。

**`struct spi_dt_spec`**

它不是单纯“SPI 句柄”，而是把：

- SPI 控制器设备
- 片选信息
- 从设备地址/配置

这些和某个 DTS 节点相关的 SPI 描述打包在一起。

这样驱动里就不必手写一堆板级常量。

**`struct gpio_dt_spec`**

和 `spi_dt_spec` 类似，它把：

- GPIO 控制器
- pin 编号
- flags

整理成一个可直接传给 `gpio_*_dt()` 系列 API 的对象。

**`K_NO_WAIT`**

表示“如果现在拿不到资源，就立刻失败，不要睡眠等待”。它适合：

- callback
- 定时敏感路径
- 明确想要无阻塞行为的场景

**`K_FOREVER`**

表示“愿意一直等”。它通常适合普通线程里的锁获取或同步等待，但不适合随便塞进时延敏感回调。

**`K_USEC(x)` / `K_MSEC(x)`**

这类宏把整数时间字面量转换成内核理解的超时对象。它们的好处是：

- 代码语义直观
- 单位不容易写错

比起直接传裸数字，更不容易出现“这到底是 tick、ms 还是 us”的歧义。

**`K_KERNEL_STACK_MEMBER`**

它用于在数据结构里直接放一块线程栈内存。Host 把服务线程栈放进 `spi_can_mfd_data`，意味着：

- 每个实例都有自己的线程栈
- 栈大小由 Kconfig 控制

这是 Zephyr 驱动里很常见的线程资源组织方式。

## 12. 接口设计上的几个取舍

### 12.1 为什么 `spi_can_mfd_send()` 不直接做 SPI 事务

因为这样会把每个 `can_send()` 变成一次 SPI 往返，吞吐很差。

现在它只做：

1. 入队
2. 调度服务线程

这把“用户请求节奏”和“总线事务节奏”解耦了。

### 12.2 为什么 RX filter 放在 Host 本地软件层

优点：

- 协议更简单
- 不需要同步 filter 规则到 Slave
- Host 侧逻辑和普通 CAN 驱动更接近

代价：

- Slave 仍然要上传更多原始帧
- 总线带宽利用率不是最优

### 12.3 为什么 Slave 现在还是 demo

因为当前更重要的是把 Host 协议、节奏和联调收敛。

把 Slave 也做成正式驱动当然更“整洁”，但会立刻引入：

- 另一套 driver model 接入
- 更多 DTS/Kconfig/init 顺序问题
- 协议头文件共享问题

所以当前版本先保留为应用层参考实现。

## 13. 当前性能边界怎么理解

当前代码侧硬边界来自：

- Host 兜底轮询 `500 us`
- 每次同步最多 `4` 帧
- 双通道共享这一拍的搬运额度

所以 Host demo 目前按代码上限压测到：

- `4000 fps/方向/通道`

而不是按物理 `1 MHz CAN` 的理想值去压。

这说明现在性能瓶颈首先在桥协议和调度模型，而不是物理 CAN 总线本身。

## 14. 继续改这个项目时最容易踩的坑

### 改 `SPI_CAN_SYNC_MAX_FRAMES`

不能只改一个常量。

还要同时检查：

- Host/Slave 两侧结构体
- `reserved1` 大小
- `BUILD_ASSERT`
- 文档
- Host 压测上限推导

### 改同步节奏

不能只改 `500 us` 或 `50 us`。

还要重新评估：

- Host demo 压测参数
- 代码上限
- Slave INT 刷新节奏

### 改协议字段

Host 和 Slave 目前没有共享公共协议头，双方有重复定义。

所以协议结构一旦变动，必须双边同时对齐。

## 15. 推荐的阅读顺序

如果你第一次接手这套代码，建议按这个顺序读：

1. `src/vcan_host_demo/app.overlay`
2. `dts/bindings/can/custom,spi-can-mfd.yaml`
3. `dts/bindings/can/custom,spi-can-node.yaml`
4. `drivers/vcan/spi_can_mfd.h`
5. `drivers/vcan/spi_can_node.c`
6. `drivers/vcan/spi_can_mfd.c`
7. `src/vcan_slave_demo/src/main.c`
8. `src/vcan_host_demo/src/main.c`

这样会先知道“设备长什么样”，再理解“协议怎么跑”，最后看“怎么验证”。

## 16. 一句话总结

这套 VCAN 实现，本质上是：

- 用 `spi_can_mfd` 把“双通道 SPI 协议调度器”藏在父设备里
- 用 `spi_can_node` 把“远端逻辑 CAN 通道”包装成标准 Zephyr `can` 设备
- 用最小协议面在 Host 和 Slave 之间同步帧、bitrate 和状态
- 用“轮询 + 短聚合唤醒”在简单性和吞吐之间做折中
