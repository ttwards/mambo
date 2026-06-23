# DM CAN 调度 Demo 审查说明

## 1. 范围

这份文档用于审查本轮新增的 CAN 中间层测试代码。

本次审查涉及的文件：

- `drivers/motor/common/motor_can_sched.h`
- `drivers/motor/common/motor_can_sched.c`
- `src/dm_can_sched_demo/src/main.c`
- `src/dm_can_sched_demo/app.overlay`
- `src/dm_can_sched_demo/prj.conf`
- `src/dm_can_sched_demo/CMakeLists.txt`

设计目标来自 `drivers/motor/common/prompt.ini`：

- 先不要改现有电机驱动
- 新建代码做测试
- 先用 DM 电机验证
- 验证优先级、节拍调度、回复留白、特定帧回复追踪、超时重发、总线占用率统计

这次实现遵循了这个约束：

- 没有替换现有 `can_send_queued()`
- 没有修改 `drivers/motor/dm/motor_dm.c`
- 新增了一个独立 demo app：`src/dm_can_sched_demo`

## 2. 实现内容

### 2.1 新增调度模块

`motor_can_sched` 是一个独立的 CAN 调度层，具备：

- 多优先级环形缓冲区
- 1000 Hz 调度节拍
- 基于标准帧/扩展帧时间的发送成本估算
- 针对“紧跟回复帧”的留白能力
- 针对“需要回复确认”的帧做等待与追踪
- 超时重发
- 每 2 秒输出一次总线利用率与统计
- 周期帧自动错峰排布

对外接口定义在 `motor_can_sched.h`，并且已经收敛成更接近旧中间层风格的形式。

### 2.2 DM 验证 demo

这个 demo app 会：

- 在 `app.overlay` 中定义 4 个 DM 电机
- 为每个 DM 回复 ID 安装 RX filter
- 发送初始化帧
- 通过调度器发送周期 MIT 控制帧
- 让目标角度来回扫动
- 额外加入一个低频“探测帧”，专门验证特定帧回复链路

这个探测帧很重要，因为它能更清晰地展示：

- 提交
- 入队
- 发出
- 回复匹配成功
- 超时
- 重发
- 最终放弃

## 3. 与 prompt 需求的对应关系

### 3.1 多级环形缓冲区

已实现。

相关代码：

- `enum motor_can_sched_prio`
- `struct motor_can_sched_ring`
- `rings[MOTOR_CAN_SCHED_PRIO_COUNT]`

当前优先级：

- `CRITICAL`
- `HIGH`
- `NORMAL`
- `LOW`

### 3.2 1000 Hz 节拍调度与回复留白

已实现。

相关代码：

- `MOTOR_CAN_SCHED_TICK_HZ 1000`
- `frame_time_us()`
- `frame_cost_us()`
- `budget_us = 1000`
- `reply_reserve_us`

当前行为：

- 标准帧按 `130 us` 估算
- 扩展帧按 `150 us` 估算
- 如果某帧声明需要紧跟回复，可预留额外空档
- 每个 CAN 总线每个 tick 按 1 ms 预算发送

### 3.3 特定帧回复确认与重发

已实现。

相关代码：

- `struct motor_can_sched_pending`
- `reserve_pending_locked()`
- `motor_can_sched_report_rx()`
- `check_timeouts_locked()`

当前行为：

- 帧可以设置 `expect_reply = true`
- 调度器记录期望的 `reply_id` 和 `reply_mask`
- 收到匹配 RX 帧后清除 pending
- 超时后触发重发
- 重发次数受 `max_retries` 限制

### 3.4 全局收发计数与占用率统计

已实现。

相关代码：

- `struct motor_can_sched_stats`
- `log_window_if_needed()`

当前窗口日志包含：

- tx 忙时
- rx 忙时
- 预留忙时
- 总线负载估算
- 总 tx 数
- 总 rx 数
- ack 匹配数
- retry 数
- timeout 数
- drop 数

### 3.5 周期控制帧自动错峰

已实现。

相关代码：

- `phase_load[MOTOR_CAN_SCHED_SUPERFRAME]`
- `choose_phase_locked()`

当前行为：

- 周期任务先计算 `period_ticks`
- 调度器自动选择较低负载的释放相位
- 用于减小同一 CAN 总线上的瞬时拥塞

### 3.6 暴露 API

已实现一套更简化、面向驱动调用的接口。

当前主 API：

- `motor_can_sched_init()`
- `motor_can_sched_register_can()`
- `motor_can_sched_send()`
- `motor_can_sched_update()`
- `motor_can_sched_remove()`
- `motor_can_sched_report_rx()`
- `motor_can_sched_get_stats()`

驱动使用方式现在简化为：

- 提交 `can_frame`
- 同时提交发送参数
- 参数里说明：
  - 是否循环
  - 是否追踪回复
  - 高/低优先级
  - 是否有紧跟回复
  - 回复匹配使用的 `id/mask`
- 如果是循环帧，则返回一个句柄
- 后续通过句柄调用 `motor_can_sched_update()` 更新循环帧内容
- 不再要求驱动直接操作内部 `periodic job` 结构体

这样更适合多个不同电机驱动共同调用，因为每次提交都可以带各自独立的：

- `reply_id`
- `reply_mask`
- `timeout`
- `retry`
- `tag`

当前限制：

- 现有电机驱动还没有正式接入这套调度器
- 当前 demo 只是先验证这套简化 API 是否够用

## 4. 日志设计

这次日志不是单纯加打印，而是专门围绕“如何证明功能真的在工作”来设计的。

### 4.1 特定帧回复生命周期日志

对于设置了 `trace_lifecycle = true` 的重要帧，调度器会输出：

- `queue ...`
- `tx ...`
- `ack ok ...`
- `ack timeout ...`
- `give up ...`
- `tx busy requeue ...`

这条日志链就是“特定帧回复功能”的主要证据。

### 4.2 probe 探测帧

在 `src/main.c` 中，每个电机每 1 秒发送一次 probe：

- `dm0-probe`
- `dm1-probe`
- `dm2-probe`
- `dm3-probe`

这样做的原因是：

- 周期控制帧频率太高，不适合作为唯一审查依据
- probe 更适合直接观察“发出某一帧后，是否收到了对应回复”

建议重点关注这些日志关键词：

- `probe submitted`
- `queue seq=`
- `tx seq=`
- `ack ok`
- `ack timeout`
- `give up`
- `util`

### 4.3 控制日志噪声

DM 状态回包日志被降到了 `LOG_DBG`，避免淹没真正需要看的回复追踪日志。

## 5. 审查发现

### 5.1 本轮审查中已顺手修复的问题

在整理 README 前，我发现并修复了一个问题：

- 重发帧之前会生成新的 `seq`
- 这样同一次“等待回复”的过程，在日志里会看起来像几次不同请求
- 现在已经改为重发时保留原始 `trace_id`

这样一来，一次特定帧的：

- `queue`
- `tx`
- `timeout`
- `retry`
- `ack`

都能用同一个 `seq` 串起来，便于审查。

### 5.2 当前已知限制

这些不是编译错误，但仍然是你审查时值得重点看的边界：

- 调度器还没有真正接入现有 motor driver
- demo 直接手动打包 DM 协议帧，没有复用 `motor_dm.c`
- 周期任务移除后，没有重新计算 `phase_load`
- 回复匹配目前只做到 ID/mask 级别，没有做 payload 语义校验
- 超时策略是固定重发，没有退避
- 总线利用率是“估算统计”，不是总线分析仪级别的精确测量

## 6. 构建状态

这个 demo 之前已经在 `.vscode/tasks.json` 所指向的虚拟环境中成功编译过。

已确认环境：

- `~/zephyrproject/.venv`
- Python `3.12`

实际使用的构建命令：

```bash
source ~/zephyrproject/.venv/bin/activate
XDG_CACHE_HOME=/tmp/xdgcache CCACHE_DISABLE=1 \
west build -d /tmp/build_dm_can_sched_demo -p always -b robomaster_board_c \
src/dm_can_sched_demo -- -DZEPHYR_EXTRA_MODULES=/home/xiexiang/zephyrproject/zephyr_ws
```

补充说明：

- `XDG_CACHE_HOME=/tmp/xdgcache` 是为了绕开当前环境里默认 Zephyr cache 路径不可写的问题
- `CCACHE_DISABLE=1` 是为了绕开当前环境里默认 ccache 临时目录只读的问题

## 7. 建议的审查顺序

建议你按下面顺序看：

1. 独立 demo 的做法，是否符合“先不动老驱动，只做新代码验证”的要求
2. 调度器 API 形态，是否适合作为后续接入各类 motor driver 的基础
3. 1 kHz 节拍模型和回复留白模型，是否符合你的预期总线负载
4. 只按 ID/mask 做回复匹配，作为第一版是否足够
5. 当前 timeout/retry 默认值，是否适合 DM 电机
6. 当前日志是否足够支撑现场排查“某帧没收到回复”
7. 下一步你更倾向于：
   直接接入 `motor_dm.c`
   还是先继续打磨这层独立中间件

## 8. 我建议的下一步

我建议的顺序是：

- 先把这个 demo 上板跑起来
- 确认 `probe -> ack ok` 是稳定的
- 再人为制造一次“无回复”场景，确认 `ack timeout -> retry -> give up` 是闭环成立的
- 确认逻辑稳定后，再决定是：
  - 直接把 `dm_motor.c` 接进这套中间层
  - 还是先抽一层更适合电机驱动调用的高级 API
