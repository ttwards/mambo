# Motor Driver Refactor Notes

本文档用于说明后续 motor driver 重构时建议采用的层级结构。当前讨论范围主要覆盖
`dm`、`lk`、`mi`、`robstride`、`vesc` 驱动，`dji` 驱动暂不纳入本轮拆分。

## 背景

当前多个电机驱动已经能完成基本功能，但单个驱动文件中混合了较多职责：

- Zephyr `motor_driver_api` glue
- 目标状态缓存
- 协议帧打包与反馈解析
- CAN filter 注册
- CAN 发送排队
- 周期发送
- 在线检测
- 超时、重发、错误处理
- PID 参数读取与下发

这种结构在早期实现阶段很直接，但后续接入统一 CAN 调度中间层、增加新电机协议、
增加测试和诊断能力时，会导致修改面过大。重构目标不是为了抽象而抽象，而是让不同变化原因的代码分离。

## 目标

- 让上层业务继续通过统一 `motor_set()`、`motor_get()`、`motor_control()` 使用电机。
- 让不同电机协议只关心自己的 CAN 帧格式。
- 让 CAN 发送调度、周期任务、优先级、总线统计由公共层处理。
- 让 reply 匹配、timeout、retry、online/offline 等通信生命周期逻辑可以复用。
- 让 DM、LK、MI、Robstride、VESC 能逐步迁移，不要求一次性推倒重写。

## 建议层级

```text
application / chassis / wheel
  |
  v
motor public API layer
  motor_set / motor_get / motor_control / motor_set_mode
  |
  v
motor control facade layer
  target state / mode / enable / disable / set zero
  |
  v
protocol codec layer
  DM / LK / MI / Robstride / VESC pack and parse CAN frames
  |
  v
transaction and lifecycle layer
  request / reply / timeout / retry / online / fault / parameter sync
  |
  v
CAN scheduler and bus layer
  priority / periodic / phase spreading / TX credit / bus stats
  |
  v
Zephyr CAN driver
```

## Layer 1: Motor Public API

这一层已经存在，定义在 `include/zephyr/drivers/motor.h`。

职责：

- 对应用层暴露统一电机接口。
- 屏蔽具体电机协议。
- 保持 `motor_driver_api` 的稳定性。

典型接口：

```c
motor_set(dev, status);
motor_get(dev, status);
motor_control(dev, ENABLE_MOTOR);
motor_set_mode(dev, MIT);
```

这一层不应该知道 CAN ID、CAN frame 格式、发送队列、重试策略。

## Layer 2: Motor Control Facade

这一层位于各具体驱动内部，是 `motor_driver_api` 到内部实现的适配层。

职责：

- 校验当前电机是否支持目标模式。
- 将 `motor_status_t` 转换为内部 target state。
- 处理 enable、disable、set zero、clear error 等电机语义命令。
- 决定某个操作应提交 one-shot request，还是更新 periodic job。

这一层可以知道“用户想让电机做什么”，但不应该直接关心 CAN 队列如何发送。

现有代码中的例子：

- `dm_set()`
- `lk_set()`
- `mi_set()`
- `rs_set()`
- `vesc_set()`

后续建议让这些函数只更新目标状态并调用下层抽象，例如：

```c
motor_txn_submit_control(dev, &target);
motor_txn_submit_command(dev, MOTOR_CMD_ENABLE);
motor_periodic_update(dev, &target);
```

## Layer 3: Protocol Codec

Codec 层只回答一个问题：

```text
这帧长什么样？
```

职责：

- 将目标状态打包成 CAN frame。
- 将 CAN frame 解析成反馈状态。
- 处理 CAN ID 编码、标准帧/扩展帧、命令字、参数 index。
- 处理 raw value 与物理量之间的缩放。

它不应该做：

- `can_send()`
- `can_send_queued()`
- `k_timer_start()`
- `k_work_submit()`
- online/offline 判断
- timeout/retry
- 总线调度

建议接口形态：

```c
int dm_codec_pack_control(const struct dm_motor_config *cfg,
                          const struct dm_target *target,
                          struct can_frame *frame);

int dm_codec_pack_command(const struct dm_motor_config *cfg,
                          enum motor_cmd cmd,
                          struct can_frame *frame);

int dm_codec_parse_feedback(const struct dm_motor_config *cfg,
                            const struct can_frame *frame,
                            struct motor_feedback *feedback);
```

现有代码中的候选拆分点：

- `dm_motor_pack()`
- `lk_motor_pack()`
- `mi_motor_pack()`
- `rs_motor_pack()`
- `vesc_motor_pack()`
- 各驱动中的 feedback raw data 解析逻辑

## Layer 4: Transaction And Lifecycle

Transaction/lifecycle 层只回答一个问题：

```text
这次通信算不算完成？
```

职责：

- 创建一次 request。
- 判断 request 是否需要 reply。
- 设置 reply matcher。
- 处理 timeout。
- 处理 retry。
- 判断电机 online/offline。
- 维护 last_rx_time、missed_count、enabled、fault state。
- 处理参数下发、模式切换、clear error 等多帧或有状态操作。

它不应该知道每个 byte 怎么编码，这些由 codec 层处理。
它也不应该直接管理 CAN TX mailbox，这些由 scheduler/bus 层处理。

建议接口形态：

```c
int motor_txn_submit(struct motor_txn_ctx *ctx,
                     const struct motor_txn_request *req);

void motor_txn_report_rx(struct motor_txn_ctx *ctx,
                         const struct can_frame *frame);

void motor_lifecycle_tick(struct motor_lifecycle *life,
                          uint32_t now_ms);
```

现有代码中的候选拆分点：

- DM `prev_recv_time`、`online`、`enabled`、`tx_cnt` 相关逻辑
- LK `missed_times`、PID 参数同步逻辑
- MI `missed_times`、双帧参数发送逻辑
- Robstride 主动上报监控与 `last_report_time`
- 各驱动 enable、disable、clear error 后是否等待反馈的策略

## Layer 5: CAN Scheduler And Bus

这一层只回答一个问题：

```text
什么时候发哪一帧？
```

职责：

- 注册 CAN bus。
- 管理 CAN TX mailbox credit。
- 提供一次性发送接口。
- 提供周期发送接口。
- 按 priority、deadline、period 调度。
- 对周期控制帧做 phase spreading。
- 统计 TX/RX、drop、retry、bus utilization。
- 对上层隐藏 Zephyr CAN 发送细节。

现有 `drivers/motor/common/common.c` 中的 `can_send_queued()` 可以作为旧 backend 保留。
新的 `motor_can_sched` 可以作为下一代 backend 演进。

建议接口形态：

```c
int motor_can_submit(const struct device *can_dev,
                     const struct can_frame *frame,
                     const struct motor_can_submit_opts *opts);

int motor_can_periodic_start(const struct device *can_dev,
                             const struct can_frame *frame,
                             const struct motor_can_periodic_opts *opts,
                             motor_can_periodic_handle_t *handle);

int motor_can_periodic_update(motor_can_periodic_handle_t handle,
                              const struct can_frame *frame);

int motor_can_periodic_stop(motor_can_periodic_handle_t handle);
```

## Layer 6: State Model

每个电机驱动内部状态建议按用途分组，而不是所有字段平铺。

```text
target state:
  用户期望的 angle / rpm / torque / mode / limits

feedback state:
  电机反馈的 angle / rpm / torque / temperature / error

communication state:
  online / enabled / last_rx_time / missed_count / pending request
```

这样可以避免 `motor_get()`、`motor_set()`、RX callback、TX scheduler 同时修改同一批字段。

## SOLID 对应关系

### Single Responsibility

每层只有一个变化原因：

- 协议手册变化：改 codec。
- 发送调度策略变化：改 scheduler。
- 超时重试策略变化：改 transaction。
- 用户 API 变化：改 public API/facade。

### Open/Closed

新增一种电机协议时，应新增 codec 和 ops，而不是修改公共 scheduler。

新增一种发送策略时，应新增 scheduler/backend，而不是修改每个驱动的 pack 函数。

### Liskov Substitution

应用层使用 `motor_set()` 时，不应该关心底层是 DM、LK、MI、Robstride 还是 VESC。

驱动层使用 `motor_can_submit()` 时，不应该关心 backend 是旧 `can_send_queued()` 还是新 scheduler。

### Interface Segregation

不要把 one-shot、periodic、reply tracking、retry、stats 全塞进一个巨大的 send 参数。

建议拆成：

- `submit_once`
- `start_periodic`
- `update_periodic`
- `request_reply`
- `get_stats`

调用方不应该被迫传入自己不关心的参数。

### Dependency Inversion

具体电机驱动不应该直接依赖 Zephyr `can_send()`。

推荐依赖方向：

```text
motor driver -> motor CAN abstraction -> backend implementation -> Zephyr CAN
```

这样可以在不改驱动协议代码的情况下替换发送实现，也方便 fake CAN 测试。

## 渐进迁移计划

### Step 1: 抽 codec

先把各驱动中的 pack/parse 函数拆出来，保持行为不变。

优先级建议：

1. DM
2. LK
3. MI
4. Robstride
5. VESC

完成标准：

- `*_set()` 不再直接手写 CAN payload。
- RX callback 不再直接散落 raw-to-physical 转换。
- codec 函数不调用 CAN/RTOS API。

### Step 2: 包一层 CAN TX abstraction

新增公共发送抽象，内部先继续调用旧 `can_send_queued()`。

完成标准：

- 各驱动不再直接调用 `can_send_queued()`。
- 后续不同驱动可选择旧 backend 和新 scheduler backend。

### Step 3: 接入周期发送 abstraction

将 DM/LK/MI 中各自的 timer/workqueue 周期发送逐步迁移到公共 periodic job。

完成标准：

- 驱动只更新 target frame。
- 周期、错峰、优先级由公共 scheduler 处理。

### Step 4: 抽 transaction/lifecycle

将 reply tracking、timeout、retry、online/offline 从具体驱动中收敛。

完成标准：

- 重要命令可以声明 matcher、timeout、retry。
- offline 规则可按协议配置。
- 诊断统计统一输出。

### Step 5: 增加 fake backend 和测试

新增 fake CAN backend，用于验证 codec、transaction 和 scheduler。

重点测试：

- pack/parse golden frame
- reply match
- timeout/retry
- periodic rate
- queue overflow/drop policy
- multi-bus behavior

## 判断代码归属的简单规则

如果改协议手册会影响它，它属于 codec。

如果改 timeout/retry/online 策略会影响它，它属于 transaction/lifecycle。

如果改 CAN 队列、优先级、周期错峰会影响它，它属于 scheduler/bus。

如果改用户如何调用电机会影响它，它属于 public API/facade。

## 重构原则

- 保持每一步行为可验证，不做一次性大重写。
- 先抽纯函数，再抽状态机，最后替换发送 backend。
- 公共层只放跨驱动真正共享的机制，不放某个协议的特例。
- 协议特例留在具体 codec 或 protocol ops 中。
- 每次移动代码都保留原有 public API。

