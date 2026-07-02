# 电机驱动 API

电机接口分成三层语义：

- `motor_setpoint_t`: 用户下发的目标设定。
- `motor_status_t`: 驱动回读的当前状态。
- `motor_controller_info_t`: 驱动声明的控制器。

`motor_set()` 是一次原子操作：控制器选择和目标量一起下发。用户可以让驱动按 `(mode, target)` 选择默认控制器，也可以显式指定 controller ID。

电机驱动只和 controller 契约交互：把 `motor_status_t` 和 `motor_setpoint_t` 组成输入交给 controller，controller 输出驱动可执行的控制量；对 DM、MI、RS、LK 这类原生闭环电机，驱动只从 controller 读取协议需要的参数并发送给电机。

```c
struct motor_driver_api {
    void (*motor_control)(const struct device *dev, enum motor_cmd cmd);
    int (*motor_set)(const struct device *dev, motor_setpoint_t *setpoint);
    int (*motor_get)(const struct device *dev, motor_status_t *status);
};
```

## 控制模式

```c
enum motor_mode {
    MIT = 0,
    PV = 1,
    VO = 2,
};
```

- `MIT`: 电机原生 MIT 控制。
- `PV`: 位置目标控制，通常是位置环和速度环串联。
- `VO`: 单目标量控制，当前用于速度或扭矩目标。

具体目标由 `enum motor_target` 指定：

```c
enum motor_target {
    MOTOR_TARGET_NONE = 0,
    MOTOR_TARGET_TORQUE = 1,
    MOTOR_TARGET_SPEED = 2,
    MOTOR_TARGET_POSITION = 3,
};
```

## 控制器选择

控制器来自 devicetree 的 `controllers` 列表，每个控制器是一个独立的 `motor-controller,*` 节点。用户态可以枚举控制器：

```c
int count = motor_get_controller_count(motor);

for (int i = 0; i < count; i++) {
    motor_controller_info_t info;

    motor_get_controller_info(motor, i, &info);
}
```

控制器信息：

```c
struct motor_controller_info {
    uint8_t id;
    enum motor_mode mode;
    enum motor_target target;
    enum motor_output_type output;
    uint32_t required_states;
    char name[32];
};
```

`required_states` 声明 controller 需要驱动提供哪些状态量，`output` 声明 controller 输出量类型。这样后续接 ADRC、MPC 等更高级 controller 时，可以继续复用同一套声明和选择接口。

controller 的执行接口：

```c
int motor_controller_update(struct motor_controller_data *data,
                            const struct motor_controller_config *cfg,
                            const struct motor_controller_input *input,
                            struct motor_controller_output *output);

int motor_controller_get_params(const struct motor_controller_config *cfg,
                                uint8_t index,
                                struct motor_controller_params *params);

void motor_controller_reset(struct motor_controller_data *data,
                            const struct motor_controller_config *cfg);
```

## 设定与状态

```c
struct motor_setpoint {
    float angle;
    float rpm;
    float torque;
    float speed_limit[2];
    float torque_limit[2];
    enum motor_mode mode;
    enum motor_target target;
    enum motor_controller_select controller_select;
    uint8_t controller_id;
};
```

```c
struct motor_status {
    float angle;
    float rpm;
    float torque;
    float temperature;
    float sum_angle;
    float speed_limit[2];
    float torque_limit[2];
    enum motor_mode mode;
    enum motor_target target;
    uint8_t controller_id;
    bool online;
    bool enabled;
    int error;
};
```

`PV + MOTOR_TARGET_POSITION` 和 `VO + MOTOR_TARGET_SPEED` 中的 `torque` 字段作为力矩前馈使用。只有控制器输出为力矩、且驱动支持叠加前馈时才会生效；例如 DJI 级联控制器会把该值加到速度环输出后的目标力矩上。`VO + MOTOR_TARGET_TORQUE` 则表示直接扭矩目标，不需要 controller 节点。

## 示例

默认控制器下发速度目标：

```c
motor_setpoint_t setpoint = {
    .mode = VO,
    .target = MOTOR_TARGET_SPEED,
    .rpm = 1000.0f,
};

motor_set(motor, &setpoint);
```

速度目标加力矩前馈：

```c
motor_setpoint_t setpoint = {
    .mode = VO,
    .target = MOTOR_TARGET_SPEED,
    .rpm = 1000.0f,
    .torque = 0.2f,
};

motor_set(motor, &setpoint);
```

下发 MIT 目标：

```c
motor_setpoint_t setpoint = {
    .mode = MIT,
    .target = MOTOR_TARGET_POSITION,
    .angle = 90.0f,
    .rpm = 10.0f,
    .torque = 0.2f,
};

motor_set(motor, &setpoint);
```

按 ID 选择控制器下发位置目标：

```c
motor_setpoint_t setpoint = {
    .mode = PV,
    .target = MOTOR_TARGET_POSITION,
    .angle = 180.0f,
    .controller_select = MOTOR_CONTROLLER_BY_ID,
    .controller_id = 1,
};

motor_set(motor, &setpoint);
```

获取当前状态：

```c
motor_status_t status;

motor_get(motor, &status);
LOG_INF("angle: %.2f rpm: %.2f torque: %.2f",
        (double)status.angle,
        (double)status.rpm,
        (double)status.torque);
```
