# 电机驱动 Device Tree 配置

## Controller 配置语义

电机节点通过 `controllers` 引用一个或多个 `motor-controller,*` 节点。每个 controller 节点自己声明模式、目标和参数，不再有电机 `capabilities`，电机控制器也不再引用旧的通用控制器设备。

当前通用 controller 类型：

| compatible | mode | target | 说明 |
| --- | --- | --- | --- |
| `"motor-controller,mit"` | `MIT` | `MOTOR_TARGET_POSITION` | 电机原生 MIT 控制参数 |
| `"motor-controller,pv"` | `PV` | `MOTOR_TARGET_POSITION` | 位置-速度串级控制 |
| `"motor-controller,vo"` | `VO` | `MOTOR_TARGET_SPEED` 或 `MOTOR_TARGET_TORQUE` | 单环控制 |

`VO + MOTOR_TARGET_TORQUE` 可以是驱动原生直通扭矩目标，不一定需要 controller 节点。例如 DJI 的直接扭矩控制就是 `motor_set_torque()` 路径。

## Controller 节点

通用属性：

- `#controller-cells`: 必须是 `<0>`
- `compatible`: 必须是 `"motor-controller,mit"`、`"motor-controller,pv"` 或 `"motor-controller,vo"`
- `k_p` / `k_i` / `k_d`: 单环或 MIT 参数
- `pos_k_p` / `pos_k_i` / `pos_k_d`: PV 位置环参数
- `vel_k_p` / `vel_k_i` / `vel_k_d`: PV 速度环参数
- `i_max` / `out_max` / `detri_lpf` / `offset`: 通用参数，可选
- `pos_i_max` / `pos_out_max` / `pos_detri_lpf` / `pos_offset`: PV 位置环覆盖参数，可选
- `vel_i_max` / `vel_out_max` / `vel_detri_lpf` / `vel_offset`: PV 速度环覆盖参数，可选
- `target`: 仅 `motor-controller,vo` 使用，`"speed"` 或 `"torque"`，默认 `"speed"`

PV controller 会优先读取 `pos_*`、`vel_*` 的分环参数；未配置时回退到通用 `i_max`、`out_max`、`detri_lpf`、`offset`。

```dts
/ {
    controller {
        speed_ctrl: speed_ctrl {
            #controller-cells = <0>;
            compatible = "motor-controller,vo";
            target = "speed";
            k_p = "2.65";
            k_d = "60";
            out_max = "3";
        };

        angle_ctrl: angle_ctrl {
            #controller-cells = <0>;
            compatible = "motor-controller,pv";
            pos_k_p = "3";
            pos_k_d = "320";
            pos_out_max = "8000";
            vel_k_p = "2.65";
            vel_k_d = "60";
            vel_out_max = "3";
        };

        mit_ctrl: mit_ctrl {
            #controller-cells = <0>;
            compatible = "motor-controller,mit";
            k_p = "1";
            k_d = "0.1";
        };
    };
};
```

## DJI Motor Device Tree Properties

- `compatible`: 必须是 `"dji,motor"`
- `is_m3508` / `is_m2006` / `is_gm6020`: 电机型号，可选
- `id`: 电机编号
- `rx_id`: 电机接收 ID
- `tx_id`: 电机发送 ID
- `gear_ratio`: 减速比
- `status = "okay"`: 启用电机
- `can_channel`: CAN 总线节点
- `controllers`: controller 节点列表，可选

```dts
/ {
    motor1 {
        compatible = "dji,motor";
        is_m3508;
        id = <1>;
        rx_id = <0x201>;
        tx_id = <0x200>;
        gear_ratio = "19.20";
        status = "okay";
        can_channel = <&can1>;
        controllers = <&angle_ctrl &speed_ctrl>;
    };
};
```

## DM / MI / RS / LK Motor Device Tree Properties

- `compatible`: 对应 `"dm,motor"`、`"mi,motor"`、`"rs,motor"` 或 `"lk,motor"`
- `id`: 电机编号
- `tx_id`: 电机发送 ID
- `rx_id`: 电机接收 ID
- `can_channel`: CAN 总线节点
- `gear_ratio`: 减速比
- `controllers`: controller 节点列表，可选
- `p_max` / `v_max` / `t_max`: 角度、速度、扭矩限制，按驱动支持情况配置
- `freq`: 控制发送频率，按驱动支持情况配置

```dts
/ {
    motor1: motor1 {
        compatible = "dm,motor";
        id = <24>;
        rx_id = <24>;
        tx_id = <86>;
        gear_ratio = "1";
        status = "okay";
        can_channel = <&can1>;
        controllers = <&mit_ctrl>;
        p_max = "12.5";
        v_max = "45";
        t_max = "10";
        freq = <1000>;
    };
};
```

一个同时声明 MIT、PV、VO 的示例：

```dts
/ {
    motor1: motor1 {
        compatible = "dm,motor";
        id = <1>;
        rx_id = <0x00>;
        tx_id = <0x01>;
        status = "okay";
        can_channel = <&can1>;
        controllers = <&mit_ctrl &angle_ctrl &speed_ctrl>;
        p_max = "12.5";
        v_max = "45";
        t_max = "10";
        freq = <1000>;
    };
};
```

## VESC Motor Device Tree Properties

VESC 支持原生 `VO + TORQUE`、`VO + SPEED` 和 `PV + POSITION` 下发路径，可以不配置 `controllers`。只有需要用户态枚举并显式选择 controller 时，才需要声明 controller 节点。
