# connector_wye_debug

`connector_wye_debug` 是 `connector_wye_motor` 的最小位置调试包。

- 上电后使能 `connector_wye_motor`，并自动移动到 prepare 位置 `-800.0f`。
- 通过 USB bulk + ARES `dual_protocol` 接收一个 SYNC 指令。
- 指令 ID 为 `0x0205`，payload 为 1 个小端 `float32 angle`。
- 收到角度后使用 `ML_ANGLE` 模式控制 `connector_wye_motor` 到目标位置。

帧格式：

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| head | 2 | SYNC magic，`0x5A5A` |
| id | 2 | `0x0205` |
| angle | 4 | 目标角度，`float32` 小端 |

构建示例：

```sh
/home/xiexiang/zephyrproject/.venv/bin/west build -b robomaster_board_c src/connector_wye_debug -d build/connector_wye_debug -p always -- -DUSER_CACHE_DIR=/home/xiexiang/zephyrproject/zephyr_ws/build/zephyr-cache -DUSE_CCACHE=0
```
