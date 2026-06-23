# usb_float51_rx

Minimal USB bulk receiver for the compact `/controller` `Float32MultiArray`
control payload.

- Board: `robomaster_board_c`
- SYNC ID: `0x0101`
- Payload: `7 * sizeof(float)`
- USB/protocol path: same style as `2026_R2usb_armv2`

Payload layout:

| Index | Meaning |
|---:|---|
| 0 | button bits `0..15` |
| 1 | button bits `16..31` |
| 2 | button bits `32..47` |
| 3 | `lx` |
| 4 | `ly` |
| 5 | `rx` |
| 6 | `ry` |

The code receives the array, decodes the three button masks with the maintained
`0..47` button-name table, and prints current pressed buttons plus left/right
stick values.

Build:

```sh
/home/xiexiang/zephyrproject/.venv/bin/west build -b robomaster_board_c src/usb_float51_rx -d build/usb_float51_rx -p always -- -DUSER_CACHE_DIR=/home/xiexiang/zephyrproject/zephyr_ws/build/zephyr-cache -DUSE_CCACHE=0
```
