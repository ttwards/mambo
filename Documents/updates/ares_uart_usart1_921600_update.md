<!--
SPDX-License-Identifier: Apache-2.0
-->

# ARES UART USART1 921600 Update

Date: 2026-07-01
Board: Damiao MC-02, STM32H723
Test UART: USART1 on PA9/PA10
Log UART: USART10 at 115200
Host UART device: `/dev/cu.usbmodem00000000050C1`

## Summary

This update brings up the ARES UART interface on MC-02 USART1 and validates it
with real ARES dual-protocol FUNC/REPL frames at 921600 baud. The test path does
not bypass the protocol stack: the host sends FUNC frames, the board runs the
sample `func_cb`, and the board sends REPL frames back through the UART
interface.

## Bugs Found

- The communication sample hard-coded `DT_NODELABEL(usart6)`, while MC-02 test
  hardware is wired to USART1.
- `CONFIG_UART_ASYNC_API` was not enabled in the UART test build, causing
  `uart_callback_set()` to fail with `-ENOSYS`.
- STM32 async UART requires DMA. Without `dmas`/`dma-names`, `uart_tx()` failed
  with `-ENODEV`.
- STM32H7 async UART DMA rejects cached buffers. RX and TX initially failed with
  `Rx/Tx buffer should be placed in a nocache memory region` and `-EFAULT`.
- The UART driver created threads with `K_NO_WAIT` and then called
  `k_thread_start()` again, which can restart an already started thread and led
  to an MPU fault during bring-up.
- UART thread stacks were declared as raw `k_thread_stack_t[]` members instead
  of Zephyr kernel stack members.

## Changes

- The sample now honors `chosen { ares,uart = ...; }` before falling back to
  `usart6`, so board-specific UART selection can be done in an overlay.
- Added `samples/communication/ares_communication/boards/dm_mc02_usart1_921600.overlay`
  for USART1 PA9/PA10 at 921600 baud with STM32H7 DMAMUX requests 42/41.
- UART RX slab is placed in `__nocache`.
- UART TX copies each outgoing `net_buf` into a dedicated `__nocache` DMA buffer
  before calling `uart_tx()`.
- UART exposes TX-complete callback support and interface caps/MTU so the dual
  protocol can clear in-flight state when real UART TX completes.
- Removed redundant `k_thread_start()` calls after `k_thread_create(..., K_NO_WAIT)`.
- Changed UART stack members to `K_KERNEL_STACK_MEMBER`.

## Build Commands

Functional/log-enabled test:

```sh
west build -p always -b dm_mc02/stm32h723xx \
  samples/communication/ares_communication \
  -d build_uart_test -- \
  -DDTC_OVERLAY_FILE="samples/communication/ares_communication/boards/dm_mc02.overlay;samples/communication/ares_communication/boards/dm_mc02_usart1_921600.overlay" \
  -DCONFIG_UART_INTERFACE=y \
  -DCONFIG_UART_ASYNC_API=y \
  -DCONFIG_NOCACHE_MEMORY=y
```

Transport benchmark without application logging:

```sh
west build -p always -b dm_mc02/stm32h723xx \
  samples/communication/ares_communication \
  -d build_uart_test -- \
  -DDTC_OVERLAY_FILE="samples/communication/ares_communication/boards/dm_mc02.overlay;samples/communication/ares_communication/boards/dm_mc02_usart1_921600.overlay" \
  -DCONFIG_UART_INTERFACE=y \
  -DCONFIG_UART_ASYNC_API=y \
  -DCONFIG_NOCACHE_MEMORY=y \
  -DCONFIG_LOG=n
```

Flash:

```sh
west flash -d build_uart_test --no-rebuild --runner openocd \
  --config boards/damiao/dm_mc02/support/openocd.cfg
```

## Functional Validation

- Board initialized `uart_protocol` successfully on USART1.
- Board transmitted ARES heartbeat frames on USART1.
- Host heartbeat frames kept `uart_protocol` online.
- Host FUNC frame:

```text
ca fe 01 00 11 00 00 00 22 00 00 00 33 00 00 00 34 12
```

Board log confirmed:

```text
func_cb
params: 11,22,33
```

## Performance

All rows use 921600 baud, FUNC requests are 18 bytes, REPL replies are 10 bytes.
RTT is measured at the host from FUNC write to matching REPL read. At overloaded
rates, REPL request IDs wrap at 8 bits in the current protocol, so tail RTT is
best read as queue/overload indication rather than precise per-frame latency.

### Log Enabled

The sample's `func_cb` logs two lines per incoming FUNC. USART10 logging at
115200 saturated around 9.4 KiB/s and became a system-level bottleneck.

| Target FPS | Sent FPS | Reply FPS | Success | Avg RTT ms | P95 RTT ms | P99 RTT ms | Max RTT ms |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 500 | 500.0 | 500.0 | 100.00% | 1.03 | 1.15 | 1.72 | 3.34 |
| 1000 | 1000.0 | 1000.0 | 100.00% | 1.03 | 1.13 | 1.33 | 3.65 |
| 2000 | 1998.7 | 1989.0 | 99.52% | 8.21 | 128.78 | 129.20 | 131.88 |
| 3000 | 2818.0 | 2777.3 | 98.56% | 16.07 | 86.65 | 170.87 | 343.31 |
| 4000 | 2759.0 | 2721.0 | 98.62% | 23.31 | 103.72 | 198.97 | 331.67 |
| 5000 | 2767.0 | 2733.7 | 98.80% | 17.27 | 97.50 | 169.64 | 271.34 |

Practical log-enabled operating point: 1000 FUNC/REPL fps with low latency and
no observed loss in the 3 s run. Higher rates still return many frames, but the
log backend is saturated and tail latency rises sharply.

### Log Disabled

This removes application log traffic while keeping the real ARES UART
FUNC/REPL path. The wire-rate request-side limit for 18-byte frames at 921600
8N1 is about 5120 FUNC fps.

| Target FPS | Sent FPS | Reply FPS | Success | Avg RTT ms | P50 RTT ms | P95 RTT ms | P99 RTT ms | Max RTT ms |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1000 | 979.7 | 969.3 | 98.95% | 22.93 | 0.93 | 256.85 | 257.17 | 573.82 |
| 2000 | 2000.0 | 1977.3 | 98.87% | 19.65 | 0.89 | 128.93 | 256.88 | 384.85 |
| 3000 | 3000.0 | 2824.0 | 94.13% | 113.50 | 86.08 | 341.76 | 342.50 | 512.89 |
| 4000 | 4000.0 | 2948.0 | 73.70% | 386.33 | 384.53 | 896.55 | 1088.59 | 1345.09 |
| 5000 | 5000.0 | 1898.3 | 37.97% | 952.40 | 914.41 | 1817.98 | 2021.38 | 2328.66 |
| 6000 | 5996.3 | 1464.0 | 24.41% | 997.69 | 928.22 | 2254.06 | 2461.94 | 2803.09 |

Practical log-disabled operating point: around 2000 FUNC/REPL fps for sustained
testing with about 98.9% replies in this run. The maximum observed reply
throughput was about 2948 REPL fps at a 4000 fps request target, but that mode
is overloaded and drops roughly 26% of replies.

The stable 2000 fps point corresponds to approximately 36.0 KiB/s of request
frame bytes host-to-board and 19.3 KiB/s of reply frame bytes board-to-host,
or about 55.3 KiB/s total protocol frame bytes across the full-duplex UART.

## Remaining Bottleneck

The UART transport is now functional at 921600, but the current dual-protocol
FUNC reply path has only one in-flight reply buffer per function mapping. When
FUNC frames arrive faster than UART TX completion can clear that in-flight bit,
extra replies are intentionally skipped. Raising the loss-free ceiling will
require queuing REPL frames or giving each pending reply its own buffer and
request ID storage.
