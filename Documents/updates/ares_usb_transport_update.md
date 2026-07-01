<!-- SPDX-License-Identifier: Apache-2.0 -->

# ARES USB Transport Update

Date: 2026-07-01
Board under test: dm_mc02 / STM32H723
USB device: ARES Bulk Interface Async, full-speed USB, VID:PID 1209:0001

## Summary

This update fixes the ARES USB bulk transport and dual protocol reconnect path. The
main symptom was that the USB endpoints could still receive host frames while the
protocol state stayed offline or the reply path stayed permanently busy after a
host disconnect/reconnect.

No benchmark-only firmware hooks are part of this update. The final validation was
done on the normal `samples/communication/ares_communication` application.

## Root Cause

The old USB path treated `USBD_STATE_CONFIGURED` as the main source of truth for
the protocol `CONNECTED` event. In practice, the Zephyr USB class `enable()`
callback is the reliable point where the interface has been enabled and the first
bulk OUT transfer can be submitted.

There was also a second reconnect bug in the dual protocol TX path. The old
implementation used mutex-style send ownership. After a host disconnect, an IN
transfer could be left without a normal completion path. The board continued to
receive and parse FUNC frames, but the matching reply path stayed marked busy, so
later FUNC replies were suppressed.

## Old Architecture

- USB protocol connection was driven mostly by USB device state messages.
- Bulk OUT submission and protocol online state were loosely coupled.
- Reply and sync sends used a lock-like ownership model.
- Disconnect cleanup cleared the backup queue, but did not clear all in-flight
  FUNC/SYNC send state.
- Reconnecting the host could leave the device in a split-brain state:
  - OUT endpoint and parser were active.
  - Protocol online/reply state could remain stale.
  - Host could send frames but receive no replies.

## New Architecture

- USB class `enable()` first marks the interface enabled, then submits the initial
  bulk OUT buffer.
- The protocol receives `ARES_PROTOCOL_EVENT_CONNECTED` only after that OUT
  submission succeeds.
- A dedicated USB connected bit deduplicates `CONNECTED` and `DISCONNECTED`
  events from class callbacks and USB state messages.
- USB class `disable()`, reset, and suspend all flow through the same disconnect
  state helper.
- The ARES interface API now exposes TX completion callbacks and simple transport
  capabilities.
- USB bulk stores the TX completion callback in `net_buf` user data and calls it
  from IN completion.
- Dual protocol FUNC/SYNC sends use atomic `DUAL_TX_IN_FLIGHT` state that is
  cleared by TX completion.
- Offline/reconnect cleanup now clears:
  - FUNC in-flight state.
  - SYNC in-flight state.
  - FUNC reply backup queue.
  - FUNC backup count.

The intended state model is:

1. USB class enable succeeds.
2. Initial OUT buffer is queued.
3. Protocol is marked connected.
4. Protocol sends use TX completion to release in-flight state.
5. Disconnect/reconnect cleanup always resets stale TX state before going online
   again.

## Files Changed

- `include/ares/interface/ares_interface.h`
- `lib/ares/interface/ares_interface.h`
- `include/ares/interface/usb/usb_bulk.h`
- `lib/ares/interface/usb/usb_bulk.h`
- `include/ares/protocol/dual/dual_protocol.h`
- `lib/ares/interface/usb/usb_bulk.c`
- `lib/ares/protocol/dual/dual_protocol.c`

## Validation

Build:

```sh
west build -d build
```

Flash:

```sh
west flash -d build --no-rebuild --runner openocd --config mambo/boards/damiao/dm_mc02/support/openocd.cfg
```

Startup log after reset showed the expected class-driven connection path:

```text
ares_usb_bulk_async: Enable ARES Bulk interface
dual_protocol: dual_protocol Connection established due to event.
```

Three consecutive host reconnect tests completed successfully. Before this fix,
later runs could drop to `replies=0` while the board still logged incoming
`func_cb` calls.

```text
Run 1: sent=45949 replies=45449 send_fps=9189.8 reply_fps_over_send=9089.8
Run 2: sent=46238 replies=45819 send_fps=9247.6 reply_fps_over_send=9163.8
Run 3: sent=46039 replies=45626 send_fps=9207.8 reply_fps_over_send=9125.2
```

## Performance Data

These numbers measure the normal ARES protocol path, not raw USB bulk maximum
throughput. The tested application also still emits serial logs and periodic
application traffic.

Best observed exec/reply rate:

- Host-to-device exec send rate: 9247.6 frames/s.
- Device-to-host reply rate during send window: 9163.8 frames/s.
- Exec frame payload on the wire: 18 bytes.
- Reply frame payload on the wire: 10 bytes.
- Approximate effective protocol payload throughput:
  - OUT: 162.6 KiB/s.
  - IN replies: 89.5 KiB/s.
  - Combined: 252 KiB/s, about 2.06 Mbit/s.

Round-trip latency for single `exec -> reply` calls:

- Typical RTT: 0.28 ms to 0.36 ms.
- Average RTT: 0.30 ms to 0.39 ms.
- p95 RTT: 0.44 ms to 0.64 ms.
- p99 RTT: 0.56 ms to 1.12 ms.
- Observed max RTT: 1.4 ms to 3.8 ms.

For application budgeting, `1 ms` is a conservative round-trip target for the
current full-speed USB ARES protocol path.

## Notes

- The board enumerated as full-speed USB during the test.
- The temporary USB bulk benchmark and UART diagnostic sample directories were
  deleted and are not part of this update.
- Raw USB bulk throughput can be higher than the protocol numbers above, but the
  protocol numbers are the relevant data for current ARES exec/reply behavior.
