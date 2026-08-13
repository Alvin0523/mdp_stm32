## Why

`mdp_stm32` firmware currently only runs a board bring-up test (LED blink + motor PWM init) — it has no way to exchange data with the host at all. Per [ADR 0001](../../../../docs/adr/0001-microros-transport.md), the host↔MCU link is supposed to be micro-ROS (rclc) over `USART3`, but that transport has never been wired in. This blocks every other MCU capability (motor commands, encoder telemetry, steering, IMU) that depends on the host actually being able to talk to the board. Now that board bring-up is verified working on real hardware, this is the next unblocking step.

## What Changes

- Import the micro-ROS Zephyr module (`micro-ROS/micro_ros_zephyr_module`) into `app/west.yml`, pinned to the commit already vetted locally at `references/micro_ros_zephyr_module/` (tested against Zephyr v4.0.0, the exact version this workspace pins).
- Add the Kconfig options micro-ROS needs to `app/prj.conf` (networking/transport, `pipe`/`serial` micro-ROS transport, entropy source, etc. — per the reference module's own `prj.conf`).
- Replace the placeholder bring-up loop in `app/src/main.c` with an rclc node that:
  - Initializes a micro-ROS serial transport over `USART3` (`PD8`/`PD9`, 115200 baud — already enabled in `app/boards/stm32f4_disco.overlay`).
  - Creates an rclc support/executor/node.
  - Publishes a minimal heartbeat/counter topic on a timer, to prove the end-to-end link (agent connects, receives messages) without touching real actuators or sensors.
- Keep the existing LED blink and motor PWM init from the bring-up test as-is, running alongside the new node — this change only adds the transport, it doesn't replace what's already verified working.

## Capabilities

### New Capabilities
- `micro-ros-transport`: MCU firmware establishes and maintains a micro-ROS (rclc) session with a host micro-ROS Agent over `USART3` serial, and can publish at least one topic over that session.

### Modified Capabilities
(none — no existing specs for `mdp_stm32` yet)

## Impact

- `app/west.yml` — new module import (micro-ROS Zephyr module).
- `app/prj.conf` — new Kconfig options for micro-ROS support.
- `app/src/main.c` — adds rclc init/executor/publisher wiring; existing LED/motor init preserved.
- `app/CMakeLists.txt` — no new source files needed unless the node logic is split out (TBD in design.md).
- Host side (`mdp_ros`): out of scope for this change. A micro-ROS Agent must be run manually on the host to observe the published topic during verification, but no `mdp_ros` code changes.
- Does not touch `motor.c`'s 4-motor vs. 2-motor discrepancy (separate, already-scoped cleanup) or add real motor/encoder/servo/IMU wiring (separate TODO items in `docs/architecture.md`).
