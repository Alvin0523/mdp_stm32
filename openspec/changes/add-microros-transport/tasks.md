## 1. Host build dependencies

- [x] 1.1 Add `catkin_pkg`, `lark-parser`, `empy`, `colcon-common-extensions` to `pixi.toml` `[pypi-dependencies]` (required by the micro-ROS module's colcon build step, per `references/micro_ros_zephyr_module/README.md`)
- [x] 1.2 Run `pixi install` and confirm the new deps resolve cleanly

## 2. West manifest

- [x] 2.1 Add the micro-ROS Zephyr module to `app/west.yml`'s manifest, pinned to commit `229ffc0e131ee7942db3bb2e731fb8851583bb25` (the commit already vetted locally at `references/micro_ros_zephyr_module/`, tested against Zephyr v4.0.0)
- [x] 2.2 Run `pixi run west-update`, confirm the module fetches (at `micro_ros_zephyr_module/`, sibling to `zephyr/` — not under `modules/`, since the module's own `modules/libmicroros` subfolder is what CMake registers, not the repo root)
- [x] 2.3 Register the module in `app/CMakeLists.txt` via `list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../micro_ros_zephyr_module/modules/libmicroros)`, placed before `find_package(Zephyr REQUIRED ...)` (must be set before Zephyr's build system consumes it)

## 3. UART transport patch (USART1 → USART3)

- [x] 3.1 Write `scripts/patch_microros_uart.sh`: idempotent script that changes `#define UART_NODE DT_NODELABEL(usart1)` to `DT_NODELABEL(usart3)` in `micro_ros_zephyr_module/modules/libmicroros/microros_transports/serial/microros_transports.c`, following the same pattern (grep-before-patch, clear error if file structure doesn't match) as `scripts/patch_pyocd_runner.sh`. Verified: applies cleanly, idempotent on rerun.
- [x] 3.2 Add a `west-patch-microros` pixi task, and add it to the `setup` task's `depends-on` chain (after `west-update`, alongside the existing `west-patch`)
- [x] 3.3 Run `pixi run setup` end-to-end on a clean `modules/` (delete and re-fetch) to confirm both patches (pyocd runner + micro-ROS UART) apply cleanly together. Verified: both patches applied in order, no conflicts.

## 4. Kconfig

- [ ] 4.1 Add to `app/prj.conf`: `CONFIG_MICROROS=y`, `CONFIG_MICROROS_TRANSPORT_SERIAL=y`, `CONFIG_CPP=y`, plus the stack/thread/libc options the reference module's `prj.conf` uses (`CONFIG_MAIN_STACK_SIZE`, `CONFIG_NEWLIB_LIBC`, `CONFIG_POSIX_API`, `CONFIG_POSIX_CLOCK`, etc.) — cross-check each against what's already set for the bring-up test so nothing conflicting gets overridden
- [ ] 4.2 Verify `CONFIG_UART_INTERRUPT_DRIVEN=y` and `CONFIG_RING_BUFFER=y` end up enabled (required by the serial transport's IRQ + ring-buffer implementation)

## 5. Firmware: micro-ROS node on its own thread

- [ ] 5.1 Add a new source file (e.g. `app/src/microros_node.c` + `.h`) containing the rclc node setup, adapted from the reference module's `src/main.c` pattern: `rmw_uros_set_custom_transport()` → `rclc_support_init()` → node → publisher → timer → executor spin loop
- [ ] 5.2 Define the heartbeat publisher: a `std_msgs/Int32` (or similar minimal type) counter, incrementing on a fixed-period timer (e.g. 1000ms, matching the reference sample)
- [ ] 5.3 Run the micro-ROS init + executor spin loop on a dedicated Zephyr thread (`K_THREAD_DEFINE` or `k_thread_create`), separate from `main()`'s existing LED/motor loop, per design.md Decision 3
- [ ] 5.4 Replace `RCCHECK`'s halt-forever-on-failure behavior with a soft-fail/retry loop appropriate for "agent not yet connected" (log and keep retrying, don't need to be fancy — just must not deadlock the boot sequence)
- [ ] 5.5 Wire the new thread's start-up into `app/src/main.c`, after the existing LED/motor PWM init, without modifying that existing bring-up logic
- [ ] 5.6 Update `app/CMakeLists.txt`'s `target_sources` to include the new source file(s)

## 6. Hardware verification

- [ ] 6.1 `pixi run build && pixi run flash`, confirm it still boots clean with no Agent connected (LED blinks, motor PWM initializes, per existing bring-up behavior) — verifies the "Agent absent at boot" scenario
- [ ] 6.2 In `mdp_ros`, run `pixi run agent-build` (one-time, builds the standalone eProsima `Micro-XRCE-DDS-Agent` v3.0.1 — see `docs/troubleshooting.md` for why not the ROS2 `micro_ros_agent` colcon package) then `pixi run agent` (serial, `/dev/ttyUSB0` @ 115200) and confirm the MCU's session establishes (agent logs show a client connecting)
- [ ] 6.3 `ros2 topic list` / `ros2 topic echo <heartbeat-topic>` on the host, confirm the counter value is observable and increasing — verifies the "Heartbeat observable on host" scenario
- [ ] 6.4 Reconfirm LED blink + RTT logs still show the original bring-up success message while the Agent is connected — verifies the "Bring-up behavior unaffected" scenario

## 7. Docs

- [ ] 7.1 Update `docs/architecture.md`'s "What's implemented vs. TODO" table: mark micro-ROS integration `[x]` (or `[!]` with a note, if scoped narrower than full integration) once verified
- [ ] 7.2 Update `docs/stm32/drivers.md`'s implementation status table for the micro-ROS Transport row
