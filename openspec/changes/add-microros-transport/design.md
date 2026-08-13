## Context

Investigated the locally vetted reference module (`references/micro_ros_zephyr_module/`, official `micro-ROS/micro_ros_zephyr_module`, commit `229ffc0`, tested against Zephyr v4.0.0) directly rather than guessing at the integration:

- It's imported as a `ZEPHYR_EXTRA_MODULES` CMake module (`modules/libmicroros`), not a `west.yml` project — the module's own `CMakeLists.txt` does `list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/libmicroros)`. So the west manifest brings the *source*, and our `app/CMakeLists.txt` has to register it the same way.
- `Kconfig` exposes `MICROROS_TRANSPORT_SERIAL` (real UART), `MICROROS_TRANSPORT_SERIAL_USB` (USB CDC-ACM device mode), and `MICROROS_TRANSPORT_UDP` (WiFi). We need `MICROROS_TRANSPORT_SERIAL` — `USART3` on this board is a real hardware UART bridged to USB by the onboard CH9102F chip, not native USB device mode.
- **Critical finding**: `modules/libmicroros/microros_transports/serial/microros_transports.c` hardcodes `#define UART_NODE DT_NODELABEL(usart1)`. The `MICROROS_SERIAL_PORT` Kconfig string is not actually read by this file — it's vestigial. So out of the box, this transport talks over `USART1`, not `USART3`. This needs a source patch, the same category of problem as the pyocd-runner fix already applied to `zephyr/boards/st/stm32f4_disco/board.cmake` (see `scripts/patch_pyocd_runner.sh`): a small, necessary edit inside a west-fetched directory that isn't part of this git repo and gets wiped/re-fetched on every `west update`.
- The reference module's own `src/main.c` is a working end-to-end example: `rmw_uros_set_custom_transport()` → `rclc_support_init()` → node → publisher → timer → `rclc_executor_spin_some()` loop. This is the pattern to adapt, not reinvent.

See proposal.md - Why / What Changes for motivation and scope.

## Goals / Non-Goals

**Goals:**
- Get a real, working micro-ROS session over `USART3` between the MCU and a host Agent, publishing one heartbeat topic, verified against actual hardware.
- Make the `USART1`→`USART3` transport fix durable across fresh clones and `west update` re-fetches (not a one-off manual edit).

**Non-Goals:**
- Choosing the specific message content/type of the heartbeat beyond "a counter that increases" — implementation detail, not worth pre-deciding here.
- Any change to how `mdp_ros` runs its micro-ROS Agent — out of scope per proposal.md.
- Reconnect/session-recovery logic beyond "don't hang boot if the agent isn't there yet" — that's a robustness concern for a later change once basic transport is proven.

## Decisions

**1. Pin the micro-ROS module import to the exact commit already vetted locally (`229ffc0`), not `main`.**
Alternative considered: track the module's default branch. Rejected — this workspace already pins Zephyr itself to `v4.0.0` for reproducibility (`app/west.yml`); floating the micro-ROS module while everything else is pinned would reintroduce exactly the kind of non-reproducible setup this repo has been actively fixing (see the `west-init`/workspace-layout and pyocd-runner fixes earlier in this project). The locally-cloned commit is also the one already confirmed compatible with Zephyr v4.0.0 by the module's own README.

**2. Patch `UART_NODE` from `usart1` to `usart3` via a tracked, idempotent script, not a manual edit.**
Alternative considered: vendor/copy the whole `libmicroros` module directly into `app/` instead of importing it via `west.yml`, so it's directly git-tracked and editable. Rejected — that would fork us off upstream micro-ROS entirely (losing the ability to pull fixes) for the sake of a one-line `DT_NODELABEL` change. Instead, follow the exact pattern already established for the pyocd-runner fix: `scripts/patch_microros_uart.sh`, idempotent (checks before patching), wired into the `setup` pixi task chain right after `west-update`/`west-patch`.

**3. Run the micro-ROS session (init + executor spin loop) on its own Zephyr thread, separate from `main()`'s existing LED/motor loop.**
The reference `main.c` sample uses `RCCHECK`, which spins forever (`for(;;){}`) on any failure — including a missing Agent at boot. That directly conflicts with this change's "Agent absent at boot" requirement (LED/motor bring-up must not be blocked). Alternative considered: keep everything in `main()` and make the micro-ROS init calls non-fatal (log-and-continue instead of `RCCHECK`'s halt). Rejected in favor of a separate thread — Zephyr's `rclc_support_init`/agent handshake behavior under "agent not yet connected" isn't something to assume is non-blocking without testing on this hardware; a dedicated thread makes the LED/motor loop's independence structural rather than relying on getting every micro-ROS error path right on the first pass.

**4. Reuse the existing `app/boards/stm32f4_disco.overlay` `&usart3` node as-is.**
It's already `status = "okay"` at 115200 baud with the correct pinctrl (`PD8`/`PD9`) from the board bring-up. No overlay changes needed — confirmed by cross-checking against `docs/stm32/pinouts.md`.

## Risks / Trade-offs

- **[Risk]** The `microros_transports.c` patch is fragile if the upstream file's surrounding context changes between module versions → **Mitigation**: same as the pyocd-runner patch — the module commit is pinned (Decision 1), so the file's content is fixed too; the patch script only needs to work against that one known commit.
- **[Risk]** Running micro-ROS on a separate thread introduces real concurrency (shared UART IRQ, ring buffers) → **Mitigation**: the transport layer (ring buffers + UART IRQ callback) is already designed by the module for this; no shared state is added on the application side, since motor/LED code doesn't touch `USART3`.
- **[Trade-off]** Colcon/micro-ROS build step adds real build time and a new host dependency (`catkin_pkg`, `lark-parser`, `empy`, `colcon-common-extensions` per the module's README) → accepted as a one-time `pixi` dependency addition, consistent with how this workspace already manages all other build tooling.

## Open Questions

None — the transport-blocking concern from Decision 3 is resolved structurally (separate thread) rather than deferred.
