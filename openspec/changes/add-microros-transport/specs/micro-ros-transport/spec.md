## Purpose

Gives the MCU firmware a working micro-ROS (rclc) session with a host-side micro-ROS Agent over serial, so that later capabilities (motor commands, encoder/IMU telemetry, steering) have a transport to ride on instead of having to build their own.

## ADDED Requirements

### Requirement: Serial Transport over USART3
The MCU firmware SHALL establish the micro-ROS serial transport on `USART3` at 115200 baud, matching the host-side micro-ROS Agent's serial configuration documented in `docs/stm32/pinouts.md`.

#### Scenario: Agent connects after boot
- **WHEN** a micro-ROS Agent is started on the host, connected to the MCU's `USART3` (Type-C USB-UART) after the MCU has already booted
- **THEN** the MCU's micro-ROS session SHALL complete initialization (ping/handshake with the agent) without requiring a manual MCU reset

#### Scenario: Agent absent at boot
- **WHEN** no micro-ROS Agent is connected to `USART3` when the MCU boots
- **THEN** the MCU SHALL still complete its normal boot sequence (LED blink + motor PWM init from the existing bring-up code) without blocking indefinitely on transport initialization

### Requirement: Heartbeat Publication
Once a micro-ROS session is established, the MCU firmware SHALL publish a minimal heartbeat/counter message on a fixed-period timer, observable from the host as proof the end-to-end link works.

#### Scenario: Heartbeat observable on host
- **WHEN** the micro-ROS Agent is connected and the MCU's rclc executor is spinning
- **THEN** the heartbeat topic SHALL be observable via standard ROS2 tooling (e.g. `ros2 topic echo`) on the host, with its value increasing over time

### Requirement: Coexistence with Existing Bring-up Behavior
Adding the micro-ROS transport SHALL NOT remove or block the LED blink and motor PWM initialization already verified in the board bring-up.

#### Scenario: Bring-up behavior unaffected
- **WHEN** the firmware boots, regardless of whether a micro-ROS Agent is connected
- **THEN** the status LED SHALL continue blinking at its existing ~500ms interval and motor PWM channels SHALL still initialize successfully
