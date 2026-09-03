/**
 * @file servo.h
 * @brief Ackermann Front-Wheel Steering Servo Driver (TIM12_CH2 / PB15)
 */

#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Angles below are in RADIANS, not degrees - matching REP-103 (ROS's
 * standard units convention: SI units, radians for angles), the same unit
 * ackermann_steering_controller/URDF/the CommandPacket's steer_rad field
 * already use. */

/* PWM-vs-angle conversion (servo.c) uses WHEELTEC's reference cubic fit
 * (R550_C30D(2.0) balance.c, Drive_Motor()'s Akm_Car branch) - adopted
 * because the underlying nonlinearity is a property of the physical
 * rigid-linkage Ackermann mechanism itself (confirmed against WHEELTEC's
 * own kinematics manual - no such linkage holds a constant ratio across its
 * range), not something specific to their sample unit.
 *
 * Sign convention CONFIRMED ON PHYSICAL HARDWARE (2026-09-03, observer
 * standing behind the robot, facing the same way it drives): POSITIVE
 * commanded angle = wheel turns LEFT, NEGATIVE = RIGHT. This matches
 * REP-103 (positive yaw/turn = CCW = left) directly - this servo's
 * physical wiring/linkage already agrees with the ROS convention, so no
 * sign translation is needed anywhere in the stack (see
 * mdp_bridge/serial_bridge_node.cpp's onJointCommand()/onTelemetry(),
 * which pass steer_rad through unmodified for exactly this reason).
 *
 * Clamp bounds are this unit's own hardware-measured values (selftest.c's
 * fine sweeps, servo_set_angle_raw() past the operating clamp,
 * watching/listening for stall or knuckle binding). The sweeps that found
 * these were run before the sign convention above was confirmed, so the
 * magnitudes below are correct but were originally found under swapped
 * LEFT/RIGHT labels - see docs/stm32/tuning.md for the full story:
 * - LEFT (positive): real limit is chassis contact, not a servo stall (no
 *   stall found up to 55deg) - 50deg confirmed clean, 55deg confirmed the
 *   wheel touching the chassis. Exact contact-onset point between 50-55deg
 *   not pinned down further (didn't want more contact just to find it
 *   exactly) - clamped 2deg back from the last confirmed-clean point
 *   (50deg).
 * - RIGHT (negative): real stall confirmed at 26deg. Clamped 2deg back
 *   from that, not exactly at it, to avoid repeatedly stalling the servo. */
#define SERVO_ANGLE_MAX_LEFT_RAD  (48.0f * 3.14159265f / 180.0f) /* ~0.8378 rad - 2deg back from 50deg confirmed-clean; real contact point is somewhere in 50-55deg */
#define SERVO_ANGLE_MAX_RIGHT_RAD (24.0f * 3.14159265f / 180.0f) /* ~0.4189 rad - real stall at 26deg, 2deg safety margin */

void servo_init(void);

/**
 * @brief Steer the front wheels.
 * @param angle_rad Desired steering angle in RADIANS (REP-103 /
 *                   ackermann_steering_controller convention), positive =
 *                   left, negative = right - confirmed on physical
 *                   hardware, see the sign-convention note above. Clamped
 *                   to +SERVO_ANGLE_MAX_LEFT_RAD / -SERVO_ANGLE_MAX_RIGHT_RAD -
 *                   found on physical hardware, not placeholders.
 */
void servo_set_angle(float angle_rad);

/**
 * @brief Steer the front wheels WITHOUT the SERVO_ANGLE_MAX_LEFT/RIGHT_RAD
 * operating clamp - only bounded by WHEELTEC's tested-safe PWM range
 * (servo.c). CALIBRATION/SELF-TEST USE ONLY - do not call this from the
 * normal driving path (main.c). The operating clamp in servo_set_angle()
 * exists to protect the linkage during real driving; this function exists
 * so calibration tooling (selftest.c's servo sweep) can probe past it for
 * measurement purposes.
 */
void servo_set_angle_raw(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
