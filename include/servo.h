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
 * These two clamp bounds ARE WHEELTEC's own numbers, though, and those ARE
 * per-unit/per-assembly - confirmed by hardware test on this chassis
 * (2026-09-03): sign convention matches (positive=right, negative=left),
 * -28.1deg lands close to this unit's real left lock, but +18.3deg is
 * confirmed conservative - this unit's real right lock is further out.
 * PROVISIONAL pending our own fine-sweep + refit (see
 * docs/stm32/tuning.md "Servo Range & Steering Calibration") - safe to
 * drive on today, not yet accurate/using the full range. */
#define SERVO_ANGLE_MAX_RIGHT_RAD (18.3f * 3.14159265f / 180.0f) /* ~0.3195 rad - confirmed conservative, real limit is further out */
#define SERVO_ANGLE_MAX_LEFT_RAD  (28.1f * 3.14159265f / 180.0f) /* ~0.4904 rad - close to this unit's real left lock */

void servo_init(void);

/**
 * @brief Steer the front wheels.
 * @param angle_rad Desired steering angle in RADIANS (REP-103 /
 *                   ackermann_steering_controller convention), positive =
 *                   right, negative = left. Clamped to
 *                   +SERVO_ANGLE_MAX_RIGHT_RAD / -SERVO_ANGLE_MAX_LEFT_RAD -
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
 * measurement purposes (e.g. finding this unit's real right-side lock,
 * since SERVO_ANGLE_MAX_RIGHT_RAD is confirmed conservative - see above).
 */
void servo_set_angle_raw(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
