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

/* THE ANGLE ARGUMENT IS A REAL WHEEL ANGLE (2026-09-11).
 *
 * This changed. It used to be an input to WHEELTEC's borrowed cubic, a unit
 * that turned out not to correspond to any real deflection on this chassis -
 * protractor measurement showed the cubic's 52.3deg was a real 35.0deg, an
 * overstatement of ~1.5x. The cubic is retired; servo.c now interpolates
 * per-side between measured endpoints, so this argument means the same thing
 * the URDF and ackermann_steering_controller mean by a steering angle.
 *
 * Consequence worth noting: mdp_bridge/serial_bridge_node.cpp passes
 * steer_rad straight through, which was previously a unit mismatch (real
 * angle fed into a differently-scaled unit, under-steering left and clipping
 * right) and is now correct without any bridge change.
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
 * Clamp bounds are protractor readings at the wheel, taken at the pulse
 * width each side was driven to via selftest.c's raw-pulse calibration
 * sweeps (see servo_set_pulse_us() below for why calibration is done in
 * microseconds, not in this angle unit):
 * - LEFT (positive): 35.0deg real at 840us. The limit here is the wheel
 *   contacting the chassis, not a servo stall.
 * - RIGHT (negative): 29.5deg real at 2400us. NOT a confirmed mechanical
 *   limit - the wheel was still tracking at 2400us, which was simply the
 *   calibration ceiling at the time. Taken as the working limit for now;
 *   this side may extend further (selftest.c PHASE 2 sweeps 2380-2500us).
 *
 * These replace the previous 48deg/24deg, which were in the retired cubic's
 * commanded unit and are NOT comparable numbers. Do not reintroduce them:
 * fed through the current mapping, 48deg real would command ~595us, far past
 * the 840us chassis-contact point.
 *
 * The 35.0 vs 29.5 gap may be inner-vs-outer rather than left-vs-right -
 * which wheel each reading came from was not recorded, and Ackermann geometry
 * steers the inner wheel harder than the outer through the shared tie-rod.
 * Center is measured, not assumed: 1490us, found by pushing the car by hand
 * at candidate pulses with the motors off. The nominal 1500us curved right.
 * See docs/stm32/tuning.md. */
#define SERVO_ANGLE_MAX_LEFT_RAD  (35.0f * 3.14159265f / 180.0f) /* ~0.6109 rad - measured at 840us, chassis contact */
#define SERVO_ANGLE_MAX_RIGHT_RAD (29.5f * 3.14159265f / 180.0f) /* ~0.5149 rad - measured at 2400us, still tracking there */

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

/**
 * @brief Set the servo pulse width DIRECTLY, in microseconds.
 *
 * CALIBRATION/SELF-TEST USE ONLY - do not call this from the normal driving
 * path (main.c).
 *
 * Bypasses everything: the cubic angle mapping, the
 * SERVO_ANGLE_MAX_LEFT/RIGHT_RAD operating clamp, and the 800-2200us
 * WHEELTEC PWM clamp. Bounded instead by servo.c's wider
 * SERVO_CAL_PULSE_MIN/MAX_US (600-2400us).
 *
 * WHY THIS EXISTS: microseconds is the only unit in this driver that means
 * something physically verifiable on its own. The "angle" taken by
 * servo_set_angle()/servo_set_angle_raw() is an input to WHEELTEC's
 * borrowed cubic, and has never been confirmed to correspond to a real
 * wheel deflection on this chassis (see docs/stm32/tuning.md's terminology
 * warning) - so calibrating against it would be calibrating against the
 * very mapping under test. Two further reasons to prefer this entry point
 * for measurement:
 *   - The angle path's 800-2200us clamp saturates at only ~-25.6deg
 *     commanded on the right side, so angle sweeps past that point cannot
 *     move the servo at all - which is what made the previously recorded
 *     "right stall at 26deg" unsafe to trust.
 *   - The cubic's slope varies ~3x across the range (-10.8us/deg at the
 *     left extreme vs. -35.2us/deg at the right), so equal angle steps are
 *     not equal physical steps, while equal microsecond steps are.
 *
 * @param pulse_us Pulse width in microseconds, clamped to
 *                 SERVO_CAL_PULSE_MIN/MAX_US. Straight-ahead is the measured
 *                 1490us (SERVO_PULSE_CENTER_US), not the nominal 1500us.
 */
void servo_set_pulse_us(uint16_t pulse_us);

/**
 * @brief Current servo pulse width in microseconds (TIM12_CH2's CCR).
 *
 * Reads back whatever was last written, by any of the three setters - so it
 * also reveals when the cubic path has been clamped (e.g. a commanded angle
 * beyond ~-25.6deg reads back as a flat 2200us).
 */
uint16_t servo_get_pulse_us(void);

/**
 * @brief Measured straight-ahead pulse width in microseconds (1490us).
 */
uint16_t servo_pulse_center_us(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
