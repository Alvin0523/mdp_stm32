/**
 * @file motor.h
 * @brief AT8236 Dual H-Bridge Motor Driver Interface
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief Initialize GPIO and PWM Timers for AT8236 Motor Driver
 * @return 0 on success
 */
int motor_init(void);

/**
 * @brief Drive both rear motors.
 * @param left_pct  Motor A (rear left) speed, -100..100, negative = reverse.
 * @param right_pct Motor B (rear right) speed, -100..100, negative = reverse.
 */
void motor_set_speed(int16_t left_pct, int16_t right_pct);

/**
 * @brief Drive both rear motors from a target wheel angular velocity.
 * Open-loop: converts rad/s to a PWM percent using the motor's rated max
 * speed (see MOTOR_MAX_WHEEL_RAD_S in motor.c). Actual speed at a given
 * percent varies with battery voltage/load - there is no closed-loop
 * encoder-based speed regulation yet.
 * @param left_rad_s  Motor A (rear left) target angular velocity, rad/s.
 * @param right_rad_s Motor B (rear right) target angular velocity, rad/s.
 */
void motor_set_speed_rad_s(float left_rad_s, float right_rad_s);

/**
 * @brief Read the board's onboard enable/e-stop switch (PD3).
 * @return 1 if e-stop is engaged (motors should not drive), 0 if ready.
 *         Wired as input pull-up per vendor reference; polarity (switch-to-
 *         GND = engaged) is assumed from the vendor schematic and not yet
 *         physically verified on this board - flip the comparison in
 *         motor.c if it reads backwards.
 */
uint8_t motor_estop_engaged(void);

/**
 * @brief Initialize the closed-loop wheel-speed PID control loop.
 * Must be called after motor_init() AND encoder_init(). Starts TIM7 as a
 * 100Hz periodic interrupt that reads encoder deltas and drives PWM output
 * toward the target set via motor_pid_set_target() - see
 * docs/stm32/control_loop.md for the full design rationale.
 *
 * NOT YET BENCH-TUNED: MOTOR_PID_KP/KI in motor.c are a starting-point
 * guess only, not verified on hardware. Do not trust this beyond
 * controlled bench testing until it's been tuned on the real chassis.
 */
void motor_pid_init(void);

/**
 * @brief Enable/disable the closed-loop control loop.
 * While disabled, the PID ISR forces both wheels to 0% PWM every cycle and
 * resets its integrators (avoids windup while idle, and guarantees an
 * immediate stop rather than a ramped one). Call this with 0 anywhere the
 * old code called motor_set_speed(0, 0) directly for a safety fail-safe
 * (stale command timeout, motor ON/OFF switch off) - the PID loop must
 * never be allowed to override those.
 */
void motor_pid_enable(uint8_t enable);

/**
 * @brief Set the target wheel angular velocities for the PID loop.
 * Only takes effect while the loop is enabled via motor_pid_enable(1).
 * @param left_rad_s  Motor A (rear left) target angular velocity, rad/s.
 * @param right_rad_s Motor B (rear right) target angular velocity, rad/s.
 */
void motor_pid_set_target(float left_rad_s, float right_rad_s);

/**
 * @brief Read back the PID loop's most recent encoder-measured wheel
 * angular velocities (sampled at the same 100Hz rate the PID runs at).
 * Use this instead of calling encoder_get_delta_a/b() directly once the
 * PID loop is running - those are now consumed exclusively by the PID ISR
 * (they reset the hardware tick counter on every call, so a second
 * consumer would starve the PID loop of ticks and get incomplete reads
 * itself).
 */
void motor_pid_get_measured_rad_s(float *left_rad_s, float *right_rad_s);

/**
 * @brief Pause the PID loop's TIM7 interrupt entirely (not just disable
 * its output - stops it from touching the motors OR consuming encoder
 * ticks at all). Call this before anything else needs direct, exclusive
 * access to motor_set_speed()/encoder_get_delta_a/b() - currently only
 * selftest.c, which drives motors and reads encoder deltas directly and
 * would otherwise have both stolen out from under it every 10ms.
 */
void motor_pid_pause(void);

/**
 * @brief Resume the PID loop after motor_pid_pause(). Resets integrators,
 * same as a fresh motor_pid_enable(1) - whatever accumulated before the
 * pause is stale relative to whatever drove the motors while paused.
 */
void motor_pid_resume(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
