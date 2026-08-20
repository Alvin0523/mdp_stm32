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

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
