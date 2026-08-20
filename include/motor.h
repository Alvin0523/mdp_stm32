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
