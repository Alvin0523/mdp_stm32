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

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
