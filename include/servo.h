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

void servo_init(void);

/**
 * @brief Steer the front wheels.
 * @param angle_deg Desired steering angle in degrees, positive = right, negative = left.
 *                   Range is clamped to +/- SERVO_ANGLE_MAX_DEG (see servo.c) which maps
 *                   to a 1000-2000us pulse width; SERVO_ANGLE_MAX_DEG is a placeholder
 *                   estimate and should be tuned to the servo linkage's actual mechanical
 *                   lock-to-lock limit on this chassis.
 */
void servo_set_angle(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
