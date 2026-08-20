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

typedef enum {
	MOTOR_A = 0,
	MOTOR_B = 1,
	MOTOR_COUNT
} motor_id_t;

/**
 * @brief Initialize GPIO and PWM Timers for AT8236 Motor Driver
 * @return 0 on success
 */
int motor_init(void);

/**
 * @brief Set motor direction and speed.
 * @param motor Motor A or B.
 * @param speed -1000 to 1000; sign selects direction.
 * @return HAL status.
 */
HAL_StatusTypeDef motor_set_speed(motor_id_t motor, int16_t speed);

/**
 * @brief Set the commanded steering angle.
 * @param angle_rad Steering angle from center, limited to +/-0.39 rad.
 * @return HAL status.
 */
HAL_StatusTypeDef motor_set_steering_angle(float angle_rad);

/**
 * @brief Set the steering servo pulse width directly.
 * @param pulse_us Pulse width in microseconds, limited to 500..2500.
 * @return HAL status.
 */
HAL_StatusTypeDef motor_set_steering_pulse_us(uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
