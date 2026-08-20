/**
 * @file encoder.h
 * @brief Rear Wheel Quadrature Encoder Driver (Motor A: TIM2, Motor B: TIM3)
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void encoder_init(void);

/**
 * @brief Read and reset the tick delta since the last call.
 * Ticks are signed: positive = forward rotation, negative = reverse.
 * Intended to be called once per control loop iteration (e.g. every 100ms)
 * to get a per-tick velocity sample.
 */
int32_t encoder_get_delta_a(void); /* Motor A, rear left */
int32_t encoder_get_delta_b(void); /* Motor B, rear right */

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
