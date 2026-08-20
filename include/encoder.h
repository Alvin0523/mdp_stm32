#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define ENCODER_PULSES_PER_WHEEL_REV 330

void encoder_init(void);
void encoder_handle_exti(uint16_t gpio_pin);
int32_t encoder_get_left_count(void);
int32_t encoder_get_right_count(void);
void encoder_reset_counts(void);
float encoder_counts_to_distance_mm(int32_t count, float wheel_diameter_mm);

#endif /* __ENCODER_H */