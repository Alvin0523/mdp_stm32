/**
 * @file ir.h
 * @brief Analog IR sensor driver on PC1 (ADC1 channel 11).
 */

#ifndef __IR_H
#define __IR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void ir_init(void);
/**
 * @brief Read analog IR sensor voltage.
 * @return Raw ADC value (0-4095, 12-bit), where higher values indicate closer objects.
 */
uint16_t ir_read_raw(void);
/**
 * @brief Read analog IR sensor voltage in volts.
 * @return Sensor output voltage (0.0 to 3.3V).
 */
float ir_read_voltage(void);
/**
 * @brief Estimate distance from the sensor output voltage.
 * @return Approximate distance in centimeters, or -1.0f below the valid
 *         voltage range.
 */
float ir_raw_to_distance_cm(uint16_t raw);

float ir_read_distance_cm(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_H */
