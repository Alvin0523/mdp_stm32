/**
 * @file ir_sensor.h
 * @brief IR1 on PC2 (ADC1 channel 12), IR2 on PC1 (ADC1 channel 11).
 */

#ifndef __IR_H
#define __IR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Configure both inputs. Blocking reads are main-loop-only: ADC1 is shared
 * with battery.c. Both sensors use the Sharp GP2Y0A21YK 10-80 cm curve. */
void ir_sensor_init(void);
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
 * @return Approximate distance clamped to 10-80 centimeters. Raw zero
 *         (including a failed ADC read) returns 80 cm.
 */
float ir_raw_to_distance_cm(uint16_t raw);

float ir_read_distance_cm(void);

/* Second sensor; same units and error behavior as the original API above. */
uint16_t ir_sensor2_read_raw(void);
float ir_sensor2_read_voltage(void);
float ir_sensor2_read_distance_cm(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_SENSOR_H */
