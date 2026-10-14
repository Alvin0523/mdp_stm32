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
 * with battery.c. Both sensors are Sharp GP2Y0A21YK (10-80 cm). */
void ir_init(void);
/**
 * @brief Read IR1 (PC2), the mean of IR_OVERSAMPLE conversions (ir.c).
 * @return Raw ADC value (0-4095, 12-bit), higher = closer; 0 if the ADC read failed.
 *         The Pi turns it into cm with the sensor's calibrated curve (bridges.yaml).
 */
uint16_t ir_read_raw(void);
/* Second IR (PC1): same as ir_read_raw(). */
uint16_t ir_sensor2_read_raw(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_SENSOR_H */
