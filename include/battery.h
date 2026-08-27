/**
 * @file battery.h
 * @brief Battery Voltage ADC Driver (PB0 / ADC1_CH8)
 */

#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief Initialize PB0 as an analog input on ADC1_CH8.
 * @return 0 on success, -1 on HAL init failure.
 */
int battery_init(void);

/**
 * @brief Sample the battery voltage divider and return pack voltage.
 * Divider ratio (11.0x) and conversion formula are taken from WHEELTEC's
 * own C30D vendor reference firmware (references/WHEELTEC/.../03-ADC.../
 * bsp_adc.c: `Get_Adc(Battery_Ch)/4095.0f * 3.3f * 11.0f`) for this exact
 * board line - not independently re-derived from the schematic.
 * @return Battery pack voltage in volts.
 */
float battery_read_voltage(void);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
