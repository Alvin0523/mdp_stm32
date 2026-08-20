/**
 * @file oled.h
 * @brief SSD1306 0.96" OLED Driver (Bit-Bang GPIO PD11/12/13/14)
 */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern uint8_t g_oled_page;

void oled_init(void);
void oled_clear(void);
void oled_show_string_8x16_offset(uint8_t row, uint8_t x_offset, const char *str);
void oled_render_page1(float battery_v, float left_speed, float right_speed, float steer_deg, float yaw_deg);
void oled_render_page2(float us_cm, float ir_left_cm, float ir_right_cm);
void oled_render_page3(uint8_t estop_state, int32_t enc_left, int32_t enc_right, uint32_t uptime_sec);
void oled_render_page4(void);
void oled_next_page(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
