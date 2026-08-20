/**
 * @file button.h
 * @brief User Button PE0 GPIO EXTI Interrupt Driver
 */

#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief Initialize PE0 User Button as falling-edge EXTI0 interrupt input
 */
void button_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BUTTON_H */
