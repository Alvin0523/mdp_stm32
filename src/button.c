/**
 * @file button.c
 * @brief User Button PE0 EXTI Interrupt Handler
 *
 * HOW THIS WORKS:
 * 1. PE0 Pin Setup:
 *    PE0 is connected to the onboard USER button on WHEELTEC C30D board.
 *    Configured as GPIO_MODE_IT_FALLING with Pull-Up.
 *
 * 2. EXTI0 Interrupt:
 *    When PE0 is pressed (pulled to GND), the hardware generates EXTI Line 0 Interrupt.
 *    EXTI0_IRQHandler() handles the interrupt and calls oled_next_page() to cycle OLED pages!
 */

#include "button.h"
#include "encoder.h"
#include "oled.h"

static uint32_t s_last_button_tick = 0;

void button_init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Enable EXTI Line 0 Interrupt in NVIC */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
 * @brief EXTI Line 0 Interrupt Handler for PE0 User Button
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief EXTI Falling Edge Callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    encoder_handle_exti(GPIO_Pin);

    if (GPIO_Pin == GPIO_PIN_0) {
        /* Debounce button press (200ms cooldown) */
        uint32_t now = HAL_GetTick();
        if (now - s_last_button_tick > 200) {
            s_last_button_tick = now;
            oled_next_page(); /* Cycle OLED display page */
        }
    }
}
