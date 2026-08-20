/**
 * @file encoder.c
 * @brief Rear Wheel Quadrature Encoder Driver Implementation
 *
 * Hardware Pin Mapping (per "STM32F407VET6(C30D-V2.1)主板资源分配说明.pdf"):
 *   Motor A (rear left)  encoder: PA15, PB3 -> TIM2_CH1/CH2 (AF1)
 *   Motor B (rear right) encoder: PB4,  PB5 -> TIM3_CH1/CH2 (AF2)
 *
 * TIM2/TIM3 are configured in hardware encoder-interface mode, which
 * quadrature-decodes the A/B channel pulses in hardware (direction and
 * count both handled by the timer peripheral, no ISR needed).
 */

#include "encoder.h"

static TIM_HandleTypeDef s_htim2;
static TIM_HandleTypeDef s_htim3;
static int32_t s_total_a = 0;
static int32_t s_total_b = 0;

static void encoder_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    htim->Instance = instance;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    TIM_Encoder_InitTypeDef sConfig = {0};
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    HAL_TIM_Encoder_Init(htim, &sConfig);

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void encoder_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    /* Motor A encoder: PA15 (TIM2_CH1), PB3 (TIM2_CH2) */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Motor B encoder: PB4 (TIM3_CH1), PB5 (TIM3_CH2) */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    encoder_timer_init(&s_htim2, TIM2);
    encoder_timer_init(&s_htim3, TIM3);
}

int32_t encoder_get_delta_a(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&s_htim2);
    __HAL_TIM_SET_COUNTER(&s_htim2, 0);
    s_total_a += count;
    return count;
}

int32_t encoder_get_delta_b(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&s_htim3);
    __HAL_TIM_SET_COUNTER(&s_htim3, 0);
    s_total_b += count;
    return count;
}

int32_t encoder_get_count_a(void)
{
    return s_total_a;
}

int32_t encoder_get_count_b(void)
{
    return s_total_b;
}
