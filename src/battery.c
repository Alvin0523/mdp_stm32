/**
 * @file battery.c
 * @brief Battery Voltage ADC Driver Implementation (PB0 / ADC1_CH8)
 */

#include "battery.h"

static ADC_HandleTypeDef s_hadc1;

int battery_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
    s_hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    s_hadc1.Init.ScanConvMode = DISABLE;
    s_hadc1.Init.ContinuousConvMode = DISABLE;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion = 1;
    s_hadc1.Init.DMAContinuousRequests = DISABLE;
    s_hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&s_hadc1) != HAL_OK) {
        return -1;
    }

    return 0;
}

float battery_read_voltage(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_8; /* PB0 */
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(&s_hadc1, &sConfig) != HAL_OK) {
        return 0.0f;
    }

    HAL_ADC_Start(&s_hadc1);
    if (HAL_ADC_PollForConversion(&s_hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&s_hadc1);
        return 0.0f;
    }

    uint32_t raw = HAL_ADC_GetValue(&s_hadc1);
    HAL_ADC_Stop(&s_hadc1);

    /* See battery.h - divider ratio and formula sourced from WHEELTEC's
     * C30D vendor reference firmware, not re-derived from the schematic. */
    return (float)raw / 4095.0f * 3.3f * 11.0f;
}
