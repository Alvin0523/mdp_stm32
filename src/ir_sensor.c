/**
 * @file ir_sensor.c
 * @brief Analog IR sensor driver implementation for PC1 (ADC1 channel 11).
 *
 * Analog IR sensor connected to PC1. Reads raw 12-bit ADC values where higher
 * values typically indicate closer object proximity.
 */

#include "ir_sensor.h"
#include <math.h>

static ADC_HandleTypeDef s_hadc1_ir;

void ir_sensor_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    s_hadc1_ir.Instance = ADC1;
    s_hadc1_ir.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
    s_hadc1_ir.Init.Resolution = ADC_RESOLUTION_12B;
    s_hadc1_ir.Init.ScanConvMode = DISABLE;
    s_hadc1_ir.Init.ContinuousConvMode = DISABLE;
    s_hadc1_ir.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1_ir.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_hadc1_ir.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    s_hadc1_ir.Init.NbrOfConversion = 1;
    s_hadc1_ir.Init.DMAContinuousRequests = DISABLE;
    s_hadc1_ir.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    HAL_ADC_Init(&s_hadc1_ir);
}

uint16_t ir_sensor_read_raw(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    HAL_StatusTypeDef status;

    status = HAL_ADC_ConfigChannel(&s_hadc1_ir, &sConfig);
    if (status != HAL_OK) {
        printf("ADC config failed: %d\r\n", status);
        return 0U;
    }

    status = HAL_ADC_Start(&s_hadc1_ir);
    if (status != HAL_OK) {
        printf("ADC start failed: %d\r\n", status);
        return 0U;
    }

    status = HAL_ADC_PollForConversion(&s_hadc1_ir, 10U);
    if (status != HAL_OK) {
        printf("ADC poll failed: %d, error=0x%08lx\r\n",
               status,
               (unsigned long)HAL_ADC_GetError(&s_hadc1_ir));

        HAL_ADC_Stop(&s_hadc1_ir);
        return 0U;
    }

    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&s_hadc1_ir);
    HAL_ADC_Stop(&s_hadc1_ir);

    printf("ADC raw: %u\r\n", raw);
    return raw;
}

float ir_sensor_read_voltage(void)
{
    uint16_t raw = ir_sensor_read_raw();
    return (float)raw / 4095.0f * 3.3f;
}

// id clean this up later -songli
#define IR_DISTANCE_MIN_CM     10.0f
#define IR_DISTANCE_MAX_CM     80.0f
#define IR_DISTANCE_OFFSET_CM   0.0f

float ir_sensor_raw_to_distance_cm(uint16_t raw)
{
    if (raw == 0U) {
        return IR_DISTANCE_MAX_CM;
    }

    float normalized = (float)raw / 4095.0f;
    float distance = 6.3028f / powf(normalized, 1.226f);

    distance -= IR_DISTANCE_OFFSET_CM;

    if (distance > IR_DISTANCE_MAX_CM) {
        distance = IR_DISTANCE_MAX_CM;
    }

    if (distance < IR_DISTANCE_MIN_CM) {
        distance = IR_DISTANCE_MIN_CM;
    }

    return distance;
}

float ir_sensor_read_distance_cm(void)
{
    return ir_sensor_raw_to_distance_cm(ir_sensor_read_raw());
}