/**
 * @file ir_sensor.c
 * @brief Analog IR sensors: IR1 on PC2/ADC1_CH12, IR2 on PC1/ADC1_CH11.
 *
 * Two analog IR sensors connected to PC2 and PC1. Reads raw 12-bit ADC values where higher
 * values typically indicate closer object proximity.
 */

#include "ir.h"
#include <math.h>
#include <stdio.h>

static ADC_HandleTypeDef s_hadc1_ir;

void ir_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_1;
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

/* ADC1 is shared with battery.c. Read sequentially from the main loop;
 * select the channel on every read and stop before the next user. */
static uint16_t ir_read_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
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

    /* Was unconditional on every read (5Hz, forever) - drowned out anything
     * else on USART1. The distance is visible via ROS telemetry and the
     * OLED now, so this isn't needed for normal operation. Re-enable only
     * for standalone bench bring-up with no Pi/ROS connected. */
    /* printf("ADC raw: %u\r\n", raw); */
    return raw;
}

uint16_t ir_read_raw(void)
{
    return ir_read_channel(ADC_CHANNEL_12);
}

uint16_t ir_sensor2_read_raw(void)
{
    return ir_read_channel(ADC_CHANNEL_11);
}

float ir_sensor2_read_voltage(void)
{
    return (float)ir_sensor2_read_raw() / 4095.0f * 3.3f;
}

float ir_read_voltage(void)
{
    uint16_t raw = ir_read_raw();
    return (float)raw / 4095.0f * 3.3f;
}

/* Each sensor's own curve, distance = A / (raw/4095)^B, from `pixi run calib ir`
 * on the car (2026-10-07, after the IR mounts were fixed; IR1 within 1.0 cm and
 * IR2 within 0.4 cm at 13-25 cm). The SAME numbers as mdp_ros
 * src/mdp_bringup/config/bridges.yaml (ir1_curve / ir2_curve), which the Pi
 * applies to the raw values it gets in the telemetry - so the OLED shows what
 * /ir and /ir2 show. Recalibrated: change both places. */
#define IR1_CURVE_A            6.23f
#define IR1_CURVE_B            1.210f
#define IR2_CURVE_A            6.53f
#define IR2_CURVE_B            1.157f
#define IR_DISTANCE_MIN_CM     4.0f    /* as bridges.yaml ir_min_cm (Sharp is unreliable below ~7 cm) */
#define IR_DISTANCE_MAX_CM     80.0f

static float ir_curve_cm(uint16_t raw, float a, float b)
{
    if (raw == 0U) {
        return IR_DISTANCE_MAX_CM;
    }
    float distance = a / powf((float)raw / 4095.0f, b);
    if (distance > IR_DISTANCE_MAX_CM) {
        distance = IR_DISTANCE_MAX_CM;
    }
    if (distance < IR_DISTANCE_MIN_CM) {
        distance = IR_DISTANCE_MIN_CM;
    }
    return distance;
}

float ir_raw_to_distance_cm(uint16_t raw)
{
    return ir_curve_cm(raw, IR1_CURVE_A, IR1_CURVE_B);
}

float ir2_raw_to_distance_cm(uint16_t raw)
{
    return ir_curve_cm(raw, IR2_CURVE_A, IR2_CURVE_B);
}

float ir_sensor2_read_distance_cm(void)
{
    return ir2_raw_to_distance_cm(ir_sensor2_read_raw());
}

float ir_read_distance_cm(void)
{
    return ir_raw_to_distance_cm(ir_read_raw());
}
