/**
 * @file servo.c
 * @brief Ackermann Front-Wheel Steering Servo Driver Implementation
 *
 * Hardware: WHEELTEC C30D board, servo header PB15 (TIM12_CH2, AF9).
 * Confirmed via "STM32F407VET6(C30D-V2.1)主板资源分配说明.pdf":
 *   PC6/PC7/PC8/PC9/PB14/PB15 -> TIM8/TIM12, "阿克曼小车(舵机)使用PB15引脚"
 *
 * TIM12 is on APB1; since APB1 prescaler != 1, TIM12CLK = 2 x PCLK1 = 84MHz.
 * PSC=83 -> 1MHz (1us) counter tick. ARR=19999 -> 20ms period (50Hz), the
 * standard RC servo refresh rate. CCR is therefore the pulse width in
 * microseconds directly.
 */

#include "servo.h"

/* Standard RC servo pulse range; adjust SERVO_ANGLE_MAX_DEG to match the
 * chassis's actual mechanical steering limit if the linkage can't reach
 * full lock at these pulse widths. */
#define SERVO_PULSE_MIN_US    1000U
#define SERVO_PULSE_CENTER_US 1500U
#define SERVO_PULSE_MAX_US    2000U
#define SERVO_ANGLE_MAX_DEG   30.0f

static TIM_HandleTypeDef s_htim12;

void servo_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    s_htim12.Instance = TIM12;
    s_htim12.Init.Prescaler = 83;
    s_htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim12.Init.Period = 19999;
    s_htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&s_htim12);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_PULSE_CENTER_US;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&s_htim12, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&s_htim12, TIM_CHANNEL_2);
}

/**
 * @brief Steer the front wheels.
 * @param angle_deg Desired steering angle, positive = right, negative = left.
 *                   Clamped to +/- SERVO_ANGLE_MAX_DEG.
 */
void servo_set_angle(float angle_deg)
{
    if (angle_deg > SERVO_ANGLE_MAX_DEG) angle_deg = SERVO_ANGLE_MAX_DEG;
    if (angle_deg < -SERVO_ANGLE_MAX_DEG) angle_deg = -SERVO_ANGLE_MAX_DEG;

    float span_us = (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 2.0f;
    uint32_t pulse_us = (uint32_t)(SERVO_PULSE_CENTER_US + (angle_deg / SERVO_ANGLE_MAX_DEG) * span_us);

    __HAL_TIM_SET_COMPARE(&s_htim12, TIM_CHANNEL_2, pulse_us);
}
