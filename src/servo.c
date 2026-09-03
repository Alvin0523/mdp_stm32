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

#define SERVO_PULSE_CENTER_US 1500U

/* WHEELTEC's reference cubic PWM-vs-angle fit, adopted here as the real
 * conversion (not just a comparison tool) - see servo.h for the full
 * rationale/provenance (R550_C30D(2.0) balance.c, Drive_Motor()'s Akm_Car
 * branch) and why the clamp bounds (SERVO_ANGLE_MAX_LEFT/RIGHT_RAD, servo.h)
 * are provisional pending our own fine-sweep data. */
#define SERVO_WT_RATIO       636.56f
#define SERVO_WT_FIT_CENTER  1.572f
/* WHEELTEC's own tested-safe PWM bound - kept as the hard safety clamp since
 * this cubic's behavior outside their tested envelope is unverified on our
 * unit. The angle-level clamp in servo_set_angle() should keep the formula
 * well inside this anyway; this is belt-and-suspenders. */
#define SERVO_WT_PWM_MIN_US 800.0f
#define SERVO_WT_PWM_MAX_US 2200.0f

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

/* Shared conversion, used by both the clamped and raw entry points below.
 * Cubic coefficients copied verbatim from WHEELTEC's R550_C30D(2.0) chassis
 * firmware (BALANCE/balance.c):
 *
 *   Angle_Servo = -0.628*angle^3 + 1.269*angle^2 - 1.772*angle + 1.573;
 *   Servo = SERVO_INIT + (Angle_Servo - 1.572) * 636.56;
 *
 * Their Servo/SERVO_INIT are already in microseconds (their PWM clamp,
 * 800-2200us, is a plausible RC pulse range - same assumption made here by
 * using our own SERVO_PULSE_CENTER_US as SERVO_INIT). */
static void servo_write_angle_unclamped(float angle_rad)
{
    float angle_servo = -0.628f * angle_rad * angle_rad * angle_rad
                       +  1.269f * angle_rad * angle_rad
                       -  1.772f * angle_rad
                       +  1.573f;
    float pulse_us = (float)SERVO_PULSE_CENTER_US + (angle_servo - SERVO_WT_FIT_CENTER) * SERVO_WT_RATIO;

    if (pulse_us > SERVO_WT_PWM_MAX_US) pulse_us = SERVO_WT_PWM_MAX_US;
    if (pulse_us < SERVO_WT_PWM_MIN_US) pulse_us = SERVO_WT_PWM_MIN_US;

    __HAL_TIM_SET_COMPARE(&s_htim12, TIM_CHANNEL_2, (uint32_t)pulse_us);
}

void servo_set_angle(float angle_rad)
{
    if (angle_rad > SERVO_ANGLE_MAX_RIGHT_RAD) angle_rad = SERVO_ANGLE_MAX_RIGHT_RAD;
    if (angle_rad < -SERVO_ANGLE_MAX_LEFT_RAD) angle_rad = -SERVO_ANGLE_MAX_LEFT_RAD;
    servo_write_angle_unclamped(angle_rad);
}

void servo_set_angle_raw(float angle_rad)
{
    /* No SERVO_ANGLE_MAX_LEFT/RIGHT_RAD clamp - calibration/self-test use
     * only, see the warning in servo.h. Still bounded by SERVO_WT_PWM_MIN/
     * MAX_US above regardless. */
    servo_write_angle_unclamped(angle_rad);
}
