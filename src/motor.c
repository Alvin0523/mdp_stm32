/**
 * @file motor.c
 * @brief AT8236 Dual H-Bridge Motor Driver Hardware Initialization
 *
 * HOW THIS WORKS:
 * 1. Hardware Motor Pin Mapping (WHEELTEC C30D Board):
 *    - Rear Left Motor (A):  PB8 (TIM10_CH1) & PB9 (TIM11_CH1)
 *    - Rear Right Motor (B): PE5 (TIM9_CH1)  & PE6 (TIM9_CH2)
 *
 * 2. PWM Drive Principle:
 *    - AT8236 motor driver accepts 2 PWM inputs per channel (IN1 / IN2).
 *    - PWM Duty Cycle (0-100%) controls motor speed via pulse-width modulation.
 *    - Direction:
 *      * IN1 = PWM, IN2 = 0: Forward
 *      * IN1 = 0, IN2 = PWM: Reverse
 *      * IN1 = 0, IN2 = 0: Coast / Stop
 */

#include "motor.h"

#define MOTOR_TIMER_PRESCALER 167U
#define MOTOR_TIMER_PERIOD 999U
#define STEERING_MAX_ANGLE_RAD 0.39f
#define STEERING_CENTER_PULSE_US 1500U
#define STEERING_RANGE_PULSE_US 1000U

static TIM_HandleTypeDef htim9;
static TIM_HandleTypeDef htim10;
static TIM_HandleTypeDef htim11;
static TIM_HandleTypeDef htim3;

static HAL_StatusTypeDef motor_timer_init(TIM_HandleTypeDef *timer,
                                          TIM_TypeDef *instance,
                                          uint32_t prescaler,
                                          uint32_t period)
{
    timer->Instance = instance;
    timer->Init.Prescaler = prescaler;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = period;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    return HAL_TIM_PWM_Init(timer);
}

static HAL_StatusTypeDef motor_pwm_channel_init(TIM_HandleTypeDef *timer,
                                                uint32_t channel)
{
    TIM_OC_InitTypeDef config = {0};

    config.OCMode = TIM_OCMODE_PWM1;
    config.Pulse = 0;
    config.OCPolarity = TIM_OCPOLARITY_HIGH;
    config.OCFastMode = TIM_OCFAST_DISABLE;

    return HAL_TIM_PWM_ConfigChannel(timer, &config, channel);
}

int motor_init(void)
{
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_TIM11_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Step 1: Enable GPIO Clocks for Port B and Port E */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Step 2: Configure Motor A (Rear Left) Pins PB8 & PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;            /* Alternate Function Push-Pull */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;       /* Connect PB8/PB9 to Timer AF3 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Step 3: Configure Motor B (Rear Right) Pins PE5 & PE6 */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;        /* Connect PE5/PE6 to Timer AF3 */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Steering servo: PB1 = TIM3_CH4, 50 Hz with a 1 us timer tick. */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    if (motor_timer_init(&htim10, TIM10, MOTOR_TIMER_PRESCALER,
                        MOTOR_TIMER_PERIOD) != HAL_OK ||
        motor_timer_init(&htim11, TIM11, MOTOR_TIMER_PRESCALER,
                         MOTOR_TIMER_PERIOD) != HAL_OK ||
        motor_timer_init(&htim9, TIM9, MOTOR_TIMER_PRESCALER,
                         MOTOR_TIMER_PERIOD) != HAL_OK ||
        motor_timer_init(&htim3, TIM3, 83U, 19999U) != HAL_OK) {
        return -1;
    }

    if (motor_pwm_channel_init(&htim10, TIM_CHANNEL_1) != HAL_OK ||
        motor_pwm_channel_init(&htim11, TIM_CHANNEL_1) != HAL_OK ||
        motor_pwm_channel_init(&htim9, TIM_CHANNEL_1) != HAL_OK ||
        motor_pwm_channel_init(&htim9, TIM_CHANNEL_2) != HAL_OK ||
        motor_pwm_channel_init(&htim3, TIM_CHANNEL_4) != HAL_OK) {
        return -1;
    }

    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
        return -1;
    }

    /* Center the steering servo: 1.5 ms pulse at 50 Hz. */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, STEERING_CENTER_PULSE_US);

    return 0;
}

HAL_StatusTypeDef motor_set_speed(motor_id_t motor, int16_t speed)
{
    if (motor >= MOTOR_COUNT) {
        return HAL_ERROR;
    }

    if (speed > 1000) {
        speed = 1000;
    } else if (speed < -1000) {
        speed = -1000;
    }

    uint32_t pulse = (uint32_t)((speed < 0 ? -speed : speed) *
                                MOTOR_TIMER_PERIOD / 1000);

    if (motor == MOTOR_A) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, speed > 0 ? pulse : 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, speed < 0 ? pulse : 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed > 0 ? pulse : 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, speed < 0 ? pulse : 0);
    }

    return HAL_OK;
}

HAL_StatusTypeDef motor_set_steering_angle(float angle_rad)
{
    if (angle_rad < -STEERING_MAX_ANGLE_RAD) {
        angle_rad = -STEERING_MAX_ANGLE_RAD;
    } else if (angle_rad > STEERING_MAX_ANGLE_RAD) {
        angle_rad = STEERING_MAX_ANGLE_RAD;
    }

    uint16_t pulse_us = (uint16_t)(STEERING_CENTER_PULSE_US +
                                   (angle_rad / STEERING_MAX_ANGLE_RAD) *
                                       STEERING_RANGE_PULSE_US);
    return motor_set_steering_pulse_us(pulse_us);
}

HAL_StatusTypeDef motor_set_steering_pulse_us(uint16_t pulse_us)
{
    if (pulse_us < 500U) {
        pulse_us = 500U;
    } else if (pulse_us > 2500U) {
        pulse_us = 2500U;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_us);

    return HAL_OK;
}
