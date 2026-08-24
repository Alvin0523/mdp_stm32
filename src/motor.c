/**
 * @file motor.c
 * @brief AT8236 Dual H-Bridge Motor Driver Hardware Initialization
 *
 * HOW THIS WORKS:
 * 1. Hardware Motor Pin Mapping (WHEELTEC C30D Board):
 *    - Rear Left Motor (A):  PB8 (TIM10_CH1) & PB9 (TIM11_CH1)
 *    - Rear Right Motor (B): PE5 (TIM9_CH1)  & PE6 (TIM9_CH2)
 *
 * 2. PWM Drive Principle (locked-antiphase, matching WHEELTEC's vendor
 *    reference firmware for this board):
 *    - AT8236 motor driver accepts 2 PWM inputs per channel (IN1 / IN2).
 *    - At rest, BOTH inputs sit at ~100% duty (electrical brake). To drive
 *      a direction, one input stays pinned at 100% while the other is
 *      pulled down from 100% by the commanded speed magnitude.
 *    - A simpler "one side 0%, other side duty%" scheme was tried first and
 *      left the wheels completely dead: AT8236 needs continuous switching
 *      on both legs to keep its high-side gate drive alive, so a leg held
 *      statically low never actually turns on.
 */

#include "motor.h"

/* PWM: TIM9/TIM10/TIM11 are APB2 timers; APB2 prescaler != 1 so their clock
 * is 2 x PCLK2 = 168MHz. PSC=7 -> 21MHz counter clock, ARR=999 -> ~21kHz PWM
 * (above the audible range, standard for H-bridge motor drive). */
#define MOTOR_PWM_ARR 999U

static TIM_HandleTypeDef s_htim9;
static TIM_HandleTypeDef s_htim10;
static TIM_HandleTypeDef s_htim11;

static void motor_pwm_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    htim->Instance = instance;
    htim->Init.Prescaler = 7;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = MOTOR_PWM_ARR;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(htim);
}

static void motor_pwm_channel_start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim, channel);
}

int motor_init(void)
{
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

    /* Step 4: Bring up PWM timers and start all four IN1/IN2 channels at 0% duty */
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_TIM11_CLK_ENABLE();

    motor_pwm_timer_init(&s_htim10, TIM10); /* Motor A IN1 -> PB8 */
    motor_pwm_timer_init(&s_htim11, TIM11); /* Motor A IN2 -> PB9 */
    motor_pwm_timer_init(&s_htim9, TIM9);   /* Motor B IN1/IN2 -> PE5/PE6 */

    motor_pwm_channel_start(&s_htim10, TIM_CHANNEL_1);
    motor_pwm_channel_start(&s_htim11, TIM_CHANNEL_1);
    motor_pwm_channel_start(&s_htim9, TIM_CHANNEL_1);
    motor_pwm_channel_start(&s_htim9, TIM_CHANNEL_2);

    /* Step 5: Onboard enable/e-stop switch, PD3, input with pull-up
     * (per vendor reference "Enable_Pin" init) */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    return 0;
}

uint8_t motor_estop_engaged(void)
{
    /* Assumed: switch pulls the pin low when engaged (pull-up idles high).
     * Verify against the physical switch and flip if backwards. */
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET ? 1 : 0;
}

/* Locked-antiphase drive: both IN1/IN2 sit at ~100% duty (electrical brake)
 * at rest, and one side is pulled down from 100% by the commanded magnitude
 * to produce a direction. Ported from WHEELTEC's vendor reference firmware
 * (BALANCE/balance.c Set_Pwm) for this exact board/AT8236 combination -
 * the simpler "one side at 0%, other side at duty%" scheme this replaced
 * left the wheels completely dead despite correct-looking PWM commands,
 * because AT8236 needs continuous switching on both legs to keep its
 * high-side gate drive alive; a leg statically held low never turns on. */
static void motor_drive(TIM_HandleTypeDef *htim_in1, uint32_t ch_in1,
                         TIM_HandleTypeDef *htim_in2, uint32_t ch_in2, int16_t pct)
{
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;

    int32_t delta = ((int32_t)pct * (int32_t)MOTOR_PWM_ARR) / 100;

    if (delta < 0) {
        __HAL_TIM_SET_COMPARE(htim_in1, ch_in1, MOTOR_PWM_ARR);
        __HAL_TIM_SET_COMPARE(htim_in2, ch_in2, (uint32_t)((int32_t)MOTOR_PWM_ARR + delta));
    } else {
        __HAL_TIM_SET_COMPARE(htim_in2, ch_in2, MOTOR_PWM_ARR);
        __HAL_TIM_SET_COMPARE(htim_in1, ch_in1, (uint32_t)((int32_t)MOTOR_PWM_ARR - delta));
    }
}

void motor_set_speed(int16_t left_pct, int16_t right_pct)
{
    /* Motor A (rear left): IN1 = TIM10_CH1 (PB8), IN2 = TIM11_CH1 (PB9) */
    motor_drive(&s_htim10, TIM_CHANNEL_1, &s_htim11, TIM_CHANNEL_1, left_pct);
    /* Motor B (rear right): IN1 = TIM9_CH1 (PE5), IN2 = TIM9_CH2 (PE6) */
    motor_drive(&s_htim9, TIM_CHANNEL_1, &s_htim9, TIM_CHANNEL_2, right_pct);
}

/* MG513P3012V drive motor: 1:30 gear ratio, 330 RPM max output shaft speed
 * per the official course component spec (docs/hardware.md). Wheel is
 * mounted directly on the output shaft, so this is also the max wheel
 * speed: 330 RPM * 2*pi/60 = ~34.56 rad/s. Open-loop estimate only - real
 * max speed varies with battery voltage and load. */
#define MOTOR_MAX_WHEEL_RAD_S 34.56f

static int16_t rad_s_to_pct(float rad_s)
{
    float pct = (rad_s / MOTOR_MAX_WHEEL_RAD_S) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (pct < -100.0f) pct = -100.0f;
    return (int16_t)pct;
}

void motor_set_speed_rad_s(float left_rad_s, float right_rad_s)
{
    motor_set_speed(rad_s_to_pct(left_rad_s), rad_s_to_pct(right_rad_s));
}
