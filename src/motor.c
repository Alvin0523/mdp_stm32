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
#include "encoder.h"

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

/* ===========================================================================
 * Closed-loop wheel-speed PID (docs/stm32/control_loop.md)
 *
 * Runs from TIM7's 100Hz update interrupt, decoupled from the 10Hz main
 * loop so telemetry/OLED/IMU work never limits the control loop's rate.
 *
 * Hybrid feedforward + incremental-PI trim: rad_s_to_pct() supplies an
 * immediate open-loop estimate (so the wheel responds right away instead
 * of ramping up from a cold PI accumulator), and the PI term trims that
 * estimate toward the actual encoder-measured speed to correct for battery
 * voltage sag, load, and the two motors not being identical (see the
 * per-wheel independent tracking note in selftest.c for why they aren't).
 *
 * GAINS ARE UNTUNED. MOTOR_PID_KP/KI below are a starting-point guess
 * (percent PWM per rad/s of error), ported in *shape* only from WHEELTEC's
 * vendor firmware's Incremental_PI_A/B (BALANCE/control.c) - their
 * Velocity_KP/KI=300 are tuned against a completely different PWM/tick
 * scale and are not transferable as numbers. Bench-tune before trusting
 * this beyond controlled testing: raise KP until a step target is tracked
 * with acceptable overshoot, then add just enough KI to kill steady-state
 * error.
 * =========================================================================== */
#define MOTOR_PID_HZ         100U
#define MOTOR_PID_KP         4.0f
#define MOTOR_PID_KI         0.5f
#define MOTOR_TICKS_PER_REV  1560  /* see docs/stm32/protocol.md */
#define MOTOR_RAD_PER_TICK   (2.0f * 3.14159265f / (float)MOTOR_TICKS_PER_REV)

static TIM_HandleTypeDef s_htim7;

static volatile uint8_t s_pid_enabled = 0;
static volatile float s_target_left_rad_s = 0.0f;
static volatile float s_target_right_rad_s = 0.0f;
static volatile float s_measured_left_rad_s = 0.0f;
static volatile float s_measured_right_rad_s = 0.0f;

/* Only touched from the TIM7 ISR - no locking needed. */
static float s_pid_integral_left = 0.0f;
static float s_pid_integral_right = 0.0f;
static float s_pid_last_err_left = 0.0f;
static float s_pid_last_err_right = 0.0f;

void motor_pid_init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* TIM7 is APB1; APB1 prescaler != 1 so TIM7CLK = 2 x PCLK1 = 84MHz.
     * PSC=8399 -> 10kHz counter clock, ARR=99 -> 100Hz update interrupt. */
    s_htim7.Instance = TIM7;
    s_htim7.Init.Prescaler = 8399;
    s_htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim7.Init.Period = 99;
    s_htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&s_htim7);

    /* Highest preemption priority (lowest number) of this project's app-
     * level interrupts - the motor PID's fixed 10ms cadence is both time-
     * critical (the control math assumes a consistent period) and safety-
     * critical (this ISR is what actually enforces "PD3 off -> motors off"
     * every cycle, see motor_pid_enable()). Must preempt USART3 RX and the
     * user button, not the other way around. */
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_TIM_Base_Start_IT(&s_htim7);
}

void motor_pid_enable(uint8_t enable)
{
    if (enable && !s_pid_enabled) {
        /* Rising edge: reset integrators so we don't inherit a stale
         * accumulated PWM value left over from before the loop was last
         * disabled. */
        s_pid_integral_left = 0.0f;
        s_pid_integral_right = 0.0f;
        s_pid_last_err_left = 0.0f;
        s_pid_last_err_right = 0.0f;
    }
    s_pid_enabled = enable;
}

void motor_pid_set_target(float left_rad_s, float right_rad_s)
{
    s_target_left_rad_s = left_rad_s;
    s_target_right_rad_s = right_rad_s;
}

void motor_pid_get_measured_rad_s(float *left_rad_s, float *right_rad_s)
{
    *left_rad_s = s_measured_left_rad_s;
    *right_rad_s = s_measured_right_rad_s;
}

void motor_pid_pause(void)
{
    HAL_TIM_Base_Stop_IT(&s_htim7);
}

void motor_pid_resume(void)
{
    s_pid_integral_left = 0.0f;
    s_pid_integral_right = 0.0f;
    s_pid_last_err_left = 0.0f;
    s_pid_last_err_right = 0.0f;
    HAL_TIM_Base_Start_IT(&s_htim7);
}

static int16_t pid_clamp_pct(float pct)
{
    if (pct > 100.0f) pct = 100.0f;
    if (pct < -100.0f) pct = -100.0f;
    return (int16_t)pct;
}

/* Basic anti-windup: keep the integral term itself within the PWM range so
 * it can't accumulate far beyond what's usable and cause a lag on
 * desaturation once the target becomes reachable again. */
static float pid_clamp_integral(float val)
{
    if (val > 100.0f) val = 100.0f;
    if (val < -100.0f) val = -100.0f;
    return val;
}

void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&s_htim7);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7) {
        return;
    }

    /* Sole consumer of encoder_get_delta_a/b() - see motor_pid_get_measured_rad_s()'s
     * doc comment in motor.h for why nothing else may call these directly
     * once the PID loop is running. */
    int32_t delta_a = encoder_get_delta_a();
    int32_t delta_b = encoder_get_delta_b();
    s_measured_left_rad_s = (float)delta_a * MOTOR_RAD_PER_TICK * (float)MOTOR_PID_HZ;
    s_measured_right_rad_s = (float)delta_b * MOTOR_RAD_PER_TICK * (float)MOTOR_PID_HZ;

    if (!s_pid_enabled) {
        motor_set_speed(0, 0);
        return;
    }

    float target_left = s_target_left_rad_s;
    float target_right = s_target_right_rad_s;

    float err_left = target_left - s_measured_left_rad_s;
    s_pid_integral_left = pid_clamp_integral(
        s_pid_integral_left + MOTOR_PID_KP * (err_left - s_pid_last_err_left) + MOTOR_PID_KI * err_left);
    s_pid_last_err_left = err_left;

    float err_right = target_right - s_measured_right_rad_s;
    s_pid_integral_right = pid_clamp_integral(
        s_pid_integral_right + MOTOR_PID_KP * (err_right - s_pid_last_err_right) + MOTOR_PID_KI * err_right);
    s_pid_last_err_right = err_right;

    float pwm_left = (float)rad_s_to_pct(target_left) + s_pid_integral_left;
    float pwm_right = (float)rad_s_to_pct(target_right) + s_pid_integral_right;

    motor_set_speed(pid_clamp_pct(pwm_left), pid_clamp_pct(pwm_right));
}
