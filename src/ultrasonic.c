#include "ultrasonic.h"
#include "stm32f4xx_hal.h"

#if ULTRASONIC_ENABLED
static TIM_HandleTypeDef s_timer;
static volatile uint8_t s_waiting, s_have_rise, s_done, s_bad;
static volatile uint16_t s_rise, s_width;
static volatile uint32_t s_finished;
static uint32_t s_started, s_last_valid;
static float s_distance;
static bool s_valid, s_ready;

void ultrasonic_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_hz *= 2U;
    s_timer.Instance = TIM5;
    s_timer.Init.Prescaler = timer_hz / 1000000U - 1U;
    s_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_timer.Init.Period = 65535U;
    s_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_IC_Init(&s_timer) != HAL_OK) return;

    /* Both channels capture PA3/TI4: CH4 rising, CH3 falling.
     * Hardware timestamps remain accurate even when another ISR runs. */
    TIM_IC_InitTypeDef capture = {0};
    capture.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    capture.ICSelection = TIM_ICSELECTION_DIRECTTI;
    capture.ICPrescaler = TIM_ICPSC_DIV1;
    if (HAL_TIM_IC_ConfigChannel(&s_timer, &capture, TIM_CHANNEL_4) != HAL_OK) return;
    capture.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    capture.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    if (HAL_TIM_IC_ConfigChannel(&s_timer, &capture, TIM_CHANNEL_3) != HAL_OK) return;
    HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    if (HAL_TIM_IC_Start_IT(&s_timer, TIM_CHANNEL_4) != HAL_OK) return;
    if (HAL_TIM_IC_Start_IT(&s_timer, TIM_CHANNEL_3) != HAL_OK) return;
    s_started = HAL_GetTick();
    s_ready = true;
}

void TIM5_IRQHandler(void)
{
    uint32_t flags = TIM5->SR;
    uint16_t rise = (uint16_t)TIM5->CCR4;
    uint16_t fall = (uint16_t)TIM5->CCR3;
    TIM5->SR = ~(flags & (TIM_SR_CC3IF | TIM_SR_CC4IF |
                         TIM_SR_CC3OF | TIM_SR_CC4OF | TIM_SR_UIF));
    if (!s_waiting || s_done) return;
    if (flags & (TIM_SR_CC3OF | TIM_SR_CC4OF)) {
        s_bad = 1;
        s_done = 1;
        return;
    }
    if (flags & TIM_SR_CC4IF) {
        if (s_have_rise) s_bad = 1;
        s_rise = rise;
        s_have_rise = 1;
    }
    if ((flags & TIM_SR_CC3IF) && s_have_rise) {
        s_width = (uint16_t)(fall - s_rise);
        s_finished = HAL_GetTick();
        s_done = 1;
    }
}

void ultrasonic_update(void)
{
    if (!s_ready) return;
    uint32_t now = HAL_GetTick();
    if (s_waiting) {
        if (s_done) {
            s_waiting = 0;
            float cm = (float)s_width * 0.01715f;
            s_valid = !s_bad && s_finished - s_started < 30U &&
                      cm >= 2.0f && cm <= 400.0f;
            if (s_valid) {
                s_distance = cm;
                s_last_valid = s_finished;
            }
        } else if (now - s_started >= 30U) {
            s_waiting = 0;
            s_valid = false;
        }
    }
    if (!s_waiting && now - s_started >= 100U) {
        s_started = now;
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
            s_valid = false; /* Stuck-high echo: do not retrigger. */
            return;
        }
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        TIM5->SR = 0;
        HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);
        s_have_rise = s_done = s_bad = 0;
        s_waiting = 1;
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        uint16_t start = (uint16_t)TIM5->CNT;
        /* Only the 10us trigger blocks; echo capture is asynchronous. */
        while ((uint16_t)((uint16_t)TIM5->CNT - start) < 10U) {}
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    }
}

bool ultrasonic_get_distance_cm(float *distance_cm)
{
    if (!distance_cm || !s_valid || HAL_GetTick() - s_last_valid > 300U) return false;
    *distance_cm = s_distance;
    return true;
}
#else
void ultrasonic_init(void) {}
void ultrasonic_update(void) {}
bool ultrasonic_get_distance_cm(float *distance_cm)
{
    (void)distance_cm;
    return false;
}
#endif
