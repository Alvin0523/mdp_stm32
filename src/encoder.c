#include "encoder.h"

static volatile int32_t s_left_count;
static volatile int32_t s_right_count;

void encoder_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef config = {0};
    config.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    config.Mode = GPIO_MODE_IT_RISING;
    config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &config);

    config.Pin = GPIO_PIN_15;
    config.Mode = GPIO_MODE_INPUT;
    config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &config);

    config.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &config);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void encoder_handle_exti(uint16_t gpio_pin)
{
    if (gpio_pin == GPIO_PIN_3) {
        /* The sign may be inverted if the encoder phase wires are swapped. */
        s_left_count += HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET ? 1 : -1;
    } else if (gpio_pin == GPIO_PIN_4) {
        s_right_count += HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET ? 1 : -1;
    }
}

int32_t encoder_get_left_count(void)
{
    int32_t count;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    count = s_left_count;
    if (!primask) {
        __enable_irq();
    }
    return count;
}

int32_t encoder_get_right_count(void)
{
    int32_t count;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    count = s_right_count;
    if (!primask) {
        __enable_irq();
    }
    return count;
}

void encoder_reset_counts(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_left_count = 0;
    s_right_count = 0;
    if (!primask) {
        __enable_irq();
    }
}

float encoder_counts_to_distance_mm(int32_t count, float wheel_diameter_mm)
{
    return ((float)count / ENCODER_PULSES_PER_WHEEL_REV) *
           3.14159265358979323846f * wheel_diameter_mm;
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}