/**
 * @file stm32f4xx_hal_conf.h
 * @brief STM32 HAL Module Configuration Header
 * 
 * HOW THIS WORKS:
 * In ST Microelectronics HAL library, this header file acts as a peripheral
 * switchboard. Defining HAL_<MODULE>_MODULE_ENABLED includes the necessary
 * HAL driver headers for GPIO, Timers, UARTs, Clock (RCC), and Power (PWR).
 */

#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* 1. Module Selection: Enable only required peripherals for fast build      */
/* -------------------------------------------------------------------------- */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED     /* Analog-to-Digital Converter driver     */
#define HAL_CORTEX_MODULE_ENABLED  /* ARM Cortex-M4 core & SysTick functions */
#define HAL_DMA_MODULE_ENABLED     /* Direct Memory Access driver            */
#define HAL_FLASH_MODULE_ENABLED   /* Flash memory controller                */
#define HAL_GPIO_MODULE_ENABLED    /* General Purpose I/O driver             */
#define HAL_PWR_MODULE_ENABLED     /* Power management controller            */
#define HAL_RCC_MODULE_ENABLED     /* Reset & Clock Control driver           */
#define HAL_TIM_MODULE_ENABLED     /* Timer & PWM output driver              */
#define HAL_UART_MODULE_ENABLED    /* Universal Asynchronous Rx/Tx (Serial)  */

/* -------------------------------------------------------------------------- */
/* 2. Oscillator Frequencies for WHEELTEC C30D Board                         */
/* -------------------------------------------------------------------------- */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000U) /* 8 MHz External Crystal Oscillator */
#endif

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)   /* Timeout in ms for HSE to stabilize */
#endif

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /* 16 MHz Internal RC Oscillator */
#endif

/* System Configuration Constants */
#define  VDD_VALUE                    ((uint32_t)3300U) /* Operating voltage 3.3V in mV */
#define  TICK_INT_PRIORITY            ((uint32_t)0F)    /* SysTick interrupt priority  */
#define  PREFETCH_ENABLE              1U                /* Enable Flash prefetch buffer */
#define  INSTRUCTION_CACHE_ENABLE     1U                /* Enable CPU instruction cache */
#define  DATA_CACHE_ENABLE            1U                /* Enable CPU data cache        */

/* -------------------------------------------------------------------------- */
/* 3. Include Core Peripheral Drivers                                        */
/* -------------------------------------------------------------------------- */
#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f4xx_hal_pwr.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f4xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f4xx_hal_uart.h"
#endif

#ifdef  USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t *file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */
