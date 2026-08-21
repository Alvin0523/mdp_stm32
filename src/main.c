/**
 * @file main.c
 * @brief STM32F407VET6 Main Program Entrypoint (WHEELTEC C30D Board)
 */

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "motor.h"
#include "oled.h"
#include "button.h"
#include "imu.h"
#include "servo.h"
#include "encoder.h"
#include "selftest.h"

void SystemClock_Config(void);

void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Status LED PE8 Initialization */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Initialize Peripherals: USART3, Motors, OLED, PE0 Button, ICM-20948 IMU */
    MX_USART3_UART_Init();
    motor_init();
    encoder_init();
    oled_init();
    button_init();
    servo_init();
    int imu_status = imu_init();

    printf("\r\n=======================================================\r\n");
    printf("  WHEELTEC C30D Board Bringup (PlatformIO + STM32 HAL) \r\n");
    printf("  MCU: STM32F407VET6 @ 168 MHz                          \r\n");
    printf("  IMU Sensor: ICM-20948 (I2C2 PB10/PB11) -> %s        \r\n", imu_status == 0 ? "DETECTED OK" : "NOT DETECTED");
    printf("  OLED: 0.96-inch 128x64 (PD11/12/13/14)                \r\n");
    printf("  Button: PE0 EXTI Interrupt (Page Switcher)            \r\n");
    printf("=======================================================\r\n\r\n");

    /* Hold PE0 (user button) at boot to run the drive/steer self-test
     * before entering the main loop - see docs/stm32/protocol.md. */
    selftest_run_if_requested();

    uint32_t loop_count = 0;

    /* Steering angle currently applied to the servo. Driven by the host's
     * command packet (see protocol.h) when the link is alive; centered as
     * a fail-safe when the host command goes stale (link lost). */
    float steer_deg = 0.0f;

    /* PROTOCOL_COMMAND_TIMEOUT_MS: if no valid command packet arrives from
     * the host within this window, stop driving (fail safe) rather than
     * keep coasting on the last-received setpoint. */
    #define PROTOCOL_COMMAND_TIMEOUT_MS 500U

    /* OLED pages auto-advance on this cadence when nobody's pressed the
     * button; a self-test run also resets it so the display doesn't flip
     * mid-way through reading the just-finished results. */
    #define OLED_AUTO_PAGE_MS 5000U
    uint32_t last_page_change_tick = HAL_GetTick();

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8); // Toggle PE8 LED
        loop_count++;

        /* PE0 press during normal operation triggers the self-test
         * sequence (run here, outside the ISR, since it blocks). */
        if (button_consume_selftest_request()) {
            selftest_run();
            last_page_change_tick = HAL_GetTick();
        }

        if (HAL_GetTick() - last_page_change_tick >= OLED_AUTO_PAGE_MS) {
            oled_next_page();
            last_page_change_tick = HAL_GetTick();
        }

        /* Update IMU telemetry */
        imu_update();

        /* Sample rear wheel encoders: delta since last read (this loop is
         * 10Hz, so delta*10 = ticks/sec) and running cumulative totals. */
        int32_t enc_delta_left = encoder_get_delta_a();
        int32_t enc_delta_right = encoder_get_delta_b();

        /* Apply the host's latest command, or fail safe if the serial link
         * has gone stale (no valid packet within the timeout window). */
        if (uart_command_is_stale(PROTOCOL_COMMAND_TIMEOUT_MS)) {
            motor_set_speed(0, 0);
            steer_deg = 0.0f;
        } else {
            command_packet_t cmd = g_last_command;
            motor_set_speed_rad_s(cmd.left_wheel_rad_s, cmd.right_wheel_rad_s);
            steer_deg = cmd.steer_rad * (180.0f / 3.14159265f);
        }
        servo_set_angle(steer_deg);

        /* Send telemetry to the host every loop tick. */
        telemetry_packet_t telemetry = {0};
        telemetry.enc_left = encoder_get_count_a();
        telemetry.enc_right = encoder_get_count_b();
        telemetry.steer_deg = steer_deg;
        telemetry.accel_x = g_imu_data.accel_x;
        telemetry.accel_y = g_imu_data.accel_y;
        telemetry.accel_z = g_imu_data.accel_z;
        telemetry.gyro_x = g_imu_data.gyro_x;
        telemetry.gyro_y = g_imu_data.gyro_y;
        telemetry.gyro_z = g_imu_data.gyro_z;
        telemetry.yaw_deg = g_imu_data.yaw;
        telemetry.imu_ready = g_imu_data.ready;
        telemetry.estop = motor_estop_engaged();
        telemetry.uptime_ms = HAL_GetTick();
        uart_send_telemetry(&telemetry);

        /* Render Current OLED Display Page based on g_oled_page */
        switch (g_oled_page) {
            case 0:
                /* Page 1: Primary Drive & Battery Status
                 * NOTE: battery_v is still a placeholder - no ADC driver
                 * wired up yet (PB0/ADC1_IN8 per the resource sheet), and
                 * the sense-resistor divider ratio isn't confirmed from the
                 * schematic, so a computed voltage would just be a guess.
                 * left/right are real encoder tick-rates (ticks/sec), not
                 * calibrated m/s (no confirmed wheel diameter/gear ratio). */
                oled_render_page1(12.4f, (float)(enc_delta_left * 10), (float)(enc_delta_right * 10), steer_deg, g_imu_data.yaw);
                break;

            case 1:
                /* Page 2: Distance Sensors (Ultrasonic & IR)
                 * NOTE: still fully fake - no ultrasonic/IR driver exists
                 * in this project yet. */
                oled_render_page2(18.5f, 22.0f, 24.5f);
                break;

            case 2:
                /* Page 3: Safety & Hardware Diagnostics */
                oled_render_page3(motor_estop_engaged(), encoder_get_count_a(), encoder_get_count_b(), loop_count / 2);
                break;

            case 3:
                /* Page 4: Bezel Alignment Calibration Test Page */
                oled_render_page4();
                break;

            default:
                g_oled_page = 0;
                break;
        }

        printf("[IMU Telemetry] Yaw: %+6.1f deg | GyroZ: %+6.1f deg/s | AccelX: %+5.2f m/s^2 | Page: %d\r\n",
               g_imu_data.yaw, g_imu_data.gyro_z, g_imu_data.accel_x, g_oled_page + 1);

        HAL_Delay(100); /* 10 Hz refresh rate for smooth IMU telemetry */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
