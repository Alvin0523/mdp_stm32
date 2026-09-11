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
#include "battery.h"
#include "ir_sensor.h"

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

    /* Initialize Peripherals: USART1 (debug console), USART3 (binary
     * protocol to RPi), Motors, OLED, PE0 Button, ICM-20948 IMU */
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    motor_init();
    encoder_init();
    oled_init();
    button_init();
    servo_init();
    battery_init();
    ir_sensor_init();
    motor_pid_init(); /* must come after motor_init()/encoder_init() above */
    int imu_status = imu_init();

    printf("\r\n=======================================================\r\n");
    printf("  WHEELTEC C30D Board Bringup (PlatformIO + STM32 HAL) \r\n");
    printf("  MCU: STM32F407VET6 @ 168 MHz                          \r\n");
    printf("  IMU Sensor: ICM-20948 (I2C2 PB10/PB11) -> %s        \r\n", imu_status == 0 ? "DETECTED OK" : "NOT DETECTED");
    printf("  OLED: 0.96-inch 128x64 (PD11/12/13/14)                \r\n");
    printf("  Button: PE0 (Self-Test if motor switch ON, else Page Flip)\r\n");
    printf("=======================================================\r\n\r\n");

    /* Hold PE0 (user button) at boot to run the drive/steer self-test
     * before entering the main loop - see docs/stm32/protocol.md. */
    selftest_run_if_requested();

    /* Steering angle currently applied to the servo, in RADIANS - matches
     * servo_set_angle()'s unit (servo.h) and the command packet's steer_rad
     * field (protocol.h) directly, no internal degree conversion anymore.
     * Driven by the host's command packet when the link is alive; centered
     * as a fail-safe when the host command goes stale (link lost). Persists
     * across loop iterations since the OLED (slow tier) displays the same
     * value the command/safety tier (fast tier) last set. */
    float steer_rad = 0.0f;

    /* Cached encoder-measured wheel speed and battery voltage, likewise
     * persisted across iterations so the slow tier can display whatever
     * the fast tier (rad/s) or the slow tier itself (battery) last read,
     * without re-reading a fast-tier-owned resource out of turn. */
    float meas_left_rad_s = 0.0f, meas_right_rad_s = 0.0f;
    float battery_v = 0.0f;
    uint16_t ir_raw = 0U;
    float ir_voltage = 0.0f;
    float ir_distance_cm = -1.0f;

    /* PROTOCOL_COMMAND_TIMEOUT_MS: if no valid command packet arrives from
     * the host within this window, stop driving (fail safe) rather than
     * keep coasting on the last-received setpoint. */
    #define PROTOCOL_COMMAND_TIMEOUT_MS 500U

    /* Two independent rate tiers (see docs/stm32/control_loop.md, "Main
     * loop timing allocation") instead of one shared HAL_Delay():
     * - FAST_PERIOD_MS (100Hz): command/safety gating, IMU read, telemetry
     *   TX - matches the motor PID's own 100Hz encoder sampling 1:1, so
     *   nothing the PID measures is throttled down before it reaches the
     *   Pi. Only viable because uart_send_telemetry() is interrupt-driven
     *   (usart.c) - at this rate a blocking transmit would eat ~47% of the
     *   period on its own. Also shrinks motor-switch-off reaction time
     *   from ~100ms (pre-tiering) to ~20ms.
     * - SLOW_PERIOD_MS (~5Hz): OLED render, debug printf, battery ADC,
     *   heartbeat LED - all either human-facing (no benefit running
     *   faster than the eye can use) or slow-changing (battery voltage).
     * The loop itself free-runs (no blocking delay) and each tier fires
     * on its own schedule. */
    #define FAST_PERIOD_MS 10U
    #define SLOW_PERIOD_MS 200U
    uint32_t last_fast_tick = HAL_GetTick();
    uint32_t last_slow_tick = HAL_GetTick();

    while (1)
    {
        /* PE0 press during normal operation is dual-purpose, gated on PD3:
         * - PD3 ready (not engaged): self-test can actually drive the
         *   motors, so run it (outside the ISR, since it blocks).
         * - PD3 engaged: self-test would just refuse anyway (see
         *   selftest_run()'s own e-stop check), so cycle the OLED page
         *   instead - gives the button a use in that state too.
         * Cheap enough to check every raw loop pass, not tiered. No
         * auto-advance timer anymore - pages only change on button press. */
        if (button_consume_selftest_request()) {
            if (!motor_estop_engaged()) {
                selftest_run();
            } else {
                oled_next_page();
            }
        }

        uint32_t now = HAL_GetTick();

        if (now - last_fast_tick >= FAST_PERIOD_MS) {
            last_fast_tick = now;

            /* Update IMU telemetry */
            imu_update();

            /* Wheel speed is closed-loop, driven by motor_pid's own 100Hz
             * TIM7 ISR (see docs/stm32/control_loop.md) - this tier only
             * sets the target/enable state below, it never calls
             * motor_set_speed()/encoder_get_delta_a/b() directly. Those
             * encoder reads are owned exclusively by that ISR (they reset
             * the hardware tick counter on every call), so read back its
             * measured rad/s instead for display. */
            motor_pid_get_measured_rad_s(&meas_left_rad_s, &meas_right_rad_s);

            /* Apply the host's latest command, or fail safe (motors off,
             * steering centered) if EITHER the serial link has gone stale
             * (no valid packet within the timeout window) OR the PD3 motor
             * ON/OFF switch is OFF - the switch overrides any command from
             * the host regardless of link freshness, same as it already
             * does for self-test. */
            if (uart_command_is_stale(PROTOCOL_COMMAND_TIMEOUT_MS) || motor_estop_engaged()) {
                motor_pid_enable(0);
                steer_rad = 0.0f;
            } else {
                command_packet_t cmd = g_last_command;
                motor_pid_enable(1);
                motor_pid_set_target(cmd.left_wheel_rad_s, cmd.right_wheel_rad_s);
                steer_rad = cmd.steer_rad; /* no conversion - servo_set_angle() takes radians directly now */
            }
            servo_set_angle(steer_rad);

            telemetry_packet_t telemetry = {0};
            telemetry.enc_left = encoder_get_count_a();
            telemetry.enc_right = encoder_get_count_b();
            /* protocol.h's telemetry_packet_t.steer_deg is degrees (wire
             * format, unchanged - mdp_hardware_bridge on the Pi side still
             * expects degrees here and converts back to radians itself) -
             * derive it from the radian value we actually work in now. */
            telemetry.steer_deg = steer_rad * (180.0f / 3.14159265f);
            telemetry.accel_x = g_imu_data.accel_x;
            telemetry.accel_y = g_imu_data.accel_y;
            telemetry.accel_z = g_imu_data.accel_z;
            telemetry.gyro_x = g_imu_data.gyro_x;
            telemetry.gyro_y = g_imu_data.gyro_y;
            telemetry.gyro_z = g_imu_data.gyro_z;
            telemetry.yaw_deg = g_imu_data.yaw;
            telemetry.imu_ready = g_imu_data.ready;
            telemetry.estop = motor_estop_engaged();
            telemetry.battery_v = battery_v; /* last slow-tier reading, see below */
            telemetry.ir_raw = ir_raw;
            telemetry.ir_voltage = ir_voltage;
            telemetry.ir_distance_cm = ir_distance_cm;
            telemetry.uptime_ms = HAL_GetTick();
            uart_send_telemetry(&telemetry);
        }

        if (now - last_slow_tick >= SLOW_PERIOD_MS) {
            last_slow_tick = now;

            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8); /* heartbeat LED */

            battery_v = battery_read_voltage();

            ir_raw = ir_sensor_read_raw();
            ir_voltage = (float)ir_raw * (3.3f / 4095.0f);
            ir_distance_cm = ir_sensor_raw_to_distance_cm(ir_raw);

            /* Render Current OLED Display Page based on g_oled_page */
            switch (g_oled_page) {
                case 0:
                    /* Page 1: Primary Drive & Battery Status
                     * left/right are real encoder tick-rates (ticks/sec,
                     * back-converted from the PID loop's measured rad/s),
                     * not calibrated m/s (no confirmed wheel diameter/gear
                     * ratio). */
                    oled_render_page1(battery_v,
                                       meas_left_rad_s * (1560.0f / (2.0f * 3.14159265f)),
                                       meas_right_rad_s * (1560.0f / (2.0f * 3.14159265f)),
                                       steer_rad * (180.0f / 3.14159265f), /* oled.h's page1 is degrees, display-only */
                                       g_imu_data.yaw);
                    break;

                case 1:
                    /* Page 2: Distance Sensors (Ultrasonic & IR). */
                    oled_render_page2(18.5f, ir_raw, ir_voltage, ir_distance_cm);
                    break;

                case 2:
                    /* Page 3: Safety & Hardware Diagnostics */
                    oled_render_page3(motor_estop_engaged(), encoder_get_count_a(),
                                       encoder_get_count_b(), HAL_GetTick() / 1000U);
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
        }
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
