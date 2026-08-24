/**
 * @file selftest.c
 * @brief Boot-time self-test sequence implementation.
 *
 * Trigger: hold PE0 (the onboard user button) down while the board resets
 * or powers on. Runs once, before the main loop, so it doesn't interfere
 * with normal operation or the host command fail-safe.
 *
 * Only PE8 is a GPIO-controllable LED on this board (confirmed against the
 * WHEELTEC resource-allocation PDF) - the LED1-LED4 silkscreen labels seen
 * on the schematic are hardwired power-rail/SWD indicators, not something
 * firmware can drive. So each test phase is signalled by a distinct PE8
 * blink count instead of separate LEDs:
 *   1 blink = forward 1 wheel revolution
 *   2 blinks = backward 1 wheel revolution
 *   3 blinks = steer left
 *   4 blinks = steer right
 *   5 blinks = done (or refused, if e-stop was engaged)
 */

#include "selftest.h"
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "oled.h"
#include <stdlib.h>

/* Ticks per wheel revolution: see docs/stm32/protocol.md /
 * mdp_hardware_bridge's kTicksPerWheelRev - sourced from WHEELTEC's vendor
 * reference firmware (EncoderMultiples * Hall_13 * HALL_30F). */
#define SELFTEST_TICKS_PER_REV 1560
#define SELFTEST_DRIVE_PCT     30
#define SELFTEST_STEER_ANGLE_DEG 20.0f
#define SELFTEST_STEER_HOLD_MS 800U
#define SELFTEST_DRIVE_TIMEOUT_MS 5000U /* safety: give up if wheels don't turn */

static void blink_pe8(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(on_ms);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(off_ms);
    }
}

static void drive_ticks(int16_t pct, int32_t target_ticks)
{
    encoder_get_delta_a();
    encoder_get_delta_b();

    /* Track and stop each wheel independently - motors are never perfectly
     * matched, so averaging left+right (the old approach) could stop one
     * wheel short of a full revolution while overshooting the other. */
    int32_t accumulated_a = 0;
    int32_t accumulated_b = 0;
    uint8_t done_a = 0;
    uint8_t done_b = 0;
    uint32_t start_tick = HAL_GetTick();

    motor_set_speed(pct, pct);
    while (!done_a || !done_b) {
        if (!done_a) {
            accumulated_a += labs(encoder_get_delta_a());
            if (accumulated_a >= target_ticks) {
                done_a = 1;
            }
        }
        if (!done_b) {
            accumulated_b += labs(encoder_get_delta_b());
            if (accumulated_b >= target_ticks) {
                done_b = 1;
            }
        }
        motor_set_speed(done_a ? 0 : pct, done_b ? 0 : pct);

        if (HAL_GetTick() - start_tick > SELFTEST_DRIVE_TIMEOUT_MS) {
            break; /* wheels not turning (off ground / stuck?) - don't hang forever */
        }
        HAL_Delay(20);
    }
    motor_set_speed(0, 0);
    HAL_Delay(300); /* let it settle before the next phase */
}

void selftest_run_if_requested(void)
{
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) != GPIO_PIN_RESET) {
        return; /* PE0 not held (pull-up idles high, pressed = low) */
    }

    selftest_run();
}

void selftest_run(void)
{
    if (motor_estop_engaged()) {
        blink_pe8(5, 100, 100); /* refused */
        return;
    }

    oled_clear();
    oled_show_string_8x16_offset(0, 0, "SELF-TEST MODE");

    /* Phase 1: forward, 1 wheel revolution */
    blink_pe8(1, 150, 150);
    oled_show_string_8x16_offset(1, 0, "1: FWD 1 REV");
    drive_ticks(SELFTEST_DRIVE_PCT, SELFTEST_TICKS_PER_REV);

    /* Phase 2: backward, 1 wheel revolution */
    blink_pe8(2, 150, 150);
    oled_show_string_8x16_offset(1, 0, "2: REV 1 REV");
    drive_ticks(-SELFTEST_DRIVE_PCT, SELFTEST_TICKS_PER_REV);

    /* Phase 3: steer left */
    blink_pe8(3, 150, 150);
    oled_show_string_8x16_offset(1, 0, "3: STEER LEFT");
    servo_set_angle(-SELFTEST_STEER_ANGLE_DEG);
    HAL_Delay(SELFTEST_STEER_HOLD_MS);

    /* Phase 4: steer right */
    blink_pe8(4, 150, 150);
    oled_show_string_8x16_offset(1, 0, "4: STEER RIGHT");
    servo_set_angle(SELFTEST_STEER_ANGLE_DEG);
    HAL_Delay(SELFTEST_STEER_HOLD_MS);

    /* Return to center and finish */
    servo_set_angle(0.0f);
    oled_show_string_8x16_offset(1, 0, "DONE            ");
    blink_pe8(5, 80, 80);
}
