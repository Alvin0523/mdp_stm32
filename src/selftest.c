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
 * blink count instead of separate LEDs.
 *
 * servo_set_angle() now uses WHEELTEC's reference cubic PWM-vs-angle fit
 * (servo.c) - see servo.h for the clamp values (provisional, pending our
 * own fine-sweep + refit) and docs/stm32/tuning.md for the decision record.
 *
 * CURRENTLY (drive phases 1/2 disabled - right-side-only servo measurement
 * pass, see selftest_run()/servo_sweep()):
 *   1 blink = servo to LEFT max (SERVO_ANGLE_MAX_LEFT_RAD), hold, then
 *             RIGHT max (SERVO_ANGLE_MAX_RIGHT_RAD), hold
 *   2 blinks = done
 *
 * NORMAL sequence (restore by uncommenting phases 1/2 in selftest_run()):
 *   1 blink = forward 1 wheel revolution
 *   2 blinks = backward 1 wheel revolution
 *   3 blinks = servo to LEFT max, hold, then RIGHT max, hold
 *   4 blinks = done
 *
 * Refusal (motor switch OFF) is a distinct standalone 5-blink pattern with
 * a different on/off timing (100ms/100ms vs. the phases' 150ms/150ms) -
 * not part of either numbered sequence above.
 */

#include "selftest.h"
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "oled.h"
#include <stdlib.h>
#include <stdio.h>

/* Ticks per wheel revolution: see docs/stm32/protocol.md /
 * mdp_hardware_bridge's kTicksPerWheelRev - sourced from WHEELTEC's vendor
 * reference firmware (EncoderMultiples * Hall_13 * HALL_30F). */
#define SELFTEST_TICKS_PER_REV 1560
#define SELFTEST_DRIVE_PCT     30
#define SELFTEST_DRIVE_TIMEOUT_MS 5000U /* safety: give up if wheels don't turn */

/* Servo sweep timing - see docs/stm32/control_loop.md ("Servo range").
 * Confirmed final per-side limits live in servo.h
 * (SERVO_ANGLE_MAX_LEFT_RAD/SERVO_ANGLE_MAX_RIGHT_RAD), not duplicated here.
 * Step size is defined in degrees purely because that's a more intuitive
 * unit for a human picking a sweep resolution - converted to radians once,
 * immediately, since servo_set_angle()/servo_set_angle_raw() take radians
 * (matching REP-103 / the rest of the stack, see servo.h). */
#define SELFTEST_SERVO_SWEEP_STEP_DEG   1.0f
#define SELFTEST_SERVO_SWEEP_STEP_RAD   (SELFTEST_SERVO_SWEEP_STEP_DEG * 3.14159265f / 180.0f)
#define SELFTEST_SERVO_RIGHT_FINE_START_DEG 35.0f
#define SELFTEST_SERVO_RIGHT_FINE_END_DEG   55.0f
#define SELFTEST_SERVO_RIGHT_FINE_START_RAD (SELFTEST_SERVO_RIGHT_FINE_START_DEG * 3.14159265f / 180.0f)
#define SELFTEST_SERVO_RIGHT_FINE_END_RAD   (SELFTEST_SERVO_RIGHT_FINE_END_DEG * 3.14159265f / 180.0f)
#define SELFTEST_SERVO_SWEEP_HOLD_MS    1500U

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

/* Sweeps one range (both bounds in RADIANS, matching servo_set_angle()'s
 * unit) at SELFTEST_SERVO_SWEEP_STEP_RAD resolution. Direction is
 * auto-detected from start_rad vs end_rad - ascends if end_rad > start_rad,
 * descends otherwise. use_raw selects servo_set_angle_raw() (bypasses the
 * SERVO_ANGLE_MAX_LEFT/RIGHT_RAD operating clamp - calibration/measurement
 * only) vs. the normal clamped servo_set_angle() (safe for routine
 * self-test use). The OLED readout converts back to degrees purely for
 * human readability while watching/measuring - the underlying command and
 * comparison against a protractor is still whatever that degree number
 * represents, this is a display-only conversion. */
static void servo_sweep_range(float start_rad, float end_rad, const char *label, uint8_t use_raw)
{
    char buf[24];
    float step = (end_rad >= start_rad) ? SELFTEST_SERVO_SWEEP_STEP_RAD : -SELFTEST_SERVO_SWEEP_STEP_RAD;

    oled_show_string_8x16_offset(1, 0, label);
    HAL_Delay(500);

    for (float angle = start_rad;
         (step > 0.0f) ? (angle <= end_rad + 0.0002f) : (angle >= end_rad - 0.0002f);
         angle += step) {
        if (use_raw) {
            servo_set_angle_raw(angle);
        } else {
            servo_set_angle(angle);
        }
        float angle_deg = angle * (180.0f / 3.14159265f); /* OLED display only */
        snprintf(buf, sizeof(buf), "SWEEP: %+5.1f deg", (double)angle_deg);
        oled_show_string_8x16_offset(1, 0, buf);
        HAL_Delay(SELFTEST_SERVO_SWEEP_HOLD_MS);
    }
}

/* Verification hold: commands the servo straight to the firmware-clamped
 * left max, holds it for SELFTEST_SERVO_HOLD_MEASURE_MS (long enough to
 * measure with a protractor), then straight to the right max, holds again -
 * no sweeping through intermediate steps, just the two operating extremes
 * (SERVO_ANGLE_MAX_LEFT_RAD/RIGHT_RAD, servo.h) for direct re-measurement
 * of the real steering angle at exactly what the firmware actually clamps
 * to. See docs/stm32/control_loop.md ("Servo range"). */
#define SELFTEST_SERVO_HOLD_MEASURE_MS 8000U
static void servo_sweep(void)
{
    /* Direct jumps (no intermediate stepping): center -> left max -> center
     * -> right max -> center, holding each long enough to measure with a
     * protractor. Bounds come from servo.h (SERVO_ANGLE_MAX_LEFT/RIGHT_RAD)
     * - currently WHEELTEC's provisional clamp values, see servo.h/
     * docs/stm32/tuning.md for status. */
    oled_show_string_8x16_offset(1, 0, "LEFT MAX - HOLD");
    servo_set_angle(-SERVO_ANGLE_MAX_LEFT_RAD);
    HAL_Delay(SELFTEST_SERVO_HOLD_MEASURE_MS);

    oled_show_string_8x16_offset(1, 0, "CENTER");
    servo_set_angle(0.0f);
    HAL_Delay(500);

    oled_show_string_8x16_offset(1, 0, "RIGHT MAX - HOLD");
    servo_set_angle(SERVO_ANGLE_MAX_RIGHT_RAD);
    HAL_Delay(SELFTEST_SERVO_HOLD_MEASURE_MS);

    servo_set_angle(0.0f);
    HAL_Delay(300);
}

/* Right-side fine sweep past the current (confirmed conservative)
 * SERVO_ANGLE_MAX_RIGHT_RAD clamp, to find this unit's real right-side
 * lock point - see docs/stm32/tuning.md. Not currently wired into
 * selftest_run() - kept for the next calibration pass. Uses
 * servo_set_angle_raw() to bypass the operating clamp. */
static void servo_sweep_right_fine(void)
{
    servo_sweep_range(SELFTEST_SERVO_RIGHT_FINE_START_RAD, SELFTEST_SERVO_RIGHT_FINE_END_RAD,
                       "RIGHT FINE 35-55", 1);
    servo_set_angle(0.0f);
    HAL_Delay(300);
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

    /* This whole sequence drives motors and reads encoder deltas directly
     * - the PID loop's TIM7 ISR (motor.c, docs/stm32/control_loop.md) does
     * both of those every 10ms in the background and would otherwise fight
     * us for the motors and steal our encoder ticks. */
    motor_pid_pause();

    oled_clear();
    oled_show_string_8x16_offset(0, 0, "SELF-TEST MODE");

    /* Phases 1/2 (forward/backward 1 wheel revolution) disabled - right-
     * side-only servo measurement pass. Commented out, not deleted.
     *
     * blink_pe8(1, 150, 150);
     * oled_show_string_8x16_offset(1, 0, "1: FWD 1 REV");
     * drive_ticks(SELFTEST_DRIVE_PCT, SELFTEST_TICKS_PER_REV);
     *
     * blink_pe8(2, 150, 150);
     * oled_show_string_8x16_offset(1, 0, "2: REV 1 REV");
     * drive_ticks(-SELFTEST_DRIVE_PCT, SELFTEST_TICKS_PER_REV);
     */

    /* Phase 1 (renumbered): servo sweep, both extremes */
    blink_pe8(1, 150, 150);
    oled_show_string_8x16_offset(1, 0, "1: SERVO SWEEP");
    servo_sweep();

    /* Return to center and finish */
    servo_set_angle(0.0f);
    oled_show_string_8x16_offset(1, 0, "DONE            ");
    blink_pe8(2, 80, 80);

    motor_pid_resume();
}
