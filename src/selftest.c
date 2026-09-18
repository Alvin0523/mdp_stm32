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
 * servo_set_angle() uses WHEELTEC's reference cubic PWM-vs-angle fit
 * (servo.c). Sign convention CONFIRMED ON PHYSICAL HARDWARE (2026-09-03):
 * positive = left, negative = right (matches REP-103). Operating clamps
 * (servo.h) are this unit's own hardware-measured real limits: left
 * 50-55deg chassis contact (2deg margin -> 48deg), right 26deg stall
 * (2deg margin -> 24deg) - see docs/stm32/tuning.md for the full decision
 * record, including how the LEFT/RIGHT labels got corrected after the
 * sign convention was pinned down (the sweeps were run before that).
 *
 * MINI COMPONENT TEST (current default - steering calibration is done, see
 * servo.h; this is the routine "are the basics still working" sequence,
 * not a calibration tool). Each phase advances automatically once its own
 * motion finishes - no button press needed mid-sequence:
 *   1 blink  = FWD 1 REV  - drives both rear wheels forward one wheel
 *              revolution open-loop (PID paused), tracking each wheel's own
 *              encoder independently. Confirms both motors AND both
 *              encoders respond.
 *   2 blinks = REV 1 REV  - same, in reverse.
 *   3 blinks = SERVO SWEEP - steering to the calibrated left max
 *              (SERVO_ANGLE_MAX_LEFT_RAD), hold, back to center, then the
 *              calibrated right max (SERVO_ANGLE_MAX_RIGHT_RAD), hold, back
 *              to center. Confirms the servo can actually reach both
 *              calibrated limits without stalling.
 *   4 blinks = STRAIGHT LINE - drives forward through the real PID loop
 *              (motor_pid, not open-loop) with steering centered at the
 *              measured 1490us. End-to-end check: motors, encoders, PID,
 *              and steering center all working together.
 *   DONE     = 2 quick blinks, motors off, steering re-centered.
 *
 * Refusal (motor switch OFF) is a distinct standalone 5-blink pattern with
 * a different on/off timing (100ms/100ms vs. the phases' 150ms/150ms).
 *
 * The steering-calibration tooling used to find the numbers above (raw-pulse
 * sweeps, center trim, protractor measurement phases) is still in this file
 * but disabled in selftest_run() - re-enable only to re-calibrate, not for
 * routine testing. See the RAW-PULSE CALIBRATION block below for that.
 */

#include "selftest.h"
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "servo.h"
#include "button.h"
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
/* Ranges start just under the CURRENT operating clamp (servo.h) and sweep
 * past it - the point is to find where the real mechanical lock/stall is,
 * which is confirmed further out than the current provisional clamp on at
 * least the right side. Still safe regardless of how far these go: the
 * cubic mapping's own hard PWM clamp (servo.c, SERVO_WT_PWM_MIN/MAX_US)
 * caps the actual pulse sent to WHEELTEC's own tested-safe bound no matter
 * what angle is requested here. */
/* LEFT = positive, RIGHT = negative - confirmed on physical hardware
 * (servo.h). These ranges/values are unchanged from the original sweeps;
 * only the LEFT/RIGHT labels were corrected after that confirmation (the
 * sweeps were run before the sign convention was pinned down - see
 * docs/stm32/tuning.md). */
#define SELFTEST_SERVO_LEFT_FINE_START_DEG  40.0f
#define SELFTEST_SERVO_LEFT_FINE_END_DEG    55.0f
#define SELFTEST_SERVO_LEFT_FINE_START_RAD  (SELFTEST_SERVO_LEFT_FINE_START_DEG * 3.14159265f / 180.0f)
#define SELFTEST_SERVO_LEFT_FINE_END_RAD    (SELFTEST_SERVO_LEFT_FINE_END_DEG * 3.14159265f / 180.0f)
#define SELFTEST_SERVO_RIGHT_FINE_START_DEG (-22.0f)
#define SELFTEST_SERVO_RIGHT_FINE_END_DEG   (-40.0f)
#define SELFTEST_SERVO_RIGHT_FINE_START_RAD (SELFTEST_SERVO_RIGHT_FINE_START_DEG * 3.14159265f / 180.0f)
#define SELFTEST_SERVO_RIGHT_FINE_END_RAD   (SELFTEST_SERVO_RIGHT_FINE_END_DEG * 3.14159265f / 180.0f)
#define SELFTEST_SERVO_SWEEP_HOLD_MS    1500U

/* Straight-line PID test: target wheel speed and run duration. 5 rad/s is
 * roughly 15% of MOTOR_MAX_WHEEL_RAD_S (34.56) - slow enough to stay
 * controllable indoors, fast enough that the PID is actually working. 4s is
 * long enough to see a curve develop; the 1-wheel-revolution drive phases
 * are far too short for that. */
#define SELFTEST_STRAIGHT_RAD_S 5.0f
#define SELFTEST_STRAIGHT_MS    4000U

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
     * - this unit's own hardware-measured real limits, see servo.h/
     * docs/stm32/tuning.md. Positive = left, negative = right - confirmed
     * on physical hardware, see servo.h's sign-convention note. */
    oled_show_string_8x16_offset(1, 0, "LEFT MAX - HOLD");
    servo_set_angle(SERVO_ANGLE_MAX_LEFT_RAD);
    HAL_Delay(SELFTEST_SERVO_HOLD_MEASURE_MS);

    oled_show_string_8x16_offset(1, 0, "CENTER");
    servo_set_angle(0.0f);
    HAL_Delay(500);

    oled_show_string_8x16_offset(1, 0, "RIGHT MAX - HOLD");
    servo_set_angle(-SERVO_ANGLE_MAX_RIGHT_RAD);
    HAL_Delay(SELFTEST_SERVO_HOLD_MEASURE_MS);

    servo_set_angle(0.0f);
    HAL_Delay(300);
}

/* ------------------------------------------------------------------------
 * RAW-PULSE CALIBRATION (microseconds, button-advanced)
 *
 * Preferred tool for finding the real mechanical steering limits and for
 * protractor work, in place of the angle-based sweeps below. Rationale in
 * full at servo.h's servo_set_pulse_us() docs; the short version:
 *
 *   - Microseconds is the only unit here that is physically meaningful on
 *     its own. The angle sweeps' unit is an input to WHEELTEC's borrowed
 *     cubic, which is the very mapping being validated - measuring against
 *     it would be circular.
 *   - The angle path clamps at 800-2200us, and the cubic reaches 2200us at
 *     only ~-25.6deg commanded. Angle sweeps past that point send an
 *     identical pulse and cannot move the servo, so the previously recorded
 *     "right stall at 26deg" tells us nothing about the linkage. This path
 *     uses the wider SERVO_CAL_PULSE_MIN/MAX_US (600-2400us) instead.
 *   - The cubic's slope varies ~3x across the range, so equal angle steps
 *     are unequal physical steps. Equal microsecond steps are uniform.
 *
 * Advance is one step per PE0 press, so measuring is self-paced rather than
 * racing a fixed delay. If no press arrives within
 * SELFTEST_CAL_STEP_TIMEOUT_MS the sweep aborts and recenters - deliberate,
 * so a servo that has hit a stall is never left sitting loaded and drawing
 * current while unattended.
 * --------------------------------------------------------------------- */

/* Per-step wait before a limit/measure sweep gives up and recenters. Those
 * sweeps may be sitting against a stalled servo, so giving up promptly is the
 * safe default. The center trim does not use this - it holds position
 * indefinitely instead, see servo_cal_center_trim(). */
#define SELFTEST_CAL_STEP_TIMEOUT_MS 30000U

/* --- Center trim: find the pulse width that actually rolls straight ---
 *
 * 1500us is the nominal mechanical center but has never been verified.
 * Direction flipped from the original assumption here: this unit's hand-push
 * test at 1500us curves LEFT (not right, as first assumed) - decreasing the
 * pulse moves further left (confirmed sign convention: shorter pulse = left,
 * per the measured endpoints: 840us at +35deg left, 2400us at -29.5deg
 * right), so correcting a left curve means sweeping UPWARD instead.
 *
 * 5us steps are ~0.19deg of real wheel angle each on this side (right-side
 * resolution is 0.038deg/us, per SERVO_US_PER_RAD_RIGHT), fine enough to
 * land on straight without being tedious. If the whole range is exhausted
 * while it still curves left, raise SELFTEST_TRIM_END_US further.
 *
 * Each step is HELD indefinitely - no timeout, no auto-recenter - so the car
 * can be pushed repeatedly at one setting and power cut at the good one. */
#define SELFTEST_TRIM_START_US 1500U
#define SELFTEST_TRIM_END_US   1570U
#define SELFTEST_TRIM_STEP_US     5U

/* --- Phase A: mechanical limit finding, one side at a time ---
 *
 * Each side starts at a pulse already known to be safe and steps OUTWARD in
 * small increments, so the first thing that ever goes wrong is the limit
 * being looked for. Note both sides move AWAY from 1500us but in opposite
 * numerical directions: the cubic's slope is negative, so left is a
 * shortening pulse and right a lengthening one.
 *
 * LEFT: 887us was the old operating clamp and 866us was observed moving
 * cleanly, with wheel-to-chassis contact appearing by ~809us. Starting at
 * 950us leaves a margin inside all of that. The expected outcome here is
 * chassis contact, not a stall.
 *
 * RIGHT: genuinely unknown territory. 2144us was the old clamp, and
 * everything beyond ~2200us was previously unreachable because the angle
 * path clamped there - so the old "stall" was the firmware, not the
 * hardware. Sweeping to 2400us (SERVO_CAL_PULSE_MAX_US) covers the rest of
 * the conventional RC envelope. A real stall may well appear partway; stop
 * there and record it. */
#define SELFTEST_CAL_LEFT_START_US   950U
#define SELFTEST_CAL_LEFT_END_US     780U
#define SELFTEST_CAL_LEFT_STEP_US     10U

/* Right range narrowed to the unexplored top end: 2400us was already reached
 * with the wheel still tracking (29.5deg real), so the interesting region is
 * above it. Starts at 2380us to give one known-good step for reference
 * before entering new territory, and ends at SERVO_CAL_PULSE_MAX_US.
 *
 * WHAT TO LOOK FOR HERE - the question is specifically whether the wheel
 * still MOVES as the pulse grows:
 *   - angle keeps increasing  -> still not at any limit
 *   - angle stops changing, servo silent -> servo's internal travel limit,
 *     a real and acceptable answer for this side
 *   - audible buzz/whine, no motion -> stall against a hard stop; back off
 *     immediately, do not hold it there
 *   - visible binding/scraping -> linkage or chassis contact
 * Compare the angle at 2380 against 2500. If they are the same, the pulse
 * has stopped buying travel and this side's real maximum is whatever that
 * angle is. */
#define SELFTEST_CAL_RIGHT_START_US 2380U
#define SELFTEST_CAL_RIGHT_END_US   2500U
#define SELFTEST_CAL_RIGHT_STEP_US    10U

/* --- Phase B: protractor measurement across the full range ---
 *
 * Run only AFTER phase A, and edit these bounds to sit just inside the
 * limits it found - the defaults are placeholders based on the old clamps,
 * not measured values. Coarse steps on purpose: the goal is a dozen or so
 * (pulse, real angle) pairs to fit a mapping to, and manual protractor
 * precision is far coarser than the 1us the timer can resolve, so finer
 * steps would only add reading noise. The range is swept in one continuous
 * pass through center rather than per-side, which keeps every reading on
 * the same protractor setup. */
#define SELFTEST_CAL_MEASURE_START_US  900U
#define SELFTEST_CAL_MEASURE_END_US   2200U
#define SELFTEST_CAL_MEASURE_STEP_US   100U

/* Blocks until PE0 is pressed, or until timeout_ms elapses since entry.
 * Returns 1 on press, 0 on timeout.
 *
 * Waits for release before arming, so holding the button down advances one
 * step rather than running away through the whole sweep. Also drains
 * button.c's EXTI0 self-test request flag: every press here would otherwise
 * still be latched when this returns, and main.c's loop would immediately
 * re-run the entire self-test on seeing it. */
static uint8_t wait_for_button_press(uint32_t timeout_ms)
{
    const uint32_t start_tick = HAL_GetTick();

    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - start_tick > timeout_ms) {
            (void)button_consume_selftest_request();
            return 0;
        }
        HAL_Delay(10);
    }

    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) != GPIO_PIN_RESET) {
        if (HAL_GetTick() - start_tick > timeout_ms) {
            (void)button_consume_selftest_request();
            return 0;
        }
        HAL_Delay(10);
    }

    HAL_Delay(40); /* contact debounce */
    (void)button_consume_selftest_request();
    return 1;
}

/* Steps the pulse width from start_us to end_us in step_mag_us increments,
 * one step per PE0 press. Direction is auto-detected from start vs. end.
 * Returns 1 if the full range was covered, 0 if aborted on timeout.
 *
 * The OLED shows the absolute pulse width and its signed offset from the
 * 1500us center - the offset is what makes left/right symmetry directly
 * comparable, which the commanded-angle numbers obscure (48deg vs 24deg
 * look wildly asymmetric while being only 613us vs 644us off center).
 *
 * WATCH AND LISTEN at every step. Two different limits present differently:
 *   - servo stall: audible buzz/whine, no visible movement
 *   - linkage/chassis contact: knuckle or wheel visibly binding or
 *     scraping while the servo is still trying
 * Stop at the LAST step that moved cleanly - that step is the limit, not
 * the one that stalled. */
static uint8_t servo_pulse_sweep(uint16_t start_us, uint16_t end_us,
                                 uint16_t step_mag_us, const char *label,
                                 uint32_t step_timeout_ms)
{
    char buf[20];
    const int32_t center_us = (int32_t)servo_pulse_center_us();
    const int32_t step = (end_us >= start_us) ? (int32_t)step_mag_us
                                             : -(int32_t)step_mag_us;

    oled_clear();
    oled_show_string_8x16_offset(0, 0, label);
    oled_show_string_8x16_offset(3, 0, "PE0=next step");

    for (int32_t pulse = (int32_t)start_us;
         (step > 0) ? (pulse <= (int32_t)end_us) : (pulse >= (int32_t)end_us);
         pulse += step) {
        servo_set_pulse_us((uint16_t)pulse);

        /* Read back rather than echoing the request, so a value silenced by
         * SERVO_CAL_PULSE_MIN/MAX_US is visible instead of misleading. */
        const int32_t actual = (int32_t)servo_get_pulse_us();

        snprintf(buf, sizeof(buf), "PULSE %4ld us", (long)actual);
        oled_show_string_8x16_offset(1, 0, buf);
        snprintf(buf, sizeof(buf), "CTR   %+4ld us", (long)(actual - center_us));
        oled_show_string_8x16_offset(2, 0, buf);

        if (!wait_for_button_press(step_timeout_ms)) {
            servo_set_pulse_us((uint16_t)center_us);
            oled_show_string_8x16_offset(1, 0, "TIMEOUT-CENTER  ");
            oled_show_string_8x16_offset(2, 0, "                ");
            HAL_Delay(1000);
            return 0;
        }
    }

    servo_set_pulse_us((uint16_t)center_us);
    oled_show_string_8x16_offset(1, 0, "RANGE DONE      ");
    oled_show_string_8x16_offset(2, 0, "                ");
    HAL_Delay(800);
    return 1;
}

static void servo_cal_left_limit(void)
{
    (void)servo_pulse_sweep(SELFTEST_CAL_LEFT_START_US, SELFTEST_CAL_LEFT_END_US,
                            SELFTEST_CAL_LEFT_STEP_US, "LEFT LIMIT",
                            SELFTEST_CAL_STEP_TIMEOUT_MS);
}

static void servo_cal_right_limit(void)
{
    (void)servo_pulse_sweep(SELFTEST_CAL_RIGHT_START_US, SELFTEST_CAL_RIGHT_END_US,
                            SELFTEST_CAL_RIGHT_STEP_US, "RIGHT LIMIT",
                            SELFTEST_CAL_STEP_TIMEOUT_MS);
}

static void servo_cal_measure(void)
{
    (void)servo_pulse_sweep(SELFTEST_CAL_MEASURE_START_US, SELFTEST_CAL_MEASURE_END_US,
                            SELFTEST_CAL_MEASURE_STEP_US, "MEASURE",
                            SELFTEST_CAL_STEP_TIMEOUT_MS);
}

/* Center trim. Steps the pulse DOWN by SELFTEST_TRIM_STEP_US on each PE0
 * press, starting at 1500us, and HOLDS whatever it is on indefinitely.
 *
 * Deliberately does NOT reuse servo_pulse_sweep(): that recenters both on
 * per-step timeout and at the end of its range, which would throw away the
 * very position being hunted for. Here there is no timeout at all - the servo
 * simply stays put until the next press, so the car can be pushed by hand as
 * many times as needed at each step, and power can be cut at the step that
 * rolls straight to leave the wheels sitting there.
 *
 * Motors stay off for this: a hand push removes the motors and PID from the
 * picture entirely, so anything left over is steering geometry.
 *
 * Read the held pulse off the OLED. It becomes SERVO_PULSE_CENTER_US in
 * servo.c, and both per-side slope constants recompute from it automatically
 * since they are derived from the center-to-lock spans.
 *
 * NOTE: this never returns on its own. Once the range bottoms out it holds
 * the last value forever, waiting on a button that no longer does anything -
 * reset the board to get out. That is the intended way to use it (cut power
 * at the step you want), but it does mean the robot is parked in calibration
 * mode until reset. */
static void servo_cal_center_trim(void)
{
    char buf[20];
    int32_t pulse = (int32_t)SELFTEST_TRIM_START_US;

    oled_clear();
    oled_show_string_8x16_offset(0, 0, "CENTER TRIM");
    oled_show_string_8x16_offset(3, 0, "PE0=+5us PUSH");

    for (;;) {
        servo_set_pulse_us((uint16_t)pulse);

        snprintf(buf, sizeof(buf), "PULSE %4u us", (unsigned)servo_get_pulse_us());
        oled_show_string_8x16_offset(1, 0, buf);
        snprintf(buf, sizeof(buf), "CTR   %+4ld us",
                 (long)((int32_t)servo_get_pulse_us() - (int32_t)servo_pulse_center_us()));
        oled_show_string_8x16_offset(2, 0, buf);

        /* No timeout - hold this position until the button is pressed. */
        (void)wait_for_button_press(0xFFFFFFFFU);

        if (pulse + (int32_t)SELFTEST_TRIM_STEP_US <= (int32_t)SELFTEST_TRIM_END_US) {
            pulse += (int32_t)SELFTEST_TRIM_STEP_US;
        } else {
            /* Top of the range and still curving left - hold here rather
             * than wrapping or recentering, and raise SELFTEST_TRIM_END_US. */
            oled_show_string_8x16_offset(3, 0, "END-RAISE RANGE");
        }
    }
}

/* --- Recorded (pulse, real angle) data points, for re-measurement ---
 *
 * Every angle here is a REAL protractor reading at the wheel, NOT the
 * "commanded" value the servo_set_angle*() functions take. That distinction
 * is the entire point: at 840us the cubic's commanded angle is 52.3deg while
 * the wheel actually sits at 35.0deg, so the commanded unit overstates
 * reality by roughly 1.5x and must never be recorded as if it were an angle.
 *
 * OPEN QUESTION on both entries below: WHICH WHEEL the protractor was on was
 * not recorded, and that single omission is what stops these two numbers
 * from being usable. It is not a bookkeeping nicety - it plausibly accounts
 * for the entire 35.0 vs. 29.5deg gap:
 *
 * One servo drives both front wheels through a shared tie-rod, so a given
 * physical wheel is the INNER wheel in one turn direction and the OUTER
 * wheel in the other. Ackermann geometry deliberately steers the inner wheel
 * harder than the outer. Measuring the same wheel at both locks therefore
 * yields an inner-wheel angle for one direction and an outer-wheel angle for
 * the other, and a several-degree difference between those is the mechanism
 * working correctly - not a left/right asymmetry.
 *
 * So 35.0 vs. 29.5deg may be inner-vs-outer rather than left-vs-right. Until
 * both wheels are measured at both locks, that cannot be told apart, and
 * mdp_description's URDF (which wants left_joint and right_joint separately)
 * cannot be updated either way. Four readings settle it:
 *   left lock  -> left wheel (inner),  right wheel (outer)
 *   right lock -> right wheel (inner), left wheel (outer) */
typedef struct {
    uint16_t    pulse_us;
    float       measured_deg; /* real wheel angle previously read here */
    const char *label;
} servo_cal_point_t;

static const servo_cal_point_t s_servo_cal_points[] = {
    {  840U, 35.0f, "LEFT  840" },  /* left mechanical limit (chassis contact) */
    { 2400U, 29.5f, "RIGHT 2400" }, /* NOT a confirmed limit - this was the old
                                     * SERVO_CAL_PULSE_MAX_US ceiling, and the
                                     * wheel was still tracking when it was
                                     * reached. See PHASE 2. */
};

/* Drives to each recorded point in turn, button-advanced, showing the pulse
 * alongside the angle previously measured there so the protractor reading
 * can be compared on the spot. Disagreement is itself the result worth
 * having - it would mean the earlier reading, the wheel identification, or
 * center calibration is off.
 *
 * Uses servo_set_pulse_us(), so the SERVO_ANGLE_MAX_LEFT/RIGHT_RAD operating
 * clamp does not apply - 840us is well outside it. Bounded only by
 * SERVO_CAL_PULSE_MIN/MAX_US. */
static void servo_cal_verify_points(void)
{
    char buf[20];
    const uint16_t center_us = servo_pulse_center_us();
    const uint8_t n = (uint8_t)(sizeof(s_servo_cal_points) / sizeof(s_servo_cal_points[0]));

    oled_clear();
    oled_show_string_8x16_offset(0, 0, "VERIFY POINTS");
    oled_show_string_8x16_offset(3, 0, "PE0=next point");

    for (uint8_t i = 0; i < n; i++) {
        servo_set_pulse_us(s_servo_cal_points[i].pulse_us);

        snprintf(buf, sizeof(buf), "%-11s", s_servo_cal_points[i].label);
        oled_show_string_8x16_offset(1, 0, buf);
        snprintf(buf, sizeof(buf), "WAS %+5.1f deg", (double)s_servo_cal_points[i].measured_deg);
        oled_show_string_8x16_offset(2, 0, buf);

        if (!wait_for_button_press(SELFTEST_CAL_STEP_TIMEOUT_MS)) {
            servo_set_pulse_us(center_us);
            oled_show_string_8x16_offset(1, 0, "TIMEOUT-CENTER  ");
            oled_show_string_8x16_offset(2, 0, "                ");
            HAL_Delay(1000);
            return;
        }
    }

    servo_set_pulse_us(center_us);
    oled_show_string_8x16_offset(1, 0, "POINTS DONE     ");
    oled_show_string_8x16_offset(2, 0, "                ");
    HAL_Delay(800);
}

/* Fine sweeps past the current operating clamp on each side, to find this
 * unit's real mechanical lock point - see docs/stm32/tuning.md. Uses
 * servo_set_angle_raw() to bypass the operating clamp (still bounded by
 * the hard PWM safety clamp in servo.c). WATCH/LISTEN while running: the
 * last angle with clean, unobstructed motion is the real limit - a stall
 * sounds like an audible buzz/whine with no visible motion, or the
 * knuckle visibly binding/scraping before the servo horn itself stops. */
static void servo_sweep_right_fine(void)
{
    servo_sweep_range(SELFTEST_SERVO_RIGHT_FINE_START_RAD, SELFTEST_SERVO_RIGHT_FINE_END_RAD,
                       "RIGHT FINE -22..-40", 1);
    servo_set_angle(0.0f);
    HAL_Delay(300);
}

static void servo_sweep_left_fine(void)
{
    servo_sweep_range(SELFTEST_SERVO_LEFT_FINE_START_RAD, SELFTEST_SERVO_LEFT_FINE_END_RAD,
                       "LEFT FINE 40-55", 1);
    servo_set_angle(0.0f);
    HAL_Delay(300);
}

/* Drives straight with steering centered, THROUGH THE PID LOOP - unlike
 * drive_ticks() above, which writes open-loop PWM with the loop paused and so
 * cannot test it.
 *
 * Displays both wheels' encoder-measured rad/s live against the target, which
 * is the diagnostic that matters here: if the two measured values sit near
 * the target and near each other but the car still curves, the PID is doing
 * its job and the fault is elsewhere (steering center, or unequal effective
 * wheel radius, which an encoder cannot see). If the measured values differ
 * from the target or from each other, the loop itself is not tracking and the
 * untuned MOTOR_PID_KP/KI are the place to look.
 *
 * Encoder deltas are NOT read here - the PID ISR is their sole consumer while
 * running, so this uses motor_pid_get_measured_rad_s() and times out on the
 * clock rather than counting ticks. */
static void servo_straight_line_pid(void)
{
    char buf[20];
    float left_rad_s = 0.0f, right_rad_s = 0.0f;

    oled_clear();
    oled_show_string_8x16_offset(0, 0, "STRAIGHT PID");

    servo_set_angle(0.0f); /* real angle now - 0 maps to the measured center */
    HAL_Delay(400);

    motor_pid_resume(); /* selftest_run() paused it on entry */
    motor_pid_enable(1);
    motor_pid_set_target(SELFTEST_STRAIGHT_RAD_S, SELFTEST_STRAIGHT_RAD_S);

    const uint32_t start_tick = HAL_GetTick();
    while (HAL_GetTick() - start_tick < SELFTEST_STRAIGHT_MS) {
        motor_pid_get_measured_rad_s(&left_rad_s, &right_rad_s);

        snprintf(buf, sizeof(buf), "TGT %4.1f rad/s", (double)SELFTEST_STRAIGHT_RAD_S);
        oled_show_string_8x16_offset(1, 0, buf);
        snprintf(buf, sizeof(buf), "L%4.1f  R%4.1f", (double)left_rad_s, (double)right_rad_s);
        oled_show_string_8x16_offset(2, 0, buf);

        HAL_Delay(100);
    }

    motor_pid_set_target(0.0f, 0.0f);
    HAL_Delay(300);
    motor_pid_enable(0);
    motor_pid_pause();
    motor_set_speed(0, 0);

    oled_show_string_8x16_offset(1, 0, "STRAIGHT DONE   ");
    oled_show_string_8x16_offset(2, 0, "                ");
    HAL_Delay(500);
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

    /* Phases 1-4 (mini component test: fwd/rev 1 rev, servo sweep,
     * straight-line PID drive) disabled while center-trim recalibration is
     * in progress, so a plain PE0 click goes straight to the trim tool
     * instead of sitting through ~15s of driving first. Re-enable
     * (uncomment) once a new center value is found and set - see the git
     * history for the original phase 1-4 block if restoring from scratch.
     *
     * blink_pe8(1, 150, 150);
     * oled_show_string_8x16_offset(1, 0, "1: FWD 1 REV");
     * drive_ticks(SELFTEST_DRIVE_PCT, SELFTEST_TICKS_PER_REV);
     *
     * blink_pe8(2, 150, 150);
     * oled_show_string_8x16_offset(1, 0, "2: REV 1 REV");
     * drive_ticks(-SELFTEST_DRIVE_PCT, SELFTEST_TICKS_PER_REV);
     *
     * blink_pe8(3, 150, 150);
     * oled_show_string_8x16_offset(1, 0, "3: SERVO SWEEP");
     * servo_sweep();
     *
     * blink_pe8(4, 150, 150);
     * servo_straight_line_pid();
     */

    /* Phase 5 - center trim re-calibration. Re-enabled on request (steering
     * curves right under real driving load, static push-test center may not
     * hold up loaded). Now the ONLY thing a PE0 click runs - holds
     * INDEFINITELY - see servo_cal_center_trim()'s own doc comment. Disable
     * again (comment out) once a new center value is found and set in
     * SERVO_PULSE_CENTER_US (servo.h). */
    blink_pe8(2, 150, 150);
    servo_cal_center_trim();      // holds indefinitely, never returns

    /* ------------------------------------------------------------------
     * Remaining steering-calibration tooling below is intentionally NOT
     * part of the routine sequence above - re-enable individual calls here
     * only when actually re-calibrating, see the RAW-PULSE CALIBRATION
     * block and its functions' own doc comments further up this file.
     *
     * blink_pe8(2, 150, 150);
     * servo_cal_verify_points();
     *
     * blink_pe8(2, 150, 150);
     * servo_cal_right_limit();      // sweeps 2380-2500us
     *
     * servo_cal_left_limit();
     *
     * blink_pe8(3, 150, 150);
     * servo_cal_measure();          // PHASE B, full-range protractor sweep
     *
     * servo_sweep_left_fine();      // superseded by raw-pulse calibration -
     * servo_sweep_right_fine();     // reference only, see block comment above
     * ------------------------------------------------------------------ */

    /* Return to center and finish */
    servo_set_angle(0.0f);
    oled_show_string_8x16_offset(1, 0, "DONE            ");
    blink_pe8(2, 80, 80);

    motor_pid_resume();
}
