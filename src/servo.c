/**
 * @file servo.c
 * @brief Ackermann Front-Wheel Steering Servo Driver Implementation
 *
 * Hardware: WHEELTEC C30D board, servo header PB15 (TIM12_CH2, AF9).
 * Confirmed via "STM32F407VET6(C30D-V2.1)主板资源分配说明.pdf":
 *   PC6/PC7/PC8/PC9/PB14/PB15 -> TIM8/TIM12, "阿克曼小车(舵机)使用PB15引脚"
 *
 * TIM12 is on APB1; since APB1 prescaler != 1, TIM12CLK = 2 x PCLK1 = 84MHz.
 * PSC=83 -> 1MHz (1us) counter tick. ARR=19999 -> 20ms period (50Hz), the
 * standard RC servo refresh rate. CCR is therefore the pulse width in
 * microseconds directly.
 */

#include "servo.h"

/* MEASURED straight-ahead, not the nominal 1500us (2026-09-11).
 *
 * Found with selftest.c's servo_cal_center_trim(): motors off, car pushed by
 * hand at each candidate pulse. The nominal 1500us curved RIGHT. A hand push
 * removes the motors and the PID from the picture, so this isolates steering
 * geometry - the offset is servo horn/linkage assembly trim, not a control
 * problem.
 *
 * FINALISED at 1490us after narrowing on hardware (1445 -> 1450 -> 1475 ->
 * 1482 -> 1490us).
 *
 * This feeds the per-side slope constants below, so changing it rescales both
 * sides - the left span tightens (650us to full lock instead of the nominal
 * 660us) and the right span widens (910us instead of 900us).
 * servo_init() also seats the servo here at boot. */
#define SERVO_PULSE_CENTER_US 1490U

/* ------------------------------------------------------------------------
 * REAL-ANGLE -> PULSE MAPPING (measured on this chassis, 2026-09-11)
 *
 * RETIRED: WHEELTEC's reference cubic (R550_C30D(2.0) balance.c,
 * Drive_Motor()'s Akm_Car branch),
 *     Angle_Servo = -0.628a^3 + 1.269a^2 - 1.772a + 1.573
 *     Servo_us    = 1500 + (Angle_Servo - 1.572) * 636.56   [clamp 800-2200]
 * It was adopted on the assumption that its input unit meant a real wheel
 * angle, as it does in their firmware (their AngleR feeds R = wheelbase /
 * tan(AngleR)). Protractor measurement disproved that on this unit: at 840us
 * the cubic's input reads 52.3deg while the wheel actually sits at 35.0deg,
 * an overstatement of roughly 1.5x. Its 800-2200us clamp was also actively
 * harmful - it capped travel at 2200us, which is 200us short of this
 * chassis's right-hand range and made that side look like it stalled early.
 *
 * REPLACED BY: per-side linear interpolation between measured endpoints.
 * Two protractor readings at the wheel, one per lock direction, plus a
 * measured (not assumed) center:
 *
 *   left  (positive):  0 deg -> 1490us,  35.0 deg -> 840us   (-650us)
 *   right (negative):  0 deg -> 1490us, -29.5 deg -> 2400us   (+910us)
 *
 * Slope is therefore per-side: ~-18.6us/deg left, ~-30.8us/deg right. Both
 * are negative because a lengthening pulse steers right on this unit. The
 * sides differ by nearly 2x, which is why a single shared slope cannot work.
 *
 * The angle argument is now a REAL wheel angle, the same quantity the URDF
 * and ackermann_steering_controller use - so mdp_bridge's straight
 * pass-through of steer_rad is finally correct rather than silently feeding
 * a real angle into a differently-scaled unit.
 *
 * KNOWN LIMITATIONS, both resolvable with the selftest calibration phases:
 *   1. Linearity between center and each lock is ASSUMED, not measured. Only
 *      the two endpoints and center are known. The real curve is likely
 *      somewhat nonlinear (that is what WHEELTEC's cubic was modelling), so
 *      mid-range angles carry unknown error. Fix: measure intermediate
 *      points with selftest.c's servo_cal_measure() and fit per side.
 *   2. RESOLVED - center is measured rather than assumed: 1490us, finalised
 *      on hardware. Only 10us off the nominal 1500us, so the original
 *      right-hand drift was a small trim offset, not a large one.
 *   3. WHICH front wheel each reading came from was not recorded. One servo
 *      drives both wheels via a shared tie-rod, so Ackermann geometry puts
 *      the inner wheel at a larger angle than the outer - meaning the
 *      35.0/29.5 difference may be inner-vs-outer rather than a genuine
 *      left/right asymmetry. Until both wheels are measured at both locks,
 *      treat these as "the steering angle" in a loose single-track sense.
 * --------------------------------------------------------------------- */

/* Measured endpoints. Update these - not the derived slopes below - when
 * better calibration data exists. */
#define SERVO_CAL_LEFT_ANGLE_DEG    35.0f
#define SERVO_CAL_LEFT_PULSE_US    840.0f
#define SERVO_CAL_RIGHT_ANGLE_DEG   29.5f  /* magnitude; commanded negative */
#define SERVO_CAL_RIGHT_PULSE_US  2400.0f

#define SERVO_DEG_TO_RAD (3.14159265f / 180.0f)

/* Microseconds per radian, per side, derived from the endpoints above so the
 * two never drift out of sync. Both evaluate to negative constants. */
#define SERVO_US_PER_RAD_LEFT \
    ((SERVO_CAL_LEFT_PULSE_US - (float)SERVO_PULSE_CENTER_US) \
     / (SERVO_CAL_LEFT_ANGLE_DEG * SERVO_DEG_TO_RAD))
#define SERVO_US_PER_RAD_RIGHT \
    ((SERVO_CAL_RIGHT_PULSE_US - (float)SERVO_PULSE_CENTER_US) \
     / (-SERVO_CAL_RIGHT_ANGLE_DEG * SERVO_DEG_TO_RAD))

/* Absolute pulse safety envelope. Applies to every path in this driver,
 * including servo_set_pulse_us() and servo_set_angle_raw().
 *
 * Deliberately WIDER than the operating range the measured angle limits
 * produce (840-2400us), so calibration sweeps can probe past the current
 * limits - that headroom is the whole point. The retired cubic's 800-2200us
 * clamp is the cautionary example: it sat 200us INSIDE this chassis's actual
 * right-hand travel, so every sweep past 2200us sent an identical pulse and
 * the side looked like it stalled early when it had simply stopped being
 * commanded further.
 *
 * 2500us is the top of the conventional extended RC envelope, and this stops
 * there - beyond it a servo's internal feedback pot can be driven out of
 * range, which is not a limit worth finding by experiment. The right side is
 * currently taken as 2400us and was still tracking there, so this side may
 * extend slightly further; revisit with selftest.c's PHASE 2 sweep. */
#define SERVO_CAL_PULSE_MIN_US 600U
#define SERVO_CAL_PULSE_MAX_US 2500U

static TIM_HandleTypeDef s_htim12;

void servo_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    s_htim12.Instance = TIM12;
    s_htim12.Init.Prescaler = 83;
    s_htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim12.Init.Period = 19999;
    s_htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&s_htim12);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_PULSE_CENTER_US;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&s_htim12, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&s_htim12, TIM_CHANNEL_2);
}

/* Right-side cubic Hermite spline, replacing straight-line interpolation on
 * that side only. Reasoning: a smooth full-range PWM sweep (hand-observed on
 * this unit) showed a clearly SMALLER real wheel-angle change per
 * microsecond of pulse near the 2400us limit than near center - the true
 * response flattens in angle terms (equivalently, steepens in
 * pulse-per-angle terms) approaching that lock, not a straight line. This
 * matches the physical theory behind WHEELTEC's own reference cubic PWM fit
 * (see commit b8ed0a6): a rigid Ackermann linkage's mechanical advantage
 * genuinely varies nonlinearly - their numeric coefficients didn't transfer
 * to this chassis (confirmed wrong by protractor, see commit 4829d88), but
 * the underlying curved-not-straight shape is real.
 *
 * Built as a cubic Hermite spline through the SAME THREE REAL MEASURED
 * POINTS already used elsewhere in this file (840/1490/2400us) - not an
 * independently invented curve. A cubic has 4 degrees of freedom; 3 points
 * constrain 3, leaving exactly one free choice, resolved as follows:
 *   - Slope AT CENTER is set equal to the LEFT side's own straight-line
 *     slope (SERVO_US_PER_RAD_LEFT). This is what makes the curve SMOOTH
 *     (continuous derivative) through center rather than having a kink
 *     there - a kink at center would imply an actual mechanical
 *     discontinuity in the linkage, which isn't physically plausible.
 *   - Slope AT THE RIGHT LIMIT is set so the two Hermite endpoint slopes
 *     average to exactly the segment's real measured average slope
 *     (SERVO_US_PER_RAD_RIGHT), i.e. m0 + m1 = 2 * avg. This is the
 *     simplest reasonable way to distribute curvature given only 3 known
 *     points - it produces a limit-end slope ~2.3x steeper than center and
 *     ~1.4x steeper than the old straight-line average, matching the
 *     flattening actually observed. The EXACT curvature distribution
 *     between center and the limit is still unverified pending real
 *     intermediate protractor readings (servo_cal_measure(), selftest.c) -
 *     this is an informed estimate of the curve's SHAPE from one qualitative
 *     observation, not a fitted measurement of it.
 *
 * LEFT side is left as a straight line - no comparable flattening was
 * observed/reported there, so no comparable assumption is made for it. */
#define SERVO_RIGHT_HERMITE_M0_US_PER_RAD SERVO_US_PER_RAD_LEFT
#define SERVO_RIGHT_HERMITE_M1_US_PER_RAD \
    (2.0f * SERVO_US_PER_RAD_RIGHT - SERVO_US_PER_RAD_LEFT)

static float servo_right_hermite_pulse_us(float angle_rad)
{
    const float right_limit_rad = -SERVO_CAL_RIGHT_ANGLE_DEG * SERVO_DEG_TO_RAD;
    const float t = angle_rad / right_limit_rad; /* 0 at center, 1 at the limit */
    const float t2 = t * t;
    const float t3 = t2 * t;

    /* Standard cubic Hermite basis functions. */
    const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    const float h10 = t3 - 2.0f * t2 + t;
    const float h01 = -2.0f * t3 + 3.0f * t2;
    const float h11 = t3 - t2;

    return h00 * (float)SERVO_PULSE_CENTER_US
         + h10 * right_limit_rad * SERVO_RIGHT_HERMITE_M0_US_PER_RAD
         + h01 * SERVO_CAL_RIGHT_PULSE_US
         + h11 * right_limit_rad * SERVO_RIGHT_HERMITE_M1_US_PER_RAD;
}

/* Shared conversion, used by both the clamped and raw entry points below.
 * Left side: linear interpolation between the measured endpoints, see the
 * REAL-ANGLE -> PULSE MAPPING block above. Right side: cubic Hermite, see
 * servo_right_hermite_pulse_us() above. Bounded only by
 * SERVO_CAL_PULSE_MIN/MAX_US, the absolute safety envelope; the per-side
 * angle clamp in servo_set_angle() is what enforces the measured operating
 * limits. */
static void servo_write_angle_unclamped(float angle_rad)
{
    float pulse_us = (angle_rad >= 0.0f)
        ? (float)SERVO_PULSE_CENTER_US + angle_rad * SERVO_US_PER_RAD_LEFT
        : servo_right_hermite_pulse_us(angle_rad);

    if (pulse_us < (float)SERVO_CAL_PULSE_MIN_US) pulse_us = (float)SERVO_CAL_PULSE_MIN_US;
    if (pulse_us > (float)SERVO_CAL_PULSE_MAX_US) pulse_us = (float)SERVO_CAL_PULSE_MAX_US;

    __HAL_TIM_SET_COMPARE(&s_htim12, TIM_CHANNEL_2, (uint32_t)pulse_us);
}

void servo_set_angle(float angle_rad)
{
    /* Positive = left, negative = right - confirmed on physical hardware,
     * see servo.h's sign-convention note. */
    if (angle_rad > SERVO_ANGLE_MAX_LEFT_RAD) angle_rad = SERVO_ANGLE_MAX_LEFT_RAD;
    if (angle_rad < -SERVO_ANGLE_MAX_RIGHT_RAD) angle_rad = -SERVO_ANGLE_MAX_RIGHT_RAD;
    servo_write_angle_unclamped(angle_rad);
}

void servo_set_angle_raw(float angle_rad)
{
    /* No SERVO_ANGLE_MAX_LEFT/RIGHT_RAD clamp - calibration/self-test use
     * only, see the warning in servo.h. Still bounded by
     * SERVO_CAL_PULSE_MIN/MAX_US regardless. Note the mapping is only
     * calibrated between center and each measured lock point, so angles
     * beyond the operating clamp are linear extrapolation, not measurement. */
    servo_write_angle_unclamped(angle_rad);
}

void servo_set_pulse_us(uint16_t pulse_us)
{
    /* Bypasses BOTH the cubic and the operating angle clamp - the pulse
     * width is written to TIM12's compare register directly. This is the
     * only entry point whose unit is unambiguous and independent of
     * WHEELTEC's unverified angle mapping, which is why calibration
     * measurements should be recorded against it. See servo.h. */
    if (pulse_us < SERVO_CAL_PULSE_MIN_US) pulse_us = SERVO_CAL_PULSE_MIN_US;
    if (pulse_us > SERVO_CAL_PULSE_MAX_US) pulse_us = SERVO_CAL_PULSE_MAX_US;

    __HAL_TIM_SET_COMPARE(&s_htim12, TIM_CHANNEL_2, (uint32_t)pulse_us);
}

uint16_t servo_get_pulse_us(void)
{
    /* CCR is the pulse width in microseconds directly (1MHz counter tick,
     * see the file header), so no conversion is needed. */
    return (uint16_t)__HAL_TIM_GET_COMPARE(&s_htim12, TIM_CHANNEL_2);
}

uint16_t servo_pulse_center_us(void)
{
    return SERVO_PULSE_CENTER_US;
}
