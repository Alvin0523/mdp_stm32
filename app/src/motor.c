#include "motor.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#define MOTOR_PWM_PERIOD_NS 50000

static const struct pwm_dt_spec motor_in1[MOTOR_COUNT] = {
	[MOTOR_A] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_a_in1_pwm)),
		     .channel = 1,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
	[MOTOR_B] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_b_pwm)),
		     .channel = 1,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
	[MOTOR_C] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_cd_pwm)),
		     .channel = 1,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
	[MOTOR_D] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_cd_pwm)),
		     .channel = 3,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
};

static const struct pwm_dt_spec motor_in2[MOTOR_COUNT] = {
	[MOTOR_A] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_a_in2_pwm)),
		     .channel = 1,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
	[MOTOR_B] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_b_pwm)),
		     .channel = 2,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
	[MOTOR_C] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_cd_pwm)),
		     .channel = 2,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
	[MOTOR_D] = {.dev = DEVICE_DT_GET(DT_NODELABEL(motor_cd_pwm)),
		     .channel = 4,
		     .period = MOTOR_PWM_PERIOD_NS,
		     .flags = PWM_POLARITY_NORMAL},
};

#define MOTOR_SPEED_MAX 1000

int motor_init(void)
{
	for (motor_id_t m = 0; m < MOTOR_COUNT; m++) {
		if (!pwm_is_ready_dt(&motor_in1[m]) || !pwm_is_ready_dt(&motor_in2[m])) {
			printk("motor: motor %d PWM device not ready\n", m);
			return -ENODEV;
		}
	}

	for (motor_id_t m = 0; m < MOTOR_COUNT; m++) {
		int rc = motor_set_speed(m, 0);
		if (rc < 0) {
			return rc;
		}
	}

	return 0;
}

int motor_set_speed(motor_id_t motor, int16_t speed)
{
	if (motor >= MOTOR_COUNT) {
		return -EINVAL;
	}

	if (speed > MOTOR_SPEED_MAX) {
		speed = MOTOR_SPEED_MAX;
	} else if (speed < -MOTOR_SPEED_MAX) {
		speed = -MOTOR_SPEED_MAX;
	}

	uint32_t in1_pulse = 0;
	uint32_t in2_pulse = 0;

	if (speed > 0) {
		in1_pulse = ((uint32_t)speed * motor_in1[motor].period) / MOTOR_SPEED_MAX;
	} else if (speed < 0) {
		in2_pulse = ((uint32_t)(-speed) * motor_in2[motor].period) / MOTOR_SPEED_MAX;
	}

	int rc = pwm_set_pulse_dt(&motor_in1[motor], in1_pulse);
	if (rc < 0) {
		return rc;
	}

	return pwm_set_pulse_dt(&motor_in2[motor], in2_pulse);
}
