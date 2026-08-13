#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "motor.h"

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
	printk("=== STM32 C30D Board Bringup Test ===\n");

	if (!gpio_is_ready_dt(&led)) {
		printk("Error: LED GPIO device not ready\n");
		return 0;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printk("Error: Failed to configure LED GPIO pin\n");
		return 0;
	}

	if (motor_init() < 0) {
		printk("Error: AT8236 Motor PWM init failed\n");
		return 0;
	}

	printk("Success: LED & Motor PWM initialized cleanly!\n");

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}

	return 0;
}
