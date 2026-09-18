#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>

/* Wiring: PA2 = TRIG (H1/9), PA3 = ECHO (H1/11, TIM5_CH4, AF2).
 * These pins must be free; ECHO requires a 5V-to-3V divider.
 * Enabled by default; override with -D ULTRASONIC_ENABLED=0 to disable. */
#ifndef ULTRASONIC_ENABLED
#define ULTRASONIC_ENABLED 1
#endif

void ultrasonic_init(void);
void ultrasonic_update(void);
/* False means disabled, no echo, out of range, or older than 300 ms. */
bool ultrasonic_get_distance_cm(float *distance_cm);

#endif
