#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

typedef enum {
	MOTOR_A = 0, /* Rear Left */
	MOTOR_B = 1, /* Rear Right */
	MOTOR_C = 2,
	MOTOR_D = 3,
	MOTOR_COUNT
} motor_id_t;

int motor_init(void);
int motor_set_speed(motor_id_t motor, int16_t speed);

#endif /* MOTOR_H_ */
