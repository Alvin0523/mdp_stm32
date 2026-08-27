/**
 * @file imu.h
 * @brief ICM-20948 9-DOF IMU Driver (I2C2 PB10 SCL / PB11 SDA)
 */

#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef struct {
    float accel_x; /* Accel X (m/s^2) */
    float accel_y; /* Accel Y (m/s^2) */
    float accel_z; /* Accel Z (m/s^2) */
    float gyro_x;  /* Gyro Roll Rate (deg/s) */
    float gyro_y;  /* Gyro Pitch Rate (deg/s) */
    float gyro_z;  /* Gyro Yaw Rate (deg/s) */
    float yaw;     /* Gyro-Z integrated yaw angle (deg) - bias-corrected, no accel/mag fusion */
    uint8_t ready; /* 1 if sensor detected and operational */
} imu_data_t;

extern imu_data_t g_imu_data;

/**
 * @brief Initialize PB10/PB11 GPIOs and ICM-20948 9-axis sensor
 * @return 0 on success, -1 if sensor not found
 */
int imu_init(void);

/**
 * @brief Read accel/gyro registers and update g_imu_data
 */
void imu_update(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
