/**
 * @file protocol.h
 * @brief USART3 <-> Raspberry Pi Binary Telemetry/Command Protocol
 *
 * Custom lightweight framed protocol (not micro-ROS) linking this firmware
 * to a ROS2 bridge node on the Pi over USART3 @ 115200 baud. Both packet
 * types are fixed-size and start with the same 2 sync bytes + type byte, so
 * a receiver can resync after a dropped/corrupted byte.
 *
 * IMPORTANT: this struct layout is mirrored by hand in the ROS2 bridge node
 * (mdp_ros/src/mdp_hardware_bridge). If you change a field here, update it
 * there too - there is no shared build-time header between the two repos.
 * Both sides are little-endian (Cortex-M4 and aarch64/x86_64 Pi), so no
 * byte-swapping is done.
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PROTOCOL_SYNC0 0xAAU
#define PROTOCOL_SYNC1 0x55U

#define PROTOCOL_TYPE_TELEMETRY 0x01U
#define PROTOCOL_TYPE_COMMAND   0x02U

#pragma pack(push, 1)

/* STM32 -> Pi, sent once per main loop iteration (currently 10Hz). */
typedef struct {
    uint8_t  sync0;
    uint8_t  sync1;
    uint8_t  type;          /* PROTOCOL_TYPE_TELEMETRY */
    int32_t  enc_left;      /* Cumulative Motor A (rear left) encoder ticks */
    int32_t  enc_right;     /* Cumulative Motor B (rear right) encoder ticks */
    float    steer_deg;     /* Currently applied steering angle (deg) */
    float    accel_x;       /* m/s^2 */
    float    accel_y;       /* m/s^2 */
    float    accel_z;       /* m/s^2 */
    float    gyro_x;        /* deg/s */
    float    gyro_y;        /* deg/s */
    float    gyro_z;        /* deg/s */
    float    yaw_deg;       /* Complementary-filter yaw (deg) */
    uint8_t  imu_ready;     /* 1 = IMU detected and operational */
    uint8_t  estop;         /* 1 = e-stop engaged (PD3) */
    uint32_t uptime_ms;
    uint8_t  checksum;      /* XOR of all bytes from 'type' through 'uptime_ms' */
} telemetry_packet_t;

/* Pi -> STM32, sent whenever the host has a new command (typically matching
 * ros2_control's update_rate, e.g. 50Hz). */
typedef struct {
    uint8_t  sync0;
    uint8_t  sync1;
    uint8_t  type;              /* PROTOCOL_TYPE_COMMAND */
    float    left_wheel_rad_s;  /* Target rear-left wheel angular velocity */
    float    right_wheel_rad_s; /* Target rear-right wheel angular velocity */
    float    steer_rad;         /* Target steering angle (radians) */
    uint8_t  checksum;          /* XOR of all bytes from 'type' through 'steer_rad' */
} command_packet_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
