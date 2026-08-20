/**
 * @file imu.c
 * @brief ICM-20948 9-DOF IMU Sensor Driver Implementation (WHEELTEC C30D Board)
 *
 * Pins:
 * - PB10: I2C_SCL
 * - PB11: I2C_SDA
 */

#include "imu.h"
#include <stdio.h>
#include <math.h>

imu_data_t g_imu_data = {0};

static uint8_t s_imu_addr = 0xD0; /* 0x68 << 1 (Try 0xD0 first, fallback to 0xD2) */

#define SCL_H() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define SCL_L() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define SDA_H() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define SDA_L() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)

static void i2c_delay(void)
{
    for (volatile int i = 0; i < 20; i++);
}

static void i2c_start(void)
{
    SDA_H(); SCL_H(); i2c_delay();
    SDA_L(); i2c_delay();
    SCL_L(); i2c_delay();
}

static void i2c_stop(void)
{
    SDA_L(); SCL_H(); i2c_delay();
    SDA_H(); i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t dat)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (dat & 0x80) SDA_H(); else SDA_L();
        i2c_delay();
        SCL_H(); i2c_delay();
        SCL_L(); i2c_delay();
        dat <<= 1;
    }
    SDA_H(); i2c_delay();
    SCL_H(); i2c_delay();
    uint8_t ack = SDA_READ();
    SCL_L(); i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t dat = 0;
    SDA_H();
    for (uint8_t i = 0; i < 8; i++) {
        dat <<= 1;
        SCL_H(); i2c_delay();
        if (SDA_READ()) dat |= 0x01;
        SCL_L(); i2c_delay();
    }
    if (ack) SDA_L(); else SDA_H();
    i2c_delay();
    SCL_H(); i2c_delay();
    SCL_L(); i2c_delay();
    SDA_H();
    return dat;
}

static uint8_t imu_write_reg(uint8_t reg, uint8_t dat)
{
    i2c_start();
    if (i2c_write_byte(s_imu_addr)) { i2c_stop(); return 1; }
    i2c_write_byte(reg);
    i2c_write_byte(dat);
    i2c_stop();
    return 0;
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t dat = 0;
    i2c_start();
    if (i2c_write_byte(s_imu_addr)) { i2c_stop(); return 0xFF; }
    i2c_write_byte(reg);
    i2c_start();
    i2c_write_byte(s_imu_addr | 0x01);
    dat = i2c_read_byte(0);
    i2c_stop();
    return dat;
}

static void imu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_write_byte(s_imu_addr);
    i2c_write_byte(reg);
    i2c_start();
    i2c_write_byte(s_imu_addr | 0x01);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2c_read_byte(i == (len - 1) ? 0 : 1);
    }
    i2c_stop();
}

int imu_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; /* Open-Drain for I2C */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    SCL_H();
    SDA_H();
    HAL_Delay(10);

    /* Try 0xD0 (0x68) first, then 0xD2 (0x69) */
    uint8_t addrs[2] = {0xD0, 0xD2};
    uint8_t id = 0xFF;

    for (int a = 0; a < 2; a++) {
        s_imu_addr = addrs[a];

        /* Select Register Bank 0 */
        imu_write_reg(0x7F, 0x00);
        HAL_Delay(5);

        /* Read WHO_AM_I (Register 0x00) */
        id = imu_read_reg(0x00);
        printf("[IMU Probe] Address 0x%02X -> WHO_AM_I: 0x%02X\r\n", s_imu_addr >> 1, id);

        if (id == 0xEA) {
            /* Reset Device */
            imu_write_reg(0x06, 0x80); /* PWR_MGMT_1 reset */
            HAL_Delay(50);
            imu_write_reg(0x7F, 0x00); /* Select Bank 0 */
            imu_write_reg(0x06, 0x01); /* Auto select clock source */
            imu_write_reg(0x07, 0x00); /* PWR_MGMT_2 enable accel & gyro */
            HAL_Delay(20);

            g_imu_data.ready = 1;
            printf("[IMU Init] ICM-20948 Successfully Detected at 0x%02X!\r\n", s_imu_addr >> 1);
            return 0;
        }
    }

    g_imu_data.ready = 0;
    printf("[IMU Init Error] ICM-20948 Sensor Not Found (Returned ID: 0x%02X)\r\n", id);
    return -1;
}

void imu_update(void)
{
    if (!g_imu_data.ready) {
        return;
    }

    /* Select Bank 0 */
    imu_write_reg(0x7F, 0x00);

    uint8_t buf[12];
    imu_read_bytes(0x2D, buf, 12); /* ACCEL_XOUT_H (0x2D) to GYRO_ZOUT_L (0x38) */

    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    int16_t gx_raw = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t gy_raw = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gz_raw = (int16_t)((buf[10] << 8) | buf[11]);

    g_imu_data.accel_x = (float)ax_raw / 16384.0f * 9.81f;
    g_imu_data.accel_y = (float)ay_raw / 16384.0f * 9.81f;
    g_imu_data.accel_z = (float)az_raw / 16384.0f * 9.81f;

    g_imu_data.gyro_x = (float)gx_raw / 131.0f;
    g_imu_data.gyro_y = (float)gy_raw / 131.0f;
    g_imu_data.gyro_z = (float)gz_raw / 131.0f;

    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (last_tick > 0) {
        float dt = (float)(now - last_tick) / 1000.0f;
        if (fabsf(g_imu_data.gyro_z) > 0.5f) {
            g_imu_data.yaw += g_imu_data.gyro_z * dt;
        }
    }
    last_tick = now;
}
