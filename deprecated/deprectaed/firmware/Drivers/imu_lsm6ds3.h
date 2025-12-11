#ifndef IMU_LSM6DS3_H
#define IMU_LSM6DS3_H
#include <stdint.h>

// Register addresses (LSM6DS3/LSM6DSV)
#define LSM6DSV_I2C_ADDRESS 0x6A
#define LSM6DSV_CTRL1_XL 0x10
#define LSM6DSV_CTRL2_G  0x11
#define LSM6DSV_CTRL3_C  0x12
#define LSM6DSV_CTRL9_XL 0x18
#define LSM6DSV_FIFO_CTRL5 0x0B
#define LSM6DSV_OUTX_L_A 0x28
#define LSM6DSV_OUTX_H_A 0x29
#define LSM6DSV_OUTY_L_A 0x2A
#define LSM6DSV_OUTY_H_A 0x2B
#define LSM6DSV_OUTZ_L_A 0x2C
#define LSM6DSV_OUTZ_H_A 0x2D
#define LSM6DSV_FIFO_STATUS1 0x3A
#define LSM6DSV_FIFO_STATUS2 0x3B
#define LSM6DSV_FIFO_DATA_OUT_L 0x3E
#define LSM6DSV_FIFO_DATA_OUT_H 0x3F

void imu_lsm6ds3_init(void);
int imu_lsm6ds3_read(float *accel, float *gyro, float *temp);
int imu_lsm6ds3_selftest(float *result);
// Real-time ODR and FIFO config
void imu_lsm6ds3_set_odr(uint8_t odr);
void imu_lsm6ds3_set_fifo_mode(uint8_t mode);
#endif
