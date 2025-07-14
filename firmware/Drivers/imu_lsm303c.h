#ifndef IMU_LSM303C_H
#define IMU_LSM303C_H
#include <stdint.h>
#include <stdbool.h>

// Register addresses (LSM303C/LSM303AHTR)
#define LSM303C_I2C_ADDRESS 0x1E
#define LSM303C_CTRL_REG1_A 0x20
#define LSM303C_CTRL_REG4_A 0x23
#define LSM303C_OUT_X_L_A   0x28
#define LSM303C_OUT_X_H_A   0x29
#define LSM303C_OUT_Y_L_A   0x2A
#define LSM303C_OUT_Y_H_A   0x2B
#define LSM303C_OUT_Z_L_A   0x2C
#define LSM303C_OUT_Z_H_A   0x2D
#define LSM303C_CTRL_REG1_M 0x60
#define LSM303C_CTRL_REG2_M 0x61
#define LSM303C_CTRL_REG3_M 0x62
#define LSM303C_CTRL_REG4_M 0x63
#define LSM303C_CTRL_REG5_M 0x64
#define LSM303C_OUT_X_L_M   0x68
#define LSM303C_OUT_X_H_M   0x69
#define LSM303C_OUT_Y_L_M   0x6A
#define LSM303C_OUT_Y_H_M   0x6B
#define LSM303C_OUT_Z_L_M   0x6C
#define LSM303C_OUT_Z_H_M   0x6D

void imu_lsm303c_init(void);
int imu_lsm303c_read(float *accel, float *mag, float *temp);
int imu_lsm303c_selftest(float *result);
void imu_lsm303c_set_accel_odr(uint8_t odr);
void imu_lsm303c_set_mag_odr(uint8_t odr);
void imu_lsm303c_set_fifo_mode(uint8_t mode);
bool imu_lsm303c_is_accel_odr_supported(uint8_t odr);
bool imu_lsm303c_is_mag_odr_supported(uint8_t odr);
bool imu_lsm303c_is_fifo_mode_supported(uint8_t mode);

#endif
