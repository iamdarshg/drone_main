#ifndef IMU_KX122_H
#define IMU_KX122_H
#include <stdint.h>

// Register addresses (KX122 datasheet)
#define KX122_REG_CTRL1 0x1B
#define KX122_REG_CTRL2 0x18
#define KX122_REG_CTRL3 0x1C
#define KX122_REG_CTRL4 0x1D
#define KX122_REG_CTRL5 0x1E
#define KX122_REG_CTRL6 0x1F
#define KX122_REG_CTRL7 0x20
#define KX122_REG_CTRL8 0x21
#define KX122_REG_FFTH  0x23
#define KX122_REG_FFC   0x24

// Register values
#define KX122_CTRL1_VALUE 0xE0
#define KX122_CTRL2_VALUE 0x80
#define KX122_CTRL3_VALUE 0x02
#define KX122_CTRL4_VALUE 0x40
#define KX122_CTRL5_VALUE 0x01
#define KX122_CTRL6_VALUE 0x40
#define KX122_CTRL7_VALUE 0x10
#define KX122_CTRL8_VALUE 0x00
#define KX122_FFTH_VALUE  0x14
#define KX122_FFC_VALUE   0x03

void imu_kx122_init(void);
int imu_kx122_read(float *accel, float *temp);
int imu_kx122_selftest(float *result);
void imu_kx122_calibrate(void);
void imu_kx122_set_grange(uint8_t range);
void imu_kx122_read_fifo(float *ax, float *ay, float *az);
void imu_kx122_configure_freefall(void);
void imu_kx122_handle_freefall(void);
void imu_kx122_set_odr(uint8_t odr);
void imu_kx122_set_fifo_mode(uint8_t mode);

#endif
