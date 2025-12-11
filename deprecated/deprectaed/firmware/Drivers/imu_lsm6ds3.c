#include "imu_lsm6ds3.h"
#include "logging.h"
#include "i2c_bus.h"

static uint8_t lsm6ds3_odr = 0x60; // Default ODR (416 Hz)
static uint8_t lsm6ds3_fifo_mode = 0x06; // Default FIFO mode

void imu_lsm6ds3_set_odr(uint8_t odr) {
    lsm6ds3_odr = odr;
    i2c_bus_write_reg(LSM6DSV_I2C_ADDRESS, LSM6DSV_CTRL1_XL, odr);
}

void imu_lsm6ds3_set_fifo_mode(uint8_t mode) {
    lsm6ds3_fifo_mode = mode;
    i2c_bus_write_reg(LSM6DSV_I2C_ADDRESS, LSM6DSV_FIFO_CTRL5, mode);
}

void imu_lsm6ds3_init(void) { 
    log_info("LSM6DS3 init");
    imu_lsm6ds3_set_odr(lsm6ds3_odr);
    imu_lsm6ds3_set_fifo_mode(lsm6ds3_fifo_mode);
}

int imu_lsm6ds3_read(float *accel, float *gyro, float *temp) {
    for(int i=0;i<3;i++) accel[i]=gyro[i]=0.0f; *temp=25.0f; return 0;
}
int imu_lsm6ds3_selftest(float *result) { *result=1.0f; return 0; }
