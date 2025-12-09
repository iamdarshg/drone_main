// imu_lsm303c_all_features_example.c - Real-world usage: config, check, read, self-test
#include "imu_lsm303c.h"
#include <stdio.h>

int main(void) {
    imu_lsm303c_init();
    // Try to set ODR and FIFO, check if supported
    uint8_t odr = 0x03, fifo = 0x01;
    if (imu_lsm303c_is_accel_odr_supported(odr)) {
        imu_lsm303c_set_accel_odr(odr);
        printf("Accel ODR set to 0x%02X\n", odr);
    } else {
        printf("Accel ODR 0x%02X not supported\n", odr);
    }
    if (imu_lsm303c_is_fifo_mode_supported(fifo)) {
        imu_lsm303c_set_fifo_mode(fifo);
        printf("FIFO mode set to 0x%02X\n", fifo);
    } else {
        printf("FIFO mode 0x%02X not supported\n", fifo);
    }
    // Read data
    float accel[3], mag[3], temp;
    if (imu_lsm303c_read(accel, mag, &temp) == 0) {
        printf("Accel: %f %f %f\n", accel[0], accel[1], accel[2]);
        printf("Mag: %f %f %f\n", mag[0], mag[1], mag[2]);
        printf("Temp: %f\n", temp);
    }
    // Self-test
    float result;
    if (imu_lsm303c_selftest(&result) == 0 && result > 0.5f) {
        printf("LSM303C self-test passed\n");
    } else {
        printf("LSM303C self-test failed\n");
    }
    return 0;
}
