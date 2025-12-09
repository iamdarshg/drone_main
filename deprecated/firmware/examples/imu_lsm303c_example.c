// imu_lsm303c_example.c - Example: LSM303C IMU usage
#include "imu_lsm303c.h"
#include <stdio.h>

int main(void) {
    float accel[3], mag[3], temp;
    imu_lsm303c_init();
    if (imu_lsm303c_read(accel, mag, &temp) == 0) {
        printf("Accel: %f %f %f\n", accel[0], accel[1], accel[2]);
        printf("Mag: %f %f %f\n", mag[0], mag[1], mag[2]);
        printf("Temp: %f\n", temp);
    } else {
        printf("LSM303C read failed\n");
    }
    float result;
    if (imu_lsm303c_selftest(&result) == 0 && result > 0.5f) {
        printf("LSM303C self-test passed\n");
    } else {
        printf("LSM303C self-test failed\n");
    }
    return 0;
}
