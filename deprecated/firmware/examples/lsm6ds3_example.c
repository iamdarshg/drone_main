// Example: Basic usage of LSM6DS3 IMU driver
#include "imu_lsm6ds3.h"
#include <stdio.h>

int main(void) {
    float accel[3], gyro[3], temp;
    imu_lsm6ds3_init();
    imu_lsm6ds3_selftest();
    while (1) {
        imu_lsm6ds3_read(accel, gyro, &temp);
        printf("Accel: X=%.3fg Y=%.3fg Z=%.3fg | Gyro: X=%.3fdps Y=%.3fdps Z=%.3fdps\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
        // Add delay as needed
    }
    return 0;
}
