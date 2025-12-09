// Example: Basic usage of LSM303C IMU driver
#include "imu_lsm303c.h"
#include <stdio.h>

int main(void) {
    float accel[3], mag[3], temp;
    imu_lsm303c_init();
    imu_lsm303c_selftest();
    while (1) {
        imu_lsm303c_read(accel, mag, &temp);
        printf("Accel: X=%.3fg Y=%.3fg Z=%.3fg | Mag: X=%.3fuT Y=%.3fuT Z=%.3fuT\n", accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
        // Add delay as needed
    }
    return 0;
}
