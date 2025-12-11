// Example: Basic usage of KX122 IMU driver
#include "imu_kx122.h"
#include <stdio.h>

int main(void) {
    float accel[3], temp;
    imu_kx122_init();
    imu_kx122_set_grange(0x00); // Set range to +/-2g
    imu_kx122_calibrate();
    while (1) {
        imu_kx122_read(accel, &temp);
        printf("Accel: X=%.3fg Y=%.3fg Z=%.3fg\n", accel[0], accel[1], accel[2]);
        // Add delay as needed
    }
    return 0;
}
