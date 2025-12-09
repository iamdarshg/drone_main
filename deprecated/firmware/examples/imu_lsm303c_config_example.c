// imu_lsm303c_config_example.c - Example: Change LSM303C ODR and FIFO
#include "imu_lsm303c.h"
#include <stdio.h>

int main(void) {
    imu_lsm303c_init();
    imu_lsm303c_set_accel_odr(0x03); // Example: 400Hz
    imu_lsm303c_set_mag_odr(0x02);   // Example: 50Hz
    imu_lsm303c_set_fifo_mode(0x01); // Example: FIFO mode
    printf("LSM303C ODR and FIFO configured\n");
    return 0;
}
