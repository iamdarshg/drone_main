// imu_realtime_config_example.c - Change IMU ODR and FIFO in real time
#include "imu_kx122.h"
#include "imu_lsm6ds3.h"
#include <stdio.h>

int main(void) {
    // Set KX122 ODR to 400Hz, FIFO mode to stream
    imu_kx122_set_odr(0x05); // 400Hz (example)
    imu_kx122_set_fifo_mode(0x02); // Stream mode
    printf("KX122 ODR set to 400Hz, FIFO to stream\n");

    // Set LSM6DS3 ODR to 208Hz, FIFO mode to continuous
    imu_lsm6ds3_set_odr(0x50); // 208Hz (example)
    imu_lsm6ds3_set_fifo_mode(0x06); // Continuous mode
    printf("LSM6DS3 ODR set to 208Hz, FIFO to continuous\n");

    return 0;
}
