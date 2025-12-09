// imu_lsm303c_support_example.c - Example: Check LSM303C ODR/FIFO support
#include "imu_lsm303c.h"
#include <stdio.h>

int main(void) {
    uint8_t odr = 0x03;
    if (imu_lsm303c_is_accel_odr_supported(odr))
        printf("Accel ODR 0x%02X supported\n", odr);
    else
        printf("Accel ODR 0x%02X NOT supported\n", odr);
    uint8_t fifo = 0x01;
    if (imu_lsm303c_is_fifo_mode_supported(fifo))
        printf("FIFO mode 0x%02X supported\n", fifo);
    else
        printf("FIFO mode 0x%02X NOT supported\n", fifo);
    return 0;
}
