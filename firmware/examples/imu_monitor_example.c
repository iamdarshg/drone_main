// imu_monitor_example.c - Example: IMU error/status monitoring
#include "imu_monitor.h"
#include <stdio.h>

int main(void) {
    imu_monitor_check_all();
    printf("IMU error flags: 0x%lX\n", (unsigned long)imu_monitor_get_errors());
    printf("IMU status flags: 0x%lX\n", (unsigned long)imu_monitor_get_status());
    imu_monitor_clear_errors();
    return 0;
}
