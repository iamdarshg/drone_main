#ifndef IMU_MONITOR_H
#define IMU_MONITOR_H
#include <stdint.h>
void imu_monitor_check_all(void);
void imu_monitor_clear_errors(void);
uint32_t imu_monitor_get_errors(void);
uint32_t imu_monitor_get_status(void);
#endif
