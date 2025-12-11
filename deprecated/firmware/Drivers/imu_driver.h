#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H
#include <stdint.h>
typedef enum { IMU_OK=0, IMU_FAIL=1 } imu_status_t;
typedef struct {
    float accel[3];
    float gyro[3];
    float mag[3];
    float temp;
} imu_data_t;
void imu_init_all(void);
imu_status_t imu_read_all(imu_data_t *out);
imu_status_t imu_selftest_all(float *kalman_params, int n);
#endif
