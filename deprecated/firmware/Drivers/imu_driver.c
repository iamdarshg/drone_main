#include "imu_driver.h"
#include "logging.h"
void imu_init_all(void) {
    log_info("IMUs initialized (LSM6DS3, KX122, LSM303C)");
}
imu_status_t imu_read_all(imu_data_t *out) {
    // Stub: fill with fake data
    for(int i=0;i<3;i++) out->accel[i]=0.0f, out->gyro[i]=0.0f, out->mag[i]=0.0f;
    out->temp = 25.0f;
    return IMU_OK;
}
imu_status_t imu_selftest_all(float *kalman_params, int n) {
    log_info("Running IMU self-test and Kalman tuning");
    // Stub: tune Kalman params (e.g., noise covariances)
    if (kalman_params && n>0) {
        for(int i=0;i<n;i++) kalman_params[i]=1.0f; // Example: set all to 1.0
    }
    return IMU_OK;
}
