#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdint.h>

int attitude_estimator_init(void);
int attitude_estimator_update(float dt, const float imu[6], float out_euler[3]);

#endif // ATTITUDE_ESTIMATOR_H
