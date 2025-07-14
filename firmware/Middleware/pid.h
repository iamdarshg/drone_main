#ifndef PID_H
#define PID_H
#include <stddef.h>
typedef struct {
    float kp, ki, kd;
    float *inputs;
    size_t n_inputs;
    float integral;
    float prev_error;
} pid_t;
void pid_init(pid_t *pid, float kp, float ki, float kd, float *inputs, size_t n_inputs);
float pid_update(pid_t *pid, float setpoint, float dt);
#endif
