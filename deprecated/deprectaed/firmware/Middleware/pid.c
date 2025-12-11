// pid.c - Strong PID controller with multi-variable support
#include "pid.h"
#include <stddef.h>

void pid_init(pid_t *pid, float kp, float ki, float kd, float *inputs, size_t n_inputs) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->n_inputs = n_inputs;
    pid->inputs = inputs;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_t *pid, float setpoint, float dt) {
    float error = setpoint;
    for (size_t i = 0; i < pid->n_inputs; ++i) {
        error -= pid->inputs[i];
    }
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return output;
}
