// pid_example.c - Example: Multi-variable PID control
#include "pid.h"
#include <stdio.h>

int main(void) {
    float sensors[3] = {1.0f, 2.0f, 3.0f};
    pid_t pid;
    pid_init(&pid, 1.0f, 0.1f, 0.05f, sensors, 3);
    float setpoint = 10.0f;
    float dt = 0.01f;
    for (int i = 0; i < 10; ++i) {
        float output = pid_update(&pid, setpoint, dt);
        printf("PID output at step %d: %f\n", i, output);
        sensors[0] += 0.1f; sensors[1] += 0.05f; sensors[2] += 0.02f;
    }
    return 0;
}
