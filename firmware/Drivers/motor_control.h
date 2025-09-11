#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

int motor_control_init(void);
void motor_control_set_outputs(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif // MOTOR_CONTROL_H
