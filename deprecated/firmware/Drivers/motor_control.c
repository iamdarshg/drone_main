#include "motor_control.h"

int motor_control_init(void) {
    // Minimal stub: nothing to init in host build
    return 0;
}

void motor_control_set_outputs(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    // stub: just store values to a volatile buffer if needed
    (void)m1; (void)m2; (void)m3; (void)m4;
}
