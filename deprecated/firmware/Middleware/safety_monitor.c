#include "safety_monitor.h"

int safety_monitor_init(void) {
    // minimal init
    return 0;
}

safety_status_t safety_monitor_check(void) {
    // Always OK in stub
    return SAFETY_OK;
}
