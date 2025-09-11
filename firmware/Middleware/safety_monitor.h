#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdbool.h>

typedef enum { SAFETY_OK = 0, SAFETY_WARN, SAFETY_FAIL } safety_status_t;

int safety_monitor_init(void);
safety_status_t safety_monitor_check(void);

#endif // SAFETY_MONITOR_H
