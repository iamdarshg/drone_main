#ifndef WATCHDOG_M0_H
#define WATCHDOG_M0_H
#include <stdint.h>
void m0_watchdog_kick(void);
void m0_watchdog_task(void);
uint32_t m0_get_status(void);
#endif
