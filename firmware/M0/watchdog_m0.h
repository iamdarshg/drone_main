#ifndef WATCHDOG_M0_H
#define WATCHDOG_M0_H
#include <stdint.h>
void m0_watchdog_kick(void);
void m0_watchdog_task(void);
uint32_t m0_get_status(void);
// PID/task monitoring
void m0_monitor_pid_loop(uint32_t last_pid_tick, uint32_t now, uint32_t max_delay);
void m0_set_task_priority_flag(uint32_t flag);
void m0_clear_task_priority_flag(uint32_t flag);
uint32_t m0_get_task_priority_flags(void);
uint32_t m0_get_pid_delay_flag(void);
#endif
