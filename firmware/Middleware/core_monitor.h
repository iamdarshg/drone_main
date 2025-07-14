#ifndef CORE_MONITOR_H
#define CORE_MONITOR_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
uint32_t core_monitor_get_stack_highwater(TaskHandle_t task);
uint32_t core_monitor_get_heap_free(void);
uint32_t core_monitor_get_heap_total(void);
uint32_t core_monitor_get_core_id(void);
#endif
