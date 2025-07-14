// core_monitor.c - Monitor core and RAM usage (LPC4330)
#include "core_monitor.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

// Returns stack high water mark for a FreeRTOS task (in words)
uint32_t core_monitor_get_stack_highwater(TaskHandle_t task) {
    return uxTaskGetStackHighWaterMark(task);
}

// Returns heap free size (in bytes)
uint32_t core_monitor_get_heap_free(void) {
    return xPortGetFreeHeapSize();
}

// Returns total heap size (in bytes)
uint32_t core_monitor_get_heap_total(void) {
    return configTOTAL_HEAP_SIZE;
}

// Returns which core is running (0 = M0, 1 = M4)
uint32_t core_monitor_get_core_id(void) {
    uint32_t id;
    __asm volatile ("MRS %0, IPSR" : "=r" (id));
    return (id & 0x1) ? 1 : 0;
}
