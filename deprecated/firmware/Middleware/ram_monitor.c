// ram_monitor.c - RAM usage monitoring (simple wrapper)
#include "ram_monitor.h"
#include <stdint.h>
#include "FreeRTOS.h"

uint32_t ram_monitor_get_free(void) {
    return xPortGetFreeHeapSize();
}
uint32_t ram_monitor_get_total(void) {
    return configTOTAL_HEAP_SIZE;
}
