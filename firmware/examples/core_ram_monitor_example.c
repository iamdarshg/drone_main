// core_ram_monitor_example.c - Example: Monitor core and RAM usage
#include "core_monitor.h"
#include "ram_monitor.h"
#include <stdio.h>

int main(void) {
    printf("Heap free: %lu bytes\n", (unsigned long)ram_monitor_get_free());
    printf("Heap total: %lu bytes\n", (unsigned long)ram_monitor_get_total());
    printf("Core ID: %lu (0=M0, 1=M4)\n", (unsigned long)core_monitor_get_core_id());
    // Stack highwater example (requires FreeRTOS task handle)
    // printf("Stack highwater: %lu words\n", (unsigned long)core_monitor_get_stack_highwater(xTaskGetCurrentTaskHandle()));
    return 0;
}
