// ext_module_uart.c - Implementation of modular UART interface for external modules
#include "ext_module_uart.h"
#include "ext_uart.h" // Assumed to provide low-level UART functions
#include <string.h>

#define MAX_EXT_MODULES 8

static ext_module_uart_t ext_modules[MAX_EXT_MODULES];
static uint8_t ext_module_count = 0;

int ext_module_uart_register(const ext_module_uart_t *module) {
    if (!module || ext_module_count >= MAX_EXT_MODULES) return -1;
    for (uint8_t i = 0; i < ext_module_count; ++i) {
        if (ext_modules[i].module_id == module->module_id) return -2; // Already registered
    }
    ext_modules[ext_module_count++] = *module;
    return 0;
}

int ext_module_uart_send(uint8_t module_id, const uint8_t *data, size_t len) {
    // For now, all modules share the same UART; module_id is for logical routing
    if (!data || len == 0) return -1;
    return ext_uart_write(data, len); // ext_uart_write returns bytes sent or <0 on error
}

void ext_module_uart_poll(void) {
    uint8_t rx_buf[128];
    int rx_len = ext_uart_read(rx_buf, sizeof(rx_buf));
    if (rx_len > 0) {
        // Dispatch to all registered modules (could be improved with framing/protocol)
        for (uint8_t i = 0; i < ext_module_count; ++i) {
            if (ext_modules[i].rx_callback) {
                ext_modules[i].rx_callback(rx_buf, rx_len);
            }
        }
    }
}

int ext_module_uart_unregister(uint8_t module_id) {
    for (uint8_t i = 0; i < ext_module_count; ++i) {
        if (ext_modules[i].module_id == module_id) {
            for (uint8_t j = i; j < ext_module_count - 1; ++j) {
                ext_modules[j] = ext_modules[j + 1];
            }
            --ext_module_count;
            return 0;
        }
    }
    return -1;
}
