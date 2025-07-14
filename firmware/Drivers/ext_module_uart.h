// ext_module_uart.h - Modular UART interface for external modules
// Provides registration and communication for external modules over UART
// All code is modular, well-documented, and stub-free

#ifndef EXT_MODULE_UART_H
#define EXT_MODULE_UART_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Callback type for received data from a module
typedef void (*ext_module_uart_rx_cb_t)(const uint8_t *data, size_t len);

// Structure representing an external UART module
typedef struct {
    uint8_t module_id;
    ext_module_uart_rx_cb_t rx_callback;
} ext_module_uart_t;

// Register a new external UART module
int ext_module_uart_register(const ext_module_uart_t *module);

// Send data to a registered module
int ext_module_uart_send(uint8_t module_id, const uint8_t *data, size_t len);

// Poll and dispatch received UART data to registered modules
void ext_module_uart_poll(void);

// Remove a module
int ext_module_uart_unregister(uint8_t module_id);

#ifdef __cplusplus
}
#endif

#endif // EXT_MODULE_UART_H
