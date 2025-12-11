
#ifndef EXT_UART_H
#define EXT_UART_H

#include <stdint.h>
#include <stddef.h>

// Initialize UART port with given baud rate
void ext_uart_init(uint8_t port, uint32_t baud);

// Write data to UART (default port 0 for now). Returns bytes sent or <0 on error
int ext_uart_write(const uint8_t *data, size_t len);

// Read data from UART port. Returns bytes read or <0 on error
int ext_uart_read(uint8_t port, uint8_t *data, size_t len);

#endif // EXT_UART_H
