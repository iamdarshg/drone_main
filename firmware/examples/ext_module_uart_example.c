// ext_module_uart_example.c - Example usage of modular UART external module interface
#include "ext_module_uart.h"
#include <stdio.h>

void my_module_rx_cb(const uint8_t *data, size_t len) {
    printf("[MyModule] RX: ");
    for (size_t i = 0; i < len; ++i) printf("%02X ", data[i]);
    printf("\n");
}

int main(void) {
    ext_module_uart_t my_module = {
        .module_id = 1,
        .rx_callback = my_module_rx_cb
    };
    if (ext_module_uart_register(&my_module) == 0) {
        printf("Module registered!\n");
    }
    // Simulate polling (would be called in main loop)
    ext_module_uart_poll();
    // Send data to module (in real use, this would go out over UART)
    uint8_t msg[] = {0xAA, 0xBB, 0xCC};
    ext_module_uart_send(1, msg, sizeof(msg));
    // Unregister
    ext_module_uart_unregister(1);
    return 0;
}
