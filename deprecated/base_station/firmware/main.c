// main.c - Base station firmware entry point
#include "rf_s2lpqtr.h"
#include "usb_bridge.h"
#include "crc32.h"
#include "hamming.h"
#include "logging.h"
#include <stdio.h>

int main(void) {
    log_info("Base station firmware starting");
    // Initialize RF, USB, and error correction
    rf_s2lpqtr_init();
    usb_bridge_init();
    // Main loop: receive from RF, forward to PC, and vice versa
    while (1) {
        uint8_t rx_buf[64];
        int n = rf_receive_hamming(rx_buf, sizeof(rx_buf));
        if (n > 0) {
            usb_bridge_send(rx_buf, n);
        }
        // Check for PC commands
        uint8_t pc_buf[64];
        int m = usb_bridge_receive(pc_buf, sizeof(pc_buf));
        if (m > 0) {
            rf_send_hamming(pc_buf, m);
        }
    }
    return 0;
}
