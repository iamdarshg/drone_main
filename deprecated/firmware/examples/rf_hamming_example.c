// rf_hamming_example.c - Example: Send/receive data with Hamming code over RF
#include "rf_s2lpqtr.h"
#include <stdio.h>

int main(void) {
    uint8_t tx_data[4] = {0x1, 0x2, 0x3, 0x4};
    uint8_t rx_data[4] = {0};
    // Send data to base station
    rf_send_hamming(tx_data, 4);
    // Simulate receive (loopback for demo)
    rf_receive_hamming(rx_data, 4);
    for (int i = 0; i < 4; ++i) {
        printf("Sent: 0x%X, Received: 0x%X\n", tx_data[i], rx_data[i]);
    }
    return 0;
}
