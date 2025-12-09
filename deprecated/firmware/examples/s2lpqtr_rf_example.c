// s2lpqtr_rf_example.c - Example: Hardware S2-LPQTR comms with Hamming code
#include "rf_s2lpqtr.h"
#include <stdio.h>

int main(void) {
    uint8_t tx_data[4] = {0x1, 0x2, 0x3, 0x4};
    uint8_t rx_data[4] = {0};
    uint8_t status = 0;
    // Send data to base station
    rf_send_hamming(tx_data, 4);
    // Get status
    rf_get_status(&status);
    printf("S2-LPQTR status: 0x%02X\n", status);
    // Simulate receive (replace with real receive in hardware)
    rf_receive_hamming(rx_data, 4);
    for (int i = 0; i < 4; ++i) {
        printf("Sent: 0x%X, Received: 0x%X\n", tx_data[i], rx_data[i]);
    }
    return 0;
}
