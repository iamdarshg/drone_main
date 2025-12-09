// Example: Using LVDS comms stub
#include "lvds.h"
#include <stdio.h>

int main(void) {
    lvds_init();
    uint8_t tx_data[4] = {1,2,3,4};
    lvds_send(tx_data, 4);
    uint8_t rx_data[4];
    lvds_receive(rx_data, 4);
    printf("Received: %d %d %d %d\n", rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
    return 0;
}
