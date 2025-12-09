#ifndef RF_S2LPQTR_H
#define RF_S2LPQTR_H

#include <stdint.h>

// Send data to base station with Hamming code (full S2-LPQTR implementation)
int rf_send_hamming(const uint8_t *data, uint16_t len);
// Receive data from base station and decode Hamming
int rf_receive_hamming(uint8_t *data, uint16_t maxlen);
// Get S2-LPQTR status
int rf_get_status(uint8_t *status);

#endif
