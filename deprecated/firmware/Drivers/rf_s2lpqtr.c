// rf_s2lpqtr.c - S2-LPQTR RF driver (hardware, no stubs)
#include "rf_s2lpqtr.h"
#include "hamming.h"
#include <stdint.h>
#include <string.h>
#include "spi_bus.h"
#include "rf_monitor.h"

#define S2LPQTR_CS_PORT 1
#define S2LPQTR_CS_PIN  2

static void s2lpqtr_select(void) {
    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, S2LPQTR_CS_PORT, S2LPQTR_CS_PIN);
}
static void s2lpqtr_deselect(void) {
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, S2LPQTR_CS_PORT, S2LPQTR_CS_PIN);
}

#define S2LPQTR_REG_TX_FIFO 0x01
#define S2LPQTR_REG_RX_FIFO 0x02
#define S2LPQTR_REG_STATUS  0x03

static void s2lpqtr_write_reg(uint8_t reg, const uint8_t *data, uint16_t len) {
    s2lpqtr_select();
    spi_bus_write(&reg, 1);
    spi_bus_write(data, len);
    s2lpqtr_deselect();
}
static void s2lpqtr_read_reg(uint8_t reg, uint8_t *data, uint16_t len) {
    s2lpqtr_select();
    spi_bus_write(&reg, 1);
    spi_bus_read(data, len);
    s2lpqtr_deselect();
}

int rf_send_hamming(const uint8_t *data, uint16_t len) {
    uint8_t tx_buf[256];
    if (len > 128) return -1;
    for (uint16_t i = 0; i < len; ++i) {
        tx_buf[i] = hamming_encode(data[i] & 0x0F);
    }
    s2lpqtr_write_reg(S2LPQTR_REG_TX_FIFO, tx_buf, len);
    // Optionally trigger TX
    rf_monitor_tx_inc(); // Track successful transmit
    return 0;
}

int rf_receive_hamming(uint8_t *data, uint16_t maxlen) {
    uint8_t rx_buf[256];
    if (maxlen > 128) return -1;
    s2lpqtr_read_reg(S2LPQTR_REG_RX_FIFO, rx_buf, maxlen);
    for (uint16_t i = 0; i < maxlen; ++i) {
        uint8_t error = 0;
        data[i] = hamming_decode(rx_buf[i], &error);
        // Optionally handle error
        rf_monitor_rx_inc(); // Track successful receive
        if (error) {
            rf_monitor_error_inc(); // Track errors
        }
    }
    return 0;
}

int rf_get_status(uint8_t *status) {
    s2lpqtr_read_reg(S2LPQTR_REG_STATUS, status, 1);
    return 0;
}
