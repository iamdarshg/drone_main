/*
 * spi_interface.c - NEW FILE for firmware/Middleware/spi_interface.c
 * Simple SPI interface to provide system information to external modules
 * 
 * This module creates an SPI slave interface that other systems can query
 * for drone status, GPS coordinates, sensor health, and crash information.
 */

#include "spi_interface.h"
#include "safety_system.h"
#include "gps_uart.h"
#include "Config/config.h"
#include "logging.h"
#include <string.h>

#ifdef HOST_BUILD
#include <stdio.h>

void spi_interface_init(void) {
    printf("SPI Interface: Initialized (HOST BUILD)\n");
}

void spi_interface_update(void) {
    // Host simulation - print status periodically
    static int update_count = 0;
    update_count++;
    
    if (update_count % 100 == 0) {
        printf("SPI Interface: Status update #%d\n", update_count / 100);
    }
}

#else
// Real LPC4330 SPI slave implementation

#include <stdint.h>

// LPC4330 SSP1 registers (used as SPI slave)
#define LPC_SSP1_BASE       0x40042000
typedef struct {
    volatile uint32_t CR0;      // Control register 0
    volatile uint32_t CR1;      // Control register 1  
    volatile uint32_t DR;       // Data register
    volatile uint32_t SR;       // Status register
    volatile uint32_t CPSR;     // Clock prescale register
    volatile uint32_t IMSC;     // Interrupt mask register
    volatile uint32_t RIS;      // Raw interrupt status
    volatile uint32_t MIS;      // Masked interrupt status
    volatile uint32_t ICR;      // Interrupt clear register
} LPC_SSP_T;

#define LPC_SSP1 ((LPC_SSP_T *) LPC_SSP1_BASE)

// SPI interface state
static spi_interface_data_t interface_data = {0};
static volatile bool data_ready = false;
static volatile uint8_t current_register = 0;

void spi_interface_init(void) {
    // Configure SSP1 as SPI slave
    
    // Enable SSP1 clock
    // This would normally be done through CCU (Clock Control Unit)
    
    // Configure SSP1 for SPI slave mode
    LPC_SSP1->CR0 = (0x07 << 0) |    // 8-bit data
                    (0x00 << 4) |    // SPI frame format
                    (0x00 << 6) |    // CPOL = 0
                    (0x00 << 7);     // CPHA = 0
    
    LPC_SSP1->CR1 = (0x00 << 0) |    // Normal operation (not loopback)
                    (0x01 << 1) |    // Enable SSP
                    (0x01 << 2) |    // Slave mode
                    (0x00 << 3);     // Slave output disable = 0
    
    // Set clock prescaler (not used in slave mode)
    LPC_SSP1->CPSR = 2;
    
    // Enable interrupts
    LPC_SSP1->IMSC = (1 << 2) |     // RX FIFO half full
                     (1 << 3);      // RX timeout
    
    // Clear any pending interrupts
    LPC_SSP1->ICR = 0x03;
    
    // Enable SSP1 interrupt in NVIC
    extern void nvic_enable_irq(int irq);
    nvic_enable_irq(23); // SSP1_IRQn
    
    log_info("SPI interface initialized (SSP1 slave mode)");
}

void spi_interface_update(void) {
    // Update interface data with latest system information
    
    // System status
    interface_data.header = SPI_HEADER_MAGIC;
    interface_data.system_status = safety_is_system_healthy() ? 1 : 0;
    interface_data.degradation_mode = safety_get_degradation_mode();
    interface_data.timestamp = 0; // Would get from RTC
    
    // GPS data
    gps_data_t *gps = gps_uart_get_data();
    if (gps && gps->gps_valid) {
        interface_data.gps_latitude = (float)gps->latitude;
        interface_data.gps_longitude = (float)gps->longitude;
        interface_data.gps_altitude = gps->altitude;
        interface_data.gps_satellites = gps->satellites;
        interface_data.distance_to_home = gps->distance_to_home;
        interface_data.gps_valid = 1;
    } else {
        interface_data.gps_valid = 0;
    }
    
    // Crash detection data
    if (config_has_crash_data()) {
        interface_data.crash_detected = 1;
        // Last crash coordinates would be read from config
        interface_data.last_crash_lat = 0.0f;  // TODO: Get from config
        interface_data.last_crash_lon = 0.0f;  // TODO: Get from config
        interface_data.crash_reason = 0;       // TODO: Get from config
    } else {
        interface_data.crash_detected = 0;
    }
    
    // Calculate simple checksum
    interface_data.checksum = calculate_checksum((uint8_t*)&interface_data, 
                                                sizeof(interface_data) - sizeof(interface_data.checksum));
    
    data_ready = true;
}

// SPI slave interrupt handler
void SSP1_IRQHandler(void) {
    uint32_t status = LPC_SSP1->MIS;
    
    if (status & (1 << 2)) {  // RX FIFO half full
        // Process received data
        while (LPC_SSP1->SR & (1 << 2)) {  // RX FIFO not empty
            uint8_t received = LPC_SSP1->DR & 0xFF;
            
            // First byte is register address
            current_register = received;
            
            // Send response based on register
            spi_send_register_data(current_register);
        }
    }
    
    if (status & (1 << 3)) {  // RX timeout
        // Handle timeout
    }
    
    // Clear interrupts
    LPC_SSP1->ICR = status;
}

void spi_send_register_data(uint8_t reg) {
    uint8_t *data_ptr = (uint8_t*)&interface_data;
    
    switch (reg) {
        case SPI_REG_HEADER:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, header), 4);
            break;
            
        case SPI_REG_SYSTEM_STATUS:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, system_status), 4);
            break;
            
        case SPI_REG_GPS_LAT:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, gps_latitude), 4);
            break;
            
        case SPI_REG_GPS_LON:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, gps_longitude), 4);
            break;
            
        case SPI_REG_GPS_ALT:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, gps_altitude), 4);
            break;
            
        case SPI_REG_GPS_STATUS:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, gps_valid), 4);
            break;
            
        case SPI_REG_CRASH_DATA:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, crash_detected), 16);
            break;
            
        case SPI_REG_CHECKSUM:
            spi_send_bytes(data_ptr + offsetof(spi_interface_data_t, checksum), 4);
            break;
            
        case SPI_REG_ALL_DATA:
            // Send entire data structure
            spi_send_bytes(data_ptr, sizeof(interface_data));
            break;
            
        default:
            // Send error code
            spi_send_byte(0xFF);
            break;
    }
}

void spi_send_bytes(const uint8_t *data, int count) {
    for (int i = 0; i < count; i++) {
        spi_send_byte(data[i]);
    }
}

void spi_send_byte(uint8_t data) {
    // Wait for TX FIFO not full
    while (!(LPC_SSP1->SR & (1 << 1)));
    
    // Send data
    LPC_SSP1->DR = data;
}

uint8_t calculate_checksum(const uint8_t *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;

}

// Get interface data for other modules
const spi_interface_data_t* spi_interface_get_data(void) {
    return &interface_data;
}

// Check if data is ready
bool spi_interface_data_ready(void) {
    return data_ready;
}

#endif // HOST_BUILD
