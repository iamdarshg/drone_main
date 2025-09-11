/*
 * spi_interface.h - NEW FILE for firmware/Middleware/spi_interface.h
 * Header for SPI information interface module
 */
#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// SPI register map for external modules to query
#define SPI_REG_HEADER          0x00
#define SPI_REG_SYSTEM_STATUS   0x01
#define SPI_REG_GPS_LAT         0x02
#define SPI_REG_GPS_LON         0x03
#define SPI_REG_GPS_ALT         0x04
#define SPI_REG_GPS_STATUS      0x05
#define SPI_REG_CRASH_DATA      0x06
#define SPI_REG_CHECKSUM        0x07
#define SPI_REG_ALL_DATA        0xFF

// Magic header to identify valid data
#define SPI_HEADER_MAGIC        0x44524E45  // "DRNE" in ASCII

// Data structure exposed via SPI
typedef struct {
    uint32_t header;            // Magic header (0x44524E45)
    uint32_t timestamp;         // System timestamp
    
    // System status
    uint8_t system_status;      // 0=error, 1=healthy
    uint8_t degradation_mode;   // Current degradation level
    uint8_t reserved1[2];
    
    // GPS information
    float gps_latitude;         // Current latitude (degrees)
    float gps_longitude;        // Current longitude (degrees)  
    float gps_altitude;         // Current altitude (meters)
    uint8_t gps_satellites;     // Number of satellites
    uint8_t gps_valid;          // 1=valid GPS fix
    uint8_t reserved2[2];
    float distance_to_home;     // Distance to home (meters)
    
    // Crash detection
    uint8_t crash_detected;     // 1=crash data available
    uint8_t crash_reason;       // Reason code for crash
    uint8_t reserved3[2];
    float last_crash_lat;       // Last crash latitude
    float last_crash_lon;       // Last crash longitude
    
    uint8_t checksum;           // XOR checksum of all data
    uint8_t reserved4[3];
} __attribute__((packed)) spi_interface_data_t;

// Function prototypes
void spi_interface_init(void);
void spi_interface_update(void);
const spi_interface_data_t* spi_interface_get_data(void);
bool spi_interface_data_ready(void);

// Internal functions
void spi_send_register_data(uint8_t reg);
void spi_send_bytes(const uint8_t *data, int count);
void spi_send_byte(uint8_t data);
uint8_t calculate_checksum(const uint8_t *data, size_t length);

#endif