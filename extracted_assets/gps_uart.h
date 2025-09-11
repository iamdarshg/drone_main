/*
 * gps_uart.h - Enhanced GPS header with RTH support
 */
#ifndef GPS_UART_H
#define GPS_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Enhanced GPS data structure for RTH functionality
typedef struct {
    // Current position
    double latitude;            // Decimal degrees
    double longitude;           // Decimal degrees
    float altitude;             // Meters above sea level
    
    // Navigation data
    float speed_kmh;            // Speed in km/h
    float course_degrees;       // Course over ground in degrees (0-360)
    
    // GPS quality indicators
    uint8_t satellites;         // Number of satellites in use
    uint8_t fix_quality;        // GPS fix quality (0=invalid, 1=GPS, 2=DGPS)
    float hdop;                 // Horizontal dilution of precision
    
    // Home position for RTH
    double home_lat;            // Home latitude
    double home_lon;            // Home longitude
    bool home_set;              // True if home position is set
    
    // RTH navigation data
    float distance_to_home;     // Distance to home in meters
    float bearing_to_home;      // Bearing to home in degrees (0-360)
    
    // Status flags
    bool gps_valid;             // Overall GPS validity
    uint32_t last_update_time;  // Last successful GPS update (system ticks)
} gps_data_t;

// Function prototypes
void gps_uart_init(void);
int gps_uart_read_line(char *out, size_t maxlen);
int gps_uart_parse_gga(const char *nmea, float *lat, float *lon); // Legacy compatibility
int gps_uart_parse_sentence(const char *sentence, gps_data_t *gps_data);
gps_data_t* gps_uart_get_data(void);
int gps_uart_update(void);

// RTH-specific functions
void gps_uart_set_home_position(void);
void gps_uart_calculate_home_vector(void);
bool gps_uart_is_home_reached(float threshold_meters);
void gps_uart_print_status(void);

#endif