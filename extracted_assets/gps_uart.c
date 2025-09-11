/*
 * gps_uart.c - Enhanced GPS implementation for NEO-6M with RTH support
 * 
 * NEW FEATURES:
 * - Complete NMEA sentence parsing (GGA, RMC, VTG)
 * - RTH (Return-to-Home) navigation calculations
 * - Home position management
 * - GPS health monitoring
 */
#include "gps_uart.h"
#include "ext_uart.h"
#include "logging.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef GPS_UART_PORT
#define GPS_UART_PORT 2
#endif
#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD 9600
#endif

// GPS data storage
static gps_data_t current_gps_data = {0};
static char gps_rx_buffer[256];
static bool gps_initialized = false;

// NMEA parsing functions
static int parse_nmea_gga(const char *sentence, gps_data_t *gps);
static int parse_nmea_rmc(const char *sentence, gps_data_t *gps);
static int parse_nmea_vtg(const char *sentence, gps_data_t *gps);
static uint8_t calculate_nmea_checksum(const char *sentence);
static bool validate_nmea_checksum(const char *sentence);
static double nmea_to_decimal(double raw_coord, char direction);

void gps_uart_init(void) {
    log_info("Initializing GPS UART for NEO-6M...");
    
    ext_uart_init(GPS_UART_PORT, GPS_UART_BAUD);
    
    // Initialize GPS data structure
    memset(&current_gps_data, 0, sizeof(current_gps_data));
    current_gps_data.fix_quality = 0;
    current_gps_data.satellites = 0;
    current_gps_data.gps_valid = false;
    current_gps_data.home_set = false;
    
    gps_initialized = true;
    log_info("GPS UART initialized successfully");
}

int gps_uart_read_line(char *out, size_t maxlen) {
    size_t i = 0;
    char c = 0;
    
    while (i < maxlen - 1) {
        if (ext_uart_read(GPS_UART_PORT, (uint8_t *)&c, 1) != 1) {
            break;
        }
        
        if (c == '\n' || c == '\r') {
            break;
        }
        
        out[i++] = c;
    }
    
    out[i] = '\0';
    return (int)i;
}

int gps_uart_parse_sentence(const char *sentence, gps_data_t *gps_data) {
    if (!sentence || !gps_data || !gps_initialized) {
        return -1;
    }
    
    // Validate NMEA checksum
    if (!validate_nmea_checksum(sentence)) {
        return -1;
    }
    
    // Parse different NMEA sentence types
    if (strncmp(sentence, "$GPGGA", 6) == 0) {
        return parse_nmea_gga(sentence, gps_data);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0) {
        return parse_nmea_rmc(sentence, gps_data);
    } else if (strncmp(sentence, "$GPVTG", 6) == 0) {
        return parse_nmea_vtg(sentence, gps_data);
    }
    
    return -1; // Unsupported sentence type
}

int gps_uart_parse_gga(const char *nmea, float *lat, float *lon) {
    // Legacy function for backward compatibility
    gps_data_t temp_gps;
    if (parse_nmea_gga(nmea, &temp_gps) == 0) {
        *lat = (float)temp_gps.latitude;
        *lon = (float)temp_gps.longitude;
        return 0;
    }
    return -1;
}

gps_data_t* gps_uart_get_data(void) {
    return gps_initialized ? &current_gps_data : NULL;
}

int gps_uart_update(void) {
    if (!gps_initialized) {
        return -1;
    }
    
    char sentence[128];
    int bytes_read = gps_uart_read_line(sentence, sizeof(sentence));
    
    if (bytes_read > 0) {
        if (gps_uart_parse_sentence(sentence, &current_gps_data) == 0) {
            // Update GPS validity
            current_gps_data.gps_valid = (current_gps_data.satellites >= 4 && 
                                         current_gps_data.fix_quality > 0);
            return 1; // Successfully parsed
        }
    }
    
    return 0; // No new data or parse failure
}

void gps_uart_set_home_position(void) {
    if (!gps_initialized || !current_gps_data.gps_valid) {
        log_warning("Cannot set home - GPS not valid");
        return;
    }
    
    current_gps_data.home_lat = current_gps_data.latitude;
    current_gps_data.home_lon = current_gps_data.longitude;
    current_gps_data.home_set = true;
    
    log_info("Home position set: %.6f, %.6f", 
             current_gps_data.home_lat, current_gps_data.home_lon);
}

void gps_uart_calculate_home_vector(void) {
    if (!current_gps_data.home_set || !current_gps_data.gps_valid) {
        current_gps_data.distance_to_home = 0.0f;
        current_gps_data.bearing_to_home = 0.0f;
        return;
    }
    
    // Calculate distance and bearing using Haversine formula
    double lat1 = current_gps_data.home_lat * M_PI / 180.0;
    double lon1 = current_gps_data.home_lon * M_PI / 180.0;
    double lat2 = current_gps_data.latitude * M_PI / 180.0;
    double lon2 = current_gps_data.longitude * M_PI / 180.0;
    
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    
    // Haversine distance calculation
    double a = sin(dlat/2) * sin(dlat/2) + 
               cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    current_gps_data.distance_to_home = 6371000.0f * c; // Earth radius in meters
    
    // Bearing calculation
    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    current_gps_data.bearing_to_home = atan2(y, x) * 180.0 / M_PI;
    
    // Normalize bearing to 0-360 degrees
    if (current_gps_data.bearing_to_home < 0) {
        current_gps_data.bearing_to_home += 360.0f;
    }
}

bool gps_uart_is_home_reached(float threshold_meters) {
    if (!current_gps_data.home_set) {
        return false;
    }
    
    return (current_gps_data.distance_to_home <= threshold_meters);
}

// ==================== NMEA PARSING FUNCTIONS ====================

static int parse_nmea_gga(const char *sentence, gps_data_t *gps) {
    // $GPGGA,hhmmss.ss,ddmm.mmmmm,N/S,dddmm.mmmmm,E/W,q,nn,h.h,a.a,M,g.g,M,d.d,nnnn*hh
    
    char time_str[12];
    double raw_lat, raw_lon;
    char ns, ew;
    int quality, satellites;
    float hdop, altitude;
    
    int parsed = sscanf(sentence, 
        "$GPGGA,%11[^,],%lf,%c,%lf,%c,%d,%d,%f,%f,M",
        time_str, &raw_lat, &ns, &raw_lon, &ew, &quality, &satellites, &hdop, &altitude);
    
    if (parsed >= 9) {
        gps->latitude = nmea_to_decimal(raw_lat, ns);
        gps->longitude = nmea_to_decimal(raw_lon, ew);
        gps->fix_quality = quality;
        gps->satellites = satellites;
        gps->altitude = altitude;
        gps->hdop = hdop;
        
        // Update home vector if home is set
        if (gps->home_set) {
            gps_uart_calculate_home_vector();
        }
        
        return 0;
    }
    
    return -1;
}

static int parse_nmea_rmc(const char *sentence, gps_data_t *gps) {
    // $GPRMC,hhmmss.ss,A,ddmm.mmmmm,N/S,dddmm.mmmmm,E/W,s.s,c.c,ddmmyy,d.d,E/W*hh
    
    char time_str[12], status, date_str[8];
    double raw_lat, raw_lon;
    char ns, ew;
    float speed, course;
    
    int parsed = sscanf(sentence,
        "$GPRMC,%11[^,],%c,%lf,%c,%lf,%c,%f,%f,%7[^,]",
        time_str, &status, &raw_lat, &ns, &raw_lon, &ew, &speed, &course, date_str);
    
    if (parsed >= 9 && status == 'A') { // 'A' means valid fix
        gps->latitude = nmea_to_decimal(raw_lat, ns);
        gps->longitude = nmea_to_decimal(raw_lon, ew);
        gps->speed_kmh = speed * 1.852f; // Convert knots to km/h
        gps->course_degrees = course;
        
        // Update home vector if home is set
        if (gps->home_set) {
            gps_uart_calculate_home_vector();
        }
        
        return 0;
    }
    
    return -1;
}

static int parse_nmea_vtg(const char *sentence, gps_data_t *gps) {
    // $GPVTG,c.c,T,c.c,M,s.s,N,s.s,K*hh
    
    float course_true, course_mag, speed_knots, speed_kmh;
    
    int parsed = sscanf(sentence,
        "$GPVTG,%f,T,%f,M,%f,N,%f,K",
        &course_true, &course_mag, &speed_knots, &speed_kmh);
    
    if (parsed >= 4) {
        gps->course_degrees = course_true;
        gps->speed_kmh = speed_kmh;
        return 0;
    }
    
    return -1;
}

static uint8_t calculate_nmea_checksum(const char *sentence) {
    uint8_t checksum = 0;
    
    // Skip the '$' and calculate checksum until '*'
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
        checksum ^= sentence[i];
    }
    
    return checksum;
}

static bool validate_nmea_checksum(const char *sentence) {
    // Find the '*' character
    const char *asterisk = strchr(sentence, '*');
    if (!asterisk) {
        return false; // No checksum present
    }
    
    // Calculate checksum
    uint8_t calculated = calculate_nmea_checksum(sentence);
    
    // Parse provided checksum
    uint8_t provided;
    if (sscanf(asterisk + 1, "%2hhx", &provided) != 1) {
        return false;
    }
    
    return (calculated == provided);
}

static double nmea_to_decimal(double raw_coord, char direction) {
    // Convert DDMM.MMMMM to decimal degrees
    int degrees = (int)(raw_coord / 100);
    double minutes = raw_coord - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    
    // Apply direction
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

void gps_uart_print_status(void) {
    if (!gps_initialized) {
        log_info("GPS not initialized");
        return;
    }
    
    log_info("GPS Status:");
    log_info("  Position: %.6f, %.6f", current_gps_data.latitude, current_gps_data.longitude);
    log_info("  Satellites: %d, Fix: %d", current_gps_data.satellites, current_gps_data.fix_quality);
    log_info("  Altitude: %.1fm, Speed: %.1f km/h", current_gps_data.altitude, current_gps_data.speed_kmh);
    log_info("  Valid: %s", current_gps_data.gps_valid ? "YES" : "NO");
    
    if (current_gps_data.home_set) {
        log_info("  Home: %.6f, %.6f", current_gps_data.home_lat, current_gps_data.home_lon);
        log_info("  Distance to home: %.1fm, Bearing: %.1f°", 
                 current_gps_data.distance_to_home, current_gps_data.bearing_to_home);
    } else {
        log_info("  Home position not set");
    }
}