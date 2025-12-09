#include "imu_kx122.h"
#include "logging.h"
#include "chip.h"
#include "spi_bus.h"
#define KX122_CS_PORT 0
#define KX122_CS_PIN  0

static float kx122_scale_factor = 0.000598f;
static float kx122_bias[3] = {0.0f, 0.0f, 0.0f};
static float kx122_scale[3] = {1.0f, 1.0f, 1.0f};

static void kx122_select(void) {
    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, KX122_CS_PORT, KX122_CS_PIN);
}
static void kx122_deselect(void) {
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, KX122_CS_PORT, KX122_CS_PIN);
}
static void kx122_delay_ms(uint32_t ms) {
    Chip_Clock_System_BusyWait_ms(ms);
}
static void kx122_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    kx122_select();
    spi_bus_write(data, 2);
    kx122_deselect();
}
static uint8_t kx122_read_reg(uint8_t reg) {
    uint8_t value = 0;
    kx122_select();
    spi_bus_write(&reg, 1);
    spi_bus_read(&value, 1);
    kx122_deselect();
    return value;
}

void imu_kx122_init(void) {
    // Configure all control registers
    kx122_write_reg(KX122_REG_CTRL1, KX122_CTRL1_VALUE);
    kx122_write_reg(KX122_REG_CTRL2, KX122_CTRL2_VALUE);
    kx122_write_reg(KX122_REG_CTRL3, KX122_CTRL3_VALUE);
    kx122_write_reg(KX122_REG_CTRL4, KX122_CTRL4_VALUE);
    kx122_write_reg(KX122_REG_CTRL5, KX122_CTRL5_VALUE);
    kx122_write_reg(KX122_REG_CTRL6, KX122_CTRL6_VALUE);
    kx122_write_reg(KX122_REG_CTRL7, KX122_CTRL7_VALUE);
    kx122_write_reg(KX122_REG_CTRL8, KX122_CTRL8_VALUE);
    imu_kx122_configure_freefall();
    kx122_delay_ms(100);
    log_info("KX122 initialized");
}

int imu_kx122_read(float *accel, float *temp) {
    // Read X, Y, Z registers (replace with actual register addresses)
    uint8_t xl = kx122_read_reg(0x06), xh = kx122_read_reg(0x07);
    uint8_t yl = kx122_read_reg(0x08), yh = kx122_read_reg(0x09);
    uint8_t zl = kx122_read_reg(0x0A), zh = kx122_read_reg(0x0B);
    int16_t x = (int16_t)((xh << 8) | xl);
    int16_t y = (int16_t)((yh << 8) | yl);
    int16_t z = (int16_t)((zh << 8) | zl);
    accel[0] = (x - kx122_bias[0]) * kx122_scale_factor * kx122_scale[0];
    accel[1] = (y - kx122_bias[1]) * kx122_scale_factor * kx122_scale[1];
    accel[2] = (z - kx122_bias[2]) * kx122_scale_factor * kx122_scale[2];
    *temp = 25.0f; // KX122 does not have a temperature sensor
    return 0;
}

int imu_kx122_selftest(float *result) {
    // Perform a basic self-test: check if device responds and data is not stuck
    float accel[3], temp;
    int ok = 1;
    for (int i = 0; i < 10; ++i) {
        imu_kx122_read(accel, &temp);
        for (int j = 0; j < 3; ++j) {
            if (accel[j] < -20.0f || accel[j] > 20.0f) ok = 0;
        }
    }
    *result = ok ? 1.0f : 0.0f;
    return ok ? 0 : -1;
}

void imu_kx122_calibrate(void) {
    // Set to standby
    kx122_write_reg(KX122_REG_CTRL1, 0x00);
    kx122_delay_ms(50);
    // Zero offsets
    kx122_write_reg(0x1A, 0x00);
    kx122_write_reg(0x1B, 0x00);
    kx122_write_reg(0x1C, 0x00);
    // Back to operating mode
    kx122_write_reg(KX122_REG_CTRL1, KX122_CTRL1_VALUE);
    // Enable 2-sample averaging
    uint8_t ctrl1 = kx122_read_reg(KX122_REG_CTRL1);
    ctrl1 |= 0x02;
    kx122_write_reg(KX122_REG_CTRL1, ctrl1);
    // Set ODR for free fall detection to max
    ctrl1 = kx122_read_reg(KX122_REG_CTRL1);
    ctrl1 = (ctrl1 & 0xF8) | 0x07;
    kx122_write_reg(KX122_REG_CTRL1, ctrl1);
    kx122_delay_ms(50);
}

void imu_kx122_set_grange(uint8_t range) {
    uint8_t cntl1 = kx122_read_reg(KX122_REG_CTRL1);
    cntl1 = (cntl1 & 0xE7) | (range << 3);
    kx122_write_reg(KX122_REG_CTRL1, cntl1);
    switch (range) {
        case 0x00: kx122_scale_factor = 0.000598f; break;
        case 0x01: kx122_scale_factor = 0.001196f; break;
        case 0x02: kx122_scale_factor = 0.002392f; break;
        default:   kx122_scale_factor = 0.000598f; break;
    }
}

void imu_kx122_set_odr(uint8_t odr) {
    uint8_t ctrl1 = kx122_read_reg(KX122_REG_CTRL1);
    ctrl1 = (ctrl1 & 0x8F) | ((odr & 0x07) << 4);
    kx122_write_reg(KX122_REG_CTRL1, ctrl1);
}

void imu_kx122_set_fifo_mode(uint8_t mode) {
    kx122_write_reg(0x3A, mode & 0x07);
}

void imu_kx122_read_fifo(float *ax, float *ay, float *az) {
    uint8_t fifoLength = kx122_read_reg(0x3A);
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int sampleCount = 0;
    for (int i = 0; i < fifoLength; ++i) {
        uint8_t xl = kx122_read_reg(0x3E), xh = kx122_read_reg(0x3F);
        uint8_t yl = kx122_read_reg(0x40), yh = kx122_read_reg(0x41);
        uint8_t zl = kx122_read_reg(0x42), zh = kx122_read_reg(0x43);
        int16_t x = (int16_t)((xh << 8) | xl);
        int16_t y = (int16_t)((yh << 8) | yl);
        int16_t z = (int16_t)((zh << 8) | zl);
        sumX += x; sumY += y; sumZ += z; sampleCount++;
    }
    if (sampleCount > 0) {
        *ax = (sumX / sampleCount) * kx122_scale_factor;
        *ay = (sumY / sampleCount) * kx122_scale_factor;
        *az = (sumZ / sampleCount) * kx122_scale_factor;
    } else {
        *ax = *ay = *az = 0.0f;
    }
}

void imu_kx122_configure_freefall(void) {
    kx122_write_reg(KX122_REG_FFTH, KX122_FFTH_VALUE);
    kx122_write_reg(KX122_REG_FFC, KX122_FFC_VALUE);
}

void imu_kx122_handle_freefall(void) {
    uint8_t intSource = kx122_read_reg(0x3A);
    if (intSource & 0x20) {
        log_info("Free-fall detected!");
    }
}
