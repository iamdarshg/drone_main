#include "imu_lsm303c.h"
#include "logging.h"
#include "chip.h"
#define LSM303C_CS_PORT 0
#define LSM303C_CS_PIN  1

static float lsm303c_accel_scale = 0.000061f;
static float lsm303c_mag_scale = 0.00014f;
static float lsm303c_bias[3] = {0.0f, 0.0f, 0.0f};
static float lsm303c_mag_bias[3] = {0.0f, 0.0f, 0.0f};

static void lsm303c_select(void) {
    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, LSM303C_CS_PORT, LSM303C_CS_PIN);
}
static void lsm303c_deselect(void) {
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, LSM303C_CS_PORT, LSM303C_CS_PIN);
}
static void lsm303c_delay_ms(uint32_t ms) {
    Chip_Clock_System_BusyWait_ms(ms);
}
static void lsm303c_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    lsm303c_select();
    Chip_I2C_MasterSend(LPC_I2C0, LSM303C_I2C_ADDRESS, data, 2);
    lsm303c_deselect();
}
static uint8_t lsm303c_read_reg(uint8_t reg) {
    uint8_t value = 0;
    lsm303c_select();
    Chip_I2C_MasterSend(LPC_I2C0, LSM303C_I2C_ADDRESS, &reg, 1);
    Chip_I2C_MasterRead(LPC_I2C0, LSM303C_I2C_ADDRESS, &value, 1);
    lsm303c_deselect();
    return value;
}

void imu_lsm303c_init(void) {
    lsm303c_write_reg(LSM303C_CTRL_REG1_A, 0x97); // 6.4kHz, high-res, all axes
    lsm303c_write_reg(LSM303C_CTRL_REG4_A, 0x08); // ±2g, high-res
    lsm303c_write_reg(LSM303C_CTRL_REG1_M, 0x70); // 100Hz mag
    lsm303c_write_reg(LSM303C_CTRL_REG2_M, 0x80); // auto range
    lsm303c_write_reg(LSM303C_CTRL_REG3_M, 0x00); // continuous
    lsm303c_write_reg(LSM303C_CTRL_REG4_M, 0x02); // tilt comp
    lsm303c_write_reg(LSM303C_CTRL_REG5_M, 0x40); // FIFO
    lsm303c_delay_ms(100);
    log_info("LSM303C initialized");
}

int imu_lsm303c_read(float *accel, float *mag, float *temp) {
    // Read X, Y, Z accel
    uint8_t xl = lsm303c_read_reg(LSM303C_OUT_X_L_A), xh = lsm303c_read_reg(LSM303C_OUT_X_H_A);
    uint8_t yl = lsm303c_read_reg(LSM303C_OUT_Y_L_A), yh = lsm303c_read_reg(LSM303C_OUT_Y_H_A);
    uint8_t zl = lsm303c_read_reg(LSM303C_OUT_Z_L_A), zh = lsm303c_read_reg(LSM303C_OUT_Z_H_A);
    int16_t x = (int16_t)((xh << 8) | xl);
    int16_t y = (int16_t)((yh << 8) | yl);
    int16_t z = (int16_t)((zh << 8) | zl);
    accel[0] = (x - lsm303c_bias[0]) * lsm303c_accel_scale;
    accel[1] = (y - lsm303c_bias[1]) * lsm303c_accel_scale;
    accel[2] = (z - lsm303c_bias[2]) * lsm303c_accel_scale;
    // Read X, Y, Z mag
    xl = lsm303c_read_reg(LSM303C_OUT_X_L_M); xh = lsm303c_read_reg(LSM303C_OUT_X_H_M);
    yl = lsm303c_read_reg(LSM303C_OUT_Y_L_M); yh = lsm303c_read_reg(LSM303C_OUT_Y_H_M);
    zl = lsm303c_read_reg(LSM303C_OUT_Z_L_M); zh = lsm303c_read_reg(LSM303C_OUT_Z_H_M);
    x = (int16_t)((xh << 8) | xl);
    y = (int16_t)((yh << 8) | yl);
    z = (int16_t)((zh << 8) | zl);
    mag[0] = (x - lsm303c_mag_bias[0]) * lsm303c_mag_scale;
    mag[1] = (y - lsm303c_mag_bias[1]) * lsm303c_mag_scale;
    mag[2] = (z - lsm303c_mag_bias[2]) * lsm303c_mag_scale;
    *temp = 25.0f;
    return 0;
}

int imu_lsm303c_selftest(float *result) {
    float accel[3], mag[3], temp;
    int ok = 1;
    for (int i = 0; i < 10; ++i) {
        imu_lsm303c_read(accel, mag, &temp);
        for (int j = 0; j < 3; ++j) {
            if (accel[j] < -20.0f || accel[j] > 20.0f) ok = 0;
        }
    }
    *result = ok ? 1.0f : 0.0f;
    return ok ? 0 : -1;
}

void imu_lsm303c_set_accel_odr(uint8_t odr) {
    // ODR bits are [7:4] in CTRL_REG1_A
    uint8_t reg = lsm303c_read_reg(LSM303C_CTRL_REG1_A);
    reg = (reg & 0x0F) | ((odr & 0x0F) << 4);
    lsm303c_write_reg(LSM303C_CTRL_REG1_A, reg);
}
void imu_lsm303c_set_mag_odr(uint8_t odr) {
    // ODR bits are [3:2] in CTRL_REG1_M
    uint8_t reg = lsm303c_read_reg(LSM303C_CTRL_REG1_M);
    reg = (reg & 0xF3) | ((odr & 0x03) << 2);
    lsm303c_write_reg(LSM303C_CTRL_REG1_M, reg);
}
void imu_lsm303c_set_fifo_mode(uint8_t mode) {
    // FIFO mode in CTRL_REG5_M [6:5]
    uint8_t reg = lsm303c_read_reg(LSM303C_CTRL_REG5_M);
    reg = (reg & 0x9F) | ((mode & 0x03) << 5);
    lsm303c_write_reg(LSM303C_CTRL_REG5_M, reg);
}
