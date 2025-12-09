#include "chip.h"
#include "i2c.h" // Include I2C communication library

#define LSM6DSV_I2C_ADDRESS 0x6A // Default I2C address for LSM6DSV
#define LSM6DSV_CS_PIN 0 // Define the GPIO pin for LSM6DSV chip select

// Register addresses (LSM6DSV datasheet, page 39)
#define LSM6DSV_CTRL1_XL 0x10 // Accelerometer control register 1
#define LSM6DSV_CTRL2_G  0x11 // Gyroscope control register 2
#define LSM6DSV_CTRL3_C  0x12 // Common control register 3
#define LSM6DSV_FIFO_CTRL1 0x07 // FIFO control register 1
#define LSM6DSV_FIFO_CTRL2 0x08 // FIFO control register 2
#define LSM6DSV_FIFO_CTRL3 0x09 // FIFO control register 3
#define LSM6DSV_FIFO_CTRL4 0x0A // FIFO control register 4
#define LSM6DSV_OUTX_L_A 0x28 // Accelerometer X-axis low byte
#define LSM6DSV_OUTX_H_A 0x29 // Accelerometer X-axis high byte
#define LSM6DSV_OUTY_L_A 0x2A // Accelerometer Y-axis low byte
#define LSM6DSV_OUTY_H_A 0x2B // Accelerometer Y-axis high byte
#define LSM6DSV_OUTZ_L_A 0x2C // Accelerometer Z-axis low byte
#define LSM6DSV_OUTZ_H_A 0x2D // Accelerometer Z-axis high byte

// Additional register definitions for FIFO and dual-channel accelerometer mode (LSM6DSV datasheet, page 39)
#define LSM6DSV_CTRL9_XL 0x18 // Accelerometer control register 9: Enables dual-channel mode
#define LSM6DSV_FIFO_CTRL5 0x0B // FIFO control register 5: Configures FIFO ODR and mode
#define LSM6DSV_OUTX_L_A2 0x58 // Second accelerometer X-axis low byte
#define LSM6DSV_OUTX_H_A2 0x59 // Second accelerometer X-axis high byte
#define LSM6DSV_OUTY_L_A2 0x5A // Second accelerometer Y-axis low byte
#define LSM6DSV_OUTY_H_A2 0x5B // Second accelerometer Y-axis high byte
#define LSM6DSV_OUTZ_L_A2 0x5C // Second accelerometer Z-axis low byte
#define LSM6DSV_OUTZ_H_A2 0x5D // Second accelerometer Z-axis high byte

// Additional register definitions for FIFO status and data output (LSM6DSV datasheet, page 39)
#define LSM6DSV_FIFO_STATUS1 0x3A // FIFO status register 1
#define LSM6DSV_FIFO_STATUS2 0x3B // FIFO status register 2
#define LSM6DSV_FIFO_DATA_OUT_L 0x3E // FIFO data output low byte
#define LSM6DSV_FIFO_DATA_OUT_H 0x3F // FIFO data output high byte

// Additional register definitions for interrupts (LSM6DSV datasheet, page 39)
#define LSM6DSV_INT1_CTRL 0x0D // Interrupt control register for INT1
#define LSM6DSV_INT2_CTRL 0x0E // Interrupt control register for INT2

// Updated configuration values for 8kHz ODR using HAODR settings
#define LSM6DSV_CTRL1_XL_VALUE 0xCC // 8kHz, ±2g (HAODR enabled, ODR[3:0] = 1100)
#define LSM6DSV_CTRL2_G_VALUE  0xCC // 8kHz, 250 dps (HAODR enabled, ODR[3:0] = 1100)
#define LSM6DSV_CTRL3_C_VALUE  0x44 // Enable block data update and auto-increment
#define LSM6DSV_FIFO_CTRL5_VALUE 0x2E // FIFO ODR set to 8kHz, continuous mode (ODR[3:0] = 1100)
#define LSM6DSV_CTRL9_XL_VALUE 0x01 // Enable dual-channel accelerometer mode

class LSM6DSV {
public:
    LSM6DSV(I2C_HandleTypeDef* i2cHandle);
    void begin();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void initializeSensor();
    void readAcceleration(float* ax, float* ay, float* az);
    void setGRange(uint8_t range);
    void readGyroscope(float* gx, float* gy, float* gz); // Add method to read gyroscope data
    void setGyroRange(uint8_t range); // Add method to set gyroscope range
    void configureFIFO(); // Add method to configure FIFO
    void readSecondChannelAcceleration(float* ax2, float* ay2, float* az2); // Add method to read second channel
    void configureDataReadyInterrupt(); // Add method to configure data-ready interrupt
    void configureFIFOFullInterrupt();  // Add method to configure FIFO full interrupt
    void handleDataReadyInterrupt();    // Add method to handle data-ready interrupt
    void handleFIFOFullInterrupt();     // Add method to handle FIFO full interrupt
    void selfTestAndCalibrate();        // Add method to self-test and calibrate the sensor
    void computeCorrectedValues(float* rawAcceleration, float* correctedAcceleration, float* correctedPosition); // Add method to compute corrected values

private:
    I2C_HandleTypeDef* _i2cHandle;
    float _scaleFactor;
    float _gyroScaleFactor; // Add a member variable to store the gyroscope scale factor
    float _bias[3];         // Add member variable to store accelerometer biases
    float _scale[3];        // Add member variable to store accelerometer scales
    float _gyroBias[3];     // Add member variable to store gyroscope biases

    void delayMs(uint32_t ms);
};

LSM6DSV::LSM6DSV(I2C_HandleTypeDef* i2cHandle)
    : _i2cHandle(i2cHandle), _scaleFactor(0.000061), _gyroScaleFactor(0.00875) {} // Initialize scale factors

void LSM6DSV::begin() {
    // Initialize the I2C peripheral for LPC4330
    Chip_I2C_Init(LPC_I2C0);
    Chip_I2C_SetClockRate(LPC_I2C0, 100000); // Set I2C clock to 100 kHz
    Chip_I2C_SetMasterEventHandler(LPC_I2C0, Chip_I2C_EventHandlerPolling);

    // Configure the chip select (CS) pin as output and set it high (deselected)
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, LSM6DSV_CS_PIN); // Set CS pin as output
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM6DSV_CS_PIN, true); // Set CS pin high (deselect)
}

void LSM6DSV::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM6DSV_CS_PIN, false); // Select the chip (CS low)
    if (Chip_I2C_MasterSend(LPC_I2C0, LSM6DSV_I2C_ADDRESS, data, 2) != 2) {
        printf("Failed to write register 0x%02X\n", reg);
    }
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM6DSV_CS_PIN, true); // Deselect the chip (CS high)
}

uint8_t LSM6DSV::readRegister(uint8_t reg) {
    uint8_t value;
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM6DSV_CS_PIN, false); // Select the chip (CS low)
    if (Chip_I2C_MasterSend(LPC_I2C0, LSM6DSV_I2C_ADDRESS, &reg, 1) != 1) {
        printf("Failed to write register address 0x%02X\n", reg);
    }
    if (Chip_I2C_MasterRead(LPC_I2C0, LSM6DSV_I2C_ADDRESS, &value, 1) != 1) {
        printf("Failed to read register value\n");
    }
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM6DSV_CS_PIN, true); // Deselect the chip (CS high)
    return value;
}

void LSM6DSV::initializeSensor() {
    // Configure accelerometer for 8kHz with HAODR
    writeRegister(LSM6DSV_CTRL1_XL, LSM6DSV_CTRL1_XL_VALUE); // CTRL1_XL: ODR[3:0] = 1100 (8kHz), FS_XL[1:0] = 00 (±2g)

    // Configure gyroscope for 8kHz with HAODR
    writeRegister(LSM6DSV_CTRL2_G, LSM6DSV_CTRL2_G_VALUE);   // CTRL2_G: ODR[3:0] = 1100 (8kHz), FS_G[1:0] = 00 (250 dps)

    // Common settings
    writeRegister(LSM6DSV_CTRL3_C, LSM6DSV_CTRL3_C_VALUE);   // CTRL3_C: Enable block data update and auto-increment

    // Enable dual-channel accelerometer mode
    writeRegister(LSM6DSV_CTRL9_XL, LSM6DSV_CTRL9_XL_VALUE); // CTRL9_XL: Enable dual-channel mode

    // Set the g range of the second accelerometer channel to ±16g
    uint8_t ctrl9_xl = readRegister(LSM6DSV_CTRL9_XL);
    ctrl9_xl = (ctrl9_xl & 0xFC) | 0x03; // CTRL9_XL: FS_XL2[1:0] = 11 (±16g)
    writeRegister(LSM6DSV_CTRL9_XL, ctrl9_xl);

    // Configure FIFO for 8kHz
    configureFIFO();

    // Configure interrupts
    configureDataReadyInterrupt(); // Configure data-ready interrupt
    configureFIFOFullInterrupt();  // Configure FIFO full interrupt

    delayMs(100); // Delay for sensor to stabilize
}

void LSM6DSV::readAcceleration(float* ax, float* ay, float* az) {
    uint8_t fifoLengthL = readRegister(LSM6DSV_FIFO_STATUS1); // FIFO_STATUS1: FIFO length LSB
    uint8_t fifoLengthH = readRegister(LSM6DSV_FIFO_STATUS2); // FIFO_STATUS2: FIFO length MSB
    uint16_t fifoLength = ((fifoLengthH & 0x03) << 8) | fifoLengthL; // Combine LSB and MSB

    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t rawX, rawY, rawZ;
    uint8_t data[6];

    for (uint16_t i = 0; i < fifoLength; i++) {
        // Read accelerometer data from FIFO
        data[0] = readRegister(LSM6DSV_FIFO_DATA_OUT_L);
        data[1] = readRegister(LSM6DSV_FIFO_DATA_OUT_H);
        data[2] = readRegister(LSM6DSV_FIFO_DATA_OUT_L);
        data[3] = readRegister(LSM6DSV_FIFO_DATA_OUT_H);
        data[4] = readRegister(LSM6DSV_FIFO_DATA_OUT_L);
        data[5] = readRegister(LSM6DSV_FIFO_DATA_OUT_H);

        rawX = (int16_t)((data[1] << 8) | data[0]);
        rawY = (int16_t)((data[3] << 8) | data[2]);
        rawZ = (int16_t)((data[5] << 8) | data[4]);

        sumX += rawX;
        sumY += rawY;
        sumZ += rawZ;
    }

    // Calculate average and convert to m/s²
    *ax = (sumX / fifoLength) * _scaleFactor;
    *ay = (sumY / fifoLength) * _scaleFactor;
    *az = (sumZ / fifoLength) * _scaleFactor;
}

void LSM6DSV::setGRange(uint8_t range) {
    uint8_t ctrl1_xl = readRegister(LSM6DSV_CTRL1_XL);
    ctrl1_xl = (ctrl1_xl & 0x3F) | (range << 6); // Set the g range bits
    writeRegister(LSM6DSV_CTRL1_XL, ctrl1_xl);

    // Update the scale factor based on the selected range
    switch (range) {
        case 0x00: // ±2g
            _scaleFactor = 0.000061;
            break;
        case 0x01: // ±4g
            _scaleFactor = 0.000122;
            break;
        case 0x02: // ±8g
            _scaleFactor = 0.000244;
            break;
        case 0x03: // ±16g
            _scaleFactor = 0.000488;
            break;
        default:
            _scaleFactor = 0.000061; // Default to ±2g
            break;
    }
}

void LSM6DSV::readGyroscope(float* gx, float* gy, float* gz) {
    uint8_t fifoLengthL = readRegister(LSM6DSV_FIFO_STATUS1); // FIFO_STATUS1: FIFO length LSB
    uint8_t fifoLengthH = readRegister(LSM6DSV_FIFO_STATUS2); // FIFO_STATUS2: FIFO length MSB
    uint16_t fifoLength = ((fifoLengthH & 0x03) << 8) | fifoLengthL; // Combine LSB and MSB

    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t rawX, rawY, rawZ;
    uint8_t data[6];

    for (uint16_t i = 0; i < fifoLength; i++) {
        // Read gyroscope data from FIFO
        data[0] = readRegister(LSM6DSV_FIFO_DATA_OUT_L);
        data[1] = readRegister(LSM6DSV_FIFO_DATA_OUT_H);
        data[2] = readRegister(LSM6DSV_FIFO_DATA_OUT_L);
        data[3] = readRegister(LSM6DSV_FIFO_DATA_OUT_H);
        data[4] = readRegister(LSM6DSV_FIFO_DATA_OUT_L);
        data[5] = readRegister(LSM6DSV_FIFO_DATA_OUT_H);

        rawX = (int16_t)((data[1] << 8) | data[0]);
        rawY = (int16_t)((data[3] << 8) | data[2]);
        rawZ = (int16_t)((data[5] << 8) | data[4]);

        sumX += rawX;
        sumY += rawY;
        sumZ += rawZ;
    }

    // Calculate average and convert to dps (degrees per second)
    *gx = (sumX / fifoLength) * _gyroScaleFactor;
    *gy = (sumY / fifoLength) * _gyroScaleFactor;
    *gz = (sumZ / fifoLength) * _gyroScaleFactor;
}

void LSM6DSV::configureFIFO() {
    // Configure FIFO for continuous mode at 8kHz for both accelerometer and gyroscope
    writeRegister(LSM6DSV_FIFO_CTRL1, 0xFF); // FIFO_CTRL1: Set FIFO threshold (maximum value, LSB)
    writeRegister(LSM6DSV_FIFO_CTRL2, 0x07); // FIFO_CTRL2: Set FIFO threshold (MSB)
    writeRegister(LSM6DSV_FIFO_CTRL3, 0xCC); // FIFO_CTRL3: Enable accelerometer and gyroscope data in FIFO
    writeRegister(LSM6DSV_FIFO_CTRL5, LSM6DSV_FIFO_CTRL5_VALUE); // FIFO_CTRL5: ODR[3:0] = 1100 (8kHz), continuous mode
}

void LSM6DSV::readSecondChannelAcceleration(float* ax2, float* ay2, float* az2) {
    uint8_t data[6];
    int16_t rawX2, rawY2, rawZ2;

    // Read second accelerometer channel data
    data[0] = readRegister(LSM6DSV_OUTX_L_A2);
    data[1] = readRegister(LSM6DSV_OUTX_H_A2);
    data[2] = readRegister(LSM6DSV_OUTY_L_A2);
    data[3] = readRegister(LSM6DSV_OUTY_H_A2);
    data[4] = readRegister(LSM6DSV_OUTZ_L_A2);
    data[5] = readRegister(LSM6DSV_OUTZ_H_A2);

    rawX2 = (int16_t)((data[1] << 8) | data[0]);
    rawY2 = (int16_t)((data[3] << 8) | data[2]);
    rawZ2 = (int16_t)((data[5] << 8) | data[4]);

    // Convert raw data to m/s²
    *ax2 = rawX2 * _scaleFactor;
    *ay2 = rawY2 * _scaleFactor;
    *az2 = rawZ2 * _scaleFactor;
}

void LSM6DSV::configureDataReadyInterrupt() {
    // Configure INT1 to generate a data-ready interrupt
    writeRegister(LSM6DSV_INT1_CTRL, 0x01); // Enable accelerometer data-ready interrupt on INT1
}

void LSM6DSV::configureFIFOFullInterrupt() {
    // Configure INT2 to generate a FIFO full interrupt
    writeRegister(LSM6DSV_INT2_CTRL, 0x20); // Enable FIFO full interrupt on INT2
}

void LSM6DSV::handleDataReadyInterrupt() {
    // Handle data-ready interrupt
    uint8_t status = readRegister(0x1E); // STATUS_REG (page 39)
    if (status & 0x01) { // Check if accelerometer data is ready
        float ax, ay, az;
        readAcceleration(&ax, &ay, &az); // Read accelerometer data
        printf("Data Ready Interrupt: ax = %f, ay = %f, az = %f\n", ax, ay, az);
    }
}

void LSM6DSV::handleFIFOFullInterrupt() {
    // Handle FIFO full interrupt
    uint8_t fifoStatus2 = readRegister(LSM6DSV_FIFO_STATUS2); // Read FIFO status register
    if (fifoStatus2 & 0x20) { // Check if FIFO full flag is set
        printf("FIFO Full Interrupt: FIFO is full\n");
        // Additional handling for FIFO full can be added here
        updateSensors(); // Update sensor data
    }
}

void LSM6DSV::selfTestAndCalibrate() {
    // Self-test and calibrate the accelerometer
    printf("Starting accelerometer self-test...\n");
    writeRegister(LSM6DSV_CTRL3_C, 0x44 | 0x01); // Enable accelerometer self-test
    delayMs(100); // Wait for self-test to stabilize

    float ax, ay, az;
    readAcceleration(&ax, &ay, &az);
    printf("Accelerometer self-test readings: ax = %f, ay = %f, az = %f\n", ax, ay, az);

    // Check if the self-test readings are within expected limits
    if (ax < -0.5 || ax > 0.5 || ay < -0.5 || ay > 0.5 || az < 0.5 || az > 1.5) {
        printf("Accelerometer self-test failed!\n");
    } else {
        printf("Accelerometer self-test passed.\n");
    }

    writeRegister(LSM6DSV_CTRL3_C, 0x44); // Disable accelerometer self-test
    delayMs(100);

    // Calibrate the accelerometer
    printf("Calibrating accelerometer...\n");
    float bias[3] = {0.0f, 0.0f, 0.0f};
    float scale[3] = {1.0f, 1.0f, 1.0f};
    for (int i = 0; i < 100; i++) {
        readAcceleration(&ax, &ay, &az);
        bias[0] += ax;
        bias[1] += ay;
        bias[2] += az - 1.0f; // Subtract gravity from Z-axis
        delayMs(10);
    }
    for (int i = 0; i < 3; i++) {
        bias[i] /= 100.0f;
        scale[i] = 1.0f; // Default scale factor (can be adjusted if needed)
    }
    printf("Accelerometer biases: bx = %f, by = %f, bz = %f\n", bias[0], bias[1], bias[2]);

    // Store biases and scales for correction
    for (int i = 0; i < 3; i++) {
        _bias[i] = bias[i];
        _scale[i] = scale[i];
    }

    // Self-test and calibrate the gyroscope
    printf("Starting gyroscope self-test...\n");
    writeRegister(LSM6DSV_CTRL3_C, 0x44 | 0x02); // Enable gyroscope self-test
    delayMs(100); // Wait for self-test to stabilize

    float gx, gy, gz;
    readGyroscope(&gx, &gy, &gz);
    printf("Gyroscope self-test readings: gx = %f, gy = %f, gz = %f\n", gx, gy, gz);

    // Check if the self-test readings are within expected limits
    if (gx < -1.0 || gx > 1.0 || gy < -1.0 || gy > 1.0 || gz < -1.0 || gz > 1.0) {
        printf("Gyroscope self-test failed!\n");
    } else {
        printf("Gyroscope self-test passed.\n");
    }

    writeRegister(LSM6DSV_CTRL3_C, 0x44); // Disable gyroscope self-test
    delayMs(100);

    // Calibrate the gyroscope
    printf("Calibrating gyroscope...\n");
    float gyroBias[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; i++) {
        readGyroscope(&gx, &gy, &gz);
        gyroBias[0] += gx;
        gyroBias[1] += gy;
        gyroBias[2] += gz;
        delayMs(10);
    }
    for (int i = 0; i < 3; i++) {
        gyroBias[i] /= 100.0f;
    }
    printf("Gyroscope biases: bgx = %f, bgy = %f, bgz = %f\n", gyroBias[0], gyroBias[1], gyroBias[2]);

    // Store gyroscope biases for correction
    for (int i = 0; i < 3; i++) {
        _gyroBias[i] = gyroBias[i];
    }
}

void LSM6DSV::computeCorrectedValues(float* rawAcceleration, float* correctedAcceleration, float* correctedPosition) {
    static float prevVelocity[3] = {0.0f, 0.0f, 0.0f};
    static float prevPosition[3] = {0.0f, 0.0f, 0.0f};
    static float deltaTime = 0.01f; // Assume a fixed time step (10 ms)

    // Calculate corrected acceleration
    for (int i = 0; i < 3; i++) {
        correctedAcceleration[i] = (rawAcceleration[i] - _bias[i]) * _scale[i];
    }

    // Integrate corrected acceleration to compute velocity
    float velocity[3];
    for (int i = 0; i < 3; i++) {
        velocity[i] = prevVelocity[i] + correctedAcceleration[i] * deltaTime;
    }

    // Integrate velocity to compute position
    for (int i = 0; i < 3; i++) {
        correctedPosition[i] = prevPosition[i] + velocity[i] * deltaTime;
    }

    // Update previous values
    for (int i = 0; i < 3; i++) {
        prevVelocity[i] = velocity[i];
        prevPosition[i] = correctedPosition[i];
    }
}

void LSM6DSV::delayMs(uint32_t ms) {
    Chip_Clock_System_BusyWait_ms(ms); // Use LPCOpen delay function
}

