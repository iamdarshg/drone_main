#include "chip.h"
#include "i2c.h" // Include I2C communication library

#define LSM303AHTR_I2C_ADDRESS 0x1E // Default I2C address for LSM303AHTR
#define LSM303AHTR_CS_PIN 1         // Define the GPIO pin for LSM303AHTR chip select

// Register addresses (LSM303AHTR datasheet)
#define LSM303AHTR_CTRL_REG1_A 0x20 // Accelerometer control register 1
#define LSM303AHTR_CTRL_REG2_A 0x21 // Accelerometer control register 2
#define LSM303AHTR_CTRL_REG3_A 0x22 // Accelerometer control register 3
#define LSM303AHTR_CTRL_REG4_A 0x23 // Accelerometer control register 4
#define LSM303AHTR_CTRL_REG5_A 0x24 // Accelerometer control register 5
#define LSM303AHTR_OUT_X_L_A   0x28 // Accelerometer X-axis low byte
#define LSM303AHTR_OUT_X_H_A   0x29 // Accelerometer X-axis high byte
#define LSM303AHTR_OUT_Y_L_A   0x2A // Accelerometer Y-axis low byte
#define LSM303AHTR_OUT_Y_H_A   0x2B // Accelerometer Y-axis high byte
#define LSM303AHTR_OUT_Z_L_A   0x2C // Accelerometer Z-axis low byte
#define LSM303AHTR_OUT_Z_H_A   0x2D // Accelerometer Z-axis high byte

#define LSM303AHTR_CTRL_REG1_M 0x60 // Magnetometer control register 1
#define LSM303AHTR_CTRL_REG2_M 0x61 // Magnetometer control register 2
#define LSM303AHTR_CTRL_REG3_M 0x62 // Magnetometer control register 3
#define LSM303AHTR_CTRL_REG4_M 0x63 // Magnetometer control register 4
#define LSM303AHTR_CTRL_REG5_M 0x64 // Magnetometer control register 5
#define LSM303AHTR_OUT_X_L_M   0x68 // Magnetometer X-axis low byte
#define LSM303AHTR_OUT_X_H_M   0x69 // Magnetometer X-axis high byte
#define LSM303AHTR_OUT_Y_L_M   0x6A // Magnetometer Y-axis low byte
#define LSM303AHTR_OUT_Y_H_M   0x6B // Magnetometer Y-axis high byte
#define LSM303AHTR_OUT_Z_L_M   0x6C // Magnetometer Z-axis low byte
#define LSM303AHTR_OUT_Z_H_M   0x6D // Magnetometer Z-axis high byte
#define LSM303AHTR_OFFSET_X_L_M 0x45 // Magnetometer X-axis offset low byte
#define LSM303AHTR_OFFSET_X_H_M 0x46 // Magnetometer X-axis offset high byte
#define LSM303AHTR_OFFSET_Y_L_M 0x47 // Magnetometer Y-axis offset low byte
#define LSM303AHTR_OFFSET_Y_H_M 0x48 // Magnetometer Y-axis offset high byte
#define LSM303AHTR_OFFSET_Z_L_M 0x49 // Magnetometer Z-axis offset low byte
#define LSM303AHTR_OFFSET_Z_H_M 0x4A // Magnetometer Z-axis offset high byte

#define LSM303AHTR_INT_CTRL_REG_A 0x22 // Accelerometer interrupt control register
#define LSM303AHTR_FIFO_CTRL_REG_A 0x2E // Accelerometer FIFO control register
#define LSM303AHTR_FIFO_SRC_REG_A 0x2F // Accelerometer FIFO source register

// Updated register values for high-resolution mode, 6.4 kHz ODR, and 16-bit resolution
#define LSM303AHTR_CTRL_REG1_A_VALUE 0x97 // Accelerometer: 6.4 kHz ODR, high-resolution mode, all axes enabled
#define LSM303AHTR_CTRL_REG4_A_VALUE 0x08 // Accelerometer: ±2g range, high-resolution mode (HR bit set)
#define LSM303AHTR_CTRL_REG1_M_VALUE 0x70 // Magnetometer: 100 Hz ODR
#define LSM303AHTR_CTRL_REG2_M_VALUE 0x80 // Magnetometer: Enable automatic range detection
#define LSM303AHTR_CTRL_REG3_M_VALUE 0x00 // Magnetometer: Continuous conversion mode

class LSM303AHTR {
public:
    LSM303AHTR(I2C_HandleTypeDef* i2cHandle);
    void begin();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void initializeSensor();
    void loadHardIronCompensation(int16_t offsetX, int16_t offsetY, int16_t offsetZ);
    void readAcceleration(float* ax, float* ay, float* az);
    void readMagnetometer(float* mx, float* my, float* mz);
    void selfTestAndCalibrate();
    void configureFIFOFullInterrupt();
    void handleFIFOFullInterrupt();
    void computeCorrectedValues(float* rawAcceleration, float* correctedAcceleration, float* correctedPosition);

private:
    I2C_HandleTypeDef* _i2cHandle;
    float _bias[3] = {0.0f, 0.0f, 0.0f};
    float _scale[3] = {1.0f, 1.0f, 1.0f};
    float _magBias[3] = {0.0f, 0.0f, 0.0f};
    void select();
    void deselect();
    void delayMs(uint32_t ms);
};

LSM303AHTR::LSM303AHTR(I2C_HandleTypeDef* i2cHandle)
    : _i2cHandle(i2cHandle) {}

void LSM303AHTR::begin() {
    // Initialize the I2C peripheral
    Chip_I2C_Init(LPC_I2C0);
    Chip_I2C_SetClockRate(LPC_I2C0, 100000); // Set I2C clock to 100 kHz
    Chip_I2C_SetMasterEventHandler(LPC_I2C0, Chip_I2C_EventHandlerPolling);

    // Configure the chip select (CS) pin as output and set it high (deselected)
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, LSM303AHTR_CS_PIN); // Set CS pin as output
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM303AHTR_CS_PIN, true); // Set CS pin high (deselect)
}

void LSM303AHTR::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    select(); // Select the chip (CS low)
    if (Chip_I2C_MasterSend(LPC_I2C0, LSM303AHTR_I2C_ADDRESS, data, 2) != 2) {
        printf("Failed to write register 0x%02X\n", reg);
    }
    deselect(); // Deselect the chip (CS high)
}

uint8_t LSM303AHTR::readRegister(uint8_t reg) {
    uint8_t value;
    select(); // Select the chip (CS low)
    if (Chip_I2C_MasterSend(LPC_I2C0, LSM303AHTR_I2C_ADDRESS, &reg, 1) != 1) {
        printf("Failed to write register address 0x%02X\n", reg);
    }
    if (Chip_I2C_MasterRead(LPC_I2C0, LSM303AHTR_I2C_ADDRESS, &value, 1) != 1) {
        printf("Failed to read register value\n");
    }
    deselect(); // Deselect the chip (CS high)
    return value;
}

void LSM303AHTR::initializeSensor() {
    // Configure accelerometer
    writeRegister(LSM303AHTR_CTRL_REG1_A, LSM303AHTR_CTRL_REG1_A_VALUE); // 6.4 kHz ODR, high-resolution mode, all axes enabled
    writeRegister(LSM303AHTR_CTRL_REG4_A, LSM303AHTR_CTRL_REG4_A_VALUE); // ±2g range, high-resolution mode (HR bit set)

    // Configure magnetometer
    writeRegister(LSM303AHTR_CTRL_REG1_M, LSM303AHTR_CTRL_REG1_M_VALUE); // 100 Hz ODR
    writeRegister(LSM303AHTR_CTRL_REG2_M, LSM303AHTR_CTRL_REG2_M_VALUE); // Enable automatic range detection
    writeRegister(LSM303AHTR_CTRL_REG3_M, LSM303AHTR_CTRL_REG3_M_VALUE); // Continuous conversion mode
    writeRegister(LSM303AHTR_CTRL_REG4_M, 0x02); // Enable tilt compensation
    writeRegister(LSM303AHTR_CTRL_REG5_M, 0x40); // Enable FIFO mode
}

void LSM303AHTR::loadHardIronCompensation(int16_t offsetX, int16_t offsetY, int16_t offsetZ) {
    // Load hard iron compensation values into the magnetometer offset registers
    writeRegister(LSM303AHTR_OFFSET_X_L_M, offsetX & 0xFF); // X-axis offset low byte
    writeRegister(LSM303AHTR_OFFSET_X_H_M, (offsetX >> 8) & 0xFF); // X-axis offset high byte
    writeRegister(LSM303AHTR_OFFSET_Y_L_M, offsetY & 0xFF); // Y-axis offset low byte
    writeRegister(LSM303AHTR_OFFSET_Y_H_M, (offsetY >> 8) & 0xFF); // Y-axis offset high byte
    writeRegister(LSM303AHTR_OFFSET_Z_L_M, offsetZ & 0xFF); // Z-axis offset low byte
    writeRegister(LSM303AHTR_OFFSET_Z_H_M, (offsetZ >> 8) & 0xFF); // Z-axis offset high byte
}

void LSM303AHTR::readAcceleration(float* ax, float* ay, float* az) {
    uint8_t fifoLength = readRegister(0x2F); // FIFO_SRC_REG_A: FIFO length
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t rawX, rawY, rawZ;
    uint8_t data[6];

    for (uint8_t i = 0; i < fifoLength; i++) {
        // Read accelerometer data from FIFO
        data[0] = readRegister(LSM303AHTR_OUT_X_L_A);
        data[1] = readRegister(LSM303AHTR_OUT_X_H_A);
        data[2] = readRegister(LSM303AHTR_OUT_Y_L_A);
        data[3] = readRegister(LSM303AHTR_OUT_Y_H_A);
        data[4] = readRegister(LSM303AHTR_OUT_Z_L_A);
        data[5] = readRegister(LSM303AHTR_OUT_Z_H_A);

        rawX = (int16_t)((data[1] << 8) | data[0]);
        rawY = (int16_t)((data[3] << 8) | data[2]);
        rawZ = (int16_t)((data[5] << 8) | data[4]);

        sumX += rawX;
        sumY += rawY;
        sumZ += rawZ;
    }

    // Calculate average and convert to m/s²
    *ax = (sumX / fifoLength) * 0.000061; // Example scale factor for ±2g range
    *ay = (sumY / fifoLength) * 0.000061;
    *az = (sumZ / fifoLength) * 0.000061;
}

void LSM303AHTR::readMagnetometer(float* mx, float* my, float* mz) {
    uint8_t fifoLength = readRegister(0x2F); // FIFO_SRC_REG_M: FIFO length
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t rawX, rawY, rawZ;
    uint8_t data[6];

    for (uint8_t i = 0; i < fifoLength; i++) {
        // Read magnetometer data from FIFO
        data[0] = readRegister(LSM303AHTR_OUT_X_L_M);
        data[1] = readRegister(LSM303AHTR_OUT_X_H_M);
        data[2] = readRegister(LSM303AHTR_OUT_Y_L_M);
        data[3] = readRegister(LSM303AHTR_OUT_Y_H_M);
        data[4] = readRegister(LSM303AHTR_OUT_Z_L_M);
        data[5] = readRegister(LSM303AHTR_OUT_Z_H_M);

        rawX = (int16_t)((data[1] << 8) | data[0]);
        rawY = (int16_t)((data[3] << 8) | data[2]);
        rawZ = (int16_t)((data[5] << 8) | data[4]);

        sumX += rawX;
        sumY += rawY;
        sumZ += rawZ;
    }

    // Calculate average and convert to gauss
    *mx = (sumX / fifoLength) * 0.00014; // Example scale factor for ±4 gauss range
    *my = (sumY / fifoLength) * 0.00014;
    *mz = (sumZ / fifoLength) * 0.00014;
}

void LSM303AHTR::selfTestAndCalibrate() {
    // Self-test and calibrate the accelerometer
    printf("Starting accelerometer self-test...\n");
    writeRegister(LSM303AHTR_CTRL_REG2_A, 0x40); // Enable self-test mode
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

    writeRegister(LSM303AHTR_CTRL_REG2_A, 0x00); // Disable self-test mode
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

    // Self-test and calibrate the magnetometer
    printf("Starting magnetometer self-test...\n");
    writeRegister(LSM303AHTR_CTRL_REG2_M, 0x40); // Enable self-test mode
    delayMs(100); // Wait for self-test to stabilize

    float mx, my, mz;
    readMagnetometer(&mx, &my, &mz);
    printf("Magnetometer self-test readings: mx = %f, my = %f, mz = %f\n", mx, my, mz);

    // Check if the self-test readings are within expected limits
    if (mx < -1.0 || mx > 1.0 || my < -1.0 || my > 1.0 || mz < -1.0 || mz > 1.0) {
        printf("Magnetometer self-test failed!\n");
    } else {
        printf("Magnetometer self-test passed.\n");
    }

    writeRegister(LSM303AHTR_CTRL_REG2_M, 0x00); // Disable self-test mode
    delayMs(100);

    // Calibrate the magnetometer
    printf("Calibrating magnetometer...\n");
    float magBias[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; i++) {
        readMagnetometer(&mx, &my, &mz);
        magBias[0] += mx;
        magBias[1] += my;
        magBias[2] += mz;
        delayMs(10);
    }
    for (int i = 0; i < 3; i++) {
        magBias[i] /= 100.0f;
    }
    printf("Magnetometer biases: bmx = %f, bmy = %f, bmz = %f\n", magBias[0], magBias[1], magBias[2]);

    // Store magnetometer biases for correction
    for (int i = 0; i < 3; i++) {
        _magBias[i] = magBias[i];
    }
}

void LSM303AHTR::configureFIFOFullInterrupt() {
    // Configure FIFO for stream mode and enable FIFO full interrupt
    writeRegister(LSM303AHTR_FIFO_CTRL_REG_A, 0x9F); // FIFO_CTRL_REG_A: Stream mode, set threshold to 31
    writeRegister(LSM303AHTR_INT_CTRL_REG_A, 0x20);  // INT_CTRL_REG_A: Enable FIFO full interrupt
}

void LSM303AHTR::handleFIFOFullInterrupt() {
    // Check if the FIFO full interrupt is triggered
    uint8_t fifoStatus = readRegister(LSM303AHTR_FIFO_SRC_REG_A); // Read FIFO source register
    if (fifoStatus & 0x20) { // Check if FIFO full flag is set
        printf("FIFO Full Interrupt: FIFO is full\n");

        // Read and process all data from the FIFO
        float ax, ay, az;
        readAcceleration(&ax, &ay, &az);
        printf("FIFO Data: ax = %f, ay = %f, az = %f\n", ax, ay, az);
    }
}

void LSM303AHTR::computeCorrectedValues(float* rawAcceleration, float* correctedAcceleration, float* correctedPosition) {
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

void LSM303AHTR::select() {
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM303AHTR_CS_PIN, false); // Set CS pin low to select the chip
}

void LSM303AHTR::deselect() {
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, LSM303AHTR_CS_PIN, true); // Set CS pin high to deselect the chip
}

void LSM303AHTR::delayMs(uint32_t ms) {
    Chip_Clock_System_BusyWait_ms(ms); // Use LPCOpen delay function
}
