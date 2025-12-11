#include "chip.h"
#include "usb.h"

#define KX122_CS_PIN 0 // Define the GPIO pin for KX122 chip select

// Register addresses (KX122 datasheet, page 12)
// These registers control various aspects of the KX122 accelerometer's operation.
#define KX122_REG_CTRL1 0x1B // Control register 1: Configures ODR, g-range, and power mode
#define KX122_REG_CTRL2 0x18 // Control register 2: Enables advanced features like tilt detection
#define KX122_REG_CTRL3 0x1C // Control register 3: Configures interrupt settings
#define KX122_REG_CTRL4 0x1D // Control register 4: Configures interrupt polarity and behavior
#define KX122_REG_CTRL5 0x1E // Control register 5: Enables specific interrupt sources
#define KX122_REG_CTRL6 0x1F // Control register 6: Routes interrupt signals to INT1 or INT2
#define KX122_REG_CTRL7 0x20 // Control register 7: Enables advanced motion detection features
#define KX122_REG_CTRL8 0x21 // Control register 8: Configures additional sensor settings
#define KX122_REG_FFTH 0x23  // Free-fall threshold register (page 13)
#define KX122_REG_FFC 0x24   // Free-fall counter register (page 13)

// Register values (KX122 datasheet, page 12)
// These values are written to the corresponding registers to configure the sensor.
#define KX122_CTRL1_VALUE 0xE0 // Sets ODR to 12.5 Hz, enables ±2g range, and puts the sensor in high-resolution mode
#define KX122_CTRL2_VALUE 0x80 // Enables tilt detection
#define KX122_CTRL3_VALUE 0x02 // Configures INT1 as active high and push-pull
#define KX122_CTRL4_VALUE 0x40 // Enables data-ready interrupt
#define KX122_CTRL5_VALUE 0x01 // Enables wake-up and free-fall interrupts
#define KX122_CTRL6_VALUE 0x40 // Routes data-ready interrupt to INT1 and free-fall interrupt to INT2
#define KX122_CTRL7_VALUE 0x10 // Enables motion detection
#define KX122_CTRL8_VALUE 0x00 // Default value, no additional features enabled
#define KX122_FFTH_VALUE 0x14 // Free-fall threshold (example value, adjust as needed)
#define KX122_FFC_VALUE 0x03  // Free-fall counter (example value, adjust as needed)

class KX122 {
public:
    KX122(USB_HandleTypeDef* usbHandle);
    void begin();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void initializeSensor();
    void configureFreeFallInterrupt(); // Add method to configure free-fall detection
    void handleFreeFallInterrupt();    // Add method to handle free-fall interrupt
    void calibrate(); // Add the calibrate method declaration
    void readFIFO(float* ax, float* ay, float* az); // Add the readFIFO method declaration
    void configureFIFOFullInterrupt(); // Add the configureFIFOFullInterrupt method declaration
    void handleFIFOFullInterrupt(); // Add the handleFIFOFullInterrupt method declaration
    void setGRange(uint8_t range); // Add the setGRange method declaration
    void selfTestAndCalibrate(); // Add the selfTestAndCalibrate method declaration
    void computeCorrectedValues(float* rawAcceleration, float* correctedAcceleration, float* correctedPosition); // Add the computeCorrectedValues method declaration

private:
    USB_HandleTypeDef* _usbHandle;
    float _scaleFactor; // Add a member variable to store the scale factor
    float _bias[3]; // Add a member variable to store biases
    float _scale[3]; // Add a member variable to store scales

    void select();
    void deselect();
    void delayMs(uint32_t ms);
    void logSPIerror(const char* message);
};

KX122::KX122(USB_HandleTypeDef* usbHandle)
    : _usbHandle(usbHandle), _scaleFactor(0.000598) {
    _bias[0] = _bias[1] = _bias[2] = 0.0f;
    _scale[0] = _scale[1] = _scale[2] = 1.0f;
}

void KX122::begin() {
    // Initialize the USB peripheral if needed
    if (USB_Init(_usbHandle) != USBH_OK) {
        logSPIerror("Failed to initialize USB");
    }
}

void KX122::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    select();
    if (USB_SPI_Write(_usbHandle, data, 2) != 2) {
        logSPIerror("Failed to write register");
    }
    deselect();
}

uint8_t KX122::readRegister(uint8_t reg) {
    uint8_t value;
    select();
    if (USB_SPI_Write(_usbHandle, &reg, 1) != 1) {
        logSPIerror("Failed to write register address");
    }
    if (USB_SPI_Read(_usbHandle, &value, 1) != 1) {
        logSPIerror("Failed to read register value");
    }
    deselect();
    return value;
}

void KX122::initializeSensor() {
    // Write configuration values to the control registers to initialize the sensor.

    // Configure Control Register 1 (KX122_REG_CTRL1)
    // - Sets ODR to 12.5 Hz
    // - Enables ±2g range
    // - Puts the sensor in high-resolution mode
    writeRegister(KX122_REG_CTRL1, KX122_CTRL1_VALUE);

    // Configure Control Register 2 (KX122_REG_CTRL2)
    // - Enables tilt detection
    writeRegister(KX122_REG_CTRL2, KX122_CTRL2_VALUE);

    // Configure Control Register 3 (KX122_REG_CTRL3)
    // - Configures INT1 as active high and push-pull
    writeRegister(KX122_REG_CTRL3, KX122_CTRL3_VALUE);

    // Configure Control Register 4 (KX122_REG_CTRL4)
    // - Enables data-ready interrupt
    writeRegister(KX122_REG_CTRL4, KX122_CTRL4_VALUE);

    // Configure Control Register 5 (KX122_REG_CTRL5)
    // - Enables wake-up and free-fall interrupts
    writeRegister(KX122_REG_CTRL5, KX122_CTRL5_VALUE);

    // Configure Control Register 6 (KX122_REG_CTRL6)
    // - Routes data-ready interrupt to INT1
    // - Routes free-fall interrupt to INT2
    writeRegister(KX122_REG_CTRL6, KX122_CTRL6_VALUE);

    // Configure Control Register 7 (KX122_REG_CTRL7)
    // - Enables motion detection
    writeRegister(KX122_REG_CTRL7, KX122_CTRL7_VALUE);

    // Configure Control Register 8 (KX122_REG_CTRL8)
    // - No additional features enabled
    writeRegister(KX122_REG_CTRL8, KX122_CTRL8_VALUE);

    // Configure free-fall detection
    configureFreeFallInterrupt();

    // Delay to allow the sensor to stabilize
    delayMs(100);
}

void KX122::calibrate() {
    // Example calibration steps based on the datasheet
    // Set the sensor to standby mode
    writeRegister(KX122_REG_CTRL1, 0x00);
    delayMs(50);

    // Set the offset registers (example values, adjust as needed)
    writeRegister(0x1A, 0x00); // X-axis offset
    writeRegister(0x1B, 0x00); // Y-axis offset
    writeRegister(0x1C, 0x00); // Z-axis offset

    // Set the sensor back to operating mode
    writeRegister(KX122_REG_CTRL1, KX122_CTRL1_VALUE);
    // Enable 2-sample averaging by modifying Control Register 1
    uint8_t ctrl1 = readRegister(KX122_REG_CTRL1);
    ctrl1 |= 0x02; // Enable 2-sample averaging
    writeRegister(KX122_REG_CTRL1, ctrl1);

    // Set the ODR for free fall detection to the maximum value
    ctrl1 = readRegister(KX122_REG_CTRL1);
    ctrl1 = (ctrl1 & 0xF8) | 0x07; // Set ODR bits to 111 (maximum ODR)
    writeRegister(KX122_REG_CTRL1, ctrl1);
    delayMs(50);
}

void KX122::setGRange(uint8_t range) {
    uint8_t cntl1 = readRegister(KX122_REG_CTRL1);
    cntl1 = (cntl1 & 0xE7) | (range << 3); // Set the g range bits
    writeRegister(KX122_REG_CTRL1, cntl1);

    // Update the scale factor based on the selected range
    switch (range) {
        case 0x00: // ±2g
            _scaleFactor = 0.000598;
            break;
        case 0x01: // ±4g
            _scaleFactor = 0.001196;
            break;
        case 0x02: // ±8g
            _scaleFactor = 0.002392;
            break;
        default:
            _scaleFactor = 0.000598; // Default to ±2g
            break;
    }
}

void KX122::readFIFO(float* ax, float* ay, float* az) {
    uint8_t fifoLength = readRegister(0x3A); // Read FIFO length register (example address, adjust as needed)
    uint8_t fifoData[32 * fifoLength]; // Buffer to hold FIFO data
    int16_t x, y, z;
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int sampleCount = 0;

    // Read FIFO buffer
    for (int i = 0; i < fifoLength * 6; i += 6) {
        fifoData[i] = readRegister(0x3E); // Read X LSB
        fifoData[i + 1] = readRegister(0x3F); // Read X MSB
        fifoData[i + 2] = readRegister(0x40); // Read Y LSB
        fifoData[i + 3] = readRegister(0x41); // Read Y MSB
        fifoData[i + 4] = readRegister(0x42); // Read Z LSB
        fifoData[i + 5] = readRegister(0x43); // Read Z MSB

        x = (int16_t)((fifoData[i + 1] << 8) | fifoData[i]);
        y = (int16_t)((fifoData[i + 3] << 8) | fifoData[i + 2]);
        z = (int16_t)((fifoData[i + 5] << 8) | fifoData[i + 4]);

        sumX += x;
        sumY += y;
        sumZ += z;
        sampleCount++;
    }

    // Calculate average and convert to m/s²
    *ax = (sumX / sampleCount) * _scaleFactor;
    *ay = (sumY / sampleCount) * _scaleFactor;
    *az = (sumZ / sampleCount) * _scaleFactor;
}

void KX122::configureFreeFallInterrupt() {
    // Configure free-fall threshold and counter
    writeRegister(KX122_REG_FFTH, KX122_FFTH_VALUE); // Set free-fall threshold
    writeRegister(KX122_REG_FFC, KX122_FFC_VALUE);   // Set free-fall counter
}

void KX122::handleFreeFallInterrupt() {
    // Handle free-fall interrupt
    uint8_t intSource = readRegister(0x3A); // Read interrupt source register
    if (intSource & 0x20) { // Check if free-fall interrupt is set
        printf("Free-fall detected!\n");
    }
}

void KX122::logSPIerror(const char* message) {
    printf("SPI (K122) Error: %s\n", message);    // Log SPI communication error
}

void KX122::delayMs(uint32_t ms) {
    Chip_Clock_System_BusyWait_ms(ms);    // Use the LPCOpen delay function
}

void KX122::deselect() {
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, KX122_CS_PIN, true);    // Set the CS pin high to deselect the KX122
}

void KX122::select() {
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, KX122_CS_PIN, false);    // Set the CS pin low to select the KX122
}

void KX122::handleFIFOFullInterrupt() {
    // Check if the interrupt is from FIFO full
    uint8_t intSource = readRegister(0x3A); // Read interrupt source register (example address, adjust as needed)
    if (intSource & 0x40) { // Check if FIFO full interrupt is set
        float ax, ay, az;
        readFIFO(&ax, &ay, &az); // Read FIFO data
        // Handle the FIFO data (e.g., log it, process it, etc.)
        printf("FIFO Full Interrupt: ax = %f, ay = %f, az = %f\n", ax, ay, az);
    }
}

void KX122::configureFIFOFullInterrupt() {
    // Configure the interrupt pin (example GPIO pin, adjust as needed)
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 1); // Set GPIO pin as input

    // Enable FIFO full interrupt in the sensor
    writeRegister(KX122_REG_CTRL5, 0x40); // Enable FIFO full interrupt
    writeRegister(KX122_REG_CTRL6, 0x01); // Route interrupt to INT2 pin
}

void KX122::selfTestAndCalibrate() {
    // Self-test the accelerometer
    printf("Starting accelerometer self-test...\n");
    writeRegister(KX122_REG_CTRL2, 0x80); // Enable self-test mode
    delayMs(100); // Wait for self-test to stabilize

    float ax, ay, az;
    readFIFO(&ax, &ay, &az);
    printf("Accelerometer self-test readings: ax = %f, ay = %f, az = %f\n", ax, ay, az);

    // Check if the self-test readings are within expected limits
    if (ax < -0.5 || ax > 0.5 || ay < -0.5 || ay > 0.5 || az < 0.5 || az > 1.5) {
        printf("Accelerometer self-test failed!\n");
    } else {
        printf("Accelerometer self-test passed.\n");
    }

    writeRegister(KX122_REG_CTRL2, 0x00); // Disable self-test mode
    delayMs(100);

    // Calibrate the accelerometer
    printf("Calibrating accelerometer...\n");
    float bias[3] = {0.0f, 0.0f, 0.0f};
    float scale[3] = {1.0f, 1.0f, 1.0f};
    for (int i = 0; i < 100; i++) {
        readFIFO(&ax, &ay, &az);
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
}

void KX122::computeCorrectedValues(float* rawAcceleration, float* correctedAcceleration, float* correctedPosition) {
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