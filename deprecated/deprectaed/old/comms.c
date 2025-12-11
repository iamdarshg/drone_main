#ifndef S2_LPQTR_H
#define S2_LPQTR_H

#include "chip.h" // Include the LPCOpen library
#include "usb.h"  // Include the USB library for LPC4330

#define FLASH_LOCATION 0x08000000 // Example flash location to store received packets
#define S2_LPQTR_CS_PIN 1 // Define the GPIO pin for S2_LPQTR chip select

// Define constants for register addresses and values
#define REG_RESET 0x70
#define RESET_COMMAND 0x80

#define REG_FREQ_1 0x10
#define REG_FREQ_2 0x11
#define REG_FREQ_3 0x12
#define REG_FREQ_4 0x13
#define FREQ_915_MHZ_1 0x6C
#define FREQ_915_MHZ_2 0x80
#define FREQ_915_MHZ_3 0x00
#define FREQ_915_MHZ_4 0x00

#define REG_MODULATION 0x5A
#define MODULATION_4GFSK 0x4F
#define REG_DATA_RATE_1 0x5B
#define REG_DATA_RATE_2 0x5C
#define HIGHEST_DATA_RATE 0x00

#define REG_AFC 0x30
#define ENABLE_AFC 0x8C
#define REG_FREQ_DEV_1 0x31
#define REG_FREQ_DEV_2 0x32
#define FREQ_DEV 0x00

#define REG_TX_POWER_1 0x50
#define REG_TX_POWER_2 0x51
#define MAX_TX_POWER 0xC0

#define REG_INPUT_GAIN 0x60
#define MAX_INPUT_GAIN 0x1F
#define REG_AGC 0x61
#define ENABLE_AGC 0x00

#define REG_FIFO 0x70
#define ENABLE_FIFO 0x01

#define REG_TX_COMMAND 0x72
#define START_TX 0x01
#define START_RX 0x02

#define REG_UART_OTA 0x73
#define ENABLE_UART_OTA 0x01

#define REG_PREAMBLE_PATTERN 0x35
#define PREAMBLE_PATTERN 0xAA // Example value for alternating 1s and 0s

#define REG_IRQ_MASK 0x05
#define IRQ_RX_DATA_READY 0x01

class S2_LPQTR {
public:
    S2_LPQTR(USB_HandleTypeDef* usbHandle);
    void begin();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void readMultipleRegisters(uint8_t startReg, uint8_t* buffer, size_t length);
    void initializeDevice();
    void startTransmission();
    void startReceiving();
    void transmit(uint8_t* payload, size_t length);

private:
    USB_HandleTypeDef* _usbHandle;

    void select();
    void deselect();
    void delayMs(uint32_t ms);
    void logSPIerror(const char* message);
};

S2_LPQTR::S2_LPQTR(USB_HandleTypeDef* usbHandle)
    : _usbHandle(usbHandle) {}

void S2_LPQTR::begin() {
    // Initialize the USB peripheral if needed
    if (USB_Init(_usbHandle) != USBH_OK) {
        logSPIerror("Failed to initialize USB");
    }
}

void S2_LPQTR::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    select();
    if (USB_SPI_Write(_usbHandle, data, 2) != 2) {
        logSPIerror("Failed to write register");
    }
    deselect();
}

uint8_t S2_LPQTR::readRegister(uint8_t reg) {
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

void S2_LPQTR::readMultipleRegisters(uint8_t startReg, uint8_t* buffer, size_t length) {
    select();
    if (USB_SPI_Write(_usbHandle, &startReg, 1) != 1) {
        logSPIerror("Failed to write start register address");
    }
    if (USB_SPI_Read(_usbHandle, buffer, length) != length) {
        logSPIerror("Failed to read multiple registers");
    }
    deselect();
}

void S2_LPQTR::initializeDevice() {
    // Reset the device
    writeRegister(REG_RESET, RESET_COMMAND); // Reset command
    delayMs(20); // 20 ms delay for reset

    // Configure the device for 915 MHz, 4-GFSK, highest data rate, AFC, etc.
    writeRegister(REG_FREQ_1, FREQ_915_MHZ_1); // Set frequency to 915 MHz
    writeRegister(REG_FREQ_2, FREQ_915_MHZ_2); // Set frequency to 915 MHz
    writeRegister(REG_FREQ_3, FREQ_915_MHZ_3); // Set frequency to 915 MHz
    writeRegister(REG_FREQ_4, FREQ_915_MHZ_4); // Set frequency to 915 MHz

    writeRegister(REG_MODULATION, MODULATION_4GFSK); // Set modulation to 4-GFSK
    writeRegister(REG_DATA_RATE_1, HIGHEST_DATA_RATE); // Set highest data rate
    writeRegister(REG_DATA_RATE_2, HIGHEST_DATA_RATE); // Set highest data rate

    writeRegister(REG_AFC, ENABLE_AFC); // Enable AFC
    writeRegister(REG_FREQ_DEV_1, FREQ_DEV); // Set frequency deviation
    writeRegister(REG_FREQ_DEV_2, FREQ_DEV); // Set frequency deviation

    writeRegister(REG_TX_POWER_1, MAX_TX_POWER); // Maximize transmit power
    writeRegister(REG_TX_POWER_2, MAX_TX_POWER); // Maximize transmit power

    writeRegister(REG_INPUT_GAIN, MAX_INPUT_GAIN); // Maximize input gain
    writeRegister(REG_AGC, ENABLE_AGC); // Enable automatic gain control

    writeRegister(REG_FIFO, ENABLE_FIFO); // Enable FIFO

    // Enable UART OTA packets
    writeRegister(REG_UART_OTA, ENABLE_UART_OTA); // Enable UART OTA packets
    writeRegister(REG_PREAMBLE_PATTERN, PREAMBLE_PATTERN); // Set preamble pattern
    writeRegister(REG_IRQ_MASK, IRQ_RX_DATA_READY); // Enable RX data ready interrupt
}

void S2_LPQTR::startTransmission() {
    // Start transmission
    writeRegister(REG_TX_COMMAND, START_TX); // Start TX command
}

void S2_LPQTR::startReceiving() {
    // Start receiving
    writeRegister(REG_TX_COMMAND, START_RX); // Start RX command
}

void S2_LPQTR::delayMs(uint32_t ms) {
    // Assuming the chip is running at its 204 MHz maximum clock rate
    // Use the LPCOpen delay function
    Chip_Clock_System_BusyWait_ms(ms);
}

#define FLASH_LOCATION 0x08000000 // Example flash location

void S2_LPQTR::packetReceivedHandler() {
    uint8_t buffer[256]; // Adjust buffer size as needed
    size_t length = 256; // Adjust length as needed

    // Read the received packet from the S2-LP
    readMultipleRegisters(REG_FIFO, buffer, length);

    // Store the received packet into the flash location
    storePacketInFlash(buffer, length);
}

void S2_LPQTR::storePacketInFlash(uint8_t* buffer, size_t length) {
    // Example function to store data in flash
    // This will depend on your specific microcontroller and flash memory
    // Ensure that the flash memory is properly erased before writing

    // Unlock flash memory for writing
    Chip_FMC_Unlock();

    // Erase the flash sector
    Chip_FMC_EraseSector(FLASH_LOCATION);

    // Write the data to flash
    for (size_t i = 0; i < length; i++) {
        Chip_FMC_WriteByte(FLASH_LOCATION + i, buffer[i]);
    }

    // Lock flash memory after writing
    Chip_FMC_Lock();
}

void GPIO_IRQHandler(void) {
    // Check if the interrupt was triggered by the S2-LP
    if (Chip_GPIO_GetPinIntStatus(LPC_GPIO_PIN_INT, GPIOINT_PORT, GPIOINT_PIN)) {
        // Clear the interrupt
        Chip_GPIO_ClearInts(LPC_GPIO_PIN_INT, GPIOINT_PORT, GPIOINT_PIN);

        // Call the packet received handler
        S2_LPQTR::packetReceivedHandler();
    }
}

void S2_LPQTR::transmit(uint8_t* payload, size_t length) {
    // Write the payload to the FIFO
    for (size_t i = 0; i < length; i++) {
        writeRegister(REG_FIFO, payload[i]);
    }

    // Start transmission
    startTransmission();
}

void S2_LPQTR::select() {
    // Set the CS pin low to select the S2_LPQTR
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, S2_LPQTR_CS_PIN, false);
}

void S2_LPQTR::deselect() {
    // Set the CS pin high to deselect the S2_LPQTR
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, S2_LPQTR_CS_PIN, true);
}

// USB-SPI functions
int USB_SPI_Write(USB_HandleTypeDef* usbHandle, uint8_t* data, size_t length) {
    // Send data to the USB-SPI bridge
    int result = USBH_BulkSendData(usbHandle, data, length, USB_SPI_ENDPOINT_OUT, USB_SPI_TIMEOUT);
    if (result != USBH_OK) {
        // Handle USB communication error
        logSPIerror("Failed to write USB data");
        return -1;
    }
    return length;
}

int USB_SPI_Read(USB_HandleTypeDef* usbHandle, uint8_t* data, size_t length) {
    // Receive data from the USB-SPI bridge
    int result = USBH_BulkReceiveData(usbHandle, data, length, USB_SPI_ENDPOINT_IN, USB_SPI_TIMEOUT);
    if (result != USBH_OK) {
        // Handle USB communication error
        logSPIerror("Failed to read USB data");
        return -1;
    }
    return length;
}

void logSPIerror(const char* message) {
    // Log SPI communication error
    // Implement error logging mechanism as needed
    printf("SPI (comms) Error: %s\n", message);
}

#endif // S2_LPQTR_H