#include "imu_backup.h" // Include LSM6DSV header
#include "imu_fast.h"   // Include KX122 header
#include "imu_mag.h"    // Include LSM303AHTR header
#include <math.h>       // Include math library for trigonometric functions

// Global variable to track the current orientation
static float currentOrientation[3] = {0.0f, 0.0f, 0.0f};

// Global variable to track the current position
static float currentPosition[3] = {0.0f, 0.0f, 0.0f};

typedef struct {
    LSM6DSV* lsm6dsv;
    KX122* kx122;
    LSM303AHTR* lsm303;
} IMUInstances;

IMUInstances initializeAndTestIMUs(I2C_HandleTypeDef* i2cHandle, USB_HandleTypeDef* usbHandle) {
    // Allocate memory for IMU instances
    static LSM6DSV lsm6dsv(i2cHandle);
    static KX122 kx122(usbHandle);
    static LSM303AHTR lsm303(i2cHandle);

    // Initialize sensors
    lsm6dsv.begin();
    kx122.begin();
    lsm303.begin();

    // Initialize and configure sensors
    lsm6dsv.initializeSensor();
    kx122.initializeSensor();
    lsm303.initializeSensor();

    // Perform self-test and calibration
    lsm6dsv.selfTestAndCalibrate();
    kx122.selfTestAndCalibrate();
    lsm303.selfTestAndCalibrate();

    // Return the instances
    IMUInstances instances = {&lsm6dsv, &kx122, &lsm303};
    return instances;
}

void updateOrientation(float* orientationChange) {
    // Integrate orientation change into the current orientation
    currentOrientation[0] += orientationChange[0];
    currentOrientation[1] += orientationChange[1];
    currentOrientation[2] += orientationChange[2];
}

void getCurrentOrientation(float* orientation) {
    // Return the current orientation
    orientation[0] = currentOrientation[0];
    orientation[1] = currentOrientation[1];
    orientation[2] = currentOrientation[2];
}

void updatePosition(float* velocity, float deltaTime) {
    // Integrate velocity to update the current position
    currentPosition[0] += velocity[0] * deltaTime;
    currentPosition[1] += velocity[1] * deltaTime;
    currentPosition[2] += velocity[2] * deltaTime;
}

void getCurrentPosition(float* position) {
    // Return the current position  
    position[0] = currentPosition[0];
    position[1] = currentPosition[1];
    position[2] = currentPosition[2];
}

void computeDroneStatus(
    IMUInstances* imuInstances,
    float* acceleration, float* velocity, float* position,
    float* compassHeading, float* orientation, float* orientationChange) {
    // Variables for sensor readings
    float ax1, ay1, az1; // LSM6DSV accelerometer
    float gx, gy, gz;    // LSM6DSV gyroscope
    float ax2, ay2, az2; // KX122 accelerometer
    float mx, my, mz;    // LSM303AHTR magnetometer

    // Read data from LSM6DSV
    imuInstances->lsm6dsv->readAcceleration(&ax1, &ay1, &az1);
    imuInstances->lsm6dsv->readGyroscope(&gx, &gy, &gz);

    // Read data from KX122
    imuInstances->kx122->readFIFO(&ax2, &ay2, &az2);

    // Read data from LSM303AHTR
    imuInstances->lsm303->readMagnetometer(&mx, &my, &mz);

    // Combine accelerometer data (weighted average for higher accuracy)
    float ax = (ax1 + ax2) / 2.0f;
    float ay = (ay1 + ay2) / 2.0f;
    float az = (az1 + az2) / 2.0f;

    // Normalize accelerometer data
    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    ax /= accelMagnitude;
    ay /= accelMagnitude;
    az /= accelMagnitude;

    // Compute pitch and roll from accelerometer
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float roll = atan2(ay, az);

    // Compute yaw (compass heading) from magnetometer
    float magX = mx * cos(pitch) + mz * sin(pitch);
    float magY = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    float yaw = atan2(-magY, magX);

    // Convert yaw to compass heading in degrees
    *compassHeading = yaw * (180.0f / M_PI);
    if (*compassHeading < 0) {
        *compassHeading += 360.0f;
    }

    // Integrate gyroscope data to compute orientation change (using trapezoidal integration)
    static float prevGx = 0.0f, prevGy = 0.0f, prevGz = 0.0f;
    static float deltaTime = 0.01f; // Assume a fixed time step (10 ms)
    orientationChange[0] = (gx + prevGx) * 0.5f * deltaTime;
    orientationChange[1] = (gy + prevGy) * 0.5f * deltaTime;
    orientationChange[2] = (gz + prevGz) * 0.5f * deltaTime;
    prevGx = gx;
    prevGy = gy;
    prevGz = gz;

    // Update the global orientation tracker
    updateOrientation(orientationChange);

    // Get the current orientation from the tracker
    getCurrentOrientation(orientation);

    // Integrate accelerometer data to compute velocity and position (using trapezoidal integration)
    static float prevAx = 0.0f, prevAy = 0.0f, prevAz = 0.0f;
    static float velocityX = 0.0f, velocityY = 0.0f, velocityZ = 0.0f;

    velocityX += (ax + prevAx) * 0.5f * deltaTime;
    velocityY += (ay + prevAy) * 0.5f * deltaTime;
    velocityZ += (az + prevAz) * 0.5f * deltaTime;

    prevAx = ax;
    prevAy = ay;
    prevAz = az;

    // Update the global position tracker
    float velocityTemp[3] = {velocityX, velocityY, velocityZ};
    updatePosition(velocityTemp, deltaTime);

    // Get the current position from the tracker
    getCurrentPosition(position);

    // Output acceleration, velocity, and position
    acceleration[0] = ax;
    acceleration[1] = ay;
    acceleration[2] = az;

    velocity[0] = velocityX;
    velocity[1] = velocityY;
    velocity[2] = velocityZ;

    position[0] = currentPosition[0];
    position[1] = currentPosition[1];
    position[2] = currentPosition[2];
}

