#include "LSM6DS3.h"
#include "Wire.h"

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

// Accelerometer Bias Values (from calibration)
const float accelBiasX = 0.0041;
const float accelBiasY = 0;
const float accelBiasZ = 0.068;

// Accelerometer Scale Factors (from calibration, used for scaling)
const float accelScaleX = 1;
const float accelScaleY = 1;
const float accelScaleZ = 1;

const float gyroBiasX = 0.4154;  // Update if calibration for gyro was performed
const float gyroBiasY = -1.9169;
const float gyroBiasZ = -0.9495;

// Gyroscope Scale Factors (from calibration, used for scaling)
const float gyroScaleX = 1;
const float gyroScaleY = 1;
const float gyroScaleZ = 1;

// Desired frequency in Hz
const float frequency = 1.0; // Set frequency to 1 Hz (update as needed)
const unsigned long interval = 1000 / frequency; // Interval in milliseconds

unsigned long previousMillis = 0; // Stores last time the data was updated
unsigned long elapsedTime = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    // Call .begin() to configure the IMU
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
}

void loop() {
    // Check if the interval has passed based on the frequency
    if (millis() - previousMillis >= interval) {
        previousMillis = millis(); // Update the last time data was collected

        // Initialize sums for accelerometer and gyroscope readings
        float sumAccelX = 0.0;
        float sumAccelY = 0.0;
        float sumAccelZ = 0.0;
        float sumGyroX = 0.0;
        float sumGyroY = 0.0;
        float sumGyroZ = 0.0;
        int numSamples = 0;

        unsigned long startTime = millis();
        unsigned long elapsedTime = 0;

        // Collect data for the duration of the interval
        while (elapsedTime < interval) {
            // Read accelerometer data, apply bias correction and scaling
            float accelX = (myIMU.readFloatAccelX() - accelBiasX) * accelScaleX;
            float accelY = (myIMU.readFloatAccelY() - accelBiasY) * accelScaleY;
            float accelZ = (myIMU.readFloatAccelZ() - accelBiasZ) * accelScaleZ;

            // Read gyroscope data, apply bias correction and scaling
            float gyroX = (myIMU.readFloatGyroX() - gyroBiasX) * gyroScaleX;
            float gyroY = (myIMU.readFloatGyroY() - gyroBiasY) * gyroScaleY;
            float gyroZ = (myIMU.readFloatGyroZ() - gyroBiasZ) * gyroScaleZ;

            // Sum accelerometer and gyroscope readings
            sumAccelX += accelX;
            sumAccelY += accelY;
            sumAccelZ += accelZ;
            sumGyroX += gyroX;
            sumGyroY += gyroY;
            sumGyroZ += gyroZ;
            numSamples++;

            elapsedTime = millis() - startTime;
        }

        // Calculate the averages for accelerometer and gyroscope
        float avgAccelX = sumAccelX / numSamples;
        float avgAccelY = sumAccelY / numSamples;
        float avgAccelZ = sumAccelZ / numSamples;

        float avgGyroX = sumGyroX / numSamples;
        float avgGyroY = sumGyroY / numSamples;
        float avgGyroZ = sumGyroZ / numSamples;

        // Print average results for accelerometer and gyroscope
        Serial.print("Average Accelerometer Readings at ");
        Serial.print(frequency);
        Serial.println(" Hz:");
        Serial.print(" X = ");
        Serial.println(avgAccelX, 4);
        Serial.print(" Y = ");
        Serial.println(avgAccelY, 4);
        Serial.print(" Z = ");
        Serial.println(avgAccelZ, 4);
        
        Serial.print("Average Gyroscope Readings at ");
        Serial.print(frequency);
        Serial.println(" Hz:");
        Serial.print(" X = ");
        Serial.println(avgGyroX, 4);
        Serial.print(" Y = ");
        Serial.println(avgGyroY, 4);
        Serial.print(" Z = ");
        Serial.println(avgGyroZ, 4);
        
        Serial.print("Number of Samples: ");
        Serial.println(numSamples);
    }
}
