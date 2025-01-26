#include "LSM6DS3.h"
#include "Wire.h"

//Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// Gyroscope Bias Values
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    // Call .begin() to configure the IMU
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    // Calibrate Gyroscope
    calibrateGyro();
}

void loop() {
    // Read Gyroscope Data and Apply Bias Correction
    Serial.print("\nGyroscope:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatGyroX() - gyroBiasX, 4);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatGyroY() - gyroBiasY, 4);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatGyroZ() - gyroBiasZ, 4);

    delay(1000);
}

void calibrateGyro() {
    const int numSamples = 100;
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;

    Serial.println("Calibrating gyroscope... Please keep the IMU still.");

    for (int i = 0; i < numSamples; i++) {
        sumX += myIMU.readFloatGyroX();
        sumY += myIMU.readFloatGyroY();
        sumZ += myIMU.readFloatGyroZ();
        delay(20); // Wait 20ms between samples
    }

    gyroBiasX = sumX / numSamples;
    gyroBiasY = sumY / numSamples;
    gyroBiasZ = sumZ / numSamples;

    Serial.println("Gyroscope calibration complete.");
    Serial.print("Gyro Bias X: "); Serial.println(gyroBiasX, 4);
    Serial.print("Gyro Bias Y: "); Serial.println(gyroBiasY, 4);
    Serial.print("Gyro Bias Z: "); Serial.println(gyroBiasZ, 4);
}
