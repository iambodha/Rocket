#include "LSM6DS3.h"
#include "Wire.h"

// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

// Accelerometer Bias Values
const float accelBiasX = 0.0037;
const float accelBiasY = 0.0009;
const float accelBiasZ = 0.0719;

// Gyroscope Bias Values
const float gyroBiasX = 1.2992;
const float gyroBiasY = -2.9302;
const float gyroBiasZ = -0.6944;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial);
    // Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
}

void loop() {
    // Read Accelerometer Data and Apply Bias Correction
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatAccelX() - accelBiasX, 4);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatAccelY() - accelBiasY, 4);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatAccelZ() - accelBiasZ, 4);

    // Read Gyroscope Data and Apply Bias Correction
    Serial.print("\nGyroscope:\n");
    Serial.print(" X = ");
    Serial.println(myIMU.readFloatGyroX() - gyroBiasX, 4);
    Serial.print(" Y = ");
    Serial.println(myIMU.readFloatGyroY() - gyroBiasY, 4);
    Serial.print(" Z = ");
    Serial.println(myIMU.readFloatGyroZ() - gyroBiasZ, 4);

    // Thermometer
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C = ");
    Serial.println(myIMU.readTempC(), 4);
    Serial.print(" Degrees F = ");
    Serial.println(myIMU.readTempF(), 4);

    delay(100);
}
