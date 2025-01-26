#include "LSM6DS3.h"
#include "Wire.h"

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

// Accelerometer Bias Values (in g)
const float accelBiasX = 0.0037;
const float accelBiasY = 0.0009;
const float accelBiasZ = 0.0719;

// Gravitational acceleration constant in m/s²
const float GRAVITY = 9.81;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    // Initialize the IMU
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
}

void loop() {
    // Read Accelerometer Data (in g) and Apply Bias Correction
    float accelX_g = myIMU.readFloatAccelX() - accelBiasX;
    float accelY_g = myIMU.readFloatAccelY() - accelBiasY;
    float accelZ_g = myIMU.readFloatAccelZ() - accelBiasZ;

    // Convert to m/s²
    float accelX_ms2 = accelX_g * GRAVITY;
    float accelY_ms2 = accelY_g * GRAVITY;
    float accelZ_ms2 = accelZ_g * GRAVITY;

    // Calculate vertical acceleration
    // Assume Z-axis is aligned with gravity (downward)
    float verticalAccel = accelZ_ms2 - GRAVITY;

    // Output data
    //Serial.print("\nAccelerometer (Corrected and Converted to m/s²):\n");
    //Serial.print(" X = ");
    //Serial.println(accelX_ms2, 4);
    //Serial.print(" Y = ");
    //Serial.println(accelY_ms2, 4);
    //Serial.print(" Z = ");
    //Serial.println(accelZ_ms2, 4);

    //Serial.print("\nVertical Acceleration:\n");
    //Serial.print(" Vertical Accel = ");
    Serial.println(verticalAccel, 4); // In m/s²

    // Delay for readability
    delay(100);
}
