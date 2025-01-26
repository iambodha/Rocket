#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Bias Values
const float accelBiasX = 0.0037;
const float accelBiasY = 0.0009;
const float accelBiasZ = 0.0719;
const float gyroBiasX = 1.2992;
const float gyroBiasY = -2.9302;
const float gyroBiasZ = -0.6944;

// Variables for velocity calculation
float verticalVelocity = 0.0;
unsigned long lastTime = 0;
const float gravity = 9.81; // m/s^2

// Function to calculate roll and pitch angles from accelerometer data
void calculateAngles(float ax, float ay, float az, float& roll, float& pitch) {
    // Roll (rotation around X-axis)
    roll = atan2(ay, sqrt(ax * ax + az * az));
    // Pitch (rotation around Y-axis)
    pitch = atan2(-ax, sqrt(ay * ay + az * az));
}

// Function to calculate vertical acceleration considering tilt
float calculateVerticalAcceleration(float ax, float ay, float az, float roll, float pitch) {
    // Convert accelerometer readings from g's to m/s^2
    ax *= gravity;
    ay *= gravity;
    az *= gravity;
    
    // Compensate for tilt using rotation matrices
    float verticalAccel = az * cos(pitch) * cos(roll) +
                         ay * sin(roll) +
                         ax * sin(pitch) * cos(roll);
    
    // Subtract gravity to get true acceleration
    return verticalAccel - gravity;
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
    
    lastTime = millis();
}

void loop() {
    // Read accelerometer data with bias correction
    float ax = myIMU.readFloatAccelX() - accelBiasX;
    float ay = myIMU.readFloatAccelY() - accelBiasY;
    float az = myIMU.readFloatAccelZ() - accelBiasZ;
    
    // Calculate roll and pitch angles
    float roll, pitch;
    calculateAngles(ax, ay, az, roll, pitch);
    
    // Calculate vertical acceleration
    float verticalAccel = calculateVerticalAcceleration(ax, ay, az, roll, pitch);
    
    // Calculate time delta
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;
    
    // Update vertical velocity using trapezoidal integration
    verticalVelocity += verticalAccel * deltaTime;
    
    // Print results
    //Serial.print("Roll (rad): ");
    //Serial.println(roll, 4);
    //Serial.print("Pitch (rad): ");
    //Serial.println(pitch, 4);
    //Serial.print("Vertical Acceleration (m/s^2): ");
    Serial.println(verticalAccel, 4);
    //Serial.print("Vertical Velocity (m/s): ");
    //Serial.println(verticalVelocity, 4);
    
    delay(100); // Shorter delay for more frequent measurements
}

