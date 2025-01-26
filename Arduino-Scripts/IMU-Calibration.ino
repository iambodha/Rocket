#include "LSM6DS3.h"
#include "Wire.h"

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

float accelBias[3] = {0, 0, 0};
float accelScale[3] = {1, 1, 1};
const int NUM_SAMPLES = 100;
float readings[6][3]; // Store mean values for each orientation
bool calibrationComplete = false;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    // Initialize the IMU
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    Serial.println("Enter 'CALIB <position>' to calibrate (e.g., 'CALIB 1' for +X).");
    Serial.println("Enter 'FINISH' after completing all 6 positions to calculate calibration.");
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("CALIB")) {
            int position = command.substring(6).toInt();
            if (position >= 1 && position <= 6) {
                calibratePoint(position - 1); // Calibrate the corresponding position
            } else {
                Serial.println("Invalid position. Enter a number between 1 and 6.");
            }
        } else if (command == "FINISH") {
            if (checkAllCalibrated()) {
                calculateCalibration();
                calibrationComplete = true;
                Serial.println("Calibration complete. Calibration applied to readings.");
            } else {
                Serial.println("Please calibrate all 6 positions before finishing.");
            }
        } else {
            Serial.println("Invalid command. Use 'CALIB <position>' or 'FINISH'.");
        }
    }

    if (calibrationComplete) {
        // Apply calibration to accelerometer readings
        float ax = (myIMU.readFloatAccelX() - accelBias[0]) * accelScale[0];
        float ay = (myIMU.readFloatAccelY() - accelBias[1]) * accelScale[1];
        float az = (myIMU.readFloatAccelZ() - accelBias[2]) * accelScale[2];

        // Print calibrated accelerometer values
        Serial.print("\nCalibrated Accelerometer:\n");
        Serial.print(" X = ");
        Serial.println(ax, 4);
        Serial.print(" Y = ");
        Serial.println(ay, 4);
        Serial.print(" Z = ");
        Serial.println(az, 4);

        delay(500);
    }
}

void calibratePoint(int index) {
    Serial.print("Calibrating position ");
    Serial.print(index + 1);
    Serial.println(". Keep the IMU still in this position.");

    float meanVals[3] = {0, 0, 0};
    for (int j = 0; j < NUM_SAMPLES; j++) {
        meanVals[0] += myIMU.readFloatAccelX();
        meanVals[1] += myIMU.readFloatAccelY();
        meanVals[2] += myIMU.readFloatAccelZ();
        delay(10); // Sampling interval
    }
    readings[index][0] = meanVals[0] / NUM_SAMPLES;
    readings[index][1] = meanVals[1] / NUM_SAMPLES;
    readings[index][2] = meanVals[2] / NUM_SAMPLES;

    // Print the raw values captured for the position
    Serial.print("Measured values for position ");
    Serial.print(index + 1);
    Serial.print(": X = ");
    Serial.print(readings[index][0], 4);
    Serial.print(", Y = ");
    Serial.print(readings[index][1], 4);
    Serial.print(", Z = ");
    Serial.println(readings[index][2], 4);

    // Print bias for the current position
    Serial.print("Bias for position ");
    Serial.print(index + 1);
    Serial.print(": ");
    Serial.print(" X = ");
    Serial.print(-readings[index][0], 4);
    Serial.print(", Y = ");
    Serial.print(-readings[index][1], 4);
    Serial.print(", Z = ");
    Serial.println(-readings[index][2], 4);
}

bool checkAllCalibrated() {
    for (int i = 0; i < 6; i++) {
        if (readings[i][0] == 0 && readings[i][1] == 0 && readings[i][2] == 0) {
            return false; // At least one position hasn't been calibrated
        }
    }
    return true;
}

void calculateCalibration() {
    // Compute scale and bias
    accelBias[0] = -0.5 * (readings[0][0] + readings[1][0]);
    accelBias[1] = -0.5 * (readings[2][1] + readings[3][1]);
    accelBias[2] = -0.5 * (readings[4][2] + readings[5][2]);

    accelScale[0] = 2.0 / abs(readings[0][0] - readings[1][0]);
    accelScale[1] = 2.0 / abs(readings[2][1] - readings[3][1]);
    accelScale[2] = 2.0 / abs(readings[4][2] - readings[5][2]);

    // Print the final calculated calibration parameters
    Serial.println("Calibration parameters calculated:");
    Serial.print("Accel Bias: X = ");
    Serial.print(accelBias[0], 4);
    Serial.print(", Y = ");
    Serial.print(accelBias[1], 4);
    Serial.print(", Z = ");
    Serial.println(accelBias[2], 4);

    Serial.print("Accel Scale: X = ");
    Serial.print(accelScale[0], 4);
    Serial.print(", Y = ");
    Serial.print(accelScale[1], 4);
    Serial.print(", Z = ");
    Serial.println(accelScale[2], 4);
}