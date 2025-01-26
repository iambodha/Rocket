#include "LSM6DS3.h"
#include "Wire.h"

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  // I2C device address 0x6A

// Accelerometer Bias Values
const float accelBiasX = 0.0037;
const float accelBiasY = 0.0009;
const float accelBiasZ = 0.0719;

// Gyroscope Bias Values
const float gyroBiasX = 1.2992;
const float gyroBiasY = -2.9302;
const float gyroBiasZ = -0.6944;

// Constants
const int SAMPLE_RATE_HZ = 81;  // Adjusted sample rate (81 Hz)
const unsigned long FLIGHT_DURATION_MS = 120000; // 2 minutes (120,000 ms)
const int MAX_SAMPLES = 1000;   // Reduced max samples for memory optimization
const int COMMAND_BUFFER_SIZE = 32;

// Data storage
struct SensorData {
    unsigned long timestamp;   // Timestamp in milliseconds
    int16_t accelX;            // Scaled accelerometer X (multiplied by 1000)
    int16_t accelY;            // Scaled accelerometer Y
    int16_t accelZ;            // Scaled accelerometer Z
    int16_t gyroX;             // Scaled gyroscope X
    int16_t gyroY;             // Scaled gyroscope Y
    int16_t gyroZ;             // Scaled gyroscope Z
};

SensorData data[MAX_SAMPLES];
int sampleCount = 0;
bool isLogging = false;
unsigned long startTime = 0;
char cmdBuffer[COMMAND_BUFFER_SIZE];
int cmdIndex = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for Serial connection

    // Initialize IMU
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
        while (1);
    }

    printHelp();
}

void loop() {
    // Handle command input
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);
                cmdIndex = 0;
            }
        } else if (cmdIndex < COMMAND_BUFFER_SIZE - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
    }

    // If logging is active, collect data
    if (isLogging && sampleCount < MAX_SAMPLES) {
        static unsigned long lastSampleTime = 0;
        unsigned long currentTime = millis();
        unsigned long sampleInterval = 1000 / SAMPLE_RATE_HZ;

        if (currentTime - lastSampleTime >= sampleInterval) {
            data[sampleCount].timestamp = currentTime - startTime;  // Relative time since logging started
            data[sampleCount].accelX = (myIMU.readFloatAccelX() - accelBiasX) * 1000;
            data[sampleCount].accelY = (myIMU.readFloatAccelY() - accelBiasY) * 1000;
            data[sampleCount].accelZ = (myIMU.readFloatAccelZ() - accelBiasZ) * 1000;
            data[sampleCount].gyroX = (myIMU.readFloatGyroX() - gyroBiasX) * 1000;
            data[sampleCount].gyroY = (myIMU.readFloatGyroY() - gyroBiasY) * 1000;
            data[sampleCount].gyroZ = (myIMU.readFloatGyroZ() - gyroBiasZ) * 1000;
            sampleCount++;
            lastSampleTime = currentTime;

            if (sampleCount >= MAX_SAMPLES) {
                isLogging = false;
                Serial.println("Buffer full! Logging stopped.");
            }
        }
    }
}

void processCommand(const char* cmd) {
    if (strcmp(cmd, "start") == 0) {
        if (!isLogging) {
            isLogging = true;
            startTime = millis();  // Reset start time
            sampleCount = 0;       // Reset sample count
            Serial.println("Starting data logging...");
        } else {
            Serial.println("Already logging!");
        }
    }
    else if (strcmp(cmd, "stop") == 0) {
        if (isLogging) {
            isLogging = false;
            Serial.println("Logging stopped.");
            Serial.print("Samples collected: ");
            Serial.println(sampleCount);
        } else {
            Serial.println("Not currently logging.");
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        printStatus();
    }
    else if (strcmp(cmd, "download") == 0) {
        downloadData();
    }
    else if (strcmp(cmd, "clear") == 0) {
        clearData();
    }
    else if (strcmp(cmd, "help") == 0) {
        printHelp();
    }
    else {
        Serial.println("Unknown command. Type 'help' for available commands.");
    }
}

void printStatus() {
    Serial.println("\n=== Status ===");
    Serial.print("Logging: ");
    Serial.println(isLogging ? "Active" : "Inactive");
    Serial.print("Samples stored: ");
    Serial.println(sampleCount);
    Serial.print("Buffer capacity: ");
    Serial.println(MAX_SAMPLES);
    Serial.print("Sample rate: ");
    Serial.print(SAMPLE_RATE_HZ);
    Serial.println(" Hz");
}

void downloadData() {
    if (sampleCount == 0) {
        Serial.println("No data to download.");
        return;
    }

    Serial.println("\nTime(ms),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    for (int i = 0; i < sampleCount; i++) {
        Serial.print(data[i].timestamp);
        Serial.print(",");
        Serial.print(data[i].accelX / 1000.0, 4);
        Serial.print(",");
        Serial.print(data[i].accelY / 1000.0, 4);
        Serial.print(",");
        Serial.print(data[i].accelZ / 1000.0, 4);
        Serial.print(",");
        Serial.print(data[i].gyroX / 1000.0, 4);
        Serial.print(",");
        Serial.print(data[i].gyroY / 1000.0, 4);
        Serial.print(",");
        Serial.println(data[i].gyroZ / 1000.0, 4);
    }
    Serial.println("Download complete.");
}

void clearData() {
    if (isLogging) {
        Serial.println("Cannot clear data while logging is active!");
        return;
    }

    sampleCount = 0;
    Serial.println("All data cleared.");
}

void printHelp() {
    Serial.println("\n=== Available Commands ===");
    Serial.println("start    - Begin data logging");
    Serial.println("stop     - Stop data logging");
    Serial.println("status   - Show current status");
    Serial.println("download - Download stored data as CSV");
    Serial.println("clear    - Erase all stored data");
    Serial.println("help     - Show this help message");
}
