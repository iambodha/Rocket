#include "LSM6DS3.h"
#include "Wire.h"

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Bias and scale values
const float accelBiasX = 0.0041, accelBiasY = 0, accelBiasZ = 0.068;
const float gyroBiasX = 0.4154, gyroBiasY = -1.9169, gyroBiasZ = -0.9495;

// Timing configuration
const float FREQUENCY = 50.0;  // 50 Hz sampling
const unsigned long INTERVAL = 1000 / FREQUENCY;
const int COMMAND_BUFFER_SIZE = 16;

// Memory-optimized data structure (14 bytes per sample)
struct SensorData {
    uint16_t timestamp;  // Relative time in 20ms increments (covers up to 21.8 minutes)
    int16_t accelX;     // Scaled by 1000 to store as integer
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
};

// Calculate maximum samples based on available memory
const int MAX_SAMPLES = 6000;  // 2 minutes at 50Hz
SensorData* data = nullptr;    // Dynamic allocation
int sampleCount = 0;
bool isLogging = false;
char cmdBuffer[COMMAND_BUFFER_SIZE];
int cmdIndex = 0;
unsigned long startTime = 0;
unsigned long previousMillis = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Allocate memory for data storage
    data = new SensorData[MAX_SAMPLES];
    if (!data) {
        Serial.println("Memory allocation failed!");
        while(1);  // Stop execution
    }

    if (myIMU.begin() != 0) {
        Serial.println("IMU error");
        while(1);
    }
    
    printHelp();
}

void loop() {
    // Handle commands
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

    // Collect data if logging
    if (isLogging && (millis() - previousMillis >= INTERVAL)) {
        if (sampleCount >= MAX_SAMPLES) {
            isLogging = false;
            Serial.println("Memory full! Logging stopped.");
            return;
        }
        
        previousMillis = millis();
        collectData();
    }
}

void collectData() {
    float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    int numSamples = 0;
    
    unsigned long startSample = millis();
    while (millis() - startSample < INTERVAL) {
        // Read and correct accelerometer data
        sumAccelX += myIMU.readFloatAccelX() - accelBiasX;
        sumAccelY += myIMU.readFloatAccelY() - accelBiasY;
        sumAccelZ += myIMU.readFloatAccelZ() - accelBiasZ;

        // Read and correct gyroscope data
        sumGyroX += myIMU.readFloatGyroX() - gyroBiasX;
        sumGyroY += myIMU.readFloatGyroY() - gyroBiasY;
        sumGyroZ += myIMU.readFloatGyroZ() - gyroBiasZ;

        numSamples++;
    }

    // Calculate averages and store in memory
    data[sampleCount].timestamp = (millis() - startTime) / 20;  // Store in 20ms units
    data[sampleCount].accelX = (sumAccelX / numSamples) * 1000;
    data[sampleCount].accelY = (sumAccelY / numSamples) * 1000;
    data[sampleCount].accelZ = (sumAccelZ / numSamples) * 1000;
    data[sampleCount].gyroX = (sumGyroX / numSamples) * 1000;
    data[sampleCount].gyroY = (sumGyroY / numSamples) * 1000;
    data[sampleCount].gyroZ = (sumGyroZ / numSamples) * 1000;
    
    sampleCount++;
}

void processCommand(const char* cmd) {
    if (strcmp(cmd, "start") == 0) {
        if (!isLogging) {
            startTime = millis();
            isLogging = true;
            sampleCount = 0;
            Serial.println("Starting data logging...");
        } else {
            Serial.println("Already logging!");
        }
    }
    else if (strcmp(cmd, "stop") == 0) {
        if (isLogging) {
            isLogging = false;
            Serial.print("Logging stopped. Samples collected: ");
            Serial.println(sampleCount);
        } else {
            Serial.println("Not currently logging.");
        }
    }
    else if (strcmp(cmd, "download") == 0) {
        downloadData();
    }
    else if (strcmp(cmd, "status") == 0) {
        printStatus();
    }
    else if (strcmp(cmd, "clear") == 0) {
        clearData();
    }
    else if (strcmp(cmd, "help") == 0) {
        printHelp();
    }
    else {
        Serial.println("Unknown command. Type 'help' for commands.");
    }
}

void downloadData() {
    if (sampleCount == 0) {
        Serial.println("No data to download.");
        return;
    }

    Serial.println("Time(ms),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    for (int i = 0; i < sampleCount; i++) {
        Serial.print(data[i].timestamp * 20);  // Convert back to milliseconds
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
        Serial.println("Cannot clear while logging!");
        return;
    }
    sampleCount = 0;
    Serial.println("Data cleared.");
}

void printStatus() {
    Serial.println("\n=== Status ===");
    Serial.print("Logging: ");
    Serial.println(isLogging ? "Active" : "Inactive");
    Serial.print("Samples stored: ");
    Serial.println(sampleCount);
    Serial.print("Memory usage: ");
    Serial.print((sampleCount * 100) / MAX_SAMPLES);
    Serial.println("%");
    Serial.print("Sample rate: ");
    Serial.print(FREQUENCY);
    Serial.println(" Hz");
}

void printHelp() {
    Serial.println("\n=== Commands ===");
    Serial.println("start    - Begin logging");
    Serial.println("stop     - Stop logging");
    Serial.println("download - Get data as CSV");
    Serial.println("clear    - Erase data");
    Serial.println("status   - Show status");
    Serial.println("help     - Show commands");
}
