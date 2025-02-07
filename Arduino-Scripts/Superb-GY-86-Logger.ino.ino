#include <Wire.h>

// Increased sampling frequency to 1000 Hz
const float FREQUENCY = 1000.0;  
const unsigned long INTERVAL = 1000.0 / FREQUENCY;
const int COMMAND_BUFFER_SIZE = 16;

// Simplified data structure for acceleration only
struct AccelData {
    uint16_t timestamp;    // Relative time in 1ms increments
    int16_t accelX;        // Accelerometer data (scaled by 1000)
    int16_t accelY;
    int16_t accelZ;
};

// Memory management
const int32_t MAX_SAMPLES = 32000;
AccelData* data = nullptr;
int sampleCount = 0;
bool isLogging = false;
char cmdBuffer[COMMAND_BUFFER_SIZE];
int cmdIndex = 0;
unsigned long startTime = 0;
unsigned long previousMillis = 0;

// Function declarations
void initializeSensor();
void collectData();
void processCommand(const char* cmd);
void downloadData();
void clearData();
void printStatus();
void printHelp();
void writeMPURegister(byte reg, byte value);

void setup() {
    Wire.begin();
    Serial.begin(115200);
    while (!Serial);

    data = new AccelData[MAX_SAMPLES];
    if (!data) {
        Serial.println("Memory allocation failed!");
        while(1);
    }

    initializeSensor();
    printHelp();
}

void loop() {
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

    if (isLogging) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= INTERVAL) {
            previousMillis = currentMillis;
            if (sampleCount >= MAX_SAMPLES) {
                isLogging = false;
                Serial.println("Memory full! Logging stopped.");
                return;
            }
            collectData();
        }
    }
}

void initializeSensor() {
    Serial.println("Initializing MPU6050...");
    
    // Initialize MPU6050 with maximum acceleration range
    writeMPURegister(0x6B, 0x80);  // Reset
    delay(100);
    writeMPURegister(0x6B, 0x00);  // Wake up
    writeMPURegister(0x19, 0x00);  // Sample rate = 1000Hz (8kHz/(7+1))
    writeMPURegister(0x1A, 0x00);  // Minimal filtering
    writeMPURegister(0x1C, 0x18);  // Accel config - ±16g
    
    Serial.println("Initialization complete!");
}

void collectData() {
    // Record timestamp relative to start of logging
    data[sampleCount].timestamp = (millis() - startTime);
    
    // Read MPU6050 accelerometer
    Wire.beginTransmission(0x68);  // MPU6050 I2C address
    Wire.write(0x3B);  // Start with X-axis high byte
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    
    if(Wire.available() == 6) {
        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        
        // Scale for maximum range (±16g)
        data[sampleCount].accelX = ax / 2048.0 * 1000;
        data[sampleCount].accelY = ay / 2048.0 * 1000;
        data[sampleCount].accelZ = az / 2048.0 * 1000;
    }
    
    sampleCount++;
}

void downloadData() {
    if (sampleCount == 0) {
        Serial.println("No data to download.");
        return;
    }

    Serial.println("Time(ms),AccelX(g),AccelY(g),AccelZ(g)");
    for (int i = 0; i < sampleCount; i++) {
        Serial.print(data[i].timestamp);
        Serial.print(",");
        Serial.print(data[i].accelX / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].accelY / 1000.0, 3);
        Serial.print(",");
        Serial.println(data[i].accelZ / 1000.0, 3);
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
    Serial.println("\n=== High-Frequency Acceleration Logger Commands ===");
    Serial.println("start    - Begin logging acceleration data");
    Serial.println("stop     - Stop logging");
    Serial.println("download - Get data as CSV");
    Serial.println("clear    - Erase all data");
    Serial.println("status   - Show logger status");
    Serial.println("help     - Show these commands");
}

void processCommand(const char* cmd) {
    if (strcmp(cmd, "start") == 0) {
        if (!isLogging) {
            startTime = millis();
            isLogging = true;
            sampleCount = 0;
            Serial.println("Starting high-frequency acceleration logging...");
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

void writeMPURegister(byte reg, byte value) {
    Wire.beginTransmission(0x68);  // MPU6050 I2C address
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}