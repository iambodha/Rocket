#include <Wire.h>

// Device addresses and registers
#define MPU6050_ADDRESS  0x68
#define HMC5883L_ADDRESS 0x1E
#define MS5611_ADDRESS   0x77

// Timing configuration
const float FREQUENCY = 50.0;  // 50 Hz sampling
const unsigned long INTERVAL = 1000.0 / FREQUENCY;
const int COMMAND_BUFFER_SIZE = 16;

// Data structure for sensor readings (34 bytes per sample)
struct SensorData {
    uint16_t timestamp;    // Relative time in 20ms increments
    int16_t accelX;       // Accelerometer data (scaled by 1000)
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;        // Gyroscope data (scaled by 1000)
    int16_t gyroY;
    int16_t gyroZ;
    int16_t magnX;        // Magnetometer data (scaled by 1000)
    int16_t magnY;
    int16_t magnZ;
    int16_t mpuTemp;      // MPU6050 Temperature (scaled by 100)
    int16_t baroTemp;     // MS5611 Temperature (scaled by 100)
    uint32_t pressure;    // Pressure (Pa)
};

// Memory management
const int MAX_SAMPLES = 8130;  // 2.7 minutes at 50Hz
SensorData* data = nullptr;
int sampleCount = 0;
bool isLogging = false;
char cmdBuffer[COMMAND_BUFFER_SIZE];
int cmdIndex = 0;
unsigned long startTime = 0;
unsigned long previousMillis = 0;

// MS5611 calibration
uint16_t C[7];
uint32_t MS5611_D1, MS5611_D2;

// Function declarations
void initializeSensors();
void collectData();
void processCommand(const char* cmd);
void downloadData();
void clearData();
void printStatus();
void printHelp();
void writeMPURegister(byte reg, byte value);
void writeHMCRegister(byte reg, byte value);
void resetMS5611();
void readCalibrationMS5611();
void readMS5611Data();

void setup() {
    Wire.begin();
    Serial.begin(115200);
    while (!Serial);

    // Allocate memory
    data = new SensorData[MAX_SAMPLES];
    if (!data) {
        Serial.println("Memory allocation failed!");
        while(1);
    }

    // Initialize sensors
    initializeSensors();
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

void initializeSensors() {
    Serial.println("Initializing sensors...");
    
    // Initialize MPU6050
    writeMPURegister(0x6B, 0x80);  // Reset
    delay(100);
    writeMPURegister(0x6B, 0x00);  // Wake up
    writeMPURegister(0x19, 0x0F);  // Sample rate = 50Hz
    writeMPURegister(0x1A, 0x03);  // Config
    writeMPURegister(0x1B, 0x18);  // Gyro config
    writeMPURegister(0x1C, 0x08);  // Accel config
    
    // Enable I2C bypass for HMC5883L
    writeMPURegister(0x6A, 0x00);
    writeMPURegister(0x37, 0x02);
    delay(100);
    
    // Initialize HMC5883L
    writeHMCRegister(0x00, 0x70);
    writeHMCRegister(0x01, 0x20);
    writeHMCRegister(0x02, 0x00);
    
    // Initialize MS5611
    resetMS5611();
    delay(100);
    readCalibrationMS5611();
    
    Serial.println("Initialization complete!");
}

void collectData() {
    data[sampleCount].timestamp = (millis() - startTime) / 20;
    
    // Read MPU6050 (accelerometer, gyroscope, and temperature)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true);
    
    if(Wire.available() == 14) {
        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        int16_t temp = Wire.read() << 8 | Wire.read();
        int16_t gx = Wire.read() << 8 | Wire.read();
        int16_t gy = Wire.read() << 8 | Wire.read();
        int16_t gz = Wire.read() << 8 | Wire.read();
        
        data[sampleCount].accelX = ax / 8192.0 * 1000;  // Store as milli-g
        data[sampleCount].accelY = ay / 8192.0 * 1000;
        data[sampleCount].accelZ = az / 8192.0 * 1000;
        data[sampleCount].gyroX = gx * (2000.0 / 32768.0) * 1000;  // Store as milli-rad/s
        data[sampleCount].gyroY = gy * (2000.0 / 32768.0) * 1000;
        data[sampleCount].gyroZ = gz * (2000.0 / 32768.0) * 1000;
        data[sampleCount].mpuTemp = (temp / 340.0 + 36.53) * 100;  // Store as centi-degrees
    }
    
    // Read HMC5883L (magnetometer)
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(HMC5883L_ADDRESS, 6, true);
    
    if(Wire.available() == 6) {
        int16_t mx = Wire.read() << 8 | Wire.read();
        int16_t mz = Wire.read() << 8 | Wire.read();
        int16_t my = Wire.read() << 8 | Wire.read();
        
        data[sampleCount].magnX = mx * 0.92;  // Store as micro-gauss
        data[sampleCount].magnY = my * 0.92;
        data[sampleCount].magnZ = mz * 0.92;
    }
    
    // Read MS5611 (pressure and temperature)
    readMS5611Data();
    
    // Calculate pressure and temperature
    int32_t dT = MS5611_D2 - ((uint32_t)C[5] << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * C[6] >> 23);
    int64_t OFF = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT >> 7);
    int64_t SENS = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT >> 8);
    
    data[sampleCount].baroTemp = TEMP / 10;  // Store as centi-degrees
    data[sampleCount].pressure = ((((int64_t)MS5611_D1 * SENS) >> 21) - OFF) >> 15;
    
    sampleCount++;
}

void readMS5611Data() {
    // Start D1 conversion
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x48);  // Convert D1 (OSR=4096)
    Wire.endTransmission();
    delay(10);
    
    // Read D1
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    if(Wire.available() >= 3) {
        MS5611_D1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    
    // Start D2 conversion
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x58);  // Convert D2 (OSR=4096)
    Wire.endTransmission();
    delay(10);
    
    // Read D2
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    if(Wire.available() >= 3) {
        MS5611_D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
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

    Serial.println("Time(ms),AccelX(g),AccelY(g),AccelZ(g),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),MagnX(gauss),MagnY(gauss),MagnZ(gauss),MPUTemp(C),BaroTemp(C),Pressure(Pa)");
    for (int i = 0; i < sampleCount; i++) {
        Serial.print(data[i].timestamp * 20);  // Convert back to milliseconds
        Serial.print(",");
        Serial.print(data[i].accelX / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].accelY / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].accelZ / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].gyroX / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].gyroY / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].gyroZ / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].magnX / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].magnY / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].magnZ / 1000.0, 3);
        Serial.print(",");
        Serial.print(data[i].mpuTemp / 100.0, 2);
        Serial.print(",");
        Serial.print(data[i].baroTemp / 100.0, 2);
        Serial.print(",");
        Serial.println(data[i].pressure);
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

void writeMPURegister(byte reg, byte value) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void writeHMCRegister(byte reg, byte value) {
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void resetMS5611() {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x1E);
    Wire.endTransmission();
    delay(100);
}

void readCalibrationMS5611() {
    for (uint8_t i = 1; i <= 6; i++) {
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(0xA0 + (i * 2));
        Wire.endTransmission();
        Wire.requestFrom(MS5611_ADDRESS, 2);
        if(Wire.available() >= 2) {
            C[i] = (Wire.read() << 8) | Wire.read();
        }
    }
}