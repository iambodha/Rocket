#include <Wire.h>

// Device addresses and registers
#define MPU6050_ADDRESS  0x68
#define HMC5883L_ADDRESS 0x1E
#define MS5611_ADDRESS   0x77

// Timing configuration - Increased for rocket
const float FREQUENCY = 200.0;  // 200 Hz sampling
const unsigned long INTERVAL = 1000.0 / FREQUENCY;
const int COMMAND_BUFFER_SIZE = 16;

// Data structure for sensor readings
struct SensorData {
    uint16_t timestamp;    // Relative time in 5ms increments
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

// Rocket-specific metrics
struct RocketMetrics {
    // Peak values
    float peak_acceleration;
    float peak_rotation;
    float max_altitude;
    
    // Simple spike filter
    static const int SPIKE_WINDOW = 3;
    float recent_accels[SPIKE_WINDOW];
    float recent_gyros[SPIKE_WINDOW];
    int buffer_index;
    
    RocketMetrics() {
        peak_acceleration = 0;
        peak_rotation = 0;
        max_altitude = 0;
        buffer_index = 0;
        for(int i = 0; i < SPIKE_WINDOW; i++) {
            recent_accels[i] = 0;
            recent_gyros[i] = 0;
        }
    }
};

// Memory management
const int MAX_SAMPLES = 8130;  // 40 seconds at 200Hz
SensorData* data = nullptr;
RocketMetrics metrics;
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
void processRocketData(SensorData& reading);
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

    data = new SensorData[MAX_SAMPLES];
    if (!data) {
        Serial.println("Memory allocation failed!");
        while(1);
    }

    initializeSensors();
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

void initializeSensors() {
    Serial.println("Initializing sensors...");
    
    // Initialize MPU6050 with maximum ranges
    writeMPURegister(0x6B, 0x80);  // Reset
    delay(100);
    writeMPURegister(0x6B, 0x00);  // Wake up
    writeMPURegister(0x19, 0x04);  // Sample rate = 200Hz (8kHz/(39+1))
    writeMPURegister(0x1A, 0x01);  // Config - minimal filtering
    writeMPURegister(0x1B, 0x18);  // Gyro config - ±2000°/s
    writeMPURegister(0x1C, 0x18);  // Accel config - ±16g
    
    // Enable I2C bypass for HMC5883L
    writeMPURegister(0x6A, 0x00);
    writeMPURegister(0x37, 0x02);
    delay(100);
    
    // Initialize HMC5883L - highest speed
    writeHMCRegister(0x00, 0x18);  // 75Hz, no averaging
    writeHMCRegister(0x01, 0x20);  // Default gain
    writeHMCRegister(0x02, 0x00);  // Continuous measurement
    
    // Initialize MS5611
    resetMS5611();
    delay(100);
    readCalibrationMS5611();
    
    Serial.println("Initialization complete!");
}

void collectData() {
    data[sampleCount].timestamp = (millis() - startTime) / 5;  // 5ms resolution
    
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
        
        // Scale for maximum ranges
        data[sampleCount].accelX = ax / 2048.0 * 1000;  // ±16g range
        data[sampleCount].accelY = ay / 2048.0 * 1000;
        data[sampleCount].accelZ = az / 2048.0 * 1000;
        data[sampleCount].gyroX = gx * (2000.0 / 32768.0) * 1000;  // ±2000°/s
        data[sampleCount].gyroY = gy * (2000.0 / 32768.0) * 1000;
        data[sampleCount].gyroZ = gz * (2000.0 / 32768.0) * 1000;
        data[sampleCount].mpuTemp = (temp / 340.0 + 36.53) * 100;
    }
    
    // Read HMC5883L
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(HMC5883L_ADDRESS, 6, true);
    
    if(Wire.available() == 6) {
        int16_t mx = Wire.read() << 8 | Wire.read();
        int16_t mz = Wire.read() << 8 | Wire.read();
        int16_t my = Wire.read() << 8 | Wire.read();
        
        data[sampleCount].magnX = mx * 0.92;
        data[sampleCount].magnY = my * 0.92;
        data[sampleCount].magnZ = mz * 0.92;
    }
    
    // Read MS5611 with faster timing
    readMS5611Data();
    
    // Process the data
    processRocketData(data[sampleCount]);
    
    sampleCount++;
}

void processRocketData(SensorData& reading) {
    // Convert readings to real units
    float accel_x = reading.accelX / 1000.0;
    float accel_y = reading.accelY / 1000.0;
    float accel_z = reading.accelZ / 1000.0;
    
    float gyro_x = reading.gyroX / 1000.0;
    float gyro_y = reading.gyroY / 1000.0;
    float gyro_z = reading.gyroZ / 1000.0;
    
    // Calculate magnitudes
    float total_accel = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    float total_gyro = sqrt(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
    
    // Update peak values
    if(total_accel > metrics.peak_acceleration) {
        metrics.peak_acceleration = total_accel;
    }
    if(total_gyro > metrics.peak_rotation) {
        metrics.peak_rotation = total_gyro;
    }
    
    // Spike filtering
    metrics.recent_accels[metrics.buffer_index] = total_accel;
    metrics.recent_gyros[metrics.buffer_index] = total_gyro;
    
    float avg_accel = 0;
    float avg_gyro = 0;
    for(int i = 0; i < RocketMetrics::SPIKE_WINDOW; i++) {
        avg_accel += metrics.recent_accels[i];
        avg_gyro += metrics.recent_gyros[i];
    }
    avg_accel /= RocketMetrics::SPIKE_WINDOW;
    avg_gyro /= RocketMetrics::SPIKE_WINDOW;
    
    // Only filter extreme spikes
    if(total_accel > avg_accel * 3) {
        reading.accelX = reading.accelX * 0.5;
        reading.accelY = reading.accelY * 0.5;
        reading.accelZ = reading.accelZ * 0.5;
    }
    
    // Calculate altitude
    float altitude = 44330.0 * (1 - pow(reading.pressure / 101325.0, 0.1903));
    if(altitude > metrics.max_altitude) {
        metrics.max_altitude = altitude;
    }
    
    metrics.buffer_index = (metrics.buffer_index + 1) % RocketMetrics::SPIKE_WINDOW;
}

void readMS5611Data() {
    // Faster pressure readings
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x48);
    Wire.endTransmission();
    delayMicroseconds(10000);
    
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    if(Wire.available() >= 3) {
        MS5611_D1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x58);
    Wire.endTransmission();
    delayMicroseconds(10000);
    
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    if(Wire.available() >= 3) {
        MS5611_D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    
    // Calculate pressure and temperature
    int32_t dT = MS5611_D2 - ((uint32_t)C[5] << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * C[6] >> 23);
    int64_t OFF = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT >> 7);
    int64_t SENS = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT >> 8);
    
    data[sampleCount].baroTemp = TEMP / 10;
    data[sampleCount].pressure = ((((int64_t)MS5611_D1 * SENS) >> 21) - OFF) >> 15;
}

void downloadData() {
    if (sampleCount == 0) {
        Serial.println("No data to download.");
        return;
    }

    Serial.println("Time(ms),AccelX(g),AccelY(g),AccelZ(g),GyroX(deg/s),GyroY(deg/s),GyroZ(deg/s),MagnX(gauss),MagnY(gauss),MagnZ(gauss),MPUTemp(C),BaroTemp(C),Pressure(Pa)");
    for (int i = 0; i < sampleCount; i++) {
        Serial.print(data[i].timestamp * 5);
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
    // Print flight metrics at the end of downloadData()
    Serial.println("\nFlight Metrics:");
    Serial.print("Peak Acceleration: "); Serial.print(metrics.peak_acceleration, 2); Serial.println(" g");
    Serial.print("Peak Rotation Rate: "); Serial.print(metrics.peak_rotation, 2); Serial.println(" rad/s");
    Serial.print("Maximum Altitude: "); Serial.print(metrics.max_altitude, 2); Serial.println(" m");
    Serial.println("Download complete.");
}

void clearData() {
    if (isLogging) {
        Serial.println("Cannot clear while logging!");
        return;
    }
    sampleCount = 0;
    // Reset metrics
    metrics.peak_acceleration = 0;
    metrics.peak_rotation = 0;
    metrics.max_altitude = 0;
    metrics.buffer_index = 0;
    for(int i = 0; i < RocketMetrics::SPIKE_WINDOW; i++) {
        metrics.recent_accels[i] = 0;
        metrics.recent_gyros[i] = 0;
    }
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
    if (sampleCount > 0) {
        Serial.println("\nCurrent Flight Metrics:");
        Serial.print("Peak Acceleration: "); Serial.print(metrics.peak_acceleration, 2); Serial.println(" g");
        Serial.print("Peak Rotation: "); Serial.print(metrics.peak_rotation, 2); Serial.println(" rad/s");
        Serial.print("Max Altitude: "); Serial.print(metrics.max_altitude, 2); Serial.println(" m");
    }
}

void printHelp() {
    Serial.println("\n=== Rocket Data Logger Commands ===");
    Serial.println("start    - Begin logging flight data");
    Serial.println("stop     - Stop logging");
    Serial.println("download - Get data as CSV + flight metrics");
    Serial.println("clear    - Erase all data");
    Serial.println("status   - Show logger status");
    Serial.println("help     - Show these commands");
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

void processCommand(const char* cmd) {
    if (strcmp(cmd, "start") == 0) {
        if (!isLogging) {
            startTime = millis();
            isLogging = true;
            sampleCount = 0;
            // Reset metrics for new flight
            metrics.peak_acceleration = 0;
            metrics.peak_rotation = 0;
            metrics.max_altitude = 0;
            Serial.println("Starting flight data logging...");
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