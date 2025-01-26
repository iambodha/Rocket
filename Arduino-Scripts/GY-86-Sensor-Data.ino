#include <Wire.h>

// I2C device addresses
#define MPU6050_ADDRESS  0x68
#define HMC5883L_ADDRESS 0x1E
#define MS5611_ADDRESS   0x77

// MPU6050 registers
#define MPU6050_USER_CTRL        0x6A
#define MPU6050_INT_PIN_CFG      0x37
#define MPU6050_INT_ENABLE       0x38
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_I2C_MST_CTRL     0x24

// Global variables for sensor readings
float accel_x_g, accel_y_g, accel_z_g;
float mpu_temp_c;
float gyro_x_rad, gyro_y_rad, gyro_z_rad;
float magn_x_gs, magn_y_gs, magn_z_gs;
double pressure_mbar;
float baro_temp_c;

// MS5611 calibration coefficients and raw data
uint16_t C[7];
uint32_t MS5611_D1, MS5611_D2;  // Renamed from D1 and D2 to avoid conflict

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\nStarting GY-86 initialization...");
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  
  // Reset MPU6050
  writeMPURegister(MPU6050_PWR_MGMT_1, 0x80);  // Reset device
  delay(100);
  
  // Wake up MPU6050
  writeMPURegister(MPU6050_PWR_MGMT_1, 0x00);
  delay(100);
  
  // Configure MPU6050
  writeMPURegister(0x19, 0x07);    // Sample rate = 1kHz / (1 + 7) = 125Hz
  writeMPURegister(0x1A, 0x03);    // Config: Digital Low Pass Filter
  writeMPURegister(0x1B, 0x18);    // Gyro: ±2000dps
  writeMPURegister(0x1C, 0x08);    // Accel: ±4g
  
  // Enable I2C bypass mode to access HMC5883L
  writeMPURegister(MPU6050_USER_CTRL, 0x00);    // Disable master mode
  writeMPURegister(MPU6050_INT_PIN_CFG, 0x02);  // Enable bypass mode
  delay(100);
  
  // Initialize HMC5883L
  Serial.println("Initializing HMC5883L...");
  
  // Test HMC5883L communication
  Wire.beginTransmission(HMC5883L_ADDRESS);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("HMC5883L found!");
    
    writeHMCRegister(0x00, 0x70);  // 8-sample averaging, 15Hz, normal measurement
    writeHMCRegister(0x01, 0x20);  // Range = ±1.3Ga
    writeHMCRegister(0x02, 0x00);  // Continuous measurement
  } else {
    Serial.println("HMC5883L not found! Check connections");
  }
  
  // Initialize MS5611
  Serial.println("Initializing MS5611...");
  resetMS5611();
  delay(100);
  readCalibrationMS5611();
  
  // Verify MPU6050 readings
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 1);
  byte whoAmI = Wire.read();
  Serial.print("MPU6050 WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);
  
  Serial.println("Initialization complete!\n");
}

void loop() {
  readSensors();
  readMS5611();
  printReadings();
  delay(100);
}

void readSensors() {
  // Read MPU6050
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);  // Starting with register 0x3B (ACCEL_XOUT_H)
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
    
    // Convert readings
    accel_x_g = ax / 8192.0;  // ±4g scale
    accel_y_g = ay / 8192.0;
    accel_z_g = az / 8192.0;
    mpu_temp_c = temp / 340.0 + 36.53;
    gyro_x_rad = gx * (2000.0 / 32768.0) * (PI/180.0);
    gyro_y_rad = gy * (2000.0 / 32768.0) * (PI/180.0);
    gyro_z_rad = gz * (2000.0 / 32768.0) * (PI/180.0);
  }
  
  // Read HMC5883L
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03);  // Starting from register 0x03
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDRESS, 6, true);
  
  if(Wire.available() == 6) {
    int16_t mx = Wire.read() << 8 | Wire.read();
    int16_t mz = Wire.read() << 8 | Wire.read();  // Z comes before Y
    int16_t my = Wire.read() << 8 | Wire.read();
    
    // Convert to gauss
    magn_x_gs = mx * 0.92 / 1000.0;
    magn_y_gs = my * 0.92 / 1000.0;
    magn_z_gs = mz * 0.92 / 1000.0;
  }
}

void readMS5611() {
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
  
  // Calculate temperature and pressure
  int32_t dT = MS5611_D2 - ((uint32_t)C[5] << 8);
  int32_t TEMP = 2000 + ((int64_t)dT * C[6] >> 23);
  
  int64_t OFF = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT >> 7);
  int64_t SENS = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT >> 8);
  
  pressure_mbar = ((((int64_t)MS5611_D1 * SENS) >> 21) - OFF) >> 15;
  pressure_mbar /= 100.0;
  baro_temp_c = TEMP / 100.0;
}

void printReadings() {
  // Print all readings with basic formatting
  Serial.println("\n=== Sensor Readings ===");
  
  Serial.println("Accelerometer (g):");
  Serial.print("  X: "); Serial.print(accel_x_g, 3);
  Serial.print("  Y: "); Serial.print(accel_y_g, 3);
  Serial.print("  Z: "); Serial.println(accel_z_g, 3);
  
  Serial.println("Gyroscope (rad/s):");
  Serial.print("  X: "); Serial.print(gyro_x_rad, 3);
  Serial.print("  Y: "); Serial.print(gyro_y_rad, 3);
  Serial.print("  Z: "); Serial.println(gyro_z_rad, 3);
  
  Serial.println("Magnetometer (gauss):");
  Serial.print("  X: "); Serial.print(magn_x_gs, 3);
  Serial.print("  Y: "); Serial.print(magn_y_gs, 3);
  Serial.print("  Z: "); Serial.println(magn_z_gs, 3);
  
  Serial.println("Temperature:");
  Serial.print("  MPU6050: "); Serial.print(mpu_temp_c, 2); Serial.println(" °C");
  Serial.print("  MS5611:  "); Serial.print(baro_temp_c, 2); Serial.println(" °C");
  
  Serial.print("Pressure: "); Serial.print(pressure_mbar, 2); Serial.println(" mbar");
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
  Wire.write(0x1E);  // Reset command
  Wire.endTransmission();
  delay(100);
}

void readCalibrationMS5611() {
  Serial.println("Reading MS5611 calibration data:");
  
  for (uint8_t i = 1; i <= 6; i++) {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0xA0 + (i * 2));
    Wire.endTransmission();
    
    Wire.requestFrom(MS5611_ADDRESS, 2);
    if(Wire.available() >= 2) {
      C[i] = (Wire.read() << 8) | Wire.read();
      Serial.print("C[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(C[i]);
    }
  }
}
