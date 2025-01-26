#include <Wire.h>
#include <MPU6050.h>

// Create an MPU6050 object
MPU6050 mpu;

void setup() {
  // Start the serial communication
  Serial.begin(9600);

  // Initialize the MPU6050
  Wire.begin();
  mpu.initialize();

  // Check if the MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("MPU6050 is connected!");
}

void loop() {
  // Variables to hold sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Get accelerometer and gyroscope data
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Print accelerometer and gyroscope data to the console
  Serial.print("Accel X: ");
  Serial.print(ax);
  Serial.print(" | Accel Y: ");
  Serial.print(ay);
  Serial.print(" | Accel Z: ");
  Serial.println(az);

  Serial.print("Gyro X: ");
  Serial.print(gx);
  Serial.print(" | Gyro Y: ");
  Serial.print(gy);
  Serial.print(" | Gyro Z: ");
  Serial.println(gz);

  // Add a small delay for readability
  delay(500);
}
