#include <Arduino.h>
#include <stdio.h>
#include <SPI.h>
#include "BMI088.h"

// Define SPI pins for ESP32
#define PIN_MISO    13
#define PIN_MOSI    14
#define PIN_CLK     12
#define BMI_ACCEL_CS  11   // Accelerometer CS
#define BMI_GYRO_CS   15   // Gyroscope CS

// Create SPI instance with custom pins
SPIClass hspi(SPI_CS0);

// Create BMI088 instance using custom SPI
Bmi088 imu(hspi, BMI_ACCEL_CS, BMI_GYRO_CS);
bool sensorInitialized = false;

void setup() {
  Serial.begin(9600);
  delay(1000); // Wait for console to open
  
  printf("BMI088 SPI Detection Test\n");
  printf("-------------------------\n");
  printf("Accelerometer CS Pin: %d\n", BMI_ACCEL_CS);
  printf("Gyroscope CS Pin: %d\n", BMI_GYRO_CS);
  
  // Initialize SPI with custom pins
  hspi.begin(PIN_CLK, PIN_MISO, PIN_MOSI);
  
  // Configure chip select pins
  pinMode(BMI_ACCEL_CS, OUTPUT);
  pinMode(BMI_GYRO_CS, OUTPUT);
  digitalWrite(BMI_ACCEL_CS, HIGH);
  digitalWrite(BMI_GYRO_CS, HIGH);
  
  printf("Attempting to initialize BMI088...\n");
  
  // Try to initialize the sensor
  int status = imu.begin();
  
  if (status > 0) {
    printf("SUCCESS: BMI088 detected on SPI bus!\n");
    sensorInitialized = true;
  } else {
    printf("ERROR: Failed to detect BMI088. Error code: %d\n", status);
    printf("Possible causes:\n");
    printf("1. Incorrect CS pin assignments\n");
    printf("2. BMI088 not connected properly\n");
    printf("3. Power issue with the sensor\n");
    printf("4. SPI bus configuration issue\n");
    
    // Print additional debugging info
    printf("\nDebugging info:\n");
    printf("- SPI bus: SPI2_HOST (HSPI)\n");
    printf("- MISO: GPIO 13\n");
    printf("- MOSI: GPIO 14\n");
    printf("- CLK: GPIO 12\n");
  }
}

// Loop function remains the same
void loop() {
  if (sensorInitialized) {
    // Read sensor data
    imu.readSensor();
    
    // Print accelerometer values
    printf("Accel (m/s²): X= %.2f Y= %.2f Z= %.2f", 
           imu.getAccelX_mss(), imu.getAccelY_mss(), imu.getAccelZ_mss());
    
    // Print gyroscope values
    printf(" | Gyro (rad/s): X= %.2f Y= %.2f Z= %.2f", 
           imu.getGyroX_rads(), imu.getGyroY_rads(), imu.getGyroZ_rads());
    
    // Print temperature
    printf(" | Temp=%.2f °C\n", imu.getTemperature_C());
  } else {
    // Try to reinitialize every few seconds
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 5000) {
      printf("Attempting to reinitialize BMI088...\n");
      int status = imu.begin();
      if (status > 0) {
        printf("SUCCESS: BMI088 detected!\n");
        sensorInitialized = true;
      }
      lastAttempt = millis();
    }
  }
}