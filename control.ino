/*****************************************************************************/
//  control.ino
//  Hardware:      MPU6050, Seeedduino XIAO nRF52840 Sense, BLHeli-S 7A ESC, DYS BE1102 2-3S Mini Brushless Motor 10000KV
//  Arduino IDE:   Arduino-2.3.4
//  Author:        Steven Ng, Shaun Tang
//  Date:          Apr, 2025
//  Version:       v1.0
//
//  Description: This is the complete control script for the April Prototype
//
/*******************************************************************************/
#include "Wire.h"
#include "math.h"
#include "I2Cdev.h"

//IMU Dependencies
#include "LSM6DS3.h" //This is the abdomen IMU
#include "Adafruit_MPU6050.h" //This is the body IMU
#include "Adafruit_Sensor.h"

//General ESC library based on the official arduino servo library
#include "ESC.h"

// Add ArduinoBLE library for BLE functionality
#include <ArduinoBLE.h>

//Create IMU instances
LSM6DS3 abdomenIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
Adafruit_MPU6050 bodyIMU; //Default address is 0x68 according to documentation

//Definitions for the ESC, see ESC_Calibration.ino for calibrations in the ESC.h library examples
#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds

ESC myESC (7, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
int oESC;    

//Pre-allocate storage for IMU raw values for abdomen (A) and body (B)
int16_t axA, ayA, azA;
int16_t gxA, gyA, gzA;

int16_t axB, ayB, azB;
int16_t gxB, gyB, gzB;

//Complimentary Filter Parameters
//Constants
float alpha = 0.98;             // Complementary filter constant, tuneable
float dt = 0.01;                // Time step (s), adjust to match your sampling rate (e.g., 10ms = 0.01s)

// Pre-allocate storage for pitch and roll for abdomen (A) and body (B)
float pitchA = 0.0;
float rollA = 0.0;
float pitchB = 0.0;
float rollB = 0.0;

unsigned long lastTime = 0;

// BLE Service and Characteristic definitions
BLEService controlService("19B10000-E8F2-537E-4F6C-D104768A1214");
// Increased data length to 64 bytes to handle the additional timestamp data.
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, 64);

void setup() {//Leave everything in set up as is
  delay(5000); // Give some time for Serial Monitor to catch up

  //Configures I2C
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200); //Begins serial

  while (!Serial); //Gets stuck here if serial doesn't start properly
  
  //Checks abdomenIMU is ok
  if (abdomenIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }

  //Checks bodyIMU is ok
  if (!bodyIMU.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //Some MPU6050 specific configuration
  bodyIMU.setAccelerometerRange(MPU6050_RANGE_16_G);
  bodyIMU.setGyroRange(MPU6050_RANGE_250_DEG);
  bodyIMU.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");

  //Arm BLHeli-S ESC
  myESC.arm();     
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("ArduinoControl");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(dataCharacteristic);
  BLE.addService(controlService);
  dataCharacteristic.writeValue("Ready");
  BLE.advertise();
  Serial.println("BLE Initialized and advertising");
}


void loop() {
  // Poll BLE events
  BLE.poll();
  
  // Check if new data has been written to the BLE characteristic
  if (dataCharacteristic.written()) {
    // Assume the incoming data is an ASCII string representing a number
    int len = dataCharacteristic.valueLength();
    char buffer[21]; // Buffer for incoming data (20 characters max + null terminator)
    memcpy(buffer, dataCharacteristic.value(), len);
    buffer[len] = '\0';
    int receivedValue = atoi(buffer);
    Serial.print("BLE Received: ");
    Serial.println(receivedValue);
    // Update throttle with received value (as an example)
    throttle = receivedValue;
  }
  
  // Time calculation
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  // convert ms to seconds
  lastTime = currentTime;

  //Get body and abdomen raw values
  sensors_event_t a, g, temp;
  bodyIMU.getEvent(&a, &g, &temp);
  axA = abdomenIMU.readFloatAccelX();
  ayA = abdomenIMU.readFloatAccelY();
  azA = abdomenIMU.readFloatAccelZ();
  gxA = abdomenIMU.readFloatGyroX();
  gyA = abdomenIMU.readFloatGyroY();
  gzA = abdomenIMU.readFloatGyroZ();
  axB = a.acceleration.x;
  ayB = a.acceleration.y;
  azB = a.acceleration.z;
  gxB = g.gyro.x;
  gyB = g.gyro.y;
  gzB = g.gyro.z;
  //

  //Complimentary Filter for abdomen and body pitch and roll
  // Calculate pitch and roll from accelerometer
  float accPitchA = atan2(ayA, sqrt(axA * axA + azA * azA)) * 180.0 / PI;
  float accRollA  = atan2(-axA, azA) * 180.0 / PI;
  float accPitchB = atan2(ayB, sqrt(axB * axB + azB * azB)) * 180.0 / PI;
  float accRollB  = atan2(-axB, azB) * 180.0 / PI;

  //Resulting pitch and roll for abdomen and body, you may use these directly in controls
  pitchA = alpha * (pitchA + gxA * dt) + (1 - alpha) * accPitchA;
  rollA  = alpha * (rollA  + gyA * dt) + (1 - alpha) * accRollA;
  pitchB = alpha * (pitchB + gxB * dt) + (1 - alpha) * accPitchB;
  rollB = alpha * (rollB  + gyB * dt) + (1 - alpha) * accRollB;

  //Print pitch and roll in a way that can be shown on serial plotter
  Serial.print(pitchA); Serial.print(",");
  Serial.print(rollA); Serial.print(",");
  Serial.print(pitchB); Serial.print(",");
  Serial.println(rollB);
  
  // Send sensor data over BLE as a CSV string that now includes the timestamp.
  char bleData[64];
  snprintf(bleData, sizeof(bleData), "%lu,%.2f,%.2f,%.2f,%.2f", currentTime, pitchA, rollA, pitchB, rollB);
  dataCharacteristic.writeValue(bleData);
  
  delay(1); //Small delay, 1ms
}
