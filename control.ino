/*****************************************************************************/
//  control.ino
//  Hardware:      MPU6050, Seeedduino XIAO nRF52840 Sense, BLHeli-S 7A ESC, DYS BE1102 2-3S Mini Brushless Motor 10000KV
//	Arduino IDE:   Arduino-2.3.4
//	Author:	       Steven Ng, Shaun Tang
//	Date: 	       Apr, 2025
//	Version:       v1.0
//
//  Description: This is the complete control script for the April Prototype
//
//
//
//
//
//
/*******************************************************************************/
#include "Wire.h"
#include "math.h"
#include "I2Cdev.h"

//IMU Dependencies
#include "LSM6DS3.h"
#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"

#include "Servo.h"

/* Bluetooth is not working yet...
//BLE library (Native)
#include "ArduinoBLE.h"

//BLUE Service and Characteristic UUID
#define SERVICE_UUID         "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID  "abcdef01-1234-5678-1234-56789abcdef0"

// Create the BLE Service and Characteristic.
// The characteristic is set to allow reading, writing, and notifications.
BLEService customService(SERVICE_UUID);
BLECharacteristic customCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite | BLENotify, 20);
*/

//Create a instance of class LSM6DS3
//This is the embedded IMU on the Seeeduino nrf chip on the abdomen
LSM6DS3 abdomenIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
Adafruit_MPU6050 bodyIMU; //Default address is 0x68 according to documentation

/*
//Create instance of class DShot, set to dshot600 mode
DShot esc1(DShot::Mode::DSHOT600);
*/

//Since DShot library doesn't work with ARM or mbed architecture we use Servo library because they work with the same principle anyway.
Servo esc;

//Throttle and target value for esc
uint16_t throttle = 0;
uint16_t target = 0;

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

  //Initialise esc
  esc.attach(7);
  esc.writeMicroseconds(1200);
  delay(1000);

  /* BLE not working yet...
  while (!Serial);

  // Initialize BLE hardware
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Set the local name and advertise the service
  BLE.setLocalName("Flapper001");
  BLE.setAdvertisedService(customService);

  // Add the characteristic to the service, then add the service
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);

  // Optionally set an initial value for the characteristic
  customCharacteristic.writeValue("Hello, Flapper");

  // Start advertising
  BLE.advertise();
  Serial.println("BLE device is now advertising...");
  */
}

void loop() {
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
  delay(1); //Small delay, 1ms
}


//Ignore these for now, they are wireless communication code that I haven't worked through yet
/*
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // While the central is connected:
    while (central.connected()) {
      // Check if the characteristic was written to
      if (customCharacteristic.written()) {
        // Read the incoming value
        String receivedData = customCharacteristic.value();
        Serial.print("Received data: ");
        Serial.println(receivedData);
      }
      // Time calculation
      unsigned long currentTime = millis();
      dt = (currentTime - lastTime) / 1000.0;  // convert ms to seconds
      lastTime = currentTime;

      //Get body and abdomen raw values
      bodyIMU.getMotion6(&axB, &ayB, &azB, &gxB, &gyB, &gzB);
      axA = myIMU.readFloatAccelX();
      ayA = myIMU.readFloatAccelY();
      azA = myIMU.readFloatAccelZ();
      gxA = myIMU.readFloatGyroX();
      gyA = myIMU.readFloatGyroY();
      gzA = myIMU.readFloatGyroZ();

      //Complimentary Filter for abdomen and body pitch and roll
      // Calculate pitch and roll from accelerometer
      float accPitchA = atan2(ayA, sqrt(axA * axA + azA * azA)) * 180.0 / PI;
      float accRollA  = atan2(-axA, azA) * 180.0 / PI;
      float accPitchB = atan2(ayB, sqrt(axB * axB + azB * azB)) * 180.0 / PI;
      float accRollB  = atan2(-axB, azB) * 180.0 / PI;

        // Integrate gyro data
      pitchA = alpha * (pitch + gyA * dt) + (1 - alpha) * accPitchA;
      rollA  = alpha * (roll  + gxA * dt) + (1 - alpha) * accRollA;
      pitchB = alpha * (pitch + gyB * dt) + (1 - alpha) * accPitchB;
      rollB = alpha * (roll  + gxB * dt) + (1 - alpha) * accRollB;
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
*/

//This is used to ramp up or down the motor smoothly, target can be anything between 0 and 2047
/*
int throttleRamp(target) {
  if (target>2047)
      target = 2047;
  if (throttle<48){
    throttle = 48;
  }
  if (target<=48){
    esc1.setThrottle(target);
  }else{
    if (target>throttle){
      throttle ++;
      esc1.setThrottle(throttle);
    }else if (target<throttle){
      throttle --;
      esc1.setThrottle(throttle);
    }
  }
}
*/
