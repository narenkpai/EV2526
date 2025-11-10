#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// MT6835 on SPI0 pins
#define PIN_SCK   2   // GP2 SCK
#define PIN_MOSI  3   // GP3 MOSI
#define PIN_MISO  4   // GP4 MISO
#define SENSOR_CSLEFT 11 // GP11 CS
#define SENSOR_CSRIGHT 5 // GP11 CS

// MT6835 needs SPI mode 3
SPISettings mt6835SPI(1000000, MT6835_BITORDER, SPI_MODE3);

MagneticSensorMT6835 sensorLEFT(SENSOR_CSLEFT, mt6835SPI);
MagneticSensorMT6835 sensorRIGHT(SENSOR_CSRIGHT, mt6835SPI);

// Motor and driver
BLDCMotor motorLEFT = BLDCMotor(7);   
BLDCMotor motorRIGHT = BLDCMotor(7);                  // pole pairs
BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(18, 19, 21, 22); // A,B,C,EN
BLDCDriver3PWM driverLEFT = BLDCDriver3PWM(15,14,8, 9);

// Fixed target speed [rad/s]
static const float SET_VELOCITYRIGHT = -20.0f;
static const float SET_VELOCITYLEFT = 20.0f;

// Onboard LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// Use I2C1 (pins 6=SDA, 7=SCL on RP2350)
#define BNO_SDA 6
#define BNO_SCL 7

// Create the sensor object on I2C1
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);

void setup() {
  // SPI0 map
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  
  // Sensor
  sensorLEFT.init(&SPI);
  sensorRIGHT.init(&SPI);
  motorLEFT.linkSensor(&sensorLEFT);
  motorRIGHT.linkSensor(&sensorRIGHT);

  // Driver setup
 // driverLEFT.enable_active_high = true;   // EN tied high is fine
  driverLEFT.voltage_power_supply = 12.0f;
  driverLEFT.voltage_limit        = 12.0f; // keep modest to start
  driverLEFT.init();

 // driverRIGHT.enable_active_high = true;   // EN tied high is fine
  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit        = 12.0f; // keep modest to start
  driverRIGHT.init();

  // Motor setup
  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);
  motorLEFT.foc_modulation = FOCModulationType::SpaceVectorPWM; // or FOCModulationType::SpaceVectorPWM;
  motorRIGHT.foc_modulation = FOCModulationType::SpaceVectorPWM; // or FOCModulationType::SpaceVectorPWM;

  // Velocity control with internal angle from sensor
  motorLEFT.controller = MotionControlType::torque;
  motorRIGHT.controller = MotionControlType::torque;

  // Velocity loop tuning starters
  motorLEFT.PID_velocity.P   = 0.14f;
  motorLEFT.PID_velocity.I   = 0.95f;
  motorLEFT.PID_velocity.D   = 0.0f;

  motorLEFT.LPF_velocity.Tf  = 0.02f;

  motorLEFT.voltage_limit    = 12.0f; 
  
  motorRIGHT.PID_velocity.P   = 0.14f;
  motorRIGHT.PID_velocity.I   = 0.95f;
  motorRIGHT.PID_velocity.D   = 0.0f;
 // motorRIGHT.LPF_velocity.Tf  = 0.02f;

  motorRIGHT.voltage_limit    = 12.0f;     // safety limit for q-voltage

  // Let SimpleFOC find zero_electric_angle and sensor_direction
  motorRIGHT.init();
  motorRIGHT.initFOC();
  motorLEFT.init();
  motorLEFT.initFOC();
}

void loop() {
  motorRIGHT.loopFOC();
  motorRIGHT.move(SET_VELOCITYRIGHT);
    motorLEFT.loopFOC();
  motorLEFT.move(SET_VELOCITYLEFT);
}