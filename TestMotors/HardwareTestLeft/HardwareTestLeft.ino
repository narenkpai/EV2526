#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"

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
BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(18, 19, 20, 21); // A,B,C,EN
BLDCDriver3PWM driverLEFT = BLDCDriver3PWM(12, 10, 8, 9);

// Fixed target speed [rad/s]
static const float SET_VELOCITYRIGHT = -200.0f;
static const float SET_VELOCITYLEFT = 200.0f;

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
  driverLEFT.voltage_limit        = 6.0f; // keep modest to start
  driverLEFT.init();

 // driverRIGHT.enable_active_high = true;   // EN tied high is fine
  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit        = 6.0f; // keep modest to start
  driverRIGHT.init();

  // Motor setup
  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);

  // Velocity control with internal angle from sensor
  motorLEFT.controller = MotionControlType::velocity;
  motorRIGHT.controller = MotionControlType::velocity;

  // Velocity loop tuning starters
  motorLEFT.PID_velocity.P   = 1.5f;
  motorLEFT.PID_velocity.I   = 8.0f;
  motorLEFT.PID_velocity.D   = 0.0f;
 // motorLEFT.LPF_velocity.Tf  = 0.02f;

  motorLEFT.voltage_limit    = 12.0f; 
  
  motorRIGHT.PID_velocity.P   = 1.5f;
  motorRIGHT.PID_velocity.I   = 8.0f;
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