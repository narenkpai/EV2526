#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"
enum StepState { STEP1, STEP2, STOP };
StepState step = STEP1;

unsigned long t_start = 0;
// ---------- MT6835 SPI ----------
#define PIN_SCK        2
#define PIN_MOSI       3
#define PIN_MISO       4
#define SENSOR_CSLEFT  11
#define SENSOR_CSRIGHT 5

#ifndef MT6835_BITORDER
#define MT6835_BITORDER MSBFIRST
#endif
SPISettings mt6835SPI(1000000, MT6835_BITORDER, SPI_MODE3);

// Encoders
MagneticSensorMT6835 sensorLEFT(SENSOR_CSLEFT, mt6835SPI);
MagneticSensorMT6835 sensorRIGHT(SENSOR_CSRIGHT, mt6835SPI);

// ---------- Motors and drivers ----------
BLDCMotor motorLEFT  = BLDCMotor(7);  // pole pairs
BLDCMotor motorRIGHT = BLDCMotor(7);

// Use your given pinout. You had RIGHT as 19,18,21 with enable 22.
BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(19, 18, 21, 22);
BLDCDriver3PWM driverLEFT  = BLDCDriver3PWM(15, 14, 8, 9);

// ---------- User setpoint ----------
static const float TARGET_W_RAD_S = 25.0f;  // constant shaft speed, rad/s

void setup() {
  // SPI for MT6835
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  // Encoders first
  sensorLEFT.init(&SPI);
  sensorRIGHT.init(&SPI);

  // Link sensors
  motorLEFT.linkSensor(&sensorLEFT);
  motorRIGHT.linkSensor(&sensorRIGHT);

  // Drivers
  driverLEFT.voltage_power_supply = 12.0f;
  driverLEFT.voltage_limit        = 12.0f;
  driverLEFT.init();

  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit        = 12.0f;
  driverRIGHT.init();

  // Link drivers
  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);

  // Modulation
  motorLEFT.foc_modulation  = FOCModulationType::SpaceVectorPWM;
  motorRIGHT.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Velocity mode only
  motorLEFT.controller  = MotionControlType::angle;
  motorRIGHT.controller = MotionControlType::angle;

  // Minimal velocity PID
  motorLEFT.PID_velocity.P = 0.1f;
  motorLEFT.PID_velocity.I = 8.0f;
  motorLEFT.LPF_velocity.Tf = 0.01f;

  motorRIGHT.PID_velocity.P = 0.1f;
  motorRIGHT.PID_velocity.I = 8.0f;
  motorRIGHT.LPF_velocity.Tf = 0.01f;

  // Limits
  motorLEFT.voltage_limit  = 12.0f;
  motorRIGHT.voltage_limit = 12.0f;
  motorLEFT.velocity_limit  = 60.0f;  // rad/s
  motorRIGHT.velocity_limit = 60.0f;

  // Your measured alignment
  motorLEFT.sensor_direction       = Direction::CW;
  motorLEFT.zero_electric_angle    = 4.8867f;
  motorRIGHT.sensor_direction      = Direction::CCW;
  motorRIGHT.zero_electric_angle   = 3.6644f;

  // Init FOC in this order to match what worked for you
  motorRIGHT.init();
  motorRIGHT.initFOC();
  motorLEFT.init();
  motorLEFT.initFOC();
    t_start = millis();  // mark start time

  // Set constant targets
  motorLEFT.target  = TARGET_W_RAD_S;
  motorRIGHT.target = TARGET_W_RAD_S;
}

void loop() {
  motorLEFT.loopFOC();
  motorRIGHT.loopFOC();

  unsigned long t_now = millis();

  switch (step) {
    case STEP1:
      motorLEFT.move(1.35);
      motorRIGHT.move(0.0);
      if (t_now - t_start > 2000) {   // run step1 for 2 seconds
        step = STEP2;
        t_start = t_now;
      }
      break;

    case STEP2:
      motorLEFT.move(0.0);
      motorRIGHT.move(1.35);
      if (t_now - t_start > 2000) {   // run step2 for 2 seconds
        step = STOP;
        t_start = t_now;
      }
      break;

    case STOP:
      motorLEFT.move(0.0);
      motorRIGHT.move(0.0);
      // stays here forever
      break;
  }
}