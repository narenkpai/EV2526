#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"

// ---------- Your SPI0 pin map ----------
#define PIN_SCK        2   // GP2  SCK
#define PIN_MOSI       3   // GP3  MOSI
#define PIN_MISO       4   // GP4  MISO
#define SENSOR_CSLEFT  11  // GP11 CS for LEFT
#define SENSOR_CSRIGHT 5   // GP5  CS for RIGHT

#ifndef MT6835_BITORDER
#define MT6835_BITORDER MSBFIRST
#endif
// MT6835 runs in SPI mode 3
SPISettings mt6835SPI(1000000, MT6835_BITORDER, SPI_MODE3);

// ---------- Encoders (MT6835 on SPI) ----------
MagneticSensorMT6835 sensorLEFT(SENSOR_CSLEFT, mt6835SPI);
MagneticSensorMT6835 sensorRIGHT(SENSOR_CSRIGHT, mt6835SPI);

// ---------- Motors and drivers ----------
BLDCMotor motorLEFT  = BLDCMotor(7);                         // pole pairs
BLDCMotor motorRIGHT = BLDCMotor(7);

BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(18, 19, 20, 21); // A,B,C,EN
BLDCDriver3PWM driverLEFT  = BLDCDriver3PWM(12, 10, 8, 9);   // A,B,C,EN

// ---------- Commander ----------
Commander command = Commander(Serial);
void doLeft(char* cmd)  { command.motor(&motorLEFT,  cmd); }
void doRight(char* cmd) { command.motor(&motorRIGHT, cmd); }

void setup() {
  Serial.begin(115200);
  delay(300);

  // SPI0 routing
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  // Sensors on SPI0
  sensorLEFT.init(&SPI);
  sensorRIGHT.init(&SPI);

  // Link sensors
  motorLEFT.linkSensor(&sensorLEFT);
  motorRIGHT.linkSensor(&sensorRIGHT);

  // Drivers
  driverLEFT.voltage_power_supply = 12.0f;
  driverLEFT.voltage_limit        = 8.0f;   // start safe
  driverLEFT.init();

  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit        = 8.0f;
  driverRIGHT.init();

  // Link drivers
  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);

  // Control mode
  motorLEFT.controller  = MotionControlType::velocity;
  motorRIGHT.controller = MotionControlType::velocity;

  // Velocity loop tuning starters
  motorLEFT.PID_velocity.P = 1.5f;
  motorLEFT.PID_velocity.I = 8.0f;
  motorLEFT.PID_velocity.D = 0.0f;
  motorLEFT.LPF_velocity.Tf = 0.01f;
  motorLEFT.voltage_limit   = 12.0f;

  motorRIGHT.PID_velocity.P = 1.5f;
  motorRIGHT.PID_velocity.I = 8.0f;
  motorRIGHT.PID_velocity.D = 0.0f;
  motorRIGHT.LPF_velocity.Tf = 0.01f;
  motorRIGHT.voltage_limit   = 12.0f;

  // If you already know alignment, set these here
  // motorLEFT.sensor_direction  = Direction::CCW;
  // motorLEFT.zero_electric_angle = <your value>;
  // motorRIGHT.sensor_direction = Direction::CCW;
  // motorRIGHT.zero_electric_angle = <your value>;

  // Init and FOC
  motorLEFT.init();
  motorRIGHT.init();
  motorLEFT.initFOC();
  motorRIGHT.initFOC();

  // Defaults so Studio shows something
  motorLEFT.target  = 20.0f;   // rad/s
  motorRIGHT.target = 20.0f;

  // Enable SimpleFOC Studio commander
  command.add('A', doLeft,  "Motor LEFT");
  command.add('B', doRight, "Motor RIGHT");

  // Optional monitoring
  motorLEFT.useMonitoring(Serial);
  motorRIGHT.useMonitoring(Serial);

  Serial.println("Commander ready. Use A and B in SimpleFOCStudio.");
}

void loop() {
  // FOC loops
  motorLEFT.loopFOC();
  motorRIGHT.loopFOC();

  // Apply current targets
  motorLEFT.move(motorLEFT.target);
  motorRIGHT.move(motorRIGHT.target);

  // Monitoring and commander
  motorLEFT.monitor();
  motorRIGHT.monitor();
  command.run();
}