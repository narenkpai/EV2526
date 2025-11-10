// SimpleFOC + SimpleFOC WebController for RP2040 + MT6835 + 3PWM driver
// Serial speed for WebController: 115200

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"

// ---------- SPI0 pins (RP2040) ----------
#define PIN_SCK        2   // GP2  SCK
#define PIN_MOSI       3   // GP3  MOSI
#define PIN_MISO       4   // GP4  MISO
#define SENSOR_CSLEFT  11  // GP11 CS

#ifndef MT6835_BITORDER
#define MT6835_BITORDER MSBFIRST
#endif
// MT6835 uses SPI mode 3
SPISettings mt6835SPI(1000000, MT6835_BITORDER, SPI_MODE3);

// ---------- Sensor ----------
MagneticSensorMT6835 sensorLEFT(SENSOR_CSLEFT, mt6835SPI);

// ---------- Motor and driver ----------
BLDCMotor motorLEFT(7);              // pole pairs
BLDCDriver3PWM driverLEFT(15, 14, 8, 9);   // A, B, C, EN

// ---------- Commander ----------
Commander command = Commander(Serial);
void doMotorLEFT(char* cmd) { command.motor(&motorLEFT, cmd); }

// Basic init check
void assertInit(const char* label, bool ok) {
  if (!ok) {
    Serial.print("Init failed: ");
    Serial.println(label);
    while (true) { delay(100); }
  }
}

void setup() {
  // Serial for WebController
  Serial.begin(115200);
  delay(500);

  // SPI remap for RP2040
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  // Sensor
  sensorLEFT.init(&SPI);

  // Driver
  driverLEFT.voltage_power_supply = 12.0;   // set to your supply
  driverLEFT.voltage_limit        = 6.0;    // safe start
  // driverLEFT.pwm_frequency = 25000;
  assertInit("driverLEFT", driverLEFT.init());

  // Motor
  motorLEFT.linkDriver(&driverLEFT);
  motorLEFT.linkSensor(&sensorLEFT);

  // Control defaults
  motorLEFT.controller = MotionControlType::velocity;
  motorLEFT.torque_controller = TorqueControlType::voltage;

  // Velocity PID and LPF
  motorLEFT.PID_velocity.P = 0.2f;
  motorLEFT.PID_velocity.I = 5.0f;
  motorLEFT.PID_velocity.D = 0.0f;
  motorLEFT.PID_velocity.output_ramp = 1000.0f;
  motorLEFT.LPF_velocity.Tf = 0.02f;

  // Limits
  motorLEFT.voltage_limit = 6.0f;
  motorLEFT.velocity_limit = 20.0f;
  motorLEFT.foc_modulation = FOCModulationType::SinePWM;

  // Monitoring for WebController
  // Use same char as Commander ID
  const char motor_id = 'M';
  motorLEFT.useMonitoring(Serial);
  motorLEFT.monitor_start_char = motor_id;
  motorLEFT.monitor_end_char = motor_id;
  motorLEFT.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_VOLT_Q;
  motorLEFT.monitor_downsample = 0;

  // Init and align
  assertInit("motorLEFT", motorLEFT.init());
  motorLEFT.initFOC();

  // Commander hookup
  command.add(motor_id, doMotorLEFT, "motorLEFT");
  command.verbose = VerboseMode::machine_readable;  // best for WebController

  Serial.println("WebController ready. ID M");
  Serial.println("Try: M, M V, M C 2, M T 5.0");
}

void loop() {
  // FOC step
  motorLEFT.loopFOC();

  // Motion step, target comes from WebController
  motorLEFT.move();

  // Realtime I/O
  motorLEFT.monitor();
  command.run();
}