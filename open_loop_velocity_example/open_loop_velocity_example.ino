#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"

// Motor and driver
// Set your motor pole pairs
BLDCMotor motor = BLDCMotor(7);

// Pins: PWM_A, PWM_B, PWM_C, EN
// RP2040 GPIO 18, 19, 20 are PWM capable. 21 as enable.
BLDCDriver3PWM driver = BLDCDriver3PWM(18, 19, 20, 21);

// Fixed target velocity [rad/s]
static const float SET_VELOCITY_RAD_S = 20.0f;  // adjust as needed

void setup() {
  // No Serial, no Commander. Runs without USB.

  // Driver config
  driver.voltage_power_supply = 11.0f;   // your DC bus
  driver.voltage_limit        = 6.0f;    // start conservative
  driver.pwm_frequency        = 25000;   // 25 kHz
//  driver.dead_zone            = 0.02f;   // tune for your half bridge

  if (!driver.init()) {
    // If you want a visual error, toggle EN quickly
    pinMode(21, OUTPUT);
    while (1) { digitalWrite(21, !digitalRead(21)); delay(200); }
  }

  // Link motor to driver
  motor.linkDriver(&driver);

  // Open loop velocity control
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 6.0f;            // motor protection
  motor.velocity_limit = 1000.0f;        // not critical in open loop

  // Optional small initial q voltage to help start smoothly
  //motor.Uq = 1.5f;

  if (!motor.init()) {
    pinMode(21, OUTPUT);
    while (1) { digitalWrite(21, !digitalRead(21)); delay(200); }
  }

  // If you want a quick ramp up instead of a step, do it here
  // Simple linear ramp to SET_VELOCITY_RAD_S over ~0.5 s
  float v = 0.0f;
  const float dv = SET_VELOCITY_RAD_S / 50.0f;
  for (int i = 0; i < 50; ++i) {
    v += dv;
    motor.move(v);
    delay(10);
  }
}

void loop() {
  // Keep motor spinning at fixed velocity
  motor.move(SET_VELOCITY_RAD_S);

  // Small delay keeps CPU cool, adjust as you like
  delay(1);
}