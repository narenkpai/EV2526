/**
 * Torque control example using voltage control loop.
 */
#include <SimpleFOC.h>

// BLDC motor instance
BLDCMotor motor = BLDCMotor(7);
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 16,17, 4);

// sensor instance

void setup() { 
  
  // initialize encoder sensor hardware
  // link the motor to the sensor
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // aligning voltage
  // set motion control loop to be used
  motor.controller = MotionControlType::velocity_openloop;
  // initialize motor
  motor.init();
  // align sensor and start FOC
}

// target voltage to be set to the motor
float target_voltage = 2;

void loop() {
  // Motion control function
  motor.move(6.28);
}