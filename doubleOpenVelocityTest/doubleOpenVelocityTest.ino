// Open loop motor control example
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCMotor motor1 = BLDCMotor(7);

BLDCDriver3PWM driver = BLDCDriver3PWM(15,14,8, 9);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(18, 19, 21, 22); // A,B,C,EN

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver1.voltage_power_supply = 12;

  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  driver.init();
  driver1.voltage_limit = 6;
  driver1.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 3;   // [V]
  motor1.voltage_limit = 3;   // [V]

  // open loop control config
  motor1.controller = MotionControlType::velocity_openloop;
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
    if(!motor1.init()){
    Serial.println("Motor init failed!");
    return;
  }

}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  motor.move(7);
  motor1.move(-7);

}
