#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 16,17, 4);
//MagneticSensorI2C sensor = MagneticSensorI2C(0x06, 14, 0x0E, 6);
MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

void setup(){
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin(26,27);
  sensor.init(&Wire);

  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  // aligning voltage
  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // initialize motor
  motor.init();
  motor.initFOC();
}

void loop(){
  motor.loopFOC();
  motor.move(20);
}