#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/as5600/MagneticSensorAS5600.h"

// Motor 1 hardware (I2C0 on pins 8=SDA, 9=SCL)
MagneticSensorAS5600 sensor1;          // AS5600 on Wire
BLDCMotor motor1(7);
BLDCDriver3PWM driver1(2, 3, 4, 5);

// Motor 2 hardware (I2C1 on pins 6=SDA, 7=SCL)
MagneticSensorAS5600 sensor2;          // AS5600 on Wire1
BLDCMotor motor2(7);
BLDCDriver3PWM driver2(29, 28, 27, 26);

// Commander interface
Commander command = Commander(Serial);
void doMotor1(char* cmd) { command.motor(&motor1, cmd); }
void doMotor2(char* cmd) { command.motor(&motor2, cmd); }

void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C0 for sensor1
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();
  Wire.setClock(400000);

  // I2C1 for sensor2
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();
  Wire1.setClock(400000);

  // AS5600 init
  // closeTransactions=false keeps the bus claimed between reads for higher rate
  sensor1.closeTransactions = false;
  sensor2.closeTransactions = false;
  sensor1.init(&Wire);
  sensor2.init(&Wire1);

  // Motor 1
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::velocity;

  // known alignment
  motor1.sensor_direction = Direction::CCW;
  motor1.zero_electric_angle = 5.8721;

  motor1.init();
  motor1.initFOC();
  motor1.useMonitoring(Serial);
  motor1.target = 20.8;

  // Motor 2
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.controller = MotionControlType::velocity;

  // known alignment
  motor2.sensor_direction = Direction::CCW;
  motor2.zero_electric_angle = 0.5768;

  motor2.init();
  motor2.initFOC();
  motor2.useMonitoring(Serial);
  motor2.target = 20.8;

  // Commander bindings
  command.add('A', doMotor1, "Motor 1");
  command.add('B', doMotor2, "Motor 2");

  Serial.println("Commander ready. Use A or B in SimpleFOCStudio.");
}

void loop() {
  // FOC loops
  motor1.loopFOC();
  motor2.loopFOC();

  // Apply current targets (can be changed live from Studio)
  motor1.move(motor1.target);
  motor2.move(motor2.target);

  // Monitoring and commander
  motor1.monitor();
  motor2.monitor();
  command.run();
}