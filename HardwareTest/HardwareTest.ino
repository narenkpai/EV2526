#include <SimpleFOC.h>

// Magnetic sensor instance (AS5600 I2C)
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor instance (set pole pairs for your motor)
BLDCMotor motor = BLDCMotor(7);  // 2204 motor typically has 14 magnets -> 7 pole pairs

// BLDC driver: 3PWM
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4, 5); // IN1, IN2, IN3, Enable

// Target velocity (rad/s) — you can change this live via Serial
float target_velocity = 100.0; 

void setup() {
  // Start serial for debugging
  Serial.begin(115200);
  delay(300);

  // Set custom I2C pins
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();
  Wire.setClock(400000); // fast I2C for AS5600

  // Init the sensor
  sensor.init();
  Serial.println("AS5600 initialized");

  // Link the sensor to the motor
  motor.linkSensor(&sensor);

  // Set up the driver
  driver.voltage_power_supply = 12;
  motor.voltage_limit = 5; // Adjust to your supply voltage
  driver.init();
  motor.linkDriver(&driver);

  // Velocity PID parameters
  motor.PID_velocity.P = 0.5;  // Tune these for your setup
  motor.PID_velocity.I = 0.5;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.05; // velocity low-pass filter time constant

  // Motion control type
  motor.controller = MotionControlType::velocity;

  // Init motor
  motor.init();
  motor.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set target velocity in rad/s using Serial input.");
}

void loop() {
  // Run the FOC algorithm
  motor.loopFOC();

  // Run velocity control
  motor.move(target_velocity);
  motor.monitor();
  // Check for new Serial input to set target velocity
  if (Serial.available()) {
    float new_vel = Serial.parseFloat();
    if (Serial.available()) {
      target_velocity = Serial.parseFloat();
      Serial.print("New target velocity: ");
      Serial.println(target_velocity);
    }
  }
}