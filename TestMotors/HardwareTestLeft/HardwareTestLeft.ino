#include <SimpleFOC.h>

// AS5600 sensor on I2C0 (pins 8=SDA, 9=SCL)
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7); // 7 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4, 5); // A,B,C,Enable

float target_velocity = 20.0; // rad/s

void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C0 setup
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();
  Wire.setClock(400000);
  sensor.init(&Wire);

  // Link sensor
  motor.linkSensor(&sensor);

  // Driver setup
  driver.voltage_power_supply = 12;
  motor.voltage_limit = 6;
  driver.init();
  motor.linkDriver(&driver);

  // Velocity PID
  motor.PID_velocity.P = 1.0;
  motor.PID_velocity.I = 1.5;
  motor.LPF_velocity.Tf = 0.05;

  // Control mode
  motor.controller = MotionControlType::velocity;
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 0.0;

  // Init motor
  motor.init();
  motor.initFOC();

  Serial.println("Motor velocity test ready.");
  Serial.println("Enter velocity in rad/s (e.g. 10 or -5):");
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);

  if (Serial.available()) {
    float new_vel = Serial.parseFloat();
    if (!isnan(new_vel)) {
      target_velocity = new_vel;
      Serial.print("New velocity set to: ");
      Serial.println(target_velocity, 3);
    }
    // Clear line endings
    while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();
    }
  }
}