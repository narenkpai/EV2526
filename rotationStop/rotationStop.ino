#include <SimpleFOC.h>

// Magnetic sensor instance (AS5600 I2C)
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor instance (set pole pairs for your motor)
BLDCMotor motor = BLDCMotor(7);  // 2204 motor typically has 14 magnets -> 7 pole pairs

// BLDC driver: 3PWM
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4, 5); // IN1, IN2, IN3, Enable

// Target velocity (rad/s)
float target_velocity = 27.8;

// Number of full rotations to perform (can be decimal)
float target_rotations = 70.6850;  

// Tracking rotation progress
float start_angle = 0.0;
float cumulative_angle = 0.0;
bool movement_done = false;

void setup() {
  Serial.begin(115200);
  delay(300);

  // Custom I2C pins
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
  motor.voltage_limit = 6; 
  driver.init();
  motor.linkDriver(&driver);

  // PID tuning
  motor.PID_velocity.P = 1;
  motor.PID_velocity.I = 1.5;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.05;

  // Motion control type
  motor.controller = MotionControlType::velocity;
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 0.8023;

  // Init motor
  motor.init();
  motor.initFOC();

  // Record the starting angle
  start_angle = sensor.getAngle();
  cumulative_angle = 0.0;

  Serial.println("Motor ready.");
  Serial.print("Target rotations: ");
  Serial.println(target_rotations);
}

void loop() {
  motor.loopFOC();
  motor.monitor();

  if (!movement_done) {
    motor.move(target_velocity);

    // Track rotation distance
    static float last_angle = start_angle;
    float current_angle = sensor.getAngle();
    float delta = current_angle - last_angle;

    // Handle wrap-around for AS5600 (-PI to PI range)
    if (delta > _PI) delta -= 2 * _PI;
    else if (delta < -_PI) delta += 2 * _PI;

    cumulative_angle += delta;
    last_angle = current_angle;

    // Print exact decimal rotations
    float rotations_exact = fabs(cumulative_angle) / (2 * _PI);
    Serial.print("Rotations: ");
    Serial.println(rotations_exact, 6); // print with 6 decimal places

    // Stop if target reached (including decimal part)
    if (rotations_exact >= target_rotations) {
      motor.move(0);
      movement_done = true;
      Serial.println("Target rotations reached. Motor stopped.");
    }
  } else {
    motor.move(0); // keep stopped
  }

  // Optional: live change target velocity from Serial
  if (Serial.available()) {
    target_velocity = Serial.parseFloat();
    Serial.print("New target velocity: ");
    Serial.println(target_velocity);
    movement_done = false;
    cumulative_angle = 0.0;
  }
}