#include <SimpleFOC.h>

// Motor 1 (I2C0 on pins 8,9)
MagneticSensorI2C sensor1(AS5600_I2C);
BLDCMotor motor1(7);
BLDCDriver3PWM driver1(2, 3, 4, 5);

// Motor 2 (I2C1 on pins 6,7)
MagneticSensorI2C sensor2(AS5600_I2C);
BLDCMotor motor2(7);
BLDCDriver3PWM driver2(29, 28, 27, 26);

float target_velocity1 = 20.24;   // rad/s
float target_velocity2 = 20.24;   // rad/s, motor2 runs opposite

float target_rotations1 = 28;
float target_rotations2 = 28;

float cumulative_angle1 = 0, cumulative_angle2 = 0;
float last_angle1, last_angle2;
bool movement_done1 = false, movement_done2 = false;

unsigned long lastPrint = 0;

// Ramp settings
const float max_accel = 5.0f;   // rad/s^2
float cmd_mag = 0.0f;            // current commanded magnitude
unsigned long lastStep = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C setup for both sensors
  Wire.setSDA(8); Wire.setSCL(9);
  Wire.begin(); Wire.setClock(400000);
  sensor1.init(&Wire);

  Wire1.setSDA(6); Wire1.setSCL(7);
  Wire1.begin(); Wire1.setClock(400000);
  sensor2.init(&Wire1);
  motor1.motion_downsample = 10.0;
  motor2.motion_downsample = 10.0;
  // Motor 1 setup
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 9.0;
  motor1.PID_velocity.P = .5;
  motor1.PID_velocity.I = .5;
  motor1.PID_velocity.D = 0;
  motor1.LPF_velocity.Tf = 1.0;
  motor1.PID_velocity.output_ramp = 0.0;
  motor1.PID_velocity.limit = 12.0;
  motor1.controller = MotionControlType::velocity;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.sensor_direction = Direction::CCW;
  motor1.zero_electric_angle = 5.8721;
  motor1.init();
  motor1.initFOC();

  // Motor 2 setup
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = 9.0;
  motor2.PID_velocity.P = .5;
  motor2.PID_velocity.I = .5;
  motor2.PID_velocity.D = 0.;
  motor2.LPF_velocity.Tf = 1.0;
  motor2.PID_velocity.output_ramp = 0.0;
  motor2.PID_velocity.limit = 12.0;
  motor2.controller = MotionControlType::velocity;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.sensor_direction = Direction::CCW;
  motor2.zero_electric_angle = 0.5768;
  motor2.init();
  motor2.initFOC();

  last_angle1 = sensor1.getAngle();
  last_angle2 = sensor2.getAngle();

  lastStep = millis();
  cmd_mag = 0.0f;  // start from zero together

  Serial.println("Dual velocity control with ramp ready");
}

void loop() {
  // Run FOC for both motors
  motor1.loopFOC();
  motor2.loopFOC();

  // Compute shared ramp magnitude
  unsigned long now = millis();
  float dt = (now - lastStep) * 0.001f;
  if (dt < 0) dt = 0;
  lastStep = now;

  // While both are active, ramp toward the smaller target so they stay synced
  float tgt_mag;
  if (!(movement_done1 || movement_done2)) {
    tgt_mag = fminf(fabsf(target_velocity1), fabsf(target_velocity2));
  } else {
    // If one finished, let the other head to its own target
    float a = movement_done1 ? 0.0f : fabsf(target_velocity1);
    float b = movement_done2 ? 0.0f : fabsf(target_velocity2);
    tgt_mag = fmaxf(a, b);
  }

  // Slew limit
  float dv = max_accel * dt;
  if (cmd_mag < tgt_mag)       cmd_mag = fminf(cmd_mag + dv, tgt_mag);
  else if (cmd_mag > tgt_mag)  cmd_mag = fmaxf(cmd_mag - dv, tgt_mag);

  // Apply commands: motor1 forward, motor2 reverse
  float cmd1 = movement_done1 ? 0.0f : cmd_mag;
  float cmd2 = movement_done2 ? 0.0f : -cmd_mag;
  motor1.move(cmd1);
  motor2.move(cmd2);

  // Track rotations
  float curr1 = sensor1.getAngle();
  float d1 = curr1 - last_angle1;
  if (d1 > _PI) d1 -= 2 * _PI; else if (d1 < -_PI) d1 += 2 * _PI;
  cumulative_angle1 += d1;
  last_angle1 = curr1;
  if (fabs(cumulative_angle1) / (2 * _PI) >= target_rotations1) movement_done1 = true;

  float curr2 = sensor2.getAngle();
  float d2 = curr2 - last_angle2;
  if (d2 > _PI) d2 -= 2 * _PI; else if (d2 < -_PI) d2 += 2 * _PI;
  cumulative_angle2 += d2;
  last_angle2 = curr2;
  if (fabs(cumulative_angle2) / (2 * _PI) >= target_rotations2) movement_done2 = true;

  // Print at ~6 Hz
  if (now - lastPrint > 160) {
    lastPrint = now;
    Serial.print("W1 vel: "); Serial.print(sensor1.getVelocity(), 2);
    Serial.print("  W2 vel: "); Serial.print(sensor2.getVelocity(), 2);
    Serial.print("  cmd_mag: "); Serial.println(cmd_mag, 2);
  }
}