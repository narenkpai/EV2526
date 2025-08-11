#include <SimpleFOC.h>

// Motor 1 - AS5600 on Wire (GPIO8 SDA, GPIO9 SCL)
MagneticSensorI2C sensor1(AS5600_I2C);
BLDCMotor motor1(7);
BLDCDriver3PWM driver1(2, 3, 4, 5);

// Motor 2 - AS5600 on Wire1 (GPIO6 SDA, GPIO7 SCL)
MagneticSensorI2C sensor2(AS5600_I2C);
BLDCMotor motor2(7);
BLDCDriver3PWM driver2(29, 28, 27, 26);

// Set this to your pack under load
const float SUPPLY_V = 9.6f;

// Use full precision targets
const float TARGET1 = 19.9588f;   // rad/s
const float TARGET2 = -19.8046f;  // rad/s

// Display smoothing (does not affect control)
float v1_disp = 0.0f, v2_disp = 0.0f;
const float DISP_ALPHA = 0.15f;   // 0..1, higher = faster display

static void setupMotor(BLDCMotor& motor, BLDCDriver3PWM& driver,
                       MagneticSensorI2C& sensor, TwoWire& wire,
                       uint8_t sda, uint8_t scl,
                       float zero_elec_angle) {
  wire.setSDA(sda);
  wire.setSCL(scl);
  // RP2040 supports 1 MHz I2C. If your sensors do not like it, drop to 400000.
  wire.begin();
  wire.setClock(1000000);

  sensor.init(&wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_V;
  driver.init();
  motor.linkDriver(&driver);

  // Control config
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;
  motor.motion_downsample = 1.0f;

  // Velocity PID tuned tighter
  motor.PID_velocity.P = 0.7f;
  motor.PID_velocity.I = 1.2f;     // more I helps hold tiny offsets under load
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 0.0f;

  // Headroom
  motor.voltage_limit     = SUPPLY_V * 0.97f;
  motor.PID_velocity.limit = motor.voltage_limit;

  // Velocity estimator. Quicker helps follow without huge lag.
  motor.LPF_velocity.Tf = 0.08f;

  // Angle and current loops unchanged for voltage torque mode
  motor.P_angle.P = 20.0f;
  motor.P_angle.I = 0.0f;
  motor.P_angle.D = 0.0f;
  motor.P_angle.output_ramp = 0.0f;
  motor.P_angle.limit = 20.0f;
  motor.LPF_angle.Tf = 0.0f;

  motor.PID_current_q.P = 3.0f;
  motor.PID_current_q.I = 300.0f;
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_q.output_ramp = 0.0f;
  motor.PID_current_q.limit = SUPPLY_V;
  motor.LPF_current_q.Tf = 0.005f;

  motor.PID_current_d.P = 3.0f;
  motor.PID_current_d.I = 300.0f;
  motor.PID_current_d.D = 0.0f;
  motor.PID_current_d.output_ramp = 0.0f;
  motor.PID_current_d.limit = SUPPLY_V;
  motor.LPF_current_d.Tf = 0.005f;

  motor.velocity_limit = 80.0f;
  motor.current_limit  = 3.0f;

  // Alignment
  motor.sensor_direction    = Direction::CCW;
  motor.zero_electric_angle = zero_elec_angle;

  // PWM
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0f;

  motor.init();
  motor.initFOC();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  setupMotor(motor1, driver1, sensor1, Wire, 8, 9, 5.872f);
  setupMotor(motor2, driver2, sensor2, Wire1, 6, 7, 0.5768f);
}

void loop() {
  motor1.loopFOC();
  motor2.loopFOC();

  motor1.move(TARGET1);
  motor2.move(TARGET2);

  // Display EMA for stable 4-decimal printing
  float v1 = motor1.shaftVelocity();
  float v2 = motor2.shaftVelocity();
  v1_disp = (1.0f - DISP_ALPHA) * v1_disp + DISP_ALPHA * v1;
  v2_disp = (1.0f - DISP_ALPHA) * v2_disp + DISP_ALPHA * v2;

  // 10 Hz prints, 4 decimals
  static unsigned long lastT = 0;
  unsigned long now = millis();
  if (now - lastT > 100) {
    lastT = now;
    Serial.print("t1=");
    Serial.print(TARGET1, 4);
    Serial.print("  t2=");
    Serial.print(TARGET2, 4);
    Serial.print("  v1=");
    Serial.print(v1_disp, 4);
    Serial.print("  v2=");
    Serial.print(v2_disp, 4);
    Serial.print("  uq1=");
    Serial.print(motor1.voltage.q, 3);
    Serial.print("  uq2=");
    Serial.println(motor2.voltage.q, 3);
  }
}