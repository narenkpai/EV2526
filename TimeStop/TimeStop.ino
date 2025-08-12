#include <SimpleFOC.h>

// ==================== Motors and sensors ====================
MagneticSensorI2C sensor1(AS5600_I2C);
BLDCMotor motor1(7);
BLDCDriver3PWM driver1(2, 3, 4, 5);

MagneticSensorI2C sensor2(AS5600_I2C);
BLDCMotor motor2(7);
BLDCDriver3PWM driver2(29, 28, 27, 26);

// Supply
const float SUPPLY_V = 9.6f;

// Known electrical alignment
const float ZERO_ELEC_1 = 5.8721f;
const float ZERO_ELEC_2 = 0.5768f;

static void setupMotor(BLDCMotor& m, BLDCDriver3PWM& d,
                       MagneticSensorI2C& s, TwoWire& w,
                       uint8_t sda, uint8_t scl,
                       float zero_elec_angle) {
  w.setSDA(sda);
  w.setSCL(scl);
  w.begin();
  w.setClock(1000000); // 1 MHz I2C

  s.init(&w);
  m.linkSensor(&s);

  d.voltage_power_supply = SUPPLY_V;
  d.init();
  m.linkDriver(&d);

  m.torque_controller = TorqueControlType::voltage;
  m.controller = MotionControlType::velocity;
  m.motion_downsample = 1.0f;

  m.PID_velocity.P = 0.7f;
  m.PID_velocity.I = 1.2f;
  m.PID_velocity.D = 0.0f;
  m.PID_velocity.output_ramp = 0.0f;

  m.voltage_limit = SUPPLY_V * 0.97f;
  m.PID_velocity.limit = m.voltage_limit;

  m.LPF_velocity.Tf = 0.08f;

  m.P_angle.P = 20.0f;
  m.P_angle.I = 0.0f;
  m.P_angle.D = 0.0f;
  m.P_angle.output_ramp = 0.0f;
  m.P_angle.limit = 20.0f;

  m.PID_current_q.P = 3.0f;
  m.PID_current_q.I = 300.0f;
  m.PID_current_q.D = 0.0f;
  m.PID_current_q.output_ramp = 0.0f;
  m.PID_current_q.limit = SUPPLY_V;
  m.LPF_current_q.Tf = 0.005f;

  m.PID_current_d.P = 3.0f;
  m.PID_current_d.I = 300.0f;
  m.PID_current_d.D = 0.0f;
  m.PID_current_d.output_ramp = 0.0f;
  m.PID_current_d.limit = SUPPLY_V;
  m.LPF_current_d.Tf = 0.005f;

  m.velocity_limit = 80.0f;
  m.current_limit  = 3.0f;

  m.sensor_direction    = Direction::CCW;
  m.zero_electric_angle = zero_elec_angle;

  m.foc_modulation = FOCModulationType::SpaceVectorPWM;
  m.modulation_centered = 1.0f;

  m.init();
  m.initFOC();
}

// ==================== BNO055 over UART ====================
const uint8_t BNO_START_WRITE = 0xAA;
const uint8_t BNO_START_READ  = 0xAA;
const uint8_t BNO_CMD_WRITE   = 0x00;
const uint8_t BNO_CMD_READ    = 0x01;
const uint8_t BNO_RESP_OK     = 0xBB;
const uint8_t BNO_RESP_ERR    = 0xEE;

HardwareSerial &bno = Serial1;

static bool bno_read(uint8_t reg, uint8_t len, uint8_t *buf) {
  bno.write(BNO_START_READ);
  bno.write(BNO_CMD_READ);
  bno.write(reg);
  bno.write(len);
  bno.flush();

  unsigned long t0 = millis();
  auto readByte = [&](uint8_t &out)->bool {
    while (!bno.available()) {
      if (millis() - t0 > 5) return false;
    }
    out = bno.read();
    return true;
  };

  uint8_t hdr;
  if (!readByte(hdr)) return false;
  if (hdr == BNO_RESP_ERR) { uint8_t err; readByte(err); return false; }
  if (hdr != BNO_RESP_OK) return false;

  uint8_t rlen;
  if (!readByte(rlen)) return false;
  if (rlen != len) return false;

  for (uint8_t i = 0; i < len; i++) if (!readByte(buf[i])) return false;
  return true;
}

static bool bno_write(uint8_t reg, uint8_t len, const uint8_t *data) {
  bno.write(BNO_START_WRITE);
  bno.write(BNO_CMD_WRITE);
  bno.write(reg);
  bno.write(len);
  for (uint8_t i = 0; i < len; i++) bno.write(data[i]);
  bno.flush();

  unsigned long t0 = millis();
  while (!bno.available()) { if (millis() - t0 > 5) return false; }
  uint8_t a = bno.read();
  while (!bno.available()) { if (millis() - t0 > 5) return false; }
  uint8_t b = bno.read();
  if (a == 0xEE && b == 0x07) return true;
  if (a == 0xBB && b == 0x00) return true;
  return false;
}

static inline float deg2rad(float d){ return d * 0.01745329252f; }
static inline float wrapPI(float a){
  while(a >  M_PI) a -= 2.0f*M_PI;
  while(a < -M_PI) a += 2.0f*M_PI;
  return a;
}

static bool bno_get_heading_rad(float &head_rad){
  uint8_t raw[2];
  if (!bno_read(0x1A, 2, raw)) return false;
  int16_t h = (int16_t)((raw[1] << 8) | raw[0]);
  float heading_deg = h / 16.0f;
  head_rad = wrapPI(deg2rad(heading_deg));
  return true;
}

static void bno_setup(){
  bno.begin(115200);
  uint8_t cfg = 0x00;
  bno_write(0x3D, 1, &cfg);
  delay(25);
  uint8_t ndof = 0x0C;
  bno_write(0x3D, 1, &ndof);
  delay(50);
}

// ==================== Path generator (parabola to the right) ====================
const float WHEEL_DIAM = 0.0463f;
const float WHEEL_R    = WHEEL_DIAM * 0.5f;
const float TRACK_W    = 0.055f;

// Defaults if you do not enter values on Serial
float SPAN_M        = 8.0f;   // meters, horizontal distance left to right
float TRAVEL_TIME_S = 10.0f;  // seconds, will be clamped to 10..20
const float HEIGHT_M = 1.0f;  // peak height of parabola


const int   ARC_N = 2000;     // even

float a_parab = 0.0f;         // y = a x^2 + h
float half_span = 0.0f;
float L_center = 0.0f;        // center path length
float v_center = 0.0f;        // centerline linear speed set to finish on time

static float slope(float x){ return 2.0f * a_parab * x; }
static float curvature(float x){
  float yp = slope(x);
  float denom = powf(1.0f + yp*yp, 1.5f);
  return (2.0f * a_parab) / denom;
}
static float theta_des(float x){ return atan(slope(x)); }

static float simpson_arc_length(){
  float dx = (2.0f * half_span) / ARC_N;
  auto integrand = [](float x, float a)->float {
    float yp = 2.0f * a * x;
    return sqrtf(1.0f + yp*yp);
  };
  float sum = 0.0f;
  for(int i=0;i<=ARC_N;i++){
    float x = -half_span + i*dx;
    float coeff = (i==0 || i==ARC_N) ? 1.0f : (i%2 ? 4.0f : 2.0f);
    sum += coeff * integrand(x, a_parab);
  }
  return (dx/3.0f) * sum;
}

// ==================== Heading control with gating and slew ====================
float Kp_h = 0.50f, Ki_h = 0.1f, Kd_h = 0.00f;

const float OMEGA_MAX   = .5f;   // rad/s
const float OMEGA_SLEW  = .5f;   // rad/s^2
const float V_MIN_SCALE = 0.15f;

const float E_SLOW = 30.0f * DEG_TO_RAD;  // start slowing
const float E_FF   = 10.0f * DEG_TO_RAD;  // allow feedforward below this

const float YAW_SIGN = +1.0f;     // flip to -1.0f if steering is inverted

float e_int = 0.0f, e_prev = 0.0f;
unsigned long t_prev_ms = 0;
float yaw_meas_prev = 0.0f, yaw_unwrapped = 0.0f;

float omega_cmd_prev = 0.0f;
float wR_prev = 0.0f, wL_prev = 0.0f;
const float W_SLEW = 80.0f;       // wheel target slew [rad/s^2]

unsigned long t0_ms = 0;
bool path_done = false;

// ==================== Simple startup input ====================
static bool readFloatWithTimeout(const char* prompt, float& outVal, float timeout_s){
  Serial.print(prompt);
  Serial.print(" ");
  unsigned long t0 = millis();
  String s = "";
  while (millis() - t0 < (unsigned long)(timeout_s*1000.0f)) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (s.length() > 0) {
          outVal = s.toFloat();
          Serial.println(outVal, 4);
          return true;
        }
      } else {
        s += c;
      }
    }
  }
  Serial.println("(timeout)");
  return false;
}

static void recomputePath(){
  half_span = SPAN_M * 0.5f;
  a_parab   = +HEIGHT_M / (half_span*half_span);  // positive bends right
  L_center  = simpson_arc_length();
  v_center  = L_center / TRAVEL_TIME_S;
}

static float slewStep(float prev, float des, float rate, float dt){
  float dw = des - prev;
  float maxdw = rate * dt;
  if (dw >  maxdw) return prev + maxdw;
  if (dw < -maxdw) return prev - maxdw;
  return des;
}

void setup() {
  Serial.begin(115200);
  delay(250);

  setupMotor(motor1, driver1, sensor1, Wire, 8, 9, ZERO_ELEC_1);
  setupMotor(motor2, driver2, sensor2, Wire1, 6, 7, ZERO_ELEC_2);

  bno_setup();
  float yaw0;
  if (bno_get_heading_rad(yaw0)) {
    yaw_meas_prev = yaw0;
    yaw_unwrapped = yaw0;
  }

  recomputePath();

  float theta_start = theta_des(-half_span);
  Serial.print("Params: SPAN=");
  Serial.print(SPAN_M, 3);
  Serial.print(" m, T=");
  Serial.print(TRAVEL_TIME_S, 3);
  Serial.print(" s, L_center=");
  Serial.print(L_center, 3);
  Serial.print(" m, v_center=");
  Serial.print(v_center, 3);
  Serial.println(" m/s");

  Serial.print("Place car at left start, heading about ");
  Serial.print(theta_start * 57.2958f, 2);
  Serial.println(" deg relative to +x. Then start.");

  t0_ms = millis();
  Serial.println("Path follower ready");
}

void loop() {
  motor1.loopFOC();
  motor2.loopFOC();

  if (path_done) {
    motor1.move(0.0f);
    motor2.move(0.0f);
    return;
  }

  unsigned long t_ms = millis();
  float t = (t_ms - t0_ms) * 0.001f;
  if (t >= TRAVEL_TIME_S) path_done = true;

  float x = -half_span + (SPAN_M * (t / TRAVEL_TIME_S));
  if (x >  half_span) x =  half_span;
  if (x < -half_span) x = -half_span;

  float theta_d = theta_des(x);

  // BNO read and unwrap
  static unsigned long last_bno = 0;
  if (t_ms - last_bno > 8) {
    last_bno = t_ms;
    float yaw_wrapped;
    if (bno_get_heading_rad(yaw_wrapped)) {
      float dy = yaw_wrapped - yaw_meas_prev;
      if (dy >  M_PI) dy -= 2.0f*M_PI;
      if (dy < -M_PI) dy += 2.0f*M_PI;
      yaw_unwrapped += dy;
      yaw_meas_prev = yaw_wrapped;
    }
  }

  float dt = (t_prev_ms == 0) ? 0.001f : (t_ms - t_prev_ms) * 0.001f;
  t_prev_ms = t_ms;

  float e = wrapPI(theta_d - wrapPI(yaw_unwrapped));

  // feedforward curvature only when nearly aligned
  float kappa = curvature(x);
  float omega_ff = (fabs(e) < E_FF) ? (v_center * kappa) : 0.0f;

  // PID
  e_int += e * dt;
  float de = (dt > 0) ? (e - e_prev)/dt : 0.0f;
  e_prev = e;

  float omega_ctrl = YAW_SIGN * (Kp_h*e + Ki_h*e_int + Kd_h*de);
  float omega_cmd = omega_ff + omega_ctrl;

  // clamp and anti windup
  bool saturated = false;
  if (omega_cmd >  OMEGA_MAX) { omega_cmd =  OMEGA_MAX; saturated = true; }
  if (omega_cmd < -OMEGA_MAX) { omega_cmd = -OMEGA_MAX; saturated = true; }
  if (saturated) e_int -= e * dt;

  // slew on yaw rate
  float domega = omega_cmd - omega_cmd_prev;
  float domega_max = OMEGA_SLEW * dt;
  if (domega >  domega_max) omega_cmd = omega_cmd_prev + domega_max;
  if (domega < -domega_max) omega_cmd = omega_cmd_prev - domega_max;
  omega_cmd_prev = omega_cmd;

  // gate linear speed by heading error
  float scale = 1.0f - fminf(1.0f, fabs(e) / E_SLOW);
  scale = fmaxf(V_MIN_SCALE, scale);
  float v_cmd = v_center * scale;

  // wheel targets
  float wR_des = (v_cmd + omega_cmd * (TRACK_W*0.5f)) / WHEEL_R;
  float wL_des = (v_cmd - omega_cmd * (TRACK_W*0.5f)) / WHEEL_R;

  // per wheel slew
  wR_prev = slewStep(wR_prev, wR_des, W_SLEW, dt);
  wL_prev = slewStep(wL_prev, wL_des, W_SLEW, dt);

  // motor directions: right positive, left negative
  motor1.move(wR_prev);
  motor2.move(-wL_prev);

  // telemetry at ~10 Hz
  static unsigned long lastP = 0;
  if (t_ms - lastP > 100) {
    lastP = t_ms;
    Serial.print("t=");
    Serial.print(t, 2);
    Serial.print("  e_deg=");
    Serial.print(e * 57.2958f, 2);
    Serial.print("  v_cmd=");
    Serial.print(v_cmd, 3);
    Serial.print("  omega=");
    Serial.print(omega_cmd, 3);
    Serial.print("  wR=");
    Serial.print(wR_prev, 2);
    Serial.print("  wL=");
    Serial.print(wL_prev, 2);
    Serial.print("  v1=");
    Serial.print(motor1.shaftVelocity(), 2);
    Serial.print("  v2=");
    Serial.print(motor2.shaftVelocity(), 2);
    Serial.print("  uq1=");
    Serial.print(motor1.voltage.q, 2);
    Serial.print("  uq2=");
    Serial.println(motor2.voltage.q, 2);
  }
}