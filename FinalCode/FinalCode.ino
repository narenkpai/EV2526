#include <Arduino.h>
#include <PID_v1.h>
#include <SimpleFOC.h>

// -------- Parabola y = a x^2 + b x + c  (roots at 0 and 8) --------
const float A_parab = -0.0625f;  // -1/16
const float B_parab =  0.5f;
const float C_parab =  0.0f;
const float X_START = 0.0f;
const float X_END   = 8.0f;

// -------- Robot geometry --------
const float WHEEL_DIAMETER = 0.0463f;
const float WHEEL_R        = 0.5f * WHEEL_DIAMETER;
const float TRACK_W        = 0.055f;   // axle track

// -------- Base wheel speeds (rad/s) --------
float velocityRight = 20.0f;   // right wheel forward positive
float velocityLeft  = -20.0f;  // left wheel forward negative (your convention)

// -------- BNO055 (UART) --------
const uint8_t BNO_START_WRITE = 0xAA;
const uint8_t BNO_START_READ  = 0xAA;
const uint8_t BNO_CMD_WRITE   = 0x00;
const uint8_t BNO_CMD_READ    = 0x01;
const uint8_t BNO_RESP_OK     = 0xBB;
const uint8_t BNO_RESP_ERR    = 0xEE;
HardwareSerial &bno = Serial1;

// -------- SimpleFOC motors --------
MagneticSensorI2C sensor1(AS5600_I2C);
BLDCMotor         motor1(7);
BLDCDriver3PWM    driver1(2, 3, 4, 5);

MagneticSensorI2C sensor2(AS5600_I2C);
BLDCMotor         motor2(7);
BLDCDriver3PWM    driver2(29, 28, 27, 26);

// -------- PID_v1 for heading (degrees) --------
double heading_meas = 0.0;    // degrees
double heading_ref  = 0.0;    // degrees
double heading_out  = 0.0;    // PID output (deg/s equivalent -> we will map)
double Kp = 0.6, Ki = 0.10, Kd = 0.02;
PID headingPID(&heading_meas, &heading_out, &heading_ref, Kp, Ki, Kd, DIRECT);

// -------- Distance along path --------
float s_travel = 0.0f;    // meters
float S_total  = 0.0f;    // meters
unsigned long last_ms = 0;

// ---------- math helpers ----------
static inline float asinh_compat(float z){ return logf(z + sqrtf(z*z + 1.0f)); }
static inline float wrapDeg(float d){
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// F(x) primitive so that S(x) = F(x) - F(X_START)
static float F_of_x(float x) {
  const float k = 2.0f * A_parab;
  const float m = B_parab;
  if (fabsf(k) < 1e-9f) {
    return sqrtf(1.0f + m*m) * x;
  }
  const float z  = k*x + m;
  const float rt = sqrtf(1.0f + z*z);
  return (z*rt + asinh_compat(z)) / (2.0f*k);
}

static float S_of_x(float x) { return F_of_x(x) - F_of_x(X_START); }

// dS/dx = sqrt(1 + (2 a x + b)^2)
static inline float dSdx(float x) {
  float z = 2.0f*A_parab*x + B_parab;
  return sqrtf(1.0f + z*z);
}

// slope angle (degrees) at x
static float tangentDegAt(float x) {
  float slope = 2.0f*A_parab*x + B_parab;
  return atan(slope) * (180.0f/PI);
}

// Newton solve S(x) = s
static float x_from_s(float s) {
  if (s <= 0.0f)     return X_START;
  if (s >= S_total)  return X_END;

  float x = X_START + (s / S_total) * (X_END - X_START);
  for (int i=0; i<6; ++i) {
    float fx  = S_of_x(x) - s;
    float dfx = dSdx(x);
    if (dfx < 1e-9f) break;
    float step = fx/dfx;
    x -= step;
    if (x < X_START) x = X_START;
    if (x > X_END)   x = X_END;
    if (fabsf(step) < 1e-6f) break;
  }
  return x;
}

// ---------- BNO low level ----------
static bool bno_read(uint8_t reg, uint8_t len, uint8_t* buf){
  bno.write(BNO_START_READ); bno.write(BNO_CMD_READ); bno.write(reg); bno.write(len); bno.flush();
  unsigned long t0=millis();
  auto rb=[&](uint8_t &o)->bool{ while(!bno.available()){ if(millis()-t0>50) return false; } o=bno.read(); return true; };
  uint8_t a; if(!rb(a)) return false;
  if (a==BNO_RESP_ERR){ uint8_t e; rb(e); return false; }
  if (a!=BNO_RESP_OK) return false;
  uint8_t rlen; if(!rb(rlen)) return false; if (rlen!=len) return false;
  for(uint8_t i=0;i<len;i++){ if(!rb(buf[i])) return false; }
  return true;
}
static bool bno_write(uint8_t reg,uint8_t len,const uint8_t* data){
  bno.write(BNO_START_WRITE); bno.write(BNO_CMD_WRITE); bno.write(reg); bno.write(len);
  for(uint8_t i=0;i<len;i++) bno.write(data[i]); bno.flush();
  unsigned long t0=millis(); while(!bno.available()){ if(millis()-t0>50) return false; }
  uint8_t a=bno.read(); while(!bno.available()){ if(millis()-t0>50) return false; }
  uint8_t b=bno.read(); return (a==0xEE && b==0x07) || (a==0xBB && b==0x00);
}
static bool bno_read_heading_deg(double &degOut){
  uint8_t raw[2];
  if (!bno_read(0x1A, 2, raw)) return false;
  int16_t h = (int16_t)((raw[1] << 8) | raw[0]); // 1/16 deg
  degOut = h / 16.0;
  return true;
}

void setup() {
  Serial.begin(115200);

  // Motors
  Wire.setSDA(8); Wire.setSCL(9); Wire.begin(); Wire.setClock(400000);
  sensor1.init(&Wire); motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12; driver1.init(); motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::velocity; motor1.init(); motor1.initFOC();

  Wire1.setSDA(6); Wire1.setSCL(7); Wire1.begin(); Wire1.setClock(400000);
  sensor2.init(&Wire1); motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 12; driver2.init(); motor2.linkDriver(&driver2);
  motor2.controller = MotionControlType::velocity; motor2.init(); motor2.initFOC();

  // BNO055
  bno.begin(115200);
  uint8_t cfg=0x00; bno_write(0x3D,1,&cfg); delay(25);
  uint8_t ndof=0x0C; bno_write(0x3D,1,&ndof); delay(50);

  // Arc length range
  S_total = S_of_x(X_END);
  Serial.print("S_total = "); Serial.println(S_total,6);

  // PID setup
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-30, 30); // deg/s equivalent for trim
  headingPID.SetSampleTime(20);        // ms

  last_ms = millis();

  // Initial reference heading at x = X_START
  heading_ref = tangentDegAt(X_START);
}

void loop() {
  // dt
  unsigned long now = millis();
  float dt = (now - last_ms) * 0.001f;
  if (dt <= 0) dt = 0.001f;
  last_ms = now;

  // Distance integration from commanded wheel speeds.
  // Left forward is negative by convention, so center v uses (wR + -wL)
  float v_center = 0.5f * WHEEL_R * (velocityRight + (-velocityLeft)); // m/s
  s_travel += v_center * dt;
  if (s_travel > S_total) s_travel = S_total;

  // Map s -> x on the parabola
  float x = x_from_s(s_travel);

  // Desired tangent heading at x (degrees)
  double theta_ref_deg = tangentDegAt(x);

  // Read BNO heading (degrees)
  double theta_meas_deg = heading_meas; // fallback to previous if read fails
  bno_read_heading_deg(theta_meas_deg);

  // Wrap measured to be near reference to avoid 359/0 jumps
  double err = wrapDeg(theta_ref_deg - theta_meas_deg);
  heading_ref  = 0.0;          // make PID setpoint zero error
  heading_meas = -err;         // feed error as input so Output = correction

  // Run PID
  headingPID.Compute();        // heading_out is in "deg error units per sample"

  // Interpret PID output as desired yaw rate in deg/s, convert to rad/s
  double omega_cmd = radians(heading_out);

  // Convert yaw rate to wheel angular speed trim
  float delta_w = (omega_cmd * (TRACK_W * 0.5f)) / WHEEL_R;
  float wR_cmd  = velocityRight + delta_w;
  float wL_cmd  = velocityLeft  - delta_w;

  // Drive motors with trimmed velocities
  motor1.loopFOC(); motor2.loopFOC();
  motor1.move(wR_cmd);
  motor2.move(wL_cmd); // left forward is negative already in velocityLeft

  // Light telemetry
  static unsigned long lastP=0;
  if (now - lastP > 100) {
    lastP = now;
    Serial.print("x="); Serial.print(x,3);
    Serial.print("  s="); Serial.print(s_travel,3);
    Serial.print("  href="); Serial.print(theta_ref_deg,2);
    Serial.print("  hmeas="); Serial.print(theta_meas_deg,2);
    Serial.print("  out="); Serial.print(heading_out,2);
    Serial.print("  wR="); Serial.print(wR_cmd,2);
    Serial.print("  wL="); Serial.println(wL_cmd,2);
  }
}