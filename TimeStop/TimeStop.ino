#include <SimpleFOC.h>

// ================== Motors and AS5600 sensors ==================
MagneticSensorI2C sensorR(AS5600_I2C);    // right wheel encoder
BLDCMotor motorR(7);
BLDCDriver3PWM driverR(2, 3, 4, 5);

MagneticSensorI2C sensorL(AS5600_I2C);    // left wheel encoder
BLDCMotor motorL(7);
BLDCDriver3PWM driverL(29, 28, 27, 26);

const float SUPPLY_V = 9.6f;
const float ZERO_ELEC_R = 5.8721f;
const float ZERO_ELEC_L = 0.5768f;

// Wheel and chassis
const float WHEEL_DIAM   = 0.0463f;           // m
const float WHEEL_R      = 0.5f * WHEEL_DIAM; // m
const float TRACK_W      = 0.055f;            // m

// Base wheel speeds you want to run at (rad/s)
// Positive wR spins the right wheel forward. Left wheel forward is negative sign in move() below.
const float wR_base = 20.0f;
const float wL_base = 20.0f;

// ================== Parabola y(x) = A x^2 + B x + C ==================
// Define your curve and the x-range to traverse
const float A_parab = -0.062500; // example for height=1.0 and span=8.0 -> y = 1 - (4h/L^2)x^2
const float B_parab = 0.0f;
const float C_parab = 1.0f;

const float X_START = -4.0f;   // m
const float X_END   = +4.0f;   // m

// You place the car at X_START with this heading (deg). Use the slope there.
const float START_HEADING_DEG = atan(2.0f*A_parab*X_START + B_parab) * 57.2957795f;

// ============= BNO055 over UART (minimal read for heading) =============
const uint8_t BNO_START = 0xAA;
const uint8_t BNO_READ  = 0x01;
const uint8_t BNO_WRITE = 0x00;
const uint8_t BNO_OK    = 0xBB;
const uint8_t BNO_ERR   = 0xEE;
HardwareSerial &bno = Serial1;

static bool bno_read(uint8_t reg, uint8_t len, uint8_t *buf) {
  bno.write(BNO_START); bno.write(BNO_READ); bno.write(reg); bno.write(len); bno.flush();
  unsigned long t0 = millis();
  auto rb = [&](uint8_t &o)->bool{ while(!bno.available()){ if(millis()-t0>10) return false; } o=bno.read(); return true; };
  uint8_t h; if(!rb(h)) return false;
  if(h==BNO_ERR){ uint8_t e; rb(e); return false; }
  if(h!=BNO_OK) return false;
  uint8_t rlen; if(!rb(rlen)) return false;
  if(rlen!=len) return false;
  for(uint8_t i=0;i<len;i++){ if(!rb(buf[i])) return false; }
  return true;
}
static bool bno_write(uint8_t reg, uint8_t len, const uint8_t *data) {
  bno.write(BNO_START); bno.write(BNO_WRITE); bno.write(reg); bno.write(len);
  for(uint8_t i=0;i<len;i++) bno.write(data[i]); bno.flush();
  unsigned long t0 = millis(); while(!bno.available()){ if(millis()-t0>10) return false; }
  uint8_t a=bno.read(); while(!bno.available()){ if(millis()-t0>10) return false; }
  uint8_t b=bno.read();
  return (a==0xEE && b==0x07) || (a==0xBB && b==0x00);
}
static inline float wrapPI(float a){ while(a> M_PI) a-=2.0f*M_PI; while(a<-M_PI) a+=2.0f*M_PI; return a; }
static inline float deg2rad(float d){ return d*0.01745329252f; }
static bool bno_get_heading_rad(float &head_rad){
  uint8_t raw[2]; if(!bno_read(0x1A,2,raw)) return false;
  int16_t h = (int16_t)((raw[1]<<8)|raw[0]); head_rad = wrapPI(deg2rad(h/16.0f)); return true;
}
static void bno_setup(){
  bno.begin(115200);
  uint8_t cfg=0x00; bno_write(0x3D,1,&cfg); delay(25);
  uint8_t ndof=0x0C; bno_write(0x3D,1,&ndof); delay(50);
}

// ============= Helpers for parabola, arc length, and LUT =============
static inline float slope(float x){ return 2.0f*A_parab*x + B_parab; }
static inline float theta_des(float x){ return atanf(slope(x)); }
static inline float arcIntegrand(float x){ float yp = slope(x); return sqrtf(1.0f + yp*yp); }

// Simpson arc length over [X_START, X_END]
const int ARC_N = 2000; // even
static float simpson_arc_length(){
  float L = X_END - X_START;
  float dx = L / ARC_N, sum = 0.0f;
  for(int i=0;i<=ARC_N;i++){
    float x = X_START + i*dx;
    float c = (i==0 || i==ARC_N) ? 1.0f : (i&1 ? 4.0f : 2.0f);
    sum += c * arcIntegrand(x);
  }
  return (dx/3.0f)*sum;
}

// Arc-length LUT and inverse map s -> x
const int LUT_N = 1200;
static float x_lut[LUT_N+1], s_lut[LUT_N+1];
static float L_center = 0.0f;
static void buildArcLUT(){
  float dx = (X_END - X_START) / LUT_N;
  x_lut[0] = X_START; s_lut[0] = 0.0f;
  float s = 0.0f;
  for(int i=0;i<LUT_N;i++){
    float x0 = X_START + i*dx;
    float x1 = x0 + dx;
    float xm = 0.5f*(x0+x1);
    float ds = (dx)*(arcIntegrand(x0) + 4.0f*arcIntegrand(xm) + arcIntegrand(x1))/6.0f;
    s += ds;
    x_lut[i+1] = x1; s_lut[i+1] = s;
  }
  L_center = s;
}
static float x_from_s(float s){
  if(s<=0) return X_START;
  if(s>=L_center) return X_END;
  int lo=0, hi=LUT_N;
  while(hi-lo>1){ int m=(lo+hi)>>1; if(s_lut[m]<=s) lo=m; else hi=m; }
  float ds = s_lut[hi]-s_lut[lo];
  float t = ds>1e-9f ? (s - s_lut[lo])/ds : 0.0f;
  return x_lut[lo] + t*(x_lut[hi]-x_lut[lo]);
}

// ============= Motor setup helper =============
static void setupMotor(BLDCMotor& m, BLDCDriver3PWM& d,
                       MagneticSensorI2C& s, TwoWire& w,
                       uint8_t sda, uint8_t scl, float zero_elec){
  w.setSDA(sda); w.setSCL(scl); w.begin(); w.setClock(1000000);
  s.init(&w);
  m.linkSensor(&s);
  d.voltage_power_supply = SUPPLY_V; d.init(); m.linkDriver(&d);
  m.torque_controller = TorqueControlType::voltage;
  m.controller = MotionControlType::velocity;
  m.PID_velocity.P = 0.7f; m.PID_velocity.I = 1.2f; m.PID_velocity.D = 0.0f;
  m.PID_velocity.limit = SUPPLY_V*0.97f;
  m.voltage_limit = SUPPLY_V*0.97f;
  m.LPF_velocity.Tf = 0.08f;
  m.sensor_direction = Direction::CCW;
  m.zero_electric_angle = zero_elec;
  m.foc_modulation = FOCModulationType::SpaceVectorPWM;
  m.modulation_centered = 1.0f;
  m.init(); m.initFOC();
}

// ============= Heading control =============
// PID trims yaw rate to match desired slope angle
float Kp_h = 3.0f, Ki_h = 1.0f, Kd_h = 0.0f;
const float OMEGA_MAX  = 5.0f;   // rad/s
const float OMEGA_SLEW = 2.0f;   // rad/s^2
float e_int=0.0f, e_prev=0.0f, omega_prev=0.0f;

// progress from wheel distances
float sR=0.0f, sL=0.0f, s_center=0.0f;
float lastAngR=0.0f, lastAngL=0.0f;
static inline float unwrapDelta(float now, float prev){
  float d = now - prev; if(d> M_PI) d-=2.0f*M_PI; if(d<-M_PI) d+=2.0f*M_PI; return d;
}
// Left wheel forward is commanded with negative sign in move(), so flip when accumulating distance
const float LEFT_SIGN = -1.0f;

bool finished=false;

void setup() {
  Serial.begin(115200); delay(200);

  setupMotor(motorR, driverR, sensorR, Wire, 8, 9, ZERO_ELEC_R);
  setupMotor(motorL, driverL, sensorL, Wire1, 6, 7, ZERO_ELEC_L);

  bno_setup();

  buildArcLUT();

  // set baseline angles
  lastAngR = motorR.shaftAngle();
  lastAngL = motorL.shaftAngle();

  Serial.println(F("Parabola follower ready"));
  Serial.print(F("Start heading you should place: "));
  Serial.print(START_HEADING_DEG, 2);
  Serial.println(F(" deg"));
  Serial.print(F("Arc length L_center = "));
  Serial.print(L_center, 3);
  Serial.println(F(" m"));
}

void loop() {
  motorR.loopFOC();
  motorL.loopFOC();

  if(finished){
    static bool disableOnce=false;
    if(!disableOnce){
      motorR.move(0); motorL.move(0);
      motorR.disable(); motorL.disable();
      disableOnce = true;
      Serial.println(F("Done. Distance reached."));
    }
    return;
  }

  // Update distances from encoders
  float aR = motorR.shaftAngle();
  float aL = motorL.shaftAngle();
  float dthR = unwrapDelta(aR, lastAngR);
  float dthL = unwrapDelta(aL, lastAngL);
  lastAngR = aR; lastAngL = aL;

  sR += WHEEL_R * dthR;
  sL += WHEEL_R * (LEFT_SIGN * dthL);
  s_center = 0.5f * (sR + sL);

  // Stop by distance
  if(s_center >= L_center){
    finished = true;
  }

  // Map center distance to x along curve
  float x = x_from_s(s_center);
  float theta_d = theta_des(x);

  // IMU heading, unwrap
  static float yaw_prev_wrapped = 0.0f;
  static float yaw_unwrapped = 0.0f;
  static unsigned long last_bno = 0;
  unsigned long now = millis();
  if(now - last_bno > 8){
    last_bno = now;
    float yaw_wrapped;
    if(bno_get_heading_rad(yaw_wrapped)){
      float dy = yaw_wrapped - yaw_prev_wrapped;
      if(dy> M_PI) dy -= 2.0f*M_PI;
      if(dy<-M_PI) dy += 2.0f*M_PI;
      yaw_unwrapped += dy;
      yaw_prev_wrapped = yaw_wrapped;
    }
  }

  // Heading PID
  static unsigned long t_prev = 0;
  float dt = (t_prev==0) ? 0.001f : (now - t_prev)*0.001f;
  t_prev = now;

  float e = wrapPI(theta_d - wrapPI(yaw_unwrapped));
  e_int += e*dt;
  float de = (dt>0) ? (e - e_prev)/dt : 0.0f;
  e_prev = e;

  float omega_cmd = Kp_h*e + Ki_h*e_int + Kd_h*de;

  // limit and slew
  bool sat=false;
  if(omega_cmd >  OMEGA_MAX){ omega_cmd =  OMEGA_MAX; sat=true; }
  if(omega_cmd < -OMEGA_MAX){ omega_cmd = -OMEGA_MAX; sat=true; }
  if(sat) e_int -= e*dt; // basic anti-windup

  float domega = omega_cmd - omega_prev;
  float max_domega = OMEGA_SLEW * dt;
  if(domega >  max_domega) omega_cmd = omega_prev + max_domega;
  if(domega < -max_domega) omega_cmd = omega_prev - max_domega;
  omega_prev = omega_cmd;

  // Base linear speed from your base wheel speeds
  float v_base = WHEEL_R * 0.5f * (wR_base + wL_base);
  // Convert v and omega to desired wheel speeds
  float wR_des = (v_base + omega_cmd*(TRACK_W*0.5f)) / WHEEL_R;
  float wL_des = (v_base - omega_cmd*(TRACK_W*0.5f)) / WHEEL_R;

  // Command motors. Right forward is +, left forward is negative command.
  motorR.move(wR_des);
  motorL.move(-wL_des);

  // Telemetry at ~10 Hz
  static unsigned long lastP=0;
  if(now - lastP > 100){
    lastP = now;
    Serial.print(F("s=")); Serial.print(s_center,3);
    Serial.print(F("/")); Serial.print(L_center,3);
    Serial.print(F(" x=")); Serial.print(x,3);
    Serial.print(F(" th_d=")); Serial.print(theta_d*57.2958f,2);
    Serial.print(F(" e=")); Serial.print(e*57.2958f,2);
    Serial.print(F(" wR=")); Serial.print(wR_des,3);
    Serial.print(F(" wL=")); Serial.print(wL_des,3);
    Serial.print(F(" vR=")); Serial.print(motorR.shaftVelocity(),2);
    Serial.print(F(" vL=")); Serial.print(motorL.shaftVelocity(),2);
    Serial.println();
  }
}