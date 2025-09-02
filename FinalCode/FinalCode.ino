#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <SimpleFOC.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------- OLED --------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
const uint8_t OLED_ADDR = 0x3C;

// ---------- Parabola y = a x^2 + b x + c ----------
const float A_parab = -0.0625f;
const float B_parab =  0.5f;
const float C_parab =  0.0f;     // not used by arc length
const float X_START = 0.0f;
const float X_END   = 8.0f;

// ---------- Robot geometry ----------
const float WHEEL_DIAMETER = 0.0463f;
const float WHEEL_R        = 0.5f * WHEEL_DIAMETER;
const float TRACK_W        = 0.055f;

// ---------- Base wheel speeds (rad/s) ----------
float velocityRight = 20.0f;
float velocityLeft  = -20.0f;

// ---------- Start button ----------
const int BTN_PIN = 22;   // button to GND
bool run_enabled = false;

// ---------- BNO055 (UART) ----------
const uint8_t BNO_START_WRITE = 0xAA;
const uint8_t BNO_START_READ  = 0xAA;
const uint8_t BNO_CMD_WRITE   = 0x00;
const uint8_t BNO_CMD_READ    = 0x01;
const uint8_t BNO_RESP_OK     = 0xBB;
const uint8_t BNO_RESP_ERR    = 0xEE;
HardwareSerial &bno = Serial1;

// ---------- SimpleFOC motors ----------
MagneticSensorI2C sensor1(AS5600_I2C);
BLDCMotor         motor1(7);
BLDCDriver3PWM    driver1(2, 3, 4, 5);

MagneticSensorI2C sensor2(AS5600_I2C);
BLDCMotor         motor2(7);
BLDCDriver3PWM    driver2(28, 29, 27, 26);

// known electrical alignment
const float ZERO_ELEC_1 = 5.8720f;
const float ZERO_ELEC_2 = 0.5768f;

// ---------- PID_v1 for heading (degrees) ----------
double heading_meas = 0.0;
double heading_ref  = 0.0;
double heading_out  = 0.0;
double Kp = 0.6, Ki = 0.10, Kd = 0.02;
PID headingPID(&heading_meas, &heading_out, &heading_ref, Kp, Ki, Kd, DIRECT);

// ---------- Path distance state ----------
float s_travel = 0.0f;
float S_total  = 0.0f;
unsigned long last_ms = 0;

// ---------- helpers ----------
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
  if (fabsf(k) < 1e-9f) return sqrtf(1.0f + m*m) * x;
  const float z  = k*x + m;
  const float rt = sqrtf(1.0f + z*z);
  return (z*rt + asinh_compat(z)) / (2.0f*k);
}
static float S_of_x(float x) { return F_of_x(x) - F_of_x(X_START); }
static inline float dSdx(float x) {
  float z = 2.0f*A_parab*x + B_parab;
  return sqrtf(1.0f + z*z);
}
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

// ---------- OLED helpers ----------
static void oledShowAlignScreen(double target_deg, double meas_deg){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Align heading");

  display.setCursor(0,14);
  display.print("Target: ");
  display.print(target_deg, 2);
  display.println(" deg");

  display.setCursor(0,28);
  display.print("Actual: ");
  display.print(meas_deg, 2);
  display.println(" deg");

  display.setCursor(0,48);
  display.println("Press button to start");
  display.display();
}

static void oledShowRunning(){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 24);
  display.println("RUN");
  display.display();
}

void setup() {
  Serial.begin(115200);

  // Button
  pinMode(BTN_PIN, INPUT_PULLUP);

  // I2C0 bus for AS5600 and OLED
  Wire.setSDA(8); Wire.setSCL(9);
  Wire.begin();
  Wire.setClock(400000);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // fall back to serial if display missing
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.display();
  }

  // Motor 1 + sensor on I2C0
  sensor1.init(&Wire);
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::velocity;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.voltage_limit = 12.0f;
  motor1.velocity_limit = 80.0f;
  motor1.sensor_direction    = Direction::CCW;
  //motor1.zero_electric_angle = ZERO_ELEC_1;
  motor1.init();
  motor1.initFOC();

  // Motor 2 + sensor on I2C1
  Wire1.setSDA(6); Wire1.setSCL(7); Wire1.begin(); Wire1.setClock(400000);
  sensor2.init(&Wire1);
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.controller = MotionControlType::velocity;
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.voltage_limit = 12.0f;
  motor2.velocity_limit = 80.0f;
  motor2.sensor_direction    = Direction::CCW;
  //motor2.zero_electric_angle = ZERO_ELEC_2;
  motor2.init();
  motor2.initFOC();

  // BNO055
  bno.begin(115200);
  uint8_t cfg=0x00; bno_write(0x3D,1,&cfg); delay(25);
  uint8_t ndof=0x0C; bno_write(0x3D,1,&ndof); delay(50);

  // Arc length range
  S_total = S_of_x(X_END);

  // PID
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-30, 30);
  headingPID.SetSampleTime(20);

  last_ms = millis();
  heading_ref = tangentDegAt(X_START);

  // Show initial align screen
  double h_meas;
  if (!bno_read_heading_deg(h_meas)) h_meas = 0.0;
  oledShowAlignScreen(heading_ref, h_meas);

  Serial.println("Ready. Align to Target, then press button. 1 s delay after press.");
}

void loop() {
  // Keep FOC running
  motor1.loopFOC();
  motor2.loopFOC();

  // Alignment page until start
  if (!run_enabled) {
    static unsigned long lastOLED = 0;
    static unsigned long lastIMU  = 0;
    static double h_meas = 0.0;

    // read IMU ~20 Hz
    unsigned long now = millis();
    if (now - lastIMU > 50) {
      lastIMU = now;
      bno_read_heading_deg(h_meas);
    }
    // refresh OLED ~10 Hz
    if (now - lastOLED > 100) {
      lastOLED = now;
      oledShowAlignScreen(heading_ref, h_meas);
    }

    // wait for button
    if (digitalRead(BTN_PIN) == LOW) {
      delay(20);
      if (digitalRead(BTN_PIN) == LOW) {
        delay(1000);
        run_enabled = true;
        last_ms = millis();
        s_travel = 0.0f;
        oledShowRunning();
      }
    }
    motor1.move(0.0f);
    motor2.move(0.0f);
    return;
  }

  // After start, normal control
  unsigned long now = millis();
  float dt = (now - last_ms) * 0.001f;
  if (dt <= 0) dt = 0.001f;
  last_ms = now;

  // Integrate center distance from commanded wheel speeds
  float v_center = 0.5f * WHEEL_R * (velocityRight + (-velocityLeft)); // m/s
  s_travel += v_center * dt;
  if (s_travel > S_total) s_travel = S_total;

  // Map s -> x and compute reference heading
  float x = x_from_s(s_travel);
  double theta_ref_deg = tangentDegAt(x);

  // Read heading
  double theta_meas_deg = heading_meas;
  bno_read_heading_deg(theta_meas_deg);

  // PID on heading error
  double err = wrapDeg(theta_ref_deg - theta_meas_deg);
  heading_ref  = 0.0;
  heading_meas = -err;
  headingPID.Compute();

  // Convert PID output to wheel trim
  double omega_cmd = radians(heading_out);
  float  delta_w   = (omega_cmd * (TRACK_W * 0.5f)) / WHEEL_R;
  float  wR_cmd    = velocityRight + delta_w;
  float  wL_cmd    = velocityLeft  - delta_w;

  // Drive
  motor1.move(wR_cmd);
  motor2.move(wL_cmd);

  // Stop at end
  if (s_travel >= S_total - 1e-4f) {
    motor1.move(0.0f);
    motor2.move(0.0f);
    run_enabled = false;
    oledShowAlignScreen(heading_ref, theta_meas_deg); // go back to align screen
    Serial.println("Path done. Align for next run and press button.");
  }

  // Serial telemetry ~10 Hz
  static unsigned long lastP=0;
  if (now - lastP > 100) {
    lastP = now;
    Serial.print("x="); Serial.print(x,3);
    Serial.print("  s="); Serial.print(s_travel,3);
    Serial.print("  href="); Serial.print(theta_ref_deg,2);
    Serial.print("  out="); Serial.print(heading_out,2);
    Serial.print("  wR="); Serial.print(wR_cmd,2);
    Serial.print("  wL="); Serial.println(wL_cmd,2);
  }
}