#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"

// ---------- Pins ----------
#define PIN_SCK        2
#define PIN_MOSI       3
#define PIN_MISO       4
#define SENSOR_CSLEFT  11
#define SENSOR_CSRIGHT 5

#ifndef MT6835_BITORDER
#define MT6835_BITORDER MSBFIRST
#endif
SPISettings mt6835SPI(1000000, MT6835_BITORDER, SPI_MODE3);

// ---------- Sensors ----------
MagneticSensorMT6835 sensorLEFT(SENSOR_CSLEFT, mt6835SPI);
MagneticSensorMT6835 sensorRIGHT(SENSOR_CSRIGHT, mt6835SPI);

// ---------- Motors and drivers ----------
BLDCMotor motorLEFT  = BLDCMotor(7);
BLDCMotor motorRIGHT = BLDCMotor(7);
BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(18, 19, 20, 21);
BLDCDriver3PWM driverLEFT  = BLDCDriver3PWM(12, 10, 8, 9);

// ---------- Targets ----------
static const float SET_VELOCITYRIGHT = -30.0f; // rad/s
static const float SET_VELOCITYLEFT  =  30.0f; // rad/s
static const float TARGET_REV_LEFT   = 15.0f;   // exact revolutions to make
static const float TARGET_REV_RIGHT  = 15.0f;

// ---------- Turn counters ----------
volatile int32_t rev_count_left  = 0;
volatile int32_t rev_count_right = 0;
float prev_angle_left  = 0.0f;  // last 0..2π reading
float prev_angle_right = 0.0f;
float zero_left  = 0.0f;        // mechanical zero angle at start
float zero_right = 0.0f;

// final absolute mechanical angle targets
float target_angle_left  = 0.0f;
float target_angle_right = 0.0f;

// simple state per motor
enum class Phase { Spin, Finish, Hold };
Phase phase_left  = Phase::Spin;
Phase phase_right = Phase::Spin;

static inline void seedTurnCounters() {
  sensorLEFT.update();
  sensorRIGHT.update();
  prev_angle_left  = sensorLEFT.getAngle();
  prev_angle_right = sensorRIGHT.getAngle();
  zero_left  = prev_angle_left;
  zero_right = prev_angle_right;
  rev_count_left  = 0;
  rev_count_right = 0;

  // direction from set velocities, sign picks plus or minus turns
  float dir_left  = (SET_VELOCITYLEFT  >= 0.0f) ? 1.0f : -1.0f;
  float dir_right = (SET_VELOCITYRIGHT >= 0.0f) ? 1.0f : -1.0f;

  target_angle_left  = zero_left  + dir_left  * TARGET_REV_LEFT  * TWO_PI;
  target_angle_right = zero_right + dir_right * TARGET_REV_RIGHT * TWO_PI;

  phase_left  = Phase::Spin;
  phase_right = Phase::Spin;
}

// wrap based rev counter
static inline void updateTurnsOne(MagneticSensorMT6835& s,
                                  float& prev_angle,
                                  volatile int32_t& rev_count)
{
  float a = s.getAngle();
  float da = a - prev_angle;
  const float wrap_threshold = 3.0f; // close to π, helps avoid false wraps
  if (da < -wrap_threshold) rev_count++;
  else if (da > wrap_threshold) rev_count--;
  prev_angle = a;
}

// get continuous mechanical angle using counter and zero
static inline float mechAngle(MagneticSensorMT6835 &s,
                              float zero,
                              volatile int32_t &rev_count)
{
  return (float)rev_count * TWO_PI + (s.getAngle() - zero);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // SPI0
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  // Sensors
  sensorLEFT.init(&SPI);
  sensorRIGHT.init(&SPI);

  // Link sensors
  motorLEFT.linkSensor(&sensorLEFT);
  motorRIGHT.linkSensor(&sensorRIGHT);

  // Drivers
  driverLEFT.voltage_power_supply = 12.0f;
  driverLEFT.voltage_limit        = 8.0f;
  driverLEFT.init();

  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit        = 8.0f;
  driverRIGHT.init();

  // Motors
  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);

  // velocity loop gains
  motorLEFT.PID_velocity.P = 1.5f;
  motorLEFT.PID_velocity.I = 8.0f;
  motorLEFT.PID_velocity.D = 0.0f;
  motorLEFT.LPF_velocity.Tf = 0.01f;
  motorLEFT.voltage_limit   = 12.0f;

  motorRIGHT.PID_velocity.P = 1.5f;
  motorRIGHT.PID_velocity.I = 8.0f;
  motorRIGHT.PID_velocity.D = 0.0f;
  motorRIGHT.LPF_velocity.Tf = 0.01f;
  motorRIGHT.voltage_limit   = 12.0f;

  // angle loop gains for the final lock
  motorLEFT.P_angle.P  = 25.0f;   // increase if the rotor sags
  motorLEFT.P_angle.D  = 0.1f;    // small damping helps stop cleanly
  motorLEFT.PID_velocity.limit = 12.0f;

  motorRIGHT.P_angle.P = 25.0f;
  motorRIGHT.P_angle.D = 0.1f;
  motorRIGHT.PID_velocity.limit = 12.0f;

  // init and FOC
  motorRIGHT.init();
  motorRIGHT.initFOC();
  motorLEFT.init();
  motorLEFT.initFOC();

  // start in velocity mode
  motorLEFT.controller  = MotionControlType::velocity;
  motorRIGHT.controller = MotionControlType::velocity;

  seedTurnCounters();

  Serial.println("Ready");
}

void loop() {
  // FOC update
  motorRIGHT.loopFOC();
  motorLEFT.loopFOC();

  // update revolution counters
  updateTurnsOne(sensorLEFT,  prev_angle_left,  rev_count_left);
  updateTurnsOne(sensorRIGHT, prev_angle_right, rev_count_right);

  // current mechanical angles
  float ang_left  = mechAngle(sensorLEFT,  zero_left,  rev_count_left);
  float ang_right = mechAngle(sensorRIGHT, zero_right, rev_count_right);

  // remaining distance to final target
  float rem_left  = target_angle_left  - ang_left;
  float rem_right = target_angle_right - ang_right;

  // when close, switch to angle control and hit the exact target
  // the window protects against overshoot from velocity control
  const float switch_window = 0.50f;   // rad, roughly 28.6°
  const float done_window   = 0.02f;   // rad, about 1.1°

  // LEFT state machine
  if (phase_left == Phase::Spin) {
    motorLEFT.move(SET_VELOCITYLEFT);
    if (fabsf(rem_left) < switch_window) {
      motorLEFT.controller = MotionControlType::angle;
      phase_left = Phase::Finish;
    }
  } else if (phase_left == Phase::Finish) {
    motorLEFT.move(target_angle_left);
    // when we hit within tight window, stay and hold
    if (fabsf(rem_left) < done_window) {
      phase_left = Phase::Hold;
    }
  } else { // Hold
    motorLEFT.move(target_angle_left); // keep holding exact target
  }

  // RIGHT state machine
  if (phase_right == Phase::Spin) {
    motorRIGHT.move(SET_VELOCITYRIGHT);
    if (fabsf(rem_right) < switch_window) {
      motorRIGHT.controller = MotionControlType::angle;
      phase_right = Phase::Finish;
    }
  } else if (phase_right == Phase::Finish) {
    motorRIGHT.move(target_angle_right);
    if (fabsf(rem_right) < done_window) {
      phase_right = Phase::Hold;
    }
  } else { // Hold
    motorRIGHT.move(target_angle_right);
  }

  // debug
  static uint32_t t0 = 0;
  if (millis() - t0 > 100) {
    t0 = millis();
    auto stateStr = [](Phase p){
      switch(p){ case Phase::Spin: return "Spin"; case Phase::Finish: return "Finish"; default: return "Hold";}
    };
    Serial.print("L ang=");
    Serial.print(ang_left, 3);
    Serial.print(" tgt=");
    Serial.print(target_angle_left, 3);
    Serial.print(" rem=");
    Serial.print(rem_left, 3);
    Serial.print(" ");
    Serial.print(stateStr(phase_left));
    Serial.print(" | R ang=");
    Serial.print(ang_right, 3);
    Serial.print(" tgt=");
    Serial.print(target_angle_right, 3);
    Serial.print(" rem=");
    Serial.print(rem_right, 3);
    Serial.print(" ");
    Serial.println(stateStr(phase_right));
  }
}