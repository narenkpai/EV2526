#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------- SPI for MT6835 --------
#define PIN_SCK        2
#define PIN_MOSI       3
#define PIN_MISO       4
#define SENSOR_CSLEFT  11
#define SENSOR_CSRIGHT 5

#ifndef MT6835_BITORDER
#define MT6835_BITORDER MSBFIRST
#endif
SPISettings mt6835SPI(1000000, MT6835_BITORDER, SPI_MODE3);

MagneticSensorMT6835 sensorLEFT(SENSOR_CSLEFT, mt6835SPI);
MagneticSensorMT6835 sensorRIGHT(SENSOR_CSRIGHT, mt6835SPI);

BLDCMotor motorLEFT  = BLDCMotor(7);
BLDCMotor motorRIGHT = BLDCMotor(7);
BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(18, 19, 21, 22);
BLDCDriver3PWM driverLEFT  = BLDCDriver3PWM(15, 14, 8, 9);

// -------- BNO055 on I2C1 --------
#define BNO_SDA 6
#define BNO_SCL 7
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);

// -------- OLED on I2C1 --------
#define OLED_ADDR      0x3C
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1);

// -------- Setpoint schedule --------
// Each entry is {duration_ms, target_degrees}
struct Segment { uint32_t ms; double deg; };

Segment schedule[] = {
  { 1071,  -2.000 },
  { 1071, -5.000 },
  { 1071, -7.000 },
  { 1073, -10.471 },

  { 1428,   0.000  },

  { 1071,  +2.000 },
  { 1071, +5.000 },
  { 1071, +7.000 },
  { 1073, +10.471 }
};
const size_t NUM_SEG = sizeof(schedule)/sizeof(schedule[0]);

// Loop the schedule when it ends? (set false to stop)
const bool LOOP_SCHEDULE = false;  

// total schedule length
uint32_t scheduleTotal = 0;

// -------- Control vars --------
bool scheduleDone = false;  
// -------- Control vars --------
double Setpoint = 0.0;   // target heading in deg
double Input    = 0.0;
double Output   = 0.0;
double BaseVelo = 30.17;

double heading    = 0.0; // wrapped
double rawHeading = 0.0; // 0..360 from BNO

// PID
double Kp=0.01, Ki=0.00, Kd=0.0;
PID myPID(&heading, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// time base
uint32_t t_start = 0;

// helpers
static inline double wrap180(double d){
  if (d > 180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

// pick setpoint from schedule given elapsed ms
double scheduledSetpoint(uint32_t elapsed_ms){
  if (NUM_SEG == 0) return Setpoint;

  if (!LOOP_SCHEDULE && elapsed_ms >= scheduleTotal) {
    scheduleDone = true;  // mark as finished
    return schedule[NUM_SEG-1].deg;  // hold final setpoint
  }

  uint32_t t = (LOOP_SCHEDULE) ? elapsed_ms % scheduleTotal : elapsed_ms;

  uint32_t acc = 0;
  for (size_t i = 0; i < NUM_SEG; i++) {
    acc += schedule[i].ms;
    if (t < acc) return schedule[i].deg;
  }
  return schedule[NUM_SEG-1].deg;
}

void setup() {
  Serial.begin(115200);

  // SPI
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  // I2C1
  Wire1.setSDA(BNO_SDA);
  Wire1.setSCL(BNO_SCL);
  Wire1.begin();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (1) {}
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("OLED OK");
  display.display();
  for (size_t i = 0; i < NUM_SEG; i++) {
    scheduleTotal += schedule[i].ms;
  }
  // BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    while (1) {}
  }
  bno.setExtCrystalUse(true);

  // Encoders and motors
  sensorLEFT.init(&SPI);
  sensorRIGHT.init(&SPI);
  motorLEFT.linkSensor(&sensorLEFT);
  motorRIGHT.linkSensor(&sensorRIGHT);

  driverLEFT.voltage_power_supply = 12.0f;
  driverLEFT.voltage_limit        = 12.0f;
  driverLEFT.init();

  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit        = 12.0f;
  driverRIGHT.init();

  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);

  motorLEFT.foc_modulation  = FOCModulationType::SpaceVectorPWM;
  motorRIGHT.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motorLEFT.controller  = MotionControlType::velocity;
  motorRIGHT.controller = MotionControlType::velocity;

  motorLEFT.PID_velocity.P = 0.5f;
  motorLEFT.PID_velocity.I = 8.0f;
  //motorLEFT.LPF_velocity.Tf = 0.01f;

  motorRIGHT.PID_velocity.P = 0.5f;
  motorRIGHT.PID_velocity.I = 8.0f;
  //motorRIGHT.LPF_velocity.Tf = 0.01f;

  motorLEFT.voltage_limit = 12.0f;
  motorRIGHT.voltage_limit = 12.0f;

  motorLEFT.PID_velocity.output_ramp  = 1000;
  motorRIGHT.PID_velocity.output_ramp = 1000;

  motorRIGHT.init();
  motorRIGHT.initFOC();
  motorLEFT.init();
  motorLEFT.initFOC();

  // initial heading and PID
  sensors_event_t event;
  bno.getEvent(&event);
  rawHeading = event.orientation.x;
  heading = wrap180(rawHeading);

  Setpoint = scheduledSetpoint(0);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-0.1, +0.1); // rad/s diff
  myPID.SetSampleTime(10);
  myPID.SetControllerDirection(DIRECT);

  t_start = millis();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup complete");
  display.display();

  Serial.println("Setup complete.");
}

void loop() {
  // time
  uint32_t elapsed = millis() - t_start;

  // check schedule state
  Setpoint = scheduledSetpoint(elapsed);

  // Read BNO055
  sensors_event_t event;
  bno.getEvent(&event);
  rawHeading = event.orientation.x;
  heading = wrap180(rawHeading);

  if (!scheduleDone) {
    // PID
    Input = heading;
    myPID.Compute();

    // Motors
    motorRIGHT.loopFOC();
    motorRIGHT.move((-BaseVelo) + Output);
    motorLEFT.loopFOC();
    motorLEFT.move(BaseVelo + Output);
  } else {
    // stop motors if schedule is complete
    motorRIGHT.loopFOC();
    motorRIGHT.move(0);
    motorLEFT.loopFOC();
    motorLEFT.move(0);
  }

  // Telemetry every 100 ms
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("t=");
    Serial.print(elapsed/1000.0, 1);
    Serial.print(" s | Hd=");
    Serial.print(heading, 1);
    Serial.print(" | Set=");
    Serial.print(Setpoint, 1);
    Serial.print(" | Out=");
    Serial.print(Output, 2);
    Serial.print(" | Done=");
    Serial.println(scheduleDone ? "YES" : "NO");
  }
}