#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// SPI for MT6835
#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 4
#define SENSOR_CSLEFT 11
#define SENSOR_CSRIGHT 5

#ifndef MT6835_BITORDER
#define MT6835_BITORDER MSBFIRST
#endif
SPISettings mt6835SPI(1000000, MT6835_BITORDER, SPI_MODE3);

MagneticSensorMT6835 sensorLEFT(SENSOR_CSLEFT, mt6835SPI);
MagneticSensorMT6835 sensorRIGHT(SENSOR_CSRIGHT, mt6835SPI);

BLDCMotor motorLEFT = BLDCMotor(7);
BLDCMotor motorRIGHT = BLDCMotor(7);
//BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(18, 19, 21, 22);
BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(19, 18, 21, 22);

BLDCDriver3PWM driverLEFT = BLDCDriver3PWM(15, 14, 8, 9);

// BNO055 on I2C1
#define BNO_SDA 6
#define BNO_SCL 7
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);

// OLED on I2C1
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1);
const uint32_t PLOT_DT_MS = 50;
// Base velocity ramping   // commanded base speed in rad/s
double BaseVeloCur = 20.0;  // slewed base speed used for commands
uint32_t t_prev = 0;        // for dt
volatile bool running = true;
double targetRotations = 25.7;  // your stop point in wheel revs

// rotation counting state
bool rotInit = false;
double rotPrev = 0.0;
long wraps = 0;          // integer wraps
double rotations = 0.0;  // total revs including fraction


void plotToSerial(double vL_cmd, double vR_cmd, double baseCmd,
                  double vL_meas, double vR_meas) {
  Serial.print("vL_cmd:");
  Serial.print(vL_cmd, 3);
  Serial.print('\t');
  Serial.print("vR_cmd:");
  Serial.print(vR_cmd, 3);
  Serial.print('\t');
  Serial.print("Base:");
  Serial.print(baseCmd, 3);
  Serial.print('\t');
  Serial.print("vL_meas:");
  Serial.print(vL_meas, 3);
  Serial.print('\t');
  Serial.print("vR_meas:");
  Serial.println(vR_meas, 3);
}

// Control vars
double heading = 0.0;
double rawHeading = 0.0;

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

  // BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("BNO055 not found");
    display.display();
    while (1) {}
  }
  bno.setExtCrystalUse(true);

  // Encoders
  sensorLEFT.init(&SPI);
  rotPrev = sensorLEFT.getAngle();  // seed previous angle
  rotInit = true;
  wraps = 0;
  rotations = 0.0;
  t_prev = millis();
  sensorRIGHT.init(&SPI);
  motorLEFT.linkSensor(&sensorLEFT);
  motorRIGHT.linkSensor(&sensorRIGHT);

  // Drivers
  driverLEFT.voltage_power_supply = 12.0f;
  driverLEFT.voltage_limit = 12.0f;
  driverLEFT.init();

  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit = 12.0f;
  driverRIGHT.init();

  // Link drivers
  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);

  // Modulation
  motorLEFT.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorRIGHT.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Velocity control
  motorLEFT.controller = MotionControlType::velocity;
  motorRIGHT.controller = MotionControlType::velocity;

  // Velocity loop tuning
  motorLEFT.PID_velocity.P = 0.02f;
  motorLEFT.PID_velocity.I = 1.0f;
  motorLEFT.LPF_velocity.Tf = 0.01f;

  motorRIGHT.PID_velocity.P = 0.1f;
  motorRIGHT.PID_velocity.I = 5.0f;
  motorRIGHT.LPF_velocity.Tf = 0.01f;

  motorLEFT.voltage_limit = 12.0f;
  motorRIGHT.voltage_limit = 12.0f;

  motorLEFT.PID_velocity.output_ramp = 1000;  // rad/s^2
  motorRIGHT.PID_velocity.output_ramp = 1000;
  /*
  motorLEFT.sensor_direction=Direction::CW;
  motorLEFT.zero_electric_angle=4.8867;
  motorRIGHT.sensor_direction=Direction::CCW;
  motorRIGHT.zero_electric_angle=3.6644;
  */
  // Init FOC
  motorRIGHT.init();
  motorRIGHT.initFOC();
  motorLEFT.init();
  motorLEFT.initFOC();

  // Prime heading
  sensors_event_t event;
  bno.getEvent(&event);
  rawHeading = event.orientation.x;
  heading = (rawHeading > 180.0f) ? rawHeading - 360.0f : rawHeading;

  t_prev = millis();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup complete");
  display.display();
  Serial.println("Setup complete.");
}

void loop() {
  // dt for slew

  // Read IMU and wrap heading
  sensors_event_t event;
  bno.getEvent(&event);
  rawHeading = event.orientation.x;
  heading = (rawHeading > 180.0f) ? rawHeading - 360.0f : rawHeading;
  heading = constrain(heading, -1.0, +1.0);

  // FOC service first
  motorRIGHT.loopFOC();
  motorLEFT.loopFOC();

  // --- rotation counting from angle zero-crossings ---
  double ang = sensorLEFT.getAngle();  // 0..2π
  // update wrap counter
  if (!rotInit) {  // first pass
    rotPrev = ang;
    rotInit = true;
  } else {
    // delta in [-π, π]
    double d = ang - rotPrev;
    if (d > PI) d -= TWO_PI;
    if (d < -PI) d += TWO_PI;

    // detect crossing near 0
    const double LOW_BAND = 0.5;                            // rad
    const double HIGH_BAND = TWO_PI - 0.5;                  // rad
    if (rotPrev > HIGH_BAND && ang < LOW_BAND) wraps += 1;  // forward
    if (rotPrev < LOW_BAND && ang > HIGH_BAND) wraps -= 1;  // reverse
    rotPrev = ang;
  }

  // total rotations = integer wraps + fractional part
  double frac = ang / TWO_PI;                        // forward fraction
  if (wraps < 0) frac = -((TWO_PI - ang) / TWO_PI);  // reverse fraction
  rotations = wraps + frac;

  // --- stop logic ---
  if (running && rotations >= targetRotations) {
    running = false;    // latch stop
    BaseVeloCur = 0.0;  // hard stop, or ramp if you prefer
  }

  // command wheels once per loop
  //double vR = (BaseVeloCur) + heading/3;
  //double vL = (BaseVeloCur) - heading/3;
  double vR = (BaseVeloCur);
  double vL = (BaseVeloCur);
  motorRIGHT.move(vR);
  motorLEFT.move(vL);

  // Example: change speed later without a jump
  // if (millis() > 8000) setBaseVeloTarget(10.0, 1.5);

  // Minimal debug

  static uint32_t lastPlot = 0;
  if (millis() - lastPlot >= PLOT_DT_MS) {
    lastPlot = millis();
    // commanded velocities (what you send to move())
    double vL_cmd = vL;
    double vR_cmd = vR;
    // measured shaft velocities from SimpleFOC (rad/s)
    double vL_meas = motorLEFT.shaftVelocity();
    double vR_meas = motorRIGHT.shaftVelocity();
    plotToSerial(vL_cmd, vR_cmd, BaseVeloCur, vL_meas, vR_meas);
  }
}