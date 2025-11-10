#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// -------- SPI for MT6835 --------
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
BLDCDriver3PWM driverRIGHT = BLDCDriver3PWM(18, 19, 21, 22);
BLDCDriver3PWM driverLEFT = BLDCDriver3PWM(15, 14, 8, 9);

// -------- BNO055 on I2C1 --------
#define BNO_SDA 6
#define BNO_SCL 7
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);

// -------- OLED on I2C1 --------
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1);

// -------- Control vars --------
double BaseVelo = 20.0;    // commanded base speed [rad/s]
double BaseVeloCur = 0.0;  // slewed base speed used for commands// accel limit [rad/s^2], auto-set by setBaseVeloTarget
uint32_t t_prev = 0;

double heading = 0.0;     // wrapped
double rawHeading = 0.0;  // 0..360 from BNO

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

  // OLED first so we can message errors on Serial if needed
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
  motorLEFT.PID_velocity.P = .1f;
  motorLEFT.PID_velocity.I = 8.0f;
  motorLEFT.LPF_velocity.Tf = 0.01f;

  motorRIGHT.PID_velocity.P = .1f;
  motorRIGHT.PID_velocity.I = 8.0f;
  motorRIGHT.LPF_velocity.Tf = 0.01f;

  motorLEFT.voltage_limit = 12.0f;
  motorRIGHT.voltage_limit = 12.0f;

  motorLEFT.PID_velocity.output_ramp = 1000;  // rad/s^2
  motorRIGHT.PID_velocity.output_ramp = 1000;

  motorLEFT.sensor_direction=Direction::CW;
  motorLEFT.zero_electric_angle=4.8867;
  motorRIGHT.sensor_direction=Direction::CCW;
  motorRIGHT.zero_electric_angle=3.6644;
  // Init FOC

  // Init FOC
  motorRIGHT.init();
  motorRIGHT.initFOC();
  motorLEFT.init();
  motorLEFT.initFOC();

  // Initial heading and PID
  sensors_event_t event;
  bno.getEvent(&event);
  rawHeading = event.orientation.x;
  heading = (rawHeading > 180.0f) ? rawHeading - 360.0f : rawHeading;

  double Setpoint = 0.0;  // hold 0 deg heading

  // Warm screen
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup complete");
  display.display();

  Serial.println("Setup complete.");
  t_prev = millis();
}

void loop() {
  // Read BNO055
  sensors_event_t event;
  bno.getEvent(&event);
  rawHeading = event.orientation.x;
  heading = (rawHeading > 180.0f) ? rawHeading - 360.0f : rawHeading;
  heading = constrain(heading, -2, +2);

  // Motors
  motorRIGHT.loopFOC();
  motorRIGHT.move((BaseVelo) - heading / 9);
  motorLEFT.loopFOC();
  motorLEFT.move(BaseVelo - heading / 9);

  // Compute values for display
  float vL = motorLEFT.shaftVelocity();
  float vR = motorRIGHT.shaftVelocity();
}