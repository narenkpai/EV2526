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

// -------- Control vars --------
double Setpoint, Input, Output;
double BaseVelo = 20.0;

double heading = 0.0;    // wrapped
double rawHeading = 0.0; // 0..360 from BNO

// PID tuning
double Kp=5.2, Ki=1.5, Kd=0.0;
PID myPID(&heading, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
  driverLEFT.voltage_limit        = 6.0f;
  driverLEFT.init();

  driverRIGHT.voltage_power_supply = 12.0f;
  driverRIGHT.voltage_limit        = 6.0f;
  driverRIGHT.init();

  // Link drivers
  motorLEFT.linkDriver(&driverLEFT);
  motorRIGHT.linkDriver(&driverRIGHT);

  // Modulation
  motorLEFT.foc_modulation  = FOCModulationType::SpaceVectorPWM;
  motorRIGHT.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Velocity control
  motorLEFT.controller  = MotionControlType::velocity;
  motorRIGHT.controller = MotionControlType::velocity;

  // Velocity loop tuning
  motorLEFT.PID_velocity.P = 0.3f;
  motorLEFT.PID_velocity.I = 1.0f;
  motorRIGHT.PID_velocity.P = 0.3f;
  motorRIGHT.PID_velocity.I = 1.0f;
  motorLEFT.voltage_limit = 12.0f;
  motorRIGHT.voltage_limit = 12.0f;

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

  Setpoint = 0.0; // hold 0 deg heading
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100.0, +100.0); // rad/s diff
  myPID.SetSampleTime(10);
  myPID.SetControllerDirection(DIRECT);

  // Warm screen
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup complete");
  display.display();

  Serial.println("Setup complete.");
}

void loop() {
  // Read BNO055
  sensors_event_t event;
  bno.getEvent(&event);
  rawHeading = event.orientation.x;
  heading = (rawHeading > 180.0f) ? rawHeading - 360.0f : rawHeading;

  // PID
  Input = heading;
  myPID.Compute();

  // Motors
  motorRIGHT.loopFOC();
  motorRIGHT.move((-BaseVelo) + Output);
  motorLEFT.loopFOC();
  motorLEFT.move(BaseVelo + Output);

  // Compute values for display
  float vL = motorLEFT.shaftVelocity();
  float vR = motorRIGHT.shaftVelocity();
  float tgt = Setpoint; // degrees

  // Serial debug every 100 ms
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("Raw: ");
    Serial.print(rawHeading, 2);
    Serial.print(" deg | Wrap: ");
    Serial.print(heading, 2);
    Serial.print(" deg | Set: ");
    Serial.print(tgt, 1);
    Serial.print(" deg | vL: ");
    Serial.print(vL, 2);
    Serial.print(" | vR: ");
    Serial.print(vR, 2);
    Serial.print(" | Out: ");
    Serial.println(Output, 2);

    // OLED update
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Hd act: ");
    display.print(heading, 1);
    display.println(" deg");

    display.print("Hd tgt: ");
    display.print(tgt, 1);
    display.println(" deg");

    display.print("vL: ");
    display.print(vL, 1);
    display.println(" rad/s");

    display.print("vR: ");
    display.print(vR, 1);
    display.println(" rad/s");

    display.print("PID: ");
    display.print(Output, 1);
    display.display();
  }
}