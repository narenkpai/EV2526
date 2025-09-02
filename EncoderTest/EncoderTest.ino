#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"

// Chip select pins for the two encoders
#define SENSOR_A_nCS 5
#define SENSOR_B_nCS 11

// SPI pins (use your Pico hardware SPI0 pins)
#define PIN_SCK    2
#define PIN_MOSI   3
#define PIN_MISO   4

// SPI settings for MT6835 (mode 3, up to 10 MHz, start with 1 MHz)
SPISettings myMT6835SPISettings(1000000, MT6835_BITORDER, SPI_MODE3);

// Two sensor objects, same SPI bus, different CS
MagneticSensorMT6835 sensorA = MagneticSensorMT6835(SENSOR_A_nCS, myMT6835SPISettings);
MagneticSensorMT6835 sensorB = MagneticSensorMT6835(SENSOR_B_nCS, myMT6835SPISettings);

long ts;

void setup() {
  // Configure SPI pins for Pico
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  // Make CS pins safe
  pinMode(SENSOR_A_nCS, INPUT_PULLUP);
  pinMode(SENSOR_B_nCS, INPUT_PULLUP);

  // Enable SimpleFOC debugging
  SimpleFOCDebug::enable();

  // Init both encoders
  sensorA.init();
  sensorB.init();

  ts = millis();
}

void loop() {
  // Update both encoders
  sensorA.update();
  sensorB.update();

    // Print A
    SimpleFOCDebug::print("A angle: ");
    SimpleFOCDebug::print(sensorA.getAngle());
    SimpleFOCDebug::print(" ");

    // Print B
    SimpleFOCDebug::print("B angle: ");
    SimpleFOCDebug::println(sensorB.getAngle());

}