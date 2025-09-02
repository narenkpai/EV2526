#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Onboard LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// Use I2C1 (pins 6=SDA, 7=SCL on RP2350)
#define BNO_SDA 6
#define BNO_SCL 7

// Create the sensor object on I2C1
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("RP2350 + BNO055 I2C test (pins 6=SDA, 7=SCL)");

  // Configure Wire1 pins and speed
  Wire1.setSDA(BNO_SDA);
  Wire1.setSCL(BNO_SCL);
  Wire1.begin();

  if (!bno.begin()) {
    Serial.println("No BNO055 detected, check wiring or I2C addr (0x28/0x29)");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  // Use external crystal if available
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 ready");
}

void loop() {
  // Get Euler orientation (heading, roll, pitch)
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("Heading=");
  Serial.print(event.orientation.x, 1);
  Serial.print("  Roll=");
  Serial.print(event.orientation.z, 1);
  Serial.print("  Pitch=");
  Serial.println(event.orientation.y, 1);

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(10);
}