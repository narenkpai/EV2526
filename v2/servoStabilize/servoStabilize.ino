#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  
// Change 0x28 to 0x29 if your board uses the other address
static const int servoPin = 13;
Servo servo1;
bool started = false;
float startHeading = 0;

float wrapFloat(float x) {
  while (x > 180) x -= 360;
  while (x < -180) x += 360;
  return x;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Start I2C on custom pins
  Wire.begin(33, 32);

  if (!bno.begin()) {
    Serial.println("BNO055 not found");
    while (1);
  }

  // Use NDOF mode so heading is stable
 // bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  servo1.attach(servoPin);

  delay(200);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float raw = euler.x();      // 0 to 360 deg

  if (!started) {
    startHeading = raw;       // set boot direction as zero
    started = true;
  }

  // shift so that boot heading is now zero
  float shifted = raw - startHeading;

  // wrap to -180 to 180
  float finalHeading = wrapFloat(shifted);
  servo1.write(90-finalHeading);
  // left turn negative, right turn positive
  Serial.println(finalHeading);

  delay(20);
}