#include <SimpleFOC.h>

// AS5600 I2C sensor instance for I2C1
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  // Set I2C1 pins before starting
  Wire1.setSDA(6);  // SDA pin for motor 2
  Wire1.setSCL(7);  // SCL pin for motor 2

  Serial.begin(115200);
  delay(300);  // Give USB a moment to initialize

  // Start I2C1
  Wire1.begin();
  Wire1.setClock(400000);  // Fast I2C at 400 kHz

  // Initialize AS5600 on I2C1
  sensor.init(&Wire1);

  Serial.println("Motor 2 sensor ready");
  _delay(1000);
}

void loop() {
  sensor.update();
  Serial.print(sensor.getAngle());    // in radians
  Serial.print("\t");
  Serial.println(sensor.getVelocity()); // rad/s
}