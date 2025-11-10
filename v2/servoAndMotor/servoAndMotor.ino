#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <SimpleFOC.h>

// -------------------- Peripherals --------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);  // BNO055 on Wire1
static const int servoPin = 13;
Servo servo1;

// Magnetic sensor on primary I2C
MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

// BLDC motor and driver
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 16, 17, 4);

// -------------------- Shared state --------------------
volatile bool started = false;
volatile float startHeading = 0.0f;
volatile float finalHeading = 0.0f;

// -------------------- Tasks --------------------
TaskHandle_t taskIMUServoHandle = nullptr;  // IMU + servo on Core 0

static inline float wrapFloat(float x) {
  while (x > 180.0f) x -= 360.0f;
  while (x < -180.0f) x += 360.0f;
  return x;
}

// -------------------- IMU + Servo task (Core 0) --------------------
void imuServoTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(10); // ~100 Hz read + servo update
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    // Read Euler angles
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float raw = euler.x(); // 0..360 deg

    if (!started) {
      startHeading = raw;  // set boot direction as zero
      started = true;
    }

    float shifted = raw - startHeading;
    float fh = wrapFloat(shifted);

    // Write servo. 90 is straight. Negative is left, positive is right.
    // Clamp to sane range for write
    float servoCmd = 85.0f - fh;
    if (servoCmd < 0) servoCmd = 0;
    if (servoCmd > 180) servoCmd = 180;
    servo1.write((int)servoCmd);

    finalHeading = fh; // publish for anyone who cares

    vTaskDelayUntil(&last, period);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // ---------------- I2C setup ----------------
  // Magnetic sensor on Wire (pins 26 SDA, 27 SCL)
  Wire.begin(26, 27);
  Wire.setClock(400000);

  // BNO055 on Wire1 (pins 33 SDA, 32 SCL)
  Wire1.begin(33, 32);
  Wire1.setClock(400000);

  // ---------------- Sensors init ----------------
  sensor.init(&Wire);

  if (!bno.begin()) {
    Serial.println("BNO055 not found");
    while (1) { delay(1000); }
  }

  // ---------------- Servo init ----------------
  servo1.attach(servoPin);

  // ---------------- Motor + driver init ----------------
  driver.voltage_power_supply = 12;
  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.init();
  motor.initFOC();

  // ---------------- Disable watchdog on Core 1 ----------------
  // Core 1 will run motor control at 100%, no FreeRTOS scheduling
  disableCore1WDT();

  // ---------------- Task creation ----------------
  // IMU + Servo: scheduled task on Core 0 (default FreeRTOS core)
  xTaskCreatePinnedToCore(
    imuServoTask,
    "imuServoTask",
    4096,
    nullptr,
    1,          // priority
    &taskIMUServoHandle,
    0           // core 0 - FreeRTOS scheduled tasks
  );

  Serial.println("System initialized - motor control starting on Core 1");
}

void loop() {
  // Motor control runs at 100% on Core 1 (this core)
  // Motor + sensor must stay together, not split across cores
  const float targetVelocity = 20.0f;
  
  sensor.update();        // Read sensor
  motor.loopFOC();       // FOC algorithm
  motor.move(targetVelocity);  // Motion control
  
  // NO DELAY - runs at 100% to maintain tight timing
}