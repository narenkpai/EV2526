#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <SimpleFOC.h>
#include <math.h>

const float trackLen = 7.127;
const float tarTime = 20.0;
const float standDist = 5+2*sqrt(2);
const float straightDist = trackLen - 7.00;
const float circum = 43 * 3.14159 / 1000.0;
float totalDist = standDist+straightDist;
float constVelocity = (standDist / tarTime) / (circum / (2 * PI));

int i = 0;
//2d array with waypoints
//{{pt1}{pt2}{pt3}...}
//{dist,heading}

//Trapiz
float paths[6][2] = {{straightDist, 0.00},{1.00, 0.00},{sqrt(2),-45.00},
                   {3.00,0.00},{sqrt(2), 45.00},{1, 0.00}};


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire1);
static const int servoPin = 13;
Servo servo1;

MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 16, 17, 4);

// -------------------- Shared state --------------------
volatile bool started = false;
volatile float startHeading = 0.0f;
volatile float finalHeading = 0.0f;
volatile float targetHeading = 0.0f;
volatile float totalRevolutions = 0.0f;  // Target heading for IMU control
// -------------------- Revolution-based waypoint system --------------------
struct Waypoint {
  float targetRevolutions;  // When to trigger this waypoint
  float velocity;           // Speed of rotation (rad/s)
  float imuTargetAngle;     // Target heading in degrees for IMU servo control
};

// -------------------- Tasks --------------------
TaskHandle_t taskIMUServoHandle = nullptr;
const int buttonPin = 14;

static inline float wrapFloat(float x) {
  while (x > 180.0f) x -= 360.0f;
  while (x < -180.0f) x += 360.0f;
  return x;
}

// -------------------- IMU + Servo task (Core 0) --------------------
void imuServoTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float raw = euler.x();

    if (!started) {
      startHeading = raw;
      started = true;
    }

    float shifted = raw - startHeading;
    float currentHeading = wrapFloat(shifted);

    // Calculate error between current heading and target heading
    float headingError = targetHeading - currentHeading;
    headingError = wrapFloat(headingError);  // Wrap to -180 to 180

    // Servo control: 85 is center (straight)
    // Positive error means need to turn right, negative means turn left
    float servoCmd = 85.0f + headingError;
    if (servoCmd < 0) servoCmd = 0;
    if (servoCmd > 180) servoCmd = 180;
    servo1.write((int)servoCmd);

    finalHeading = currentHeading;

    vTaskDelayUntil(&last, period);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);

  bool buttonStatus = true;
  bool mainLoop = false;
  int state = digitalRead(buttonPin);

  while (buttonStatus) {
    state = digitalRead(buttonPin);
    if (state == LOW) {
      mainLoop = true;
      buttonStatus = false;
    }
  }

  // ---------------- I2C setup ----------------
  Wire.begin(26, 27);
  Wire.setClock(400000);
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
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 4.4033;
  
  // PID tuning
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20.0f;
  motor.PID_velocity.D = 0.001f;
  motor.velocity_limit = 30.0f;

  while (mainLoop) {
    motor.init();
    motor.initFOC();

    // Initialize revolution counting
    totalRevolutions = 0.0f;

    
    // Set initial target heading
    targetHeading = 0.00;

    disableCore1WDT();

    xTaskCreatePinnedToCore(
      imuServoTask,
      "imuServoTask",
      4096,
      nullptr,
      1,
      &taskIMUServoHandle,
      0
    );

    Serial.println("System initialized - motor control starting on Core 1");
    Serial.println("Waypoint system active:");
    for (int i = 0; i < 0; i++) {
      Serial.print("  ");
      Serial.print(i);
      Serial.print(": After ");
      //Serial.print(waypoints[i].targetRevolutions);
      Serial.print(" revs -> Velocity: ");
      //Serial.print(waypoints[i].velocity);
      Serial.print(" rad/s, Heading: ");
      //Serial.print(waypoints[i].imuTargetAngle);
      Serial.println("°");
    }
    break;
  }
}

void loop() {
  sensor.update();
  motor.loopFOC();

  // Calculate total revolutions from motor shaft angle
  totalRevolutions = motor.shaft_angle / (2.0f * PI);
  
  // Calculate distance traveled
  float distanceTraveled = totalRevolutions * circum;

  // Iterate through waypoints
  if (i < 6) {
    // Calculate cumulative distance threshold for current waypoint
    float cumulativeDistance = 0.0f;
    for (int j = 0; j <= i; j++) {
      cumulativeDistance += paths[j][0];
    }
    
    // Set target heading for current waypoint
    targetHeading = paths[i][1];
    
    // Check if we've reached the required cumulative distance for this waypoint
    if (distanceTraveled >= cumulativeDistance) {
      Serial.print("Waypoint ");
      Serial.print(i);
      Serial.print(" complete. Distance: ");
      Serial.print(distanceTraveled, 2);
      Serial.print(" m, Heading: ");
      Serial.println(paths[i][1]);
      
      i++;  // Move to next waypoint
    }
  } else {
    motor.move(0);  // Stop motor
    Serial.println("All waypoints complete!");
  }
  
  if (i < 6) {
    motor.move(constVelocity);
  }
}