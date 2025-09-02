#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25   // Onboard LED (GP25)
#endif

// I2C1 pins for your OLED
#define OLED_SDA 6
#define OLED_SCL 7

// OLED display size
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// I2C address for most 0.96" OLEDs
#define OLED_ADDR 0x3C

// Use Wire1 (I2C1) on pins GP6/GP7
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("RP2350 + OLED test");

  // Configure I2C1 pins
  Wire1.setSDA(OLED_SDA);
  Wire1.setSCL(OLED_SCL);
  Wire1.begin();

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    for (;;); // stop if OLED not found
  }

  display.clearDisplay();
  display.setTextSize(1);       // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("UCTRONICS 0.96\" OLED");
  display.println("RP2350 + I2C1");
  display.println("Addr 0x3C, SDA=6");
  display.println("SCL=7");
  display.display();

  Serial.println("OLED init complete");
}

void loop() {
  // Blink LED and update OLED
  static bool ledOn = false;
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Blink: ");
  display.println(ledOn ? "ON" : "OFF");
  display.display();

  delay(1000);
}