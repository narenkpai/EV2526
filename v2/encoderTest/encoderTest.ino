// ESP32 + MT6701 magnetic encoder over I2C
// SDA = GPIO13, SCL = GPIO12
// Prints angle at 100 Hz

#include <Arduino.h>
#include <Wire.h>

// ------------ I2C config ------------
static const int SDA_PIN = 26;
static const int SCL_PIN = 27;
static const uint32_t I2C_HZ = 400000;   // Fast mode

// ------------ MT6701 map ------------
static const uint8_t MT6701_ADDR    = 0x06;  // common default. Change if needed.
// Raw 14-bit angle spread across 0x03 (hi) and 0x04 (lo)
static const uint8_t REG_ANG_HI     = 0x03;  // upper 6 bits in [5:0]
static const uint8_t REG_ANG_LO     = 0x04;  // lower 8 bits
static const uint16_t MT6701_CPR    = 16384; // 14-bit full scale

// ---------- helpers ----------
bool i2cWriteThenRead(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  size_t got = Wire.requestFrom((int)addr, (int)n);
  if (got != n) return false;
  for (size_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}

bool mt6701_read_raw14(uint16_t& raw14) {
  uint8_t hi, lo;
  if (!i2cWriteThenRead(MT6701_ADDR, REG_ANG_HI, &hi, 1)) return false;
  if (!i2cWriteThenRead(MT6701_ADDR, REG_ANG_LO, &lo, 1)) return false;
  // hi holds bits [13:8] in its low 6 bits, lo holds [7:0]
  raw14 = ((uint16_t)(hi & 0x3F) << 8) | lo;
  return true;
}

bool mt6701_read_angle(float& deg, float& rad) {
  uint16_t raw;
  if (!mt6701_read_raw14(raw)) return false;
  deg = (360.0f * raw) / MT6701_CPR;
  rad = deg * (PI / 180.0f);
  return true;
}

void scanBus() {
  Serial.println("I2C scan:");
  int count = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) Serial.println("  No devices found");
}

// ---------- setup and loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Caution: GPIO12 is a boot strap pin. If you see boot issues, move to SDA=21 SCL=22.
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);

  scanBus();

  // Probe MT6701 address
  Wire.beginTransmission(MT6701_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.printf("MT6701 seems present at 0x%02X\n", MT6701_ADDR);
  } else {
    Serial.printf("MT6701 not found at 0x%02X. Check wiring or address.\n", MT6701_ADDR);
  }
}

void loop() {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms >= 10) {  // 100 Hz
    last_ms = now;

    float deg, rad;
    if (mt6701_read_angle(deg, rad)) {
      Serial.printf("Angle: %8.3f deg  %8.5f rad\n", deg, rad);
    } else {
      Serial.println("Read failed");
    }
  }
}