#include <SPI.h>

// SparkFun Pro Micro RP2040 hardware SPI0 pins
// Board silkscreen: SCK, COPI, CIPO, CS
constexpr uint8_t PIN_SCK =  2;   // SCK
constexpr uint8_t PIN_MOSI = 3;  // COPI (MCU -> MT6835 MOSI)
constexpr uint8_t PIN_MISO = 4;  // CIPO (MT6835 MISO -> MCU)
constexpr uint8_t PIN_CS   = 11;  // CS

// MT6835 SPI basics from datasheet:
// - Mode 3 (CPOL=1, CPHA=1)
// - Single-byte read uses command C3..C0 = 0b0011 and a 12-bit address
// - Angle bytes at 0x003, 0x004, 0x005, CRC at 0x006
// - STATUS bits are the low 3 bits of 0x005

static inline uint8_t mt6835_read_reg(uint16_t addr) {
  // Build the 24-bit frame:
  // [4 bits cmd=0b0011] [12 bits addr A11..A0] [8 dummy bits to clock out data]
  // We'll ship it as 3 bytes MSB first on MOSI while reading one data byte on MISO.
  const uint8_t cmd_hi = 0b00110000 | ((addr >> 8) & 0x0F); // cmd in upper nibble, addr[11:8] in lower nibble
  const uint8_t addr_lo = addr & 0xFF;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);

  SPI.transfer(cmd_hi);   // bits 1..8
  SPI.transfer(addr_lo);  // bits 9..16
  uint8_t data = SPI.transfer(0x00); // bits 17..24, read register byte

  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  // Per datasheet, when CS goes low the chip latches 0x003..0x006 so a sequence
  // of reads returns a consistent angle set.
  delayMicroseconds(2); // small guard time
  return data;
}

static inline uint8_t crc8_mt6835(uint8_t b0, uint8_t b1, uint8_t b2) {
  // CRC over 24 bits: ANGLE[20:0] then STATUS[2:0], MSB first.
  // Polynomial x^8 + x^2 + x + 1 -> 0x07
  uint8_t crc = 0x00;
  auto step = [&](uint8_t bit) {
    uint8_t fb = ((crc >> 7) & 1) ^ (bit & 1);
    crc = (uint8_t)((crc << 1) & 0xFF);
    if (fb) crc ^= 0x07;
  };
  // Feed b0 MSB..LSB, then b1, then top 5 bits of b2 for ANGLE, then the low 3 status bits.
  for (int i = 7; i >= 0; --i) step((b0 >> i) & 1);
  for (int i = 7; i >= 0; --i) step((b1 >> i) & 1);
  for (int i = 7; i >= 3; --i) step((b2 >> i) & 1);  // ANGLE[4:0] at bits 7..3
  for (int i = 2; i >= 0; --i) step((b2 >> i) & 1);  // STATUS[2:0] at bits 2..0
  return crc;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // Be explicit about SPI pins on RP2040
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  Serial.println("MT6835 angle read test");
}

void loop() {
  // Read the 4 angle bytes latched together
  uint8_t b0 = mt6835_read_reg(0x003); // ANGLE[20:13]
  uint8_t b1 = mt6835_read_reg(0x004); // ANGLE[12:5]
  uint8_t b2 = mt6835_read_reg(0x005); // ANGLE[4:0] + STATUS[2:0]
  uint8_t crc_read = mt6835_read_reg(0x006); // CRC over angle+status

  // Assemble 21-bit angle
  uint32_t angle21 =
      ((uint32_t)b0 << 13) |
      ((uint32_t)b1 << 5)  |
      ((uint32_t)(b2 >> 3) & 0x1F);

  // Extract STATUS bits
  uint8_t status = b2 & 0x07;
  bool warn_overspeed = status & 0x01;
  bool warn_weakfld   = status & 0x02;
  bool warn_uv        = status & 0x04;

  // Check CRC
  uint8_t crc_calc = crc8_mt6835(b0, b1, b2);
  bool crc_ok = (crc_calc == crc_read);

  // Convert to degrees
  // theta = angle21 * 360 / 2^21
  double degrees = (double)angle21 * (360.0 / 2097152.0);

  Serial.print("raw21=");
  Serial.print(angle21);
  Serial.print("  deg=");
  Serial.print(degrees, 4);
  Serial.print("  CRC ");
  Serial.print(crc_ok ? "OK" : "BAD");
  Serial.print("  status[OVR,WEAK,UV]=");
  Serial.print(warn_overspeed);
  Serial.print(",");
  Serial.print(warn_weakfld ? 1 : 0);
  Serial.print(",");
  Serial.println(warn_uv ? 1 : 0);

  delay(50);
}