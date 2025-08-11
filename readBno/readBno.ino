// RP2040 + Arduino core
// UART0 default pins: TX=GPIO0, RX=GPIO1

// BNO055 UART frame constants
const uint8_t BNO_START_WRITE = 0xAA; // host->sensor write
const uint8_t BNO_START_READ  = 0xAA; // host->sensor read cmd uses same 0xAA with different command code
const uint8_t BNO_CMD_WRITE   = 0x00;
const uint8_t BNO_CMD_READ    = 0x01;
const uint8_t BNO_RESP_OK     = 0xBB;
const uint8_t BNO_RESP_ERR    = 0xEE;

HardwareSerial &bno = Serial1;

bool bno_read(uint8_t reg, uint8_t len, uint8_t *buf) {
  // Send: 0xAA 0x01 reg len
  bno.write(BNO_START_READ);
  bno.write(BNO_CMD_READ);
  bno.write(reg);
  bno.write(len);
  bno.flush();

  // Response: 0xBB len data[len]
  unsigned long t0 = millis();
  auto readByte = [&](uint8_t &out)->bool {
    while (!bno.available()) {
      if (millis() - t0 > 50) return false;
    }
    out = bno.read();
    return true;
  };

  uint8_t hdr;
  if (!readByte(hdr)) return false;
  if (hdr == BNO_RESP_ERR) {
    uint8_t err; readByte(err);
    return false;
  }
  if (hdr != BNO_RESP_OK) return false;

  uint8_t rlen;
  if (!readByte(rlen)) return false;
  if (rlen != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    if (!readByte(buf[i])) return false;
  }
  return true;
}

bool bno_write(uint8_t reg, uint8_t len, const uint8_t *data) {
  // Send: 0xAA 0x00 reg len data[len]
  bno.write(BNO_START_WRITE);
  bno.write(BNO_CMD_WRITE);
  bno.write(reg);
  bno.write(len);
  for (uint8_t i = 0; i < len; i++) bno.write(data[i]);
  bno.flush();

  // Response: 0xEE 0x01 if error, or 0xEE 0x07 if write success on some firmwares,
  // or 0xBB 0x00 (no data) on others. We accept either BB or EE:07.
  unsigned long t0 = millis();
  while (!bno.available()) {
    if (millis() - t0 > 50) return false;
  }
  uint8_t a = bno.read();
  while (!bno.available()) {
    if (millis() - t0 > 50) return false;
  }
  uint8_t b = bno.read();
  if (a == 0xEE && b == 0x07) return true; // write OK
  if (a == 0xBB && b == 0x00) return true; // write OK, 0 data
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // If you want other pins, set them here before begin:
  // bno.setTX(0); bno.setRX(1);
  bno.begin(115200); // 8N1 default

  // Simple ping: read CHIP_ID at 0x00, expect 0xA0
  uint8_t chip;
  if (bno_read(0x00, 1, &chip)) {
    Serial.print("CHIP_ID: 0x"); Serial.println(chip, HEX);
  } else {
    Serial.println("CHIP_ID read failed");
  }

  // Put device in NDOF mode for fused Euler angles
  // Set OPR_MODE (0x3D) to 0x0C, but only from CONFIGMODE
  // 1) write 0x00 to OPR_MODE to enter CONFIGMODE
  uint8_t cfg = 0x00;
  bno_write(0x3D, 1, &cfg);
  delay(25);
  // 2) write 0x0C to OPR_MODE for NDOF
  uint8_t ndof = 0x0C;
  bno_write(0x3D, 1, &ndof);
  delay(50);
}

void loop() {
  // Read Euler heading (unit: 1/16 degree) at 0x1A LSB, 0x1B MSB
  uint8_t raw[2];
  if (bno_read(0x1A, 2, raw)) {
    int16_t h = (int16_t)((raw[1] << 8) | raw[0]);
    float heading_deg = h / 16.0f;
    Serial.print("Heading: "); Serial.println(heading_deg);
  } else {
    Serial.println("Heading read failed");
  }
  delay(100);
}