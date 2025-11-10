// RP2040 I2C scanner for SparkFun Pro Micro RP2040
// Works with both Wire (I2C0) and Wire1 (I2C1)
// Edit the configs array to try your SDA,SCL pin pairs
// Example pairs below match common setups you mentioned

#include <Arduino.h>
#include <Wire.h>

struct BusConfig {
  TwoWire* bus;     // Wire or Wire1
  int sda;          // SDA pin
  int scl;          // SCL pin
  const char* tag;  // label for printing
};

// Add or remove entries as needed
BusConfig configs[] = {
  { &Wire,  8,  9,  "I2C0 on SDA=8 SCL=9"   },
  { &Wire1, 6,  7,  "I2C1 on SDA=6 SCL=7"   },
  { &Wire,  0,  1,  "I2C0 on SDA=0 SCL=1"   },
  // If you use the Qwiic connector, add the Qwiic pins here
  // For many SparkFun RP2040 boards, Qwiic is SDA=20 SCL=21
  // { &Wire1, 20, 21, "I2C1 on SDA=20 SCL=21" },
};

void scanBus(TwoWire& bus) {
  byte count = 0;
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      Serial.println();
      count++;
    } else if (err == 4) {
      Serial.print("  Unknown error at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
    delayMicroseconds(500);  // small gap helps on some buses
  }
  if (count == 0) {
    Serial.println("  No I2C devices found.");
  } else {
    Serial.print("  Total devices found: ");
    Serial.println(count);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println();
  Serial.println("RP2040 I2C scanner");
  Serial.println("Make sure you have pullups on SDA and SCL");
  Serial.println();

  for (unsigned i = 0; i < sizeof(configs) / sizeof(configs[0]); i++) {
    TwoWire* bus = configs[i].bus;

    // Set pins before begin
    bus->setSDA(configs[i].sda);
    bus->setSCL(configs[i].scl);

    // Start the bus
    bus->begin();
    bus->setClock(400000);  // fast mode, change to 100000 if needed
    delay(5);

    Serial.print("Scanning ");
    Serial.println(configs[i].tag);
    scanBus(*bus);
    Serial.println();

    // Optional short pause between scans
    delay(50);
  }

  Serial.println("Scan complete.");
}

void loop() {
  // Nothing here. If you want periodic scans, move the for loop from setup into loop with a delay.
}