const int switchPin = 22;  // GP22 on Pico

void setup() {
  Serial.begin(115200);
  pinMode(switchPin, INPUT_PULLUP);  // enable internal pull-up
  Serial.println("Switch test ready. Press the switch to see changes.");
}

void loop() {
  int state = digitalRead(switchPin);

  if (state == LOW) {
    Serial.println("Pressed");
  } else {
    Serial.println("Released");
  }

  delay(200); // small delay to make output readable
}