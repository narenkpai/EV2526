const int buttonPin = 14;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  int state = digitalRead(buttonPin);

  if (state == LOW) {
    Serial.println("Pressed");
  } else {
    Serial.println("Released");
  }
}