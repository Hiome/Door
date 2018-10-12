#define FIRMWARE_VERSION     "V0.1"

int state = LOW;

void doorISR() {
  // do nothing
}

void initialize() {
  pinMode(PIR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIR), doorISR, CHANGE);
  blink(4);
}

void loop() {
  delay(500);
  int newState = digitalRead(PIR);
  if (state != newState) {
    state = newState;
    publish(state);
  }
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
}

