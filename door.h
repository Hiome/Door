#define FIRMWARE_VERSION     "V0.1"
#define BATTERY_POWERED

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
  int newState = digitalRead(PIR);
  if (state != newState) {
    state = newState;
    publish(state);
  }
  LOWPOWER_DELAY(SLEEP_FOREVER);
}

