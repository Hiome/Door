#define FIRMWARE_VERSION     "V0.1"
#define BATTERY_POWERED

void motionISR() {
  // do nothing
}

void initialize() {
  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), motionISR, RISING);
  blink(4);
}

void loop() {
  publish("1");
  blink(1);
  LOWPOWER_DELAY(SLEEP_FOREVER);
}

