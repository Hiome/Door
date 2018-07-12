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
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
}

