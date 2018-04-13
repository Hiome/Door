#define LUX_THRESHOLD   100

void motion() {
  publish("m");
}

void initialize() {
  pinMode(PIR0, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR0), motion, RISING);
}

static int last_lux = 0;
char LUXstr[5];
void loop() {
  int curr_lux = analogRead(LUX);
  int lux_diff = curr_lux - last_lux;
  if (abs(lux_diff) > LUX_THRESHOLD) {
    itoa(curr_lux, LUXstr, 10);
    publish(LUXstr);
  }
  last_lux = curr_lux;
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
}
