#define BLIND     400
#define DARK      700
#define DIM       800
#define BRIGHT    1024

void motion() {
  publish("m");
}

void initialize() {
  pinMode(PIR0, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR0), motion, RISING);
}

static int last_lux = 0;
void loop() {
  int curr_lux = analogRead(LUX);
  if (curr_lux <= BLIND) {
    if (last_lux != BLIND) {
      publish("blind");
      last_lux = BLIND;
    }
  } else if (curr_lux <= DARK) {
    if (last_lux != DARK) {
      publish("dark");
      last_lux = DARK;
    }
  } else if (curr_lux <= DIM) {
    if (last_lux != DIM) {
      publish("dim");
      last_lux = DIM;
    }
  } else {
    if (last_lux != BRIGHT) {
      publish("bright");
      last_lux = BRIGHT;
    }
  }
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
}
