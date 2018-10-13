#define FIRMWARE_VERSION     "V0.1"

#define BLIND     400
#define DARK      700
#define DIM       800
#define BRIGHT    1024
 
void initialize() {
  blink(4);
}
 
int last_lux = 0;
uint8_t cycles = 0;
void loop() {
  if (cycles > 60) {
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
    cycles = 0;
  }
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
}

