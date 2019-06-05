/*
Connect one end of FSR to power, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground 
*/

#define FIRMWARE_VERSION     "V0.1"
#define FSR                  0     // the FSR and 10K pulldown are connected to a0
#define PUBLISH_DELAY        15    // seconds to wait before publishing state change

uint16_t fsrReading;
uint16_t calibratedReading = 0;
bool detected = false;
uint8_t cyclesSinceChange = PUBLISH_DELAY;

void initialize() {
  blink(4);
  pinMode(FSR, INPUT_PULLUP);
  for (uint8_t i=0; i<10; i++) {
    calibratedReading += analogRead(FSR);
    LOWPOWER_DELAY(SLEEP_120MS);
  }
  calibratedReading /= 10;
}

void loop() {
  fsrReading = analogRead(FSR);

  SERIAL_PRINT(fsrReading);

  if (fsrReading < (calibratedReading + 100)) {
    SERIAL_PRINTLN(" - No pressure");
    if (detected) {
      detected = false;
      cyclesSinceChange = 0;
    } else if (cyclesSinceChange <= PUBLISH_DELAY) {
      cyclesSinceChange++;
      if (cyclesSinceChange == PUBLISH_DELAY) {
        publish("0");
      }
    }
  } else {
    SERIAL_PRINTLN(" - Sleeper");
    if (!detected) {
      detected = true;
      cyclesSinceChange = 0;
    } else if (cyclesSinceChange <= PUBLISH_DELAY) {
      cyclesSinceChange++;
      if (cyclesSinceChange == PUBLISH_DELAY) {
        publish("1");
      }
    }
  }
  SERIAL_FLUSH;
  LOWPOWER_DELAY(SLEEP_1S);
}
