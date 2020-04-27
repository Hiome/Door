#include "Hiome_AVR.h"

void Hiome_AVR::begin(uint8_t nodeid, uint8_t networkid, const char* encryptkey, bool highPowerMode) {
  // setup LED's
  DDRB  = DDRB  | B00000010;  // set pin 9 as output
  setLED(true);

  radio.initialize(RF69_915MHZ, nodeid, networkid);
  if (highPowerMode) radio.setHighPower();
  radio.encrypt(encryptkey);
  radio.enableAutoPower(HIOME_ATC_RSSI);

  if (flash.initialize()) flash.sleep();

  // check right on boot just in case this is a recovery attempt for a bricked sensor
  checkForUpdates();

  checkIfBatteryConnected();

  setLED(false);
}

void Hiome_AVR::setLED(bool on) {
  if (on)
    PORTB = PORTB | B00000010;  // pull LED pin 9 high
  else
    PORTB = PORTB & B11111101;  // pull LED pin 9 low
}

// SLEEP_15MS
// SLEEP_30MS 
// SLEEP_60MS
// SLEEP_120MS
// SLEEP_250MS
// SLEEP_500MS
// SLEEP_1S
// SLEEP_2S
// SLEEP_4S
// SLEEP_8S
// SLEEP_FOREVER
void Hiome_AVR::wait(period_t d) {
  LowPower.powerDown(d, ADC_OFF, BOD_ON);
}

void Hiome_AVR::checkForUpdates() {
  // TODO make this smarter to only check if it was previously told of an
  // impending update from an ack. Otherwise, radio should keep sleeping.
  if (radio.receiveDone()) CheckForWirelessHEX(radio, flash, false);
}

// probe BATT pin to see if it's floating or not
void Hiome_AVR::checkIfBatteryConnected() {
  uint16_t total_change = 0;
  uint16_t b = checkBattery();
  for (uint8_t k=0; k<20; k++) {
    uint16_t b2 = checkBattery();
    int16_t bd = (int16_t)b - (int16_t)b2;
    total_change += (uint16_t)abs(bd);
    if (total_change > HIOME_MAX_VOLTAGE_DRIFT ||
          b2 > HIOME_MAX_ALLOWED_VOLTAGE || b2 < HIOME_MIN_ALLOWED_VOLTAGE) {
      battConnected = false;
      return;
    }
    b = b2;
    wait(SLEEP_30MS);
  }
}

// returns raw analog reading, multiply by 1.007 to get true voltage
uint16_t Hiome_AVR::checkBattery() {
  if (!battConnected) return 0;
  return (uint16_t)analogRead(HIOME_BATT_PIN);
}

void Hiome_AVR::beatHeart(uint32_t maxBeats) {
  heartbeats++;
  if (heartbeats > maxBeats) publish("h", 0);
}

// max possible length of msg is 54 characters (including null character)
uint8_t Hiome_AVR::publish(const char* msg, uint8_t retries, bool debug) {
  char sendBuf[60];
  int8_t len = sprintf(sendBuf, "%s;%u%u", msg, checkBattery(), packetCount);
  if (len <= 0) return 0;

  bool success = radio.sendWithRetry(HIOME_GATEWAYID, sendBuf, len, retries, HIOME_RETRY_TIME);

  if (debug) {
    Serial.print(F("p "));
    Serial.print(sendBuf);
    if (!success) { Serial.print(F(" x")); }
    Serial.println(F("\n\n"));
  }

  if (success) heartbeats = 0;

  if (success || retries) {
    if (packetCount < 9) {
      return packetCount++;
    } else {
      packetCount = 1;
      return 9;
    }
  }

  return 0;
}
