#include "Hiome_AVR.h"

void Hiome_AVR::begin() {
  // setup LED's
  DDRB  = DDRB  | B00000010;  // set pin 9 as output
  ledOn();

  SERIAL_START;

  radio.initialize(RF69_915MHZ, NODEID, NETWORKID);
  #ifdef R3
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
  radio.encrypt(ENCRYPTKEY);
  radio.enableAutoPower(ATC_RSSI);

  if (flash.initialize()) flash.sleep();

  // check right on boot just in case this is a recovery attempt for a bricked sensor
  checkForUpdates();

  checkIfBatteryConnected();

  ledOff();
}

void Hiome_AVR::ledOn() {
  PORTB = PORTB | B00000010;  // pull pin 9 high
}

void Hiome_AVR::ledOff() {
  PORTB = PORTB & B11111101;  // pull LED low
}

void Hiome_AVR::sleep(period_t d) {
  LowPower.powerDown(d, ADC_OFF, BOD_ON);
}

void Hiome_AVR::checkForUpdates() {
  if (radio.receiveDone()) CheckForWirelessHEX(radio, flash, false);
}

void Hiome_AVR::checkIfBatteryConnected() {
  uint16_t total_change = 0;
  uint16_t b = checkBattery();
  for (uint8_t k=0; k<20; k++) {
    uint16_t b2 = checkBattery();
    int16_t bd = (int16_t)b - (int16_t)b2;
    total_change += (uint16_t)abs(bd);
    if (total_change > MAX_VOLTAGE_DRIFT ||
          b2 > MAX_ALLOWED_VOLTAGE || b2 < MIN_ALLOWED_VOLTAGE) {
      battConnected = false;
      return;
    }
    b = b2;
    sleep(SLEEP_30MS);
  }
  SERIAL_PRINTLN(total_change);
}

uint16_t Hiome_AVR::checkBattery() {
  if (!battConnected) return 0;
  // https://lowpowerlab.com/forum/index.php/topic,1206.0.html
  return (uint16_t)analogRead(BATT);
}

void Hiome_AVR::beatHeart(uint32_t maxBeats) {
  heartbeats++;
  if (heartbeats > maxBeats) publish("h", "0", 0);
}

uint8_t Hiome_AVR::publish(const char* msg, const char* meta, uint8_t retries, uint8_t timeout) {
  char sendBuf[60];
  int8_t len = sprintf(sendBuf, "%s;%s%u%u", msg, meta, checkBattery(), packetCount);
  if (len <= 0) return 0;

  bool success = radio.sendWithRetry(GATEWAYID, sendBuf, len, retries, timeout);

  #ifdef ENABLE_SERIAL
    SERIAL_PRINT(F("p "));
    SERIAL_PRINT(sendBuf);
    if (!success) { SERIAL_PRINT(F(" x")); }
    SERIAL_PRINTLN(F("\n\n"));
//    SERIAL_FLUSH;
  #endif

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
