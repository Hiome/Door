/*
 *  Check VL53L0X sensors repeatedly for human presence and direction of movement.
 *  Reads sensor data and streams any changes over an RFM69 radio.
 *  
 *  Copyright 2018 Neil Gupta
 *  All rights reserved.
 *  
 */

#include "config.h"
#define NETWORKID     27  // the same on all nodes that talk to each other
#define GATEWAYID     1
#define ENCRYPTKEY    "smarterisbetters" // exactly the same 16 characters/bytes on all nodes!
#define ATC_RSSI      -75
#define SERIAL_BAUD   115200
#define RETRY_TIME    60
#define RETRY_COUNT   20
#define BATT          A3

#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>
#include <LowPower.h>

RFM69_ATC radio;
SPIFlash flash(8, 0xEF30); //EF30 for windbond 4mbit flash

#ifdef ENABLE_SERIAL
  #define SERIAL_START      ( Serial.begin(SERIAL_BAUD) )
  #define SERIAL_FLUSH      ( Serial.flush() )
  #define SERIAL_PRINT(a)   ( Serial.print(a) )
  #define SERIAL_PRINTLN(a) ( Serial.println(a) )
#else
  #define SERIAL_START
  #define SERIAL_FLUSH
  #define SERIAL_PRINT(a)
  #define SERIAL_PRINTLN(a)
#endif

#define LOWPOWER_DELAY(d) ( LowPower.powerDown(d, ADC_OFF, BOD_ON) )

uint32_t heartbeats = 0;
void beatHeart(uint32_t maxBeats) {
  heartbeats++;
  if (heartbeats > maxBeats) publish("h", "0", 0);
}

bool battConnected = true;
uint16_t checkBattery() {
  if (!battConnected) return 0;
  // https://lowpowerlab.com/forum/index.php/topic,1206.0.html
  return (uint16_t)analogRead(BATT);
}

#define MAX_VOLTAGE_DRIFT   50
#define MAX_ALLOWED_VOLTAGE 700

void isBatteryConnected() {
  uint16_t total_change = 0;
  uint16_t b = checkBattery();
  for (uint8_t k=0; k<20; k++) {
    uint16_t b2 = checkBattery();
    if (b2 > MAX_ALLOWED_VOLTAGE) {
      battConnected = false;
      return;
    }
    int16_t bd = (int16_t)b - (int16_t)b2;
    total_change += abs(bd);
    if (total_change > MAX_VOLTAGE_DRIFT) break;
    b = b2;
    LOWPOWER_DELAY(SLEEP_30MS);
  }
  SERIAL_PRINTLN(total_change);
  if (total_change > MAX_VOLTAGE_DRIFT) battConnected = false;
}

uint8_t packetCount = 1;
uint8_t publish(const char* msg, const char* meta, uint8_t retries) {
  char sendBuf[57];
  int8_t len = sprintf(sendBuf, "%s;%s%u%u", msg, meta, checkBattery(), packetCount);
  if (len <= 0) return 0;

  bool success = radio.sendWithRetry(GATEWAYID, sendBuf, len, retries, RETRY_TIME);

  #ifdef ENABLE_SERIAL
    SERIAL_PRINT(F("p "));
    SERIAL_PRINT(sendBuf);
    if (!success) SERIAL_PRINT(F(" x"));
    SERIAL_PRINTLN(F("\n\n"));
    SERIAL_FLUSH;
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

#if defined LIDAR
  #include "lidar.h"
#elif defined MOTION
  #include "motion.h"
#elif defined DOOR
  #include "door.h"
#elif defined THERMAL
  #include "thermal.h"
#elif defined BED
  #include "bed.h"
#elif defined BATTERY
  #include "battery.h"
#else
  #error Missing node type
#endif

void setup() {
  // setup LED's
  DDRB  = DDRB  | B00000010;  // set pin 9 as output
  PORTB = PORTB | B00000010;  // pull pin 9 high

  SERIAL_START;

  radio.initialize(RF69_915MHZ, NODEID, NETWORKID);
  #ifdef R3
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
  radio.encrypt(ENCRYPTKEY);
  radio.enableAutoPower(ATC_RSSI);

  if (flash.initialize()) flash.sleep();

  isBatteryConnected();

  initialize();

  PORTB = PORTB & B11111101;  // pull LED low
}

void checkForUpdates() {
  if (radio.receiveDone()) CheckForWirelessHEX(radio, flash, false);
}

void loop() {
  loop_frd();
}
