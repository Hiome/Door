/*
 *  Check VL53L0X sensors repeatedly for human presence and direction of movement.
 *  Reads sensor data and streams any changes over an RFM69 radio.
 *  
 *  Copyright 2018 Neil Gupta
 *  All rights reserved.
 *  
 */

//#define ENABLE_SERIAL

#include "config.h"
#define NETWORKID     27  // the same on all nodes that talk to each other
#define GATEWAYID     1
#define ENCRYPTKEY    "smarterisbetters" // exactly the same 16 characters/bytes on all nodes!
#define ATC_RSSI      -75
#define SERIAL_BAUD   115200
#define RETRY_TIME    60
#define RETRY_COUNT   20

#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>
#include <LowPower.h>

RFM69_ATC radio;
SPIFlash flash(8, 0xEF30); //EF30 for windbond 4mbit flash

uint32_t heartbeats = 0;
void beatHeart() {
  heartbeats++;
  // 108000 = 10 (frames/sec) * 60 (sec/min) * 60 (min/hr) * 3 (hrs)
  if (heartbeats > 108000) publish("h", "0", 0);
}

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

uint8_t packetCount = 1;
uint8_t publish(const char* msg, const char* meta, uint8_t retries) {
  char sendBuf[58];
  int8_t len = sprintf(sendBuf, "%s;%s%u", msg, meta, packetCount);
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

  initialize();

  PORTB = PORTB & B11111101;  // pull LED low
}

void checkForUpdates() {
  if (radio.receiveDone()) CheckForWirelessHEX(radio, flash, false);
}

void loop() {
  loop_frd();
}
