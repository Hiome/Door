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

/* Pin Connections */
#define PIR    3
#define xshut1 6
#define xshut2 7
#define FLASH  8
#define LED    9
#define BATT   A7

#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>
#include <SPI.h>
#include <LowPower.h>

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

RFM69_ATC radio;
SPIFlash flash(FLASH, 0xEF30); //EF30 for windbond 4mbit flash

void blink(uint8_t times) {
  while (times > 0) {
    digitalWrite(LED, LOW);
    LOWPOWER_DELAY(SLEEP_120MS);
    digitalWrite(LED, HIGH);
    LOWPOWER_DELAY(SLEEP_250MS);
    digitalWrite(LED, LOW);
    LOWPOWER_DELAY(SLEEP_120MS);
    times--;
  }
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

#ifdef BATTERY_POWERED
  #define BATTERY_LEVEL ( analogRead(BATT) )
#else
  #define BATTERY_LEVEL ( 0 )
#endif

uint8_t packetCount = 1;
bool publish(char* msg, int8_t retries) {
  if (retries == -1) {
    if (!radio.canSend()) {
      SERIAL_PRINT("skipped ");
      SERIAL_PRINT(msg);
      SERIAL_PRINTLN("\n\n");
      SERIAL_FLUSH;
      return false;
    }

    char sendBuf[15];
    uint8_t len = sprintf(sendBuf, "%s;%d0", msg, BATTERY_LEVEL);
    radio.send(GATEWAYID, sendBuf, len, false);

    SERIAL_PRINT("published ");
    SERIAL_PRINT(sendBuf);
    SERIAL_PRINTLN("\n\n");
    SERIAL_FLUSH;
    return true;
  } else {
    char sendBuf[15];
    uint8_t len = sprintf(sendBuf, "%s;%d%d", msg, BATTERY_LEVEL, packetCount);
    bool success = radio.sendWithRetry(GATEWAYID, sendBuf, len, retries);

    if (success || retries > 0) {
      if (packetCount < 9)
        packetCount++;
      else
        packetCount = 1;
    }

    SERIAL_PRINT("published ");
    SERIAL_PRINT(sendBuf);
    SERIAL_PRINTLN("\n\n");
    SERIAL_FLUSH;

    return success;
  }
}

void setup() {
  SERIAL_START;

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  radio.initialize(RF69_915MHZ, NODEID, NETWORKID);
  radio.encrypt(ENCRYPTKEY);
  radio.enableAutoPower(ATC_RSSI);

  if (flash.initialize()) flash.sleep();

  initialize();

  digitalWrite(LED, LOW);
}

void loop() {
  if (radio.receiveDone()) CheckForWirelessHEX(radio, flash, false);
  loop_frd();
}
