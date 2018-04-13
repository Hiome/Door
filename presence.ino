/*
 *  Check VL53L0X sensors repeatedly for human presence and direction of movement.
 *  Reads sensor data and streams any changes over an RFM69 radio.
 *  
 *  Copyright 2018 Neil Gupta
 *  All rights reserved.
 *  
 */

#include "config.h"
#define ENCRYPTKEY    "smarterisbetters" // exactly the same 16 characters/bytes on all nodes!
#define ATC_RSSI      -90
#define SERIAL_BAUD   115200

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <RFM69_ATC.h>
#include <SPI.h>
#include <LowPower.h>
#include <SPIFlash.h>

#if DEBUGGER
  #define Sprintln(a) (Serial.println(a))
  #define Sprint(a) (Serial.print(a))
#else
  #define Sprintln(a)
  #define Sprint(a)
#endif

/* Pin Connections */
#define PIR0   3
#define PIR1   4
#define PIR2   5
#define xshut1 6
#define xshut2 7
#define LED    9
#define LUX    A6
#define BATT   A7

RFM69_ATC radio;

#define FLASH_SS      8 // FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

void blink(uint8_t times) {
  while (times > 0) {
    digitalWrite(LED, LOW);
    delay(125);
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(125);
    times--;
  }
}

char BATstr[5];
void checkBattery() {
  float batteryVolts = analogRead(BATT) * 0.00644;
  dtostrf(batteryVolts, 3,2, BATstr);
}

char sendBuf[12];
void publish(char* msg) {
  checkBattery();
  uint8_t len = sprintf(sendBuf, "%s;%sv", msg, BATstr);
  radio.sendWithRetry(GATEWAYID, sendBuf, len);
  radio.sleep();
}

#include NODETYPE

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  if (flash.initialize()) {
    Sprintln("Initialized flash...");
    flash.sleep();
  }

  radio.initialize(RF69_915MHZ, NODEID, NETWORKID);
  radio.encrypt(ENCRYPTKEY);
  radio.enableAutoPower(ATC_RSSI);

  initialize();

  publish("ON");
  digitalWrite(LED, LOW);
}

