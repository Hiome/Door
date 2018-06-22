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
#define PIR0   3
#define PIR1   4
#define PIR2   5
#define xshut1 6
#define xshut2 7
#define LED    9
#define LUX    A6
#define BATT   A7

//#if defined(LIDAR) || defined(MODAR)
//#define NEEDFORSPEED
//#define EI_NOTEXTERNAL
//#define EI_NOTPORTB
//#define EI_NOTPORTC
//#define INTERRUPT_FLAG_PIN4 motion1_triggered
//#define INTERRUPT_FLAG_PIN5 motion2_triggered
//#include <EnableInterrupt.h>
//#endif

#include <RFM69_ATC.h>
#include <LowPower.h>

#if DEBUGGER
  #define Sprintln(a) (Serial.println(a))
  #define Sprint(a) (Serial.print(a))
#else
  #define Sprintln(a)
  #define Sprint(a)
#endif

RFM69_ATC radio;

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
uint8_t packetCount = 1;
void publish(char* msg) {
  checkBattery();
  uint8_t len = sprintf(sendBuf, "%s;%s%d", msg, BATstr, packetCount);
  radio.sendWithRetry(GATEWAYID, sendBuf, len, 5);
  radio.sleep();
  if (packetCount < 9)
    packetCount++;
  else
    packetCount = 1;
  #if DEBUGGER
    Sprint("published ");
    Sprint(sendBuf);
    Sprintln("\n\n");
    Serial.flush();
  #endif
}

#if defined LIDAR
#include "lidar.h"
#elif defined MODAR
#include "modar.h"
#elif defined ROOM
#include "room.h"
#else
#error Missing node type
#endif

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  radio.initialize(RF69_915MHZ, NODEID, NETWORKID);
  radio.encrypt(ENCRYPTKEY);
  radio.enableAutoPower(ATC_RSSI);

  initialize();

  publish("A");
  digitalWrite(LED, LOW);
}

