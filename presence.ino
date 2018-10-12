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
#define LED    9
#define BATT   A7

#include <RFM69_ATC.h>
#include <LowPower.h>

#ifdef DEBUGGER
  #define Sprintln(a) (Serial.println(a))
  #define Sprint(a) (Serial.print(a))
  #define Sflush() (Serial.flush())
#else
  #define Sprintln(a)
  #define Sprint(a)
  #define Sflush()
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

float batteryLevel() {
  return analogRead(BATT) * 0.00644;
}

char sendBuf[15];
uint8_t packetCount = 1;
void publish(char* msg) {
  int batt = analogRead(BATT);
  uint8_t len = sprintf(sendBuf, "%s;%d%d", msg, batt, packetCount);
  radio.sendWithRetry(GATEWAYID, sendBuf, len, 5);
  radio.sleep();
  if (packetCount < 9)
    packetCount++;
  else
    packetCount = 1;

  Sprint("published ");
  Sprint(sendBuf);
  Sprintln("\n\n");
  Sflush();
}

#if defined LIDAR
#include "lidar.h"
#elif defined MOTION
#include "motion.h"
#elif defined DOOR
#include "door.h"
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

  publish(FIRMWARE_VERSION);
  digitalWrite(LED, LOW);
}

