/*
 *  Check VL53L0X sensors repeatedly for human presence and direction of movement.
 *  Reads sensor data and streams any changes over an RFM69 radio.
 *  
 *  Copyright 2017 Neil Gupta
 *  All rights reserved.
 *  
 */

#include <Wire.h>
#include <VL53L0X.h>

#include <RFM69_ATC.h>
#include <SPI.h>
#include <LowPower.h>
#include <SPIFlash.h>

/* Pin Connections */
#define xshut1 4
#define xshut2 5

#define SENSOR_LOW 0
#define SENSOR_HIGH 1
#define SENSOR_ERROR 2

#define TIMEOUT_THRESHOLD 7000
#define CONFIDENCE_THRESHOLD 4
#define SERIAL_BAUD    115200

#define NODEID        2   //unique for each node on same network
#define NETWORKID     27  //the same on all nodes that talk to each other
#define GATEWAYID     1
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "smarterisbetters" //exactly the same 16 characters/bytes on all nodes!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -90
#define ACK_TIME      30  // max # of ms to wait for an ack
#define LED     9  // Moteinos have LEDs on D9

RFM69_ATC radio;

#define FLASH_SS      8 // and FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

VL53L0X sensor1;
VL53L0X sensor2;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);

  Serial.begin(SERIAL_BAUD);

  if (flash.initialize()) flash.sleep(); //if Moteino has FLASH-MEM, make sure it sleeps

  Wire.begin();
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY);
  radio.enableAutoPower(ATC_RSSI);

  Serial.println("Initializing Sensor 1...");
  pinMode(xshut1, INPUT);
  delay(150);
  sensor1.init();
  delay(100);
  sensor1.setAddress((uint8_t)22);
  
  Serial.println("Initializing Sensor 2...");
  pinMode(xshut2, INPUT);
  delay(150);
  sensor2.init();
  delay(100);
  sensor2.setAddress((uint8_t)25);

  sensor1.setMeasurementTimingBudget(20000);
  sensor1.startContinuous();
  sensor2.setMeasurementTimingBudget(20000);
  sensor2.startContinuous();

  calibrate();
  radio.sendWithRetry(GATEWAYID, "START", 5);
}

void calibrate() {
  Serial.print("Calibrating for ~10 seconds... ");
  for(int i = 0; i < 500; i++){
    read_sensor1();
    read_sensor2();
    delay(1); // needed to reset watchdog timer
  }
  Serial.println("done.");
}

static const int POWER = 256;
static const int PADDING = 228; // 90% of 256
static const int SLOW_ALPHA = 18; // 7% of 256
static const int FAST_ALPHA = 38; // 15% of 256

static uint16_t sensor1_range = 0;
static uint16_t sensor2_range = 0;

uint8_t read_sensor1() {
  uint16_t range = sensor1.readRangeContinuousMillimeters();
  if (range > TIMEOUT_THRESHOLD) {
    Serial.println("sensor 1 error");
    return SENSOR_ERROR;
  }

  sensor1_range = range;
  static uint16_t avg = range;

  if (range < (PADDING*avg/POWER)) {
    Serial.print("sensor1: ");
    Serial.print(range);
    Serial.print("/");
    Serial.println(avg);
    avg = (SLOW_ALPHA * range + (POWER - SLOW_ALPHA) * avg) / POWER;
    return SENSOR_HIGH;
  } else {
    // learn faster when sensor is not active
    avg = (FAST_ALPHA * range + (POWER - FAST_ALPHA) * avg) / POWER;
    return SENSOR_LOW;
  }
}

uint8_t read_sensor2() {
  uint16_t range = sensor2.readRangeContinuousMillimeters();
  if (range > TIMEOUT_THRESHOLD) {
    Serial.println("sensor 2 error");
    return SENSOR_ERROR;
  }

  sensor2_range = range;
  static uint16_t avg = range;
  
  if (range < (PADDING*avg/POWER)) {
    Serial.print("sensor2: ");
    Serial.print(range);
    Serial.print("/");
    Serial.println(avg);
    avg = (SLOW_ALPHA * range + (POWER - SLOW_ALPHA) * avg) / POWER;
    return SENSOR_HIGH;
  } else {
    // learn faster when sensor is not active
    avg = (FAST_ALPHA * range + (POWER - FAST_ALPHA) * avg) / POWER;
    return SENSOR_LOW;
  }
}

void loop() {
  static uint8_t _start = 0;
  static uint8_t _end = 0;
  static uint8_t confidence = 0;

  uint8_t s1 = read_sensor1();
  uint8_t s2 = read_sensor2();

  if (s1 == SENSOR_ERROR || s2 == SENSOR_ERROR) {
    // at least one of the readings is definitely wrong,
    // let's just skip this cycle and try again.
    radio.sendWithRetry(GATEWAYID, "ERROR", 5);
    return;
  }

  // s1 and s2 are each either 1 or 0, depending on activity, so
  // diff can be either 1 (1 - 0), 0 (1 - 1, 0 - 0), or -1 (0 - 1)
  int diff = s1 - s2;

  if (diff == 0) {
    // either there is no activity or user is in the middle of both
    // sensors, so we know nothing about directional intent anyway.
    if (s1 == SENSOR_LOW && s2 == SENSOR_LOW) {
      // there's no activity
      if (confidence > CONFIDENCE_THRESHOLD) {
        // activity just ended with enough data points.
        // _start and _end can be either 1 or 2 (see below), so
        // dir can be either 1 (2 - 1), 0 (1 - 1, 2 - 2), or -1 (1 - 2).
        // 0 means the user walked in and out on the same side, don't care.
        int dir = _start - _end;
        if (dir == 1) {
          // moved from sensor 2 to sensor 1
          radio.sendWithRetry(GATEWAYID, "2-1", 3);
          Serial.println("---- 2-1 ----");
        } else if (dir == -1) {
          // moved from sensor 1 to sensor 2
          radio.sendWithRetry(GATEWAYID, "1-2", 3);
          Serial.println("---- 1-2 ----");
        }
      }

      if (_start != 0 || _end != 0) {
        Serial.println("all clear!");
        Serial.println();
        Serial.println();
      }
      // reset values
      _start = 0;
      _end = 0;
      confidence = 0;
    } else {
      // we are in the middle of both lasers
      confidence++;
      if (sensor1_range != sensor2_range) {
        // try to guess what direction we're moving
        uint8_t closer_sensor = (sensor1_range < sensor2_range ? 1 : 2);
        if (_start == 0) {
          // somehow we didn't pick up a starting side, let's guess it
          _start = closer_sensor;
          Serial.print("guessing start ");
          Serial.println(_start);
        } else {
          // guess ending direction, in case we don't pick up the ending side
          _end = closer_sensor;
          Serial.print("guessing direction ");
          Serial.println(_end);
        }
      }
    }
    Blink(LED,3);
  } else {
    confidence++;
    // there is activity on one side or the other
    if (_start == 0) {
      // activity is just starting.
      // diff can be either 1 or -1. Positive diff implies
      // sensor 1 is the cause of activity, else sensor 2
      _start = (diff == 1 ? 1 : 2);
    } else {
      // activity is ongoing, track where it might end.
      _end = (diff == 1 ? 1 : 2);
    }
    Blink(LED,3);
    radio.sendWithRetry(GATEWAYID, "READ", 4);
  }

  radio.sleep();
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

