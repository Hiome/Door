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
#define PIR    3
#define xshut1 4
#define xshut2 5
#define LED    9
#define BATT   A7

#define TIMEOUT_THRESHOLD 7000
#define CONFIDENCE_THRESHOLD 4
#define SERIAL_BAUD    115200

#define NODEID        2   //unique for each node on same network
#define NETWORKID     27  //the same on all nodes that talk to each other
#define GATEWAYID     1
#define ENCRYPTKEY    "smarterisbetters" //exactly the same 16 characters/bytes on all nodes!
#define ATC_RSSI      -90

RFM69_ATC radio;

#define FLASH_SS      8 // and FLASH SS on D8 on regular Moteinos (D23 on MoteinoMEGA)
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

VL53L0X sensor1;
VL53L0X sensor2;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), motion, CHANGE);

  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);

  Serial.begin(SERIAL_BAUD);

  if (flash.initialize()) {
    Serial.println("Initialized flash...");
    flash.sleep();
  }

  Wire.begin();
  radio.initialize(RF69_915MHZ, NODEID, NETWORKID);
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
  sensor2.setMeasurementTimingBudget(20000);

  calibrate();
  publish("ON");

  digitalWrite(LED, LOW);
}

volatile boolean motionEnded = true;

void motion() {
  if (digitalRead(PIR) == HIGH) {
    motionEnded = false;
  } else {
    motionEnded = true;
  }
}

static const int POWER = 256;
static const int PADDING = 228; // 90% of 256
static const int ALPHA = 38; // 15% of 256

static uint16_t avg1 = 0;
static uint16_t avg2 = 0;
static uint16_t sensor1_range = 0;
static uint16_t sensor2_range = 0;

void calibrate() {
  Serial.print("Calibrating for ~30 seconds... ");
  uint16_t range = 0;
  for(int i = 0; i < 750; i++){
    range = sensor1.readRangeSingleMillimeters();
    if (range < TIMEOUT_THRESHOLD) {
      avg1 = (avg1 == 0 ? range : (ALPHA * range + (POWER - ALPHA) * avg1) / POWER);
    }

    range = sensor2.readRangeSingleMillimeters();
    if (range < TIMEOUT_THRESHOLD) {
      avg2 = (avg2 == 0 ? range : (ALPHA * range + (POWER - ALPHA) * avg2) / POWER);
    }

    delay(1); // needed to reset watchdog timer
  }

  Serial.println("done.");
}

#define SENSOR_LOW   0
#define SENSOR_HIGH  1
#define SENSOR_ERROR 2

uint8_t read_sensor1() {
  uint16_t range = sensor1.readRangeSingleMillimeters();
  if (range > TIMEOUT_THRESHOLD) {
    Serial.println("sensor 1 error");
    return SENSOR_ERROR;
  }

  sensor1_range = range;

  if (range < (PADDING*avg1/POWER)) {
    Serial.print("sensor1: ");
    Serial.print(range);
    Serial.print("/");
    Serial.println(avg1);
    return SENSOR_HIGH;
  } else {
    return SENSOR_LOW;
  }
}

uint8_t read_sensor2() {
  uint16_t range = sensor2.readRangeSingleMillimeters();
  if (range > TIMEOUT_THRESHOLD) {
    Serial.println("sensor 2 error");
    return SENSOR_ERROR;
  }

  sensor2_range = range;
  
  if (range < (PADDING*avg2/POWER)) {
    Serial.print("sensor2: ");
    Serial.print(range);
    Serial.print("/");
    Serial.println(avg2);
    return SENSOR_HIGH;
  } else {
    return SENSOR_LOW;
  }
}

static uint8_t _start = 0;
static uint8_t _end = 0;
static uint8_t confidence = 0;

void run_sensor() {
  uint8_t s1 = read_sensor1();
  uint8_t s2 = read_sensor2();

  if (s1 == SENSOR_ERROR || s2 == SENSOR_ERROR) {
    // at least one of the readings is definitely wrong,
    // let's just skip this cycle and try again.
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
          publish("2-1");
        } else if (dir == -1) {
          // moved from sensor 1 to sensor 2
          publish("1-2");
        }
      }

      reset_sensor();
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
  }
}

void reset_sensor(){
  if (_start != 0 || _end != 0) {
    Serial.println("all clear!");
    Serial.println();
    Serial.println();
  }
  // reset values
  _start = 0;
  _end = 0;
  confidence = 0;
}

char sendBuf[10];
char BATstr[5];

void publish(char* msg) {
  checkBattery();
  sprintf(sendBuf, "%s;%sv", msg, BATstr);
  radio.sendWithRetry(GATEWAYID, sendBuf, strlen(sendBuf));
  radio.sleep();
}

void checkBattery() {
  float batteryVolts = analogRead(BATT) * 0.00644;
  dtostrf(batteryVolts, 3,2, BATstr);
}

void loop() {
  if (motionEnded) {
    reset_sensor();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  }

  run_sensor();
}

