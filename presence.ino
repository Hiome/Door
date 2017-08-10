/*
 *  Check VL53L0X sensors repeatedly for human presence and direction of movement.
 *  Reads sensor data and streams any changes to an MQTT broker.
 *  
 *  Copyright 2017 Neil Gupta
 *  All rights reserved.
 *  
 */

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

/* Pin Connections */
#define xshut1 14 // D6
#define xshut2 12 // D5

#define SENSOR_LOW 0
#define SENSOR_HIGH 1
#define SENSOR_ERROR 2

#define TIMEOUT_THRESHOLD 7000
#define CONFIDENCE_THRESHOLD 4

WiFiClient esp_client;
PubSubClient mqtt_client(esp_client);

VL53L0X sensor1;
VL53L0X sensor2;

/* Get MAC address as a char array */
static const char* mac_address() {
  static char macStr[18] = { 0 };
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return macStr;
}

static const char* client_name = mac_address();

/* Generate MQTT topic with the MAC address.
 *  Topic name will always be "smarter/presence/data/MACADDRESS"
 */
static const char* mqtt_topic() {
  static char topic[40];
  sprintf(topic, "smarter/presence/data/%s", client_name);
  return topic;
}

static const char* data_topic = mqtt_topic();

void setup() {
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);

  Serial.begin (9600);

  setup_wifi();
  mqtt_client.setServer(mqtt_host, mqtt_port);

  Wire.begin(4, 5);

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
}

/* Connect to wifi */
void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.print(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(" . ");
    delay(500);
  }

  Serial.print(WiFi.localIP());
  Serial.println(" connected.");
}

/* Connect to MQTT broker */
void setup_mqtt() {
  // if wifi disconnected, reconnect
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }

  static char lifecycle_topic[45];
  sprintf(lifecycle_topic, "smarter/presence/lifecycle/%s", client_name);

  Serial.print("Connecting to ");
  Serial.print(mqtt_host);

  // loop until reconnected
  while (!mqtt_client.connected()) {
    Serial.print(" . ");
    if (mqtt_client.connect(client_name, lifecycle_topic, 0, true, "disconnected")) {
      Serial.print(client_name);
      Serial.println(" connected.");
      mqtt_client.publish(lifecycle_topic, "connected", true);
    } else {
      delay(1000);
    }
  }
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
static const int PADDING = 28; // 10% of 256
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

  if (range < ((POWER - PADDING)*avg/POWER)) {
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
  
  if (range < ((POWER - PADDING)*avg/POWER)) {
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
  if (!mqtt_client.loop()) {
    setup_mqtt();
  }

  static uint8_t _start = 0;
  static uint8_t _end = 0;
  static uint8_t confidence = 0;

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
          mqtt_client.publish(data_topic, "2-1");
          Serial.println("---- 2-1 ----");
        } else if (dir == -1) {
          // moved from sensor 1 to sensor 2
          mqtt_client.publish(data_topic, "1-2");
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

