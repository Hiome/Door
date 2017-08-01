/*
 *  Check ultrasonic and PIR sensors repeatedly for human presence.
 *  Reads sensor data and streams any changes to an MQTT broker.
 *  
 *  Copyright 2017 Neil Gupta
 *  All rights reserved.
 *  
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

/* Pin Connections */
#define TRIG 13  //D7
#define ECHO 15  //D8
#define PIR 14   //D5
#define BLUE_LED 2

/* Constants */
#define SENSOR_TIMEOUT 50000
#define SENSOR_ERROR 0x2
#define SENSOR_DELAY 10

WiFiClient esp_client;
PubSubClient mqtt_client(esp_client);

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
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(PIR, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  setup_wifi();
  mqtt_client.setServer(mqtt_host, mqtt_port);

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
  Serial.print("Calibrating for ~30 seconds... ");
  // give the sensors ~30 seconds to calibrate
  for(int i = 0; i < 600; i++){
    read_ultrasonic_sensor();
    delay(50);
  }
  Serial.println("done.");
}

uint8_t read_ultrasonic_sensor() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);

  if (duration > SENSOR_TIMEOUT) {
    // unreasonable response (more than 3 meters away),
    // ignore it so we don't mess up our average.
    return SENSOR_ERROR;
  }

  static long avg = duration;

  static const int POWER = 256;
  static const int PADDING = 8; // 3% of 256
  static const int SLOW_ALPHA = 26; // 10% of 256
  static const int FAST_ALPHA = 78; // 30% of 256

  if (duration < ((POWER - PADDING)*avg/POWER)) {
    // activity detected: duration is within 90% of our average,
    // weight it 10% in our running average and return HIGH
    avg = (SLOW_ALPHA * duration + (POWER - SLOW_ALPHA) * avg) / POWER;
    return HIGH;
  } else {
    // no activity detected: duration is more than 90% of our average,
    // weight it 30% in our running average and return LOW
    avg = (FAST_ALPHA * duration + (POWER - FAST_ALPHA) * avg) / POWER;
    return LOW;
  }
}

void loop() {
  if (!mqtt_client.loop()) {
    setup_mqtt();
  }

  static boolean was_active = false;
  uint8_t sonar = read_ultrasonic_sensor();
  int pir_val = digitalRead(PIR);
  if (pir_val == HIGH) {
    Serial.print("PIR: ");
    Serial.println(pir_val);
    Serial.print("Sonar: ");
    Serial.println(sonar);
  }

  if (sonar == HIGH) {
    if (!was_active) {
      if (pir_val == HIGH) {
        mqtt_client.publish(data_topic, "exited", true);
        digitalWrite(BUILTIN_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);
      } else {
        mqtt_client.publish(data_topic, "entered", true);
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(BUILTIN_LED, HIGH);
      }
    }
    was_active = true;
  } else if (sonar == LOW) {
    was_active = false;
    digitalWrite(BUILTIN_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);
  }

  // give the sound waves some time to dissipate
  delay(SENSOR_DELAY);
}

