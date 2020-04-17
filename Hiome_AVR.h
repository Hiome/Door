#ifndef LIB_HIOME_AVR_H
#define LIB_HIOME_AVR_H

#define ATC_RSSI      -75
#define RETRY_TIME    100
#define RETRY_COUNT   5
#define BATT          A3
#define MAX_VOLTAGE_DRIFT   50
#define MIN_ALLOWED_VOLTAGE 200
#define MAX_ALLOWED_VOLTAGE 700

#include "auth.h"

#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>
#include <LowPower.h>

class Hiome_AVR {
  public:
    Hiome_AVR() : flash(8, 0xEF30) {}; //EF30 for windbond 4mbit flash

    void      begin(uint8_t nodeid, bool highPowerMode = true);
    void      ledOn();
    void      ledOff();
    void      sleep(period_t d);
    void      checkForUpdates();
    uint16_t  checkBattery();
    void      beatHeart(uint32_t maxBeats);
    uint8_t   publish(const char* msg, const char* meta, uint8_t retries = RETRY_COUNT, bool debug = false);

  private:
    uint8_t   packetCount = 1;
    uint32_t  heartbeats = 0;
    bool      battConnected = true;

    RFM69_ATC radio;
    SPIFlash  flash;

    void      checkIfBatteryConnected();
};

#endif
