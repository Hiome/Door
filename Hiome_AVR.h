#ifndef LIB_HIOME_AVR_H
#define LIB_HIOME_AVR_H

#define HIOME_GATEWAYID           1
#define HIOME_ATC_RSSI            -60
#define HIOME_RETRY_TIME          50
#define HIOME_RETRY_COUNT         5
#define HIOME_BATT_PIN            A3
#define HIOME_MAX_VOLTAGE_DRIFT   50
#define HIOME_MIN_ALLOWED_VOLTAGE 250
#define HIOME_MAX_ALLOWED_VOLTAGE 600
#define HIOME_MAX_MESSAGE_LENGTH  54

#include <RFM69_ATC.h>
#include <RFM69_OTA.h>
#include <SPIFlash.h>
#include <LowPower.h>

class Hiome_AVR {
  public:
    Hiome_AVR() : flash(8, 0xEF30) {}; //EF30 for windbond 4mbit flash

    void      begin(uint8_t nodeid, uint8_t networkid, const char* encryptkey, bool highPowerMode = true);
    void      setLED(bool on = true);
    void      wait(period_t d);
    void      checkForUpdates();
    void      beatHeart(uint32_t maxBeats);
    uint8_t   publish(const char* msg, uint8_t retries = HIOME_RETRY_COUNT, bool debug = false);

  private:
    uint8_t   packetCount = 1;
    uint32_t  heartbeats = 0;
    bool      battConnected = true;

    RFM69_ATC radio;
    SPIFlash  flash;

    uint16_t  checkBattery();
    void      checkIfBatteryConnected();
};

#endif
