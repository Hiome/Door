#include "Hiome_AVR.h"
#include <Hiome_AMG88xx.h>
#include "thermal/types.h"

#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
//  #define OPTIMIZE_FOR_SERIAL
//  #define TIME_CYCLES
#endif

#define FIRMWARE_VERSION        "V20.4.14"

Hiome_AVR hiome;
Hiome_AMG88xx amg;

#ifdef R3
  #define AMG_ADDR              0x68
#else
  #define AMG_ADDR              0x69
#endif

const uint8_t GRID_EXTENT            = 8;    // size of grid (8x8)
const uint8_t MIN_HISTORY            = 3;    // min number of times a point needs to be seen
const uint8_t MAX_PEOPLE             = 4;    // most people we support in a single frame
const uint8_t MAX_EMPTY_CYCLES       = 2;    // cycles to remember forgotten points
const uint8_t MAX_FORGOTTEN_COUNT    = 2;    // max number of times allowed to forget someone
const uint8_t MAX_DOOR_CHANGE_FRAMES = 5;    // cycles we keep counting after door changes
const uint8_t CONFIDENCE_THRESHOLD   = 10;   // min 10% confidence required
const uint8_t MIN_TEMP               = 2;    // ignore all points colder than 2º C
const uint8_t MAX_TEMP               = 55;   // ignore all points hotter than 55ºC
const float   BACKGROUND_GRADIENT    = 2.0;
const float   FOREGROUND_GRADIENT    = 2.0;
const coord_t UNDEF_POINT            = AMG88xx_PIXEL_ARRAY_SIZE + 10;
const idx_t   UNDEF_INDEX            = UNDEF_POINT;

#include "thermal/coordinates.h"
#include "thermal/door_contact.h"
#include "thermal/csm.h"
#include "thermal/neighbors.h"
#include "thermal/person.h"
#include "thermal/dbscan.h"
#include "thermal/debug.h"

bool processSensor() {
  if (!normalizePixels()) return false;

  // find list of peaks in current frame
  uint8_t total_masses = findCurrentPoints();

  // "I don't know who you are or what you want, but you should know that I have a
  // very particular set of skills that make me a nightmare for people like you.
  // I will find you, I will track you, and I will turn the lights on for you."
  uint8_t taken[MAX_PEOPLE*2] = { 0 };
  idx_t pairs[MAX_PEOPLE*2];

  for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
    pairs[idx] = UNDEF_INDEX;
    #include "thermal/process_person.h"
  }

  for (idx_t i=0; i<total_masses; i++) {
    if (taken[i] > 1) {
      #include "thermal/process_point.h"
    }

    if (taken[i] == 1) {
      #include "thermal/continue_person.h"
    }
    if (taken[i] == 0) {
      #include "thermal/create_person.h"
    }
  }

  // wrap up with debugging output
  #ifdef PRINT_RAW_DATA
    printDebugInfo();
  #endif

  return true;
}

void runThermalLoop() {
  if (processSensor()) {
    // publish event if any people moved through doorway yet
    publishEvents();
    // update avg_pixels
    updateBgAverage();
    // decrement counter on forgotten expirations
    expireForgottenPeople();
    // send heartbeat event if necessary
    // 108000 = 10 (frames/sec) * 60 (sec/min) * 60 (min/hr) * 3 (hrs)
    hiome.beatHeart(108000);
    // increment counter for how long door has been open
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) {
      frames_since_door_open++;
    }
    #ifdef TIME_CYCLES
      SERIAL_PRINT(F("-> "));
      SERIAL_PRINTLN(millis());
    #endif
  }
  hiome.checkForUpdates();
}

void setup() {
  hiome.begin();
  hiome.ledOn();

  amg.begin(AMG_ADDR);

  // setup reed switches
  DDRD  = DDRD  & B11100111;  // set pins 3 and 4 as inputs
  PORTD = PORTD | B00011000;  // pull pins 3 and 4 high

  hiome.sleep(SLEEP_1S);
  hiome.publish(FIRMWARE_VERSION, "0", RETRY_COUNT*2);

  // give sensor 16sec to stabilize
  hiome.sleep(SLEEP_8S);
  hiome.sleep(SLEEP_8S);

  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    known_people[i] = UNDEF_PERSON;
    forgotten_people[i] = UNDEF_PERSON;
  }

  startBgAverage();

  hiome.ledOff();
}

void loop() {
  clearPointsAfterDoorClose();
  runThermalLoop();
}
