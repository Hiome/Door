#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
//  #define OPTIMIZE_FOR_SERIAL
//  #define TIME_CYCLES
#endif

#define FIRMWARE_VERSION        "V20.4.5"
#define YAXIS                        // axis along which we expect points to move (x or y)

#include "thermal/types.h"
#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;

#ifdef R3
  #define AMG_ADDR              0x68
#else
  #define AMG_ADDR              0x69
#endif

const uint8_t GRID_EXTENT            = 8;    // size of grid (8x8)
const uint8_t MIN_HISTORY            = 3;    // min number of times a point needs to be seen
const uint8_t MAX_PEOPLE             = 5;    // most people we support in a single frame
const uint8_t MAX_EMPTY_CYCLES       = 2;    // cycles to remember forgotten points
const uint8_t MAX_FORGOTTEN_COUNT    = 2;    // max number of times allowed to forget someone
const uint8_t MAX_DOOR_CHANGE_FRAMES = 5;    // cycles we keep counting after door changes
const uint8_t CONFIDENCE_THRESHOLD   = 5;    // min 5% confidence required
const uint8_t MIN_TEMP               = 2;    // ignore all points colder than 2º C
const uint8_t MAX_TEMP               = 45;   // ignore all points hotter than 45ºC
const float   BACKGROUND_GRADIENT    = 2.0;
const float   FOREGROUND_GRADIENT    = 2.0;
const coord_t UNDEF_POINT            = AMG88xx_PIXEL_ARRAY_SIZE + 10;
const idx_t   UNDEF_INDEX            = UNDEF_POINT;

float   raw_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
fint3_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint8_t side1Point = 0;
uint8_t side2Point = 0;
float global_bgm = 0;
float global_fgm = 0;
float cavg1 = 0;
float cavg2 = 0;
uint8_t cycles_since_forgotten = MAX_EMPTY_CYCLES;

#include "thermal/coordinates.h"
#include "thermal/door_contact.h"
#include "thermal/csm.h"
#include "thermal/neighbors.h"
#include "thermal/person.h"
#include "thermal/dbscan.h"

bool processSensor() {
  if (!normalizePixels()) return false;

  // find list of peaks in current frame
  uint8_t total_masses = findCurrentPoints();

  // "I don't know who you are or what you want, but you should know that I have a
  // very particular set of skills that make me a nightmare for people like you.
  // I will find you, I will track you, and I will turn the lights on for you."
  uint8_t taken[MAX_PEOPLE];
  idx_t pairs[MAX_PEOPLE];

  // "Good luck."
  Person temp_forgotten_people[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;

  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    taken[i] = 0;
    pairs[i] = UNDEF_INDEX;
    temp_forgotten_people[i] = UNDEF_PERSON;
  }

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT (forget_person(idx, temp_forgotten_people, pairs, temp_forgotten_num))

  for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
    #include "thermal/process_person.h"
  }

  for (idx_t i=0; i<total_masses; i++) {
    if (taken[i] > 1) {
      #include "thermal/process_point.h"
    }

    if (taken[i] == 1) {
      #include "thermal/wed_point.h"
    } else if (taken[i] == 0) {
      #include "thermal/create_point.h"
    }
  }

  // copy forgotten data points for this frame to global scope

  if (temp_forgotten_num > 0) {
    for (idx_t i=0; i<MAX_PEOPLE; i++) {
      forgotten_people[i].publishMaybeEvent();
      forgotten_people[i] = temp_forgotten_people[i];
    }
    cycles_since_forgotten = 0;
    SERIAL_PRINTLN(F("s"));
  } else if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
    cycles_since_forgotten++;
    if (cycles_since_forgotten == MAX_EMPTY_CYCLES) {
      // clear forgotten points list
      for (idx_t i=0; i<MAX_PEOPLE; i++) {
        forgotten_people[i].publishMaybeEvent();
        forgotten_people[i] = UNDEF_PERSON;
      }
      SERIAL_PRINTLN(F("f"));
    }
  }

  // wrap up with debugging output
  #include "thermal/debug.h"

  return true;
}

void clearSideXPoints() {
  if (side1Point || side2Point) {
    bool conjoinedBlobExists = false;
    for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
      Person p = known_people[idx];
      if (p.real() && p.neighbors >= 3 && p.confidence > 50) {
        conjoinedBlobExists = true;
        break;
      }
    }
    if (!conjoinedBlobExists) {
      side1Point = 0;
      side2Point = 0;
      SERIAL_PRINTLN(F("csp"));
    }
  }
}

void runThermalLoop() {
  if (processSensor()) {
    // publish event if any people moved through doorway yet
    publishEvents();
    // update avg_pixels
    updateBgAverage();
    // clear sideXPoints used to track which side a merged from (naming things is hard, ok)
    clearSideXPoints();
    // send heartbeat event if necessary
    // 108000 = 10 (frames/sec) * 60 (sec/min) * 60 (min/hr) * 3 (hrs)
    beatHeart(108000);
    // increment counter for how long door has been open
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) {
      frames_since_door_open++;
    }
    #ifdef TIME_CYCLES
      SERIAL_PRINTLN(millis());
    #endif
  }
  checkForUpdates();
}

void initialize() {
  amg.begin(AMG_ADDR);

  // setup reed switches
  DDRD  = DDRD  & B11100111;  // set pins 3 and 4 as inputs
  PORTD = PORTD | B00011000;  // pull pins 3 and 4 high

  LOWPOWER_DELAY(SLEEP_1S);
  publish(FIRMWARE_VERSION, "0", RETRY_COUNT*2);

  // check right on boot just in case this is a recovery attempt for a bricked sensor
  checkForUpdates();

  // give sensor 16sec to stabilize
  LOWPOWER_DELAY(SLEEP_8S);
  LOWPOWER_DELAY(SLEEP_8S);

  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    known_people[i] = UNDEF_PERSON;
    forgotten_people[i] = UNDEF_PERSON;
  }

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    raw_pixels[i] = 0.0;
  }

  amg.readPixels(raw_pixels);

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] = floatToFint3(constrain(raw_pixels[i], MIN_TEMP, MAX_TEMP));
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!amg.readPixels(raw_pixels)) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (((uint8_t)raw_pixels[i]) <= MIN_TEMP || ((uint8_t)raw_pixels[i]) >= MAX_TEMP) {
        continue;
      }
      float std = raw_pixels[i] - bgPixel(i);
      // alpha of 0.3
      avg_pixels[i] = ((int32_t)avg_pixels[i]) + ((int32_t)(300.0 * std));
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  runThermalLoop();
}
