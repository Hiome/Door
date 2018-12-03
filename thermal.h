//#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.1"
#define GRID_EXTENT             8    // size of grid (8x8)
#define TEMP_BUFFER             2    // min increase in temp needed to register person
#define MIN_DISTANCE            3    // min distance for 2 peaks to be separate people
#define MIN_HISTORY             5    // min number of times a point needs to be seen
#define MAX_PEOPLE              5    // most people we support in a single frame
#define MAX_CALIBRATION_CYCLES  8000 // each cycle is roughly 16ms, 8000 cycles ~= 2min
#define AXIS                    x    // axis along which we expect points to move (x or y)
#define LOWER_BOUND             0
#define UPPER_BOUND             GRID_EXTENT

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float calibrated_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

uint8_t past_points[MAX_PEOPLE];
uint8_t archived_past_points[MAX_PEOPLE];
uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
uint8_t forgotten_count = 0;

uint16_t calibration_cycles = 0;

#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define PIXEL_ACTIVE(i) ( pixels[(i)] > (calibrated_pixels[(i)] + TEMP_BUFFER) )

// store in-memory so we don't have to do math every time
const uint8_t xcoordinates[64] PROGMEM = {
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8
};
const uint8_t ycoordinates[64] PROGMEM = {
  1,  1,  1,  1,  1,  1,  1,  1,
  2,  2,  2,  2,  2,  2,  2,  2,
  3,  3,  3,  3,  3,  3,  3,  3,
  4,  4,  4,  4,  4,  4,  4,  4,
  5,  5,  5,  5,  5,  5,  5,  5,
  6,  6,  6,  6,  6,  6,  6,  6,
  7,  7,  7,  7,  7,  7,  7,  7,
  8,  8,  8,  8,  8,  8,  8,  8
};

#define x(p) ( pgm_read_byte_near(xcoordinates + p) )
#define y(p) ( pgm_read_byte_near(ycoordinates + p) )

// calculate manhattan distance between 2 indices
uint8_t distance(uint8_t p1, uint8_t p2) {
  int8_t yd = y(p2) - y(p1);
  int8_t xd = x(p2) - x(p1);
  return abs(yd) + abs(xd);
}

// used only by qsort in debugging section (compiled out otherwise)
int sort_asc(const void *cmp1, const void *cmp2) {
  uint8_t a = *((uint8_t *)cmp1);
  uint8_t b = *((uint8_t *)cmp2);
  return a - b;
}

// This macro sorts array indexes based on their corresponding values.
// For example, given an array {4, 2, 0}, SORT_ARRAY(a, (a[i] > 0), 3)
// will return 2, ordered_indexes will be {1, 0}, and ordered_values
// will be {2, 4}. The third element does not pass the cmp condition.
#define SORT_ARRAY(src, cmp, sze) ({                                 \
  uint8_t count = 0;                                                 \
  for (uint8_t i=0; i<(sze); i++) {                                  \
    if ((cmp)) {                                                     \
      bool added = false;                                            \
      for (uint8_t j=0; j<count; j++) {                              \
        if ((src)[i] > ordered_values[j]) {                          \
          for (uint8_t x=count; x>j; x--) {                          \
            ordered_values[x] = ordered_values[x-1];                 \
            ordered_indexes[x] = ordered_indexes[x-1];               \
          }                                                          \
          ordered_values[j] = (src)[i];                              \
          ordered_indexes[j] = i;                                    \
          added = true;                                              \
          break;                                                     \
        }                                                            \
      }                                                              \
      if (!added) {                                                  \
        ordered_values[count] = (src)[i];                            \
        ordered_indexes[count] = i;                                  \
      }                                                              \
      count++;                                                       \
    }                                                                \
  }                                                                  \
  return count;                                                      \
})
//////////////////////////////////////////////////////////////////////

uint8_t sortPixelsByTemp(uint8_t *ordered_indexes) {
  float ordered_values[AMG88xx_PIXEL_ARRAY_SIZE];
  SORT_ARRAY(pixels, (PIXEL_ACTIVE(i)), AMG88xx_PIXEL_ARRAY_SIZE);
}

uint8_t sortPointsByHistory(uint8_t *ordered_indexes) {
  uint16_t ordered_values[MAX_PEOPLE];
  SORT_ARRAY(histories, (histories[i] > 0), MAX_PEOPLE);
}

bool readPixels() {
  float past_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(past_pixels, pixels, AMG88xx_PIXEL_ARRAY_SIZE);

  amg.readPixels(pixels);

  // return true if pixels changed
  return (memcmp(past_pixels, pixels, AMG88xx_PIXEL_ARRAY_SIZE) != 0);
}

void processSensor() {
  if (!readPixels()) return;

  // sort pixels by temperature to find peaks

  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = sortPixelsByTemp(ordered_indexes);

  // determine which points are people by comparing peaks with neighboring cells

  #define TL_IDX          ( idx - (GRID_EXTENT+1) )
  #define TM_IDX          ( idx - GRID_EXTENT )
  #define TR_IDX          ( idx - (GRID_EXTENT-1) )
  #define ML_IDX          ( idx - 1 )
  #define MR_IDX          ( idx + 1 )
  #define BL_IDX          ( idx + (GRID_EXTENT-1) )
  #define BM_IDX          ( idx + GRID_EXTENT )
  #define BR_IDX          ( idx + (GRID_EXTENT+1) )
  #define NOT_LEFT_EDGE   ( x(idx) > 1 )
  #define NOT_RIGHT_EDGE  ( x(idx) < GRID_EXTENT )
  #define NOT_TOP_EDGE    ( idx >= GRID_EXTENT )
  #define NOT_BOTTOM_EDGE ( idx < (AMG88xx_PIXEL_ARRAY_SIZE - GRID_EXTENT) )
  #define MIDDLE_LEFT     ( PIXEL_ACTIVE(ML_IDX) )
  #define MIDDLE_RIGHT    ( PIXEL_ACTIVE(MR_IDX) )
  #define TOP_LEFT        ( NOT_TOP_EDGE && PIXEL_ACTIVE(TL_IDX) )
  #define TOP_MIDDLE      ( NOT_TOP_EDGE && PIXEL_ACTIVE(TM_IDX) )
  #define TOP_RIGHT       ( NOT_TOP_EDGE && PIXEL_ACTIVE(TR_IDX) )
  #define BOTTOM_LEFT     ( NOT_BOTTOM_EDGE && PIXEL_ACTIVE(BL_IDX) )
  #define BOTTOM_MIDDLE   ( NOT_BOTTOM_EDGE && PIXEL_ACTIVE(BM_IDX) )
  #define BOTTOM_RIGHT    ( NOT_BOTTOM_EDGE && PIXEL_ACTIVE(BR_IDX) )

  uint8_t total_masses = 0;
  uint8_t points[MAX_PEOPLE];
  for (uint8_t i=0; i<active_pixel_count; i++) {
    uint8_t idx = ordered_indexes[i];
    bool distinct = true;
    for (uint8_t j=0; j<i; j++) {
      // make sure this point isn't close to another peak we already considered
      if (distance(ordered_indexes[j], idx) < MIN_DISTANCE) {
        distinct = false;
        break;
      }
    }
    if (distinct && (
      // has at least one neighboring pixel (ignore stray temp readings)
      (NOT_LEFT_EDGE && (MIDDLE_LEFT || TOP_LEFT || BOTTOM_LEFT)) ||
      (NOT_RIGHT_EDGE && (MIDDLE_RIGHT || TOP_RIGHT || BOTTOM_RIGHT)) ||
      TOP_MIDDLE || BOTTOM_MIDDLE)) {
      points[total_masses] = idx;
      total_masses++;
      if (total_masses == MAX_PEOPLE) break;  // we've hit our cap, stop looking
    }
  }

  // sort list of previously seen people by how many frames we've seen them to prioritize
  // finding people who have been in frame longest (reduces impact of noisy data)

  uint8_t ordered_past_points[MAX_PEOPLE];
  uint8_t past_total_masses = sortPointsByHistory(ordered_past_points);

  // pair previously seen points with new points to determine where people moved

  bool taken[total_masses];
  memset(taken, false, total_masses);

  uint8_t temp_forgotten_points[MAX_PEOPLE];
  memset(temp_forgotten_points, UNDEF_POINT, MAX_PEOPLE);
  uint8_t temp_forgotten_starting_points[MAX_PEOPLE];
  uint16_t temp_forgotten_histories[MAX_PEOPLE];
  uint8_t temp_forgotten_count = 0;

  for (uint8_t i=0; i<past_total_masses; i++) {
    uint8_t idx = ordered_past_points[i];
    uint8_t min_distance = (MIN_DISTANCE + 2);
    uint8_t min_index = UNDEF_POINT;
    for (uint8_t j=0; j<total_masses; j++) {
      if (!taken[j]) {
        // match with closest point
        uint8_t d = distance(past_points[idx], points[j]);
        if (d < min_distance) {
          min_distance = d;
          min_index = j;
        }
      }
    }
    if (min_index != UNDEF_POINT && total_masses < past_total_masses &&
        ((AXIS(past_points[idx]) - LOWER_BOUND) <= min_distance ||
         (UPPER_BOUND - AXIS(past_points[idx])) <= min_distance)) {
      // a point disappeared in this frame, looks like it could be this one
      min_index = UNDEF_POINT;
    }
    if (min_index == UNDEF_POINT) {
      // point disappeared (no new point found), stop tracking it
      if (AXIS(past_points[idx]) > (LOWER_BOUND + 2) &&
          AXIS(past_points[idx]) < (UPPER_BOUND - 2)) {
        // point disappeared in middle of grid, track it separately for
        // one frame so that if a new point appears in middle of grid,
        // we can pair them together below
        temp_forgotten_points[temp_forgotten_count] = past_points[idx];
        temp_forgotten_starting_points[temp_forgotten_count] = starting_points[idx];
        temp_forgotten_histories[temp_forgotten_count] = histories[idx];
        temp_forgotten_count++;
      }
      past_points[idx] = UNDEF_POINT;
      histories[idx] = 0;
    } else {
      // closest point matched, update trackers
      past_points[idx] = points[min_index];
      histories[idx]++;
      taken[min_index] = true;
    }
  }

  // look for any new points in this frame that did not match with past points

  for (uint8_t i=0; i<total_masses; i++) {
    if (!taken[i]) {
      uint16_t h = 1;
      uint8_t sp = points[i];
      // new point appeared (no past point found), start tracking it
      if (AXIS(points[i]) > (LOWER_BOUND + 2) &&
          AXIS(points[i]) < (UPPER_BOUND - 2)) {
        // this point appeared in middle of grid, let's check forgotten points for match
        for (uint8_t j=0; j<forgotten_count; j++) {
          if (forgotten_past_points[j] != UNDEF_POINT &&
              distance(forgotten_past_points[j], points[i]) < MIN_DISTANCE) {
            h = forgotten_histories[j];
            sp = forgotten_starting_points[j];
            forgotten_past_points[j] = UNDEF_POINT;
            break;
          }
        }
      }
      for (uint8_t j=0; j<MAX_PEOPLE; j++) {
        // look for first empty slot in past_points to use
        if (past_points[j] == UNDEF_POINT) {
          past_points[j] = points[i];
          starting_points[j] = sp;
          histories[j] = h;
          break;
        }
      }
    }
  }

  memcpy(forgotten_past_points, temp_forgotten_points, MAX_PEOPLE);
  memcpy(forgotten_starting_points, temp_forgotten_starting_points, MAX_PEOPLE);
  memcpy(forgotten_histories, temp_forgotten_histories, MAX_PEOPLE);
  forgotten_count = temp_forgotten_count;

  // publish event if any people moved through doorway yet

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT && histories[i] > MIN_HISTORY) {
      int diff = AXIS(starting_points[i]) - AXIS(past_points[i]);
      if (abs(diff) >= (GRID_EXTENT/2)) { // person traversed at least half the grid
        if (diff > 0) {
          SERIAL_PRINTLN("1 entered");
          // artificially shift starting point ahead 2 rows so that
          // if user turns around now, algorithm considers it an exit
          int s = past_points[i] - (2*GRID_EXTENT);
          starting_points[i] = max(s, 0);
        } else {
          SERIAL_PRINTLN("1 exited");
          int s = past_points[i] + (2*GRID_EXTENT);
          starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
        }
      }
    }
  }

  // reset calibration timer if people moved

  if (total_masses > 0) {
    bool rst = false;
    for (uint8_t i=0; i<MAX_PEOPLE; i++) {
      if (past_points[i] != UNDEF_POINT && past_points[i] != archived_past_points[i] &&
          (archived_past_points[i] == UNDEF_POINT ||
            distance(past_points[i], archived_past_points[i]) > 2)) {
        rst = true;
        break;
      }
    }
    if (rst) calibration_cycles = 0;
    memcpy(archived_past_points, past_points, MAX_PEOPLE);
  }

  // wrap up with debugging output

  #if defined(ENABLE_SERIAL) && defined(PRINT_RAW_DATA)
    if (total_masses > 0) {
      SERIAL_PRINT("Detected people at ");
      for (uint8_t i = 0; i<total_masses; i++) {
        SERIAL_PRINT(points[i]);
        SERIAL_PRINT(", ");
      }
      SERIAL_PRINTLN();

      // sort points so we can print them in chart easily
      uint8_t sorted_points[total_masses];
      memcpy(sorted_points, points, total_masses);
      qsort(sorted_points, total_masses, 1, sort_asc);

      // print chart of what sensor saw in 8x8 grid
      uint8_t next_point = sorted_points[0];
      uint8_t seen_points = 1;
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        if (PIXEL_ACTIVE(idx)) {
          idx == next_point ? SERIAL_PRINT("[") : SERIAL_PRINT(" ");
          SERIAL_PRINT(pixels[idx]);
          if (idx == next_point) {
            SERIAL_PRINT("]");
            if (seen_points < total_masses) {
              // find next point to put brackets around
              next_point = sorted_points[seen_points];
              seen_points++;
            }
          } else {
            SERIAL_PRINT(" ");
          }
        } else
          SERIAL_PRINT(" ----- ");
        if (!(NOT_RIGHT_EDGE)) SERIAL_PRINTLN();
      }
      SERIAL_PRINTLN();
    }
  #endif
}

void clearTrackers() {
  memset(histories, 0, MAX_PEOPLE);
  memset(past_points, UNDEF_POINT, MAX_PEOPLE);
  memset(archived_past_points, UNDEF_POINT, MAX_PEOPLE);
  memset(forgotten_past_points, UNDEF_POINT, MAX_PEOPLE);
}

void calibrateSensor() {
  amg.readPixels(calibrated_pixels);
  calibration_cycles = 0;
  SERIAL_PRINTLN("calibrated");
}

void initialize() {
  amg.begin();
  clearTrackers();
  blink(4);
  digitalWrite(LED, HIGH);
  LOWPOWER_DELAY(SLEEP_500MS);
  calibrateSensor();
}

void loop() {
  if (calibration_cycles == MAX_CALIBRATION_CYCLES) {
    calibrateSensor();
  } else {
    processSensor();
    calibration_cycles++;
  }
}

