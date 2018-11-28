//#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.1"
#define GRID_EXTENT             8    // size of grid (8x8)
#define TEMP_BUFFER             2    // min increase in temp needed to register person
#define MIN_DISTANCE            4    // min distance for 2 peaks to be separate people
#define MAX_PEOPLE              5    // most people we support in a single frame
#define MAX_WAKE_CYCLES         150  // each cycle is roughly 100ms, so 15s
#define MAX_CALIBRATION_CYCLES  150  // each cycle is 8s, so 150*8 = 20min
#define MIN_MOTION_BLOCK        ( MAX_CALIBRATION_CYCLES / 2 )

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float calibrated_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

#define UNDEF_POINT             ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
uint8_t past_points[MAX_PEOPLE] = {UNDEF_POINT, UNDEF_POINT, UNDEF_POINT, UNDEF_POINT, UNDEF_POINT};
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE] = {0,0,0,0,0};

uint8_t calibration_cycles = 0;
uint8_t wake_cycles = 0;
volatile boolean motion = false;

// store in-memory so we don't have to do math every time
const uint8_t xcoordinates[64] = {
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8
};
const uint8_t ycoordinates[64] = {
  1,  1,  1,  1,  1,  1,  1,  1,
  2,  2,  2,  2,  2,  2,  2,  2,
  3,  3,  3,  3,  3,  3,  3,  3,
  4,  4,  4,  4,  4,  4,  4,  4,
  5,  5,  5,  5,  5,  5,  5,  5,
  6,  6,  6,  6,  6,  6,  6,  6,
  7,  7,  7,  7,  7,  7,  7,  7,
  8,  8,  8,  8,  8,  8,  8,  8
};

// calculate manhattan distance between 2 indices
uint8_t distance(uint8_t p1, uint8_t p2) {
  int8_t yd = ycoordinates[p2] - ycoordinates[p1];
  int8_t xd = xcoordinates[p2] - xcoordinates[p1];
  return abs(yd) + abs(xd);
}

int sort_asc(const void *cmp1, const void *cmp2) {
  uint8_t a = *((uint8_t *)cmp1);
  uint8_t b = *((uint8_t *)cmp2);
  return a - b;
}

#define PIXEL_ACTIVE(i) ( pixels[(i)] > (calibrated_pixels[(i)] + TEMP_BUFFER) )

void processSensor() {
  amg.readPixels(pixels);

  // sort pixels by temperature to find peaks

  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  float ordered_temps[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
  for(uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    if (PIXEL_ACTIVE(i)) {
      bool added = false;
      for (uint8_t j=0; j<active_pixel_count; j++) {
        if (pixels[i] > ordered_temps[j]) {
          for (uint8_t x=active_pixel_count; x>j; x--) {
            ordered_temps[x] = ordered_temps[x-1];
            ordered_indexes[x] = ordered_indexes[x-1];
          }
          ordered_temps[j] = pixels[i];
          ordered_indexes[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        ordered_temps[active_pixel_count] = pixels[i];
        ordered_indexes[active_pixel_count] = i;
      }
      active_pixel_count++;
    }
  }

  // determine which points are people by comparing peaks with neighboring cells

  #define TL_IDX ( idx - (GRID_EXTENT+1) )
  #define TM_IDX ( idx - GRID_EXTENT )
  #define TR_IDX ( idx - (GRID_EXTENT-1) )
  #define ML_IDX ( idx - 1 )
  #define MR_IDX ( idx + 1 )
  #define BL_IDX ( idx + (GRID_EXTENT-1) )
  #define BM_IDX ( idx + GRID_EXTENT )
  #define BR_IDX ( idx + (GRID_EXTENT+1) )
  #define NOT_LEFT_EDGE   ( xcoordinates[idx] > 1 )
  #define NOT_RIGHT_EDGE  ( xcoordinates[idx] < GRID_EXTENT )
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
  for(uint8_t i=0; i<active_pixel_count; i++){
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
    }
  }

  // sort list of previously seen people by how many frames we've seen them
  // to prioritize finding people who have been in frame longest (reduces impact of noisy data)

  uint16_t ordered_histories[MAX_PEOPLE];
  uint8_t ordered_points[MAX_PEOPLE];
  uint8_t point_count = 0;
  for(uint8_t i=0; i<MAX_PEOPLE; i++){
    uint16_t h = histories[i];
    if (h > 0) {
      bool added = false;
      for (uint8_t j=0; j<point_count; j++) {
        if (h > ordered_histories[j]) {
          for (uint8_t x=point_count; x>j; x--) {
            ordered_histories[x] = ordered_histories[x-1];
            ordered_points[x] = ordered_points[x-1];
          }
          ordered_histories[j] = h;
          ordered_points[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        ordered_histories[point_count] = h;
        ordered_points[point_count] = i;
      }
      point_count++;
    }
  }

  // pair previously seen points with new points to determine where people moved

  bool taken[MAX_PEOPLE] = {false, false, false, false, false};
  for (uint8_t i=0; i<point_count; i++) {
    uint8_t idx = ordered_points[i];
    uint8_t p = past_points[idx];
    uint8_t min_distance = MIN_DISTANCE;
    uint8_t min_index = UNDEF_POINT;
    for (uint8_t j=0; j<total_masses; j++) {
      if (!taken[j]) {
        // match with closest point
        uint8_t d = distance(p, points[j]);
        if (d < min_distance) {
          min_distance = d;
          min_index = j;
        }
      }
    }
    if (min_index == UNDEF_POINT) {
      // point disappeared, stop tracking it
      past_points[idx] = UNDEF_POINT;
      histories[idx] = 0;
    } else {
      // closest point matched, update trackers
      past_points[idx] = points[min_index];
      histories[idx]++;
      taken[min_index] = true;
    }
  }

  for (uint8_t i=0; i<total_masses; i++) {
    if (!taken[i]) {
      // new point appeared, start tracking it
      for (uint8_t j=0; j<MAX_PEOPLE; j++) {
        if (past_points[j] == UNDEF_POINT) {
          starting_points[j] = points[i];
          past_points[j] = points[i];
          histories[j] = 1;
          break;
        }
      }
    }
  }

  // publish event if any people moved through doorway yet

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT) {
      uint8_t starting_pos = ycoordinates[starting_points[i]];
      uint8_t ending_pos = ycoordinates[past_points[i]];
      int diff = starting_pos - ending_pos;
      if (abs(diff) >= (GRID_EXTENT/2)) {
        if (starting_pos > ending_pos) {
          SERIAL_PRINTLN("1 entered");
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

  // wrap up with debugging output

  #if defined(ENABLE_SERIAL) && defined(PRINT_RAW_DATA)
    if (total_masses > 0) {
      SERIAL_PRINT("Detected people at ");
      for(uint8_t i = 0; i<total_masses; i++) {
        SERIAL_PRINT(points[i]);
        SERIAL_PRINT(", ");
      }
      SERIAL_PRINTLN();

      // sort points so we can print them in chart easily
      uint8_t sorted_points[total_masses];
      for (uint8_t i=0; i<total_masses; i++) {
        sorted_points[i] = points[i];
      }
      qsort(sorted_points, total_masses, 1, sort_asc);

      // print chart of what sensor saw in 8x8 grid
      uint8_t next_point = sorted_points[0];
      uint8_t seen_points = 1;
      for(uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++){
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

void sleepAmg() {
  if (amg.getPowerMode() != AMG88xx_SLEEP_MODE) {
    amg.setSleepMode();
  }
}

void wakeAMG() {
  if (amg.getPowerMode() != AMG88xx_NORMAL_MODE) {
    amg.setNormalMode();
  }
}

void calibrateSensor() {
  wakeAMG();
  LOWPOWER_DELAY(SLEEP_500MS);
  amg.readPixels(calibrated_pixels);
  calibration_cycles = 0;
}

void sleepSensor() {
  if (calibration_cycles == MAX_CALIBRATION_CYCLES) {
    calibrateSensor();
  } else {
    calibration_cycles++;
  }
  sleepAmg();
  LOWPOWER_DELAY(SLEEP_8S);
}

void motionISR() {
  motion = true;
}

void initialize() {
  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), motionISR, RISING);

  amg.begin();
  calibrateSensor();
}

void loop() {
  if (motion) {
    wakeAMG();
    SERIAL_PRINTLN("motion!");
    motion = false;
    wake_cycles = MAX_WAKE_CYCLES;
    // ensure a wait before trying to calibrate after motion
    calibration_cycles = min(calibration_cycles, MIN_MOTION_BLOCK);
  }
  if (wake_cycles == 0) {
    sleepSensor();
  } else {
    processSensor();
    wake_cycles--;
  }
}

