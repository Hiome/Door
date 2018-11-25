//#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION     "V0.1"
#define GRID_EXTENT          8
#define TEMP_BUFFER          2
#define MAX_DISTANCE         3
#define MAX_PEOPLE           5
#define UNDEF_POINT          70

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float calibrated_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

uint8_t past_points[MAX_PEOPLE] = {UNDEF_POINT, UNDEF_POINT, UNDEF_POINT, UNDEF_POINT, UNDEF_POINT};
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE] = {0,0,0,0,0};

volatile boolean motion = false;
void motionISR() {
  motion = true;
}

void initialize() {
  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), motionISR, RISING);

  amg.begin();

  delay(500); // let sensor boot up
  amg.readPixels(calibrated_pixels);
}

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

void loop() {
  if(motion) {
    Sprintln("motion!");
    motion = false;
  }

  amg.readPixels(pixels);

  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  float ordered_temps[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t count = 0;
  for(uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    float t = pixels[i];
    if (t - calibrated_pixels[i] > TEMP_BUFFER) {
      bool added = false;
      for (uint8_t j=0; j<count; j++) {
        if (t > ordered_temps[j]) {
          for (uint8_t x=count; x>j; x--) {
            ordered_temps[x] = ordered_temps[x-1];
            ordered_indexes[x] = ordered_indexes[x-1];
          }
          ordered_temps[j] = t;
          ordered_indexes[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        ordered_temps[count] = t;
        ordered_indexes[count] = i;
      }
      count++;
    }
  }

  // check temperatures in neighboring cells
  #define NOT_LEFT_EDGE   ( xcoordinates[idx] > 1 )
  #define NOT_RIGHT_EDGE  ( xcoordinates[idx] < GRID_EXTENT )
  #define NOT_TOP_EDGE    ( idx >= GRID_EXTENT )
  #define NOT_BOTTOM_EDGE ( idx < (AMG88xx_PIXEL_ARRAY_SIZE - GRID_EXTENT) )
  #define TL_IDX ( idx - (GRID_EXTENT+1) )
  #define TM_IDX ( idx - GRID_EXTENT )
  #define TR_IDX ( idx - (GRID_EXTENT-1) )
  #define ML_IDX ( idx - 1 )
  #define MR_IDX ( idx + 1 )
  #define BL_IDX ( idx + (GRID_EXTENT-1) )
  #define BM_IDX ( idx + GRID_EXTENT )
  #define BR_IDX ( idx + (GRID_EXTENT+1) )
  #define CHECK_PIXEL(i)( pixels[(i)] > (calibrated_pixels[(i)] + TEMP_BUFFER) )
  #define MIDDLE_LEFT   ( CHECK_PIXEL(ML_IDX) )
  #define MIDDLE_RIGHT  ( CHECK_PIXEL(MR_IDX) )
  #define TOP_LEFT      ( NOT_TOP_EDGE && CHECK_PIXEL(TL_IDX) )
  #define TOP_MIDDLE    ( NOT_TOP_EDGE && CHECK_PIXEL(TM_IDX) )
  #define TOP_RIGHT     ( NOT_TOP_EDGE && CHECK_PIXEL(TR_IDX) )
  #define BOTTOM_LEFT   ( NOT_BOTTOM_EDGE && CHECK_PIXEL(BL_IDX) )
  #define BOTTOM_MIDDLE ( NOT_BOTTOM_EDGE && CHECK_PIXEL(BM_IDX) )
  #define BOTTOM_RIGHT  ( NOT_BOTTOM_EDGE && CHECK_PIXEL(BR_IDX) )

  uint8_t total_masses = 0;
  uint8_t points[MAX_PEOPLE];
  for(uint8_t i=0; i<count; i++){
    uint8_t idx = ordered_indexes[i];
    bool distinct = true;
    for (uint8_t j=0; j<i; j++) {
      if (distance(ordered_indexes[j], idx) <= MAX_DISTANCE) {
        distinct = false;
        break;
      }
    }
    if (distinct && (
      // has at least one neighboring cell (ignore lone temp readings)
      (NOT_LEFT_EDGE && (MIDDLE_LEFT || TOP_LEFT || BOTTOM_LEFT)) ||
      (NOT_RIGHT_EDGE && (MIDDLE_RIGHT || TOP_RIGHT || BOTTOM_RIGHT)) ||
      TOP_MIDDLE || BOTTOM_MIDDLE)) {
      points[total_masses] = idx;
      total_masses++;
    }
  }

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

  bool taken[MAX_PEOPLE] = {false, false, false, false, false};
  for (uint8_t i=0; i<point_count; i++) {
    uint8_t idx = ordered_points[i];
    uint8_t p = past_points[idx];
    uint8_t min_distance = (MAX_DISTANCE + 1);
    uint8_t min_index = UNDEF_POINT;
    for (uint8_t j=0; j<total_masses; j++) {
      if (!taken[j]) {
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
      // new point appeared
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

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT) {
      uint8_t starting_pos = ycoordinates[starting_points[i]];
      uint8_t ending_pos = ycoordinates[past_points[i]];
      int diff = starting_pos - ending_pos;
      if (abs(diff) >= (GRID_EXTENT/2)) {
        if (starting_pos > ending_pos) {
          Sprintln("1 entered");
          int s = past_points[i] - (2*GRID_EXTENT);
          starting_points[i] = max(s, 0);
        } else {
          Sprintln("1 exited");
          int s = past_points[i] + (2*GRID_EXTENT);
          starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
        }
      }
    }
  }

  #if defined(DEBUGGER) && defined(PRINT_RAW_DATA)
    if (total_masses > 0) {
      for(uint8_t i = 0; i<total_masses; i++) {
        Serial.print(points[i]);
        Serial.print(", ");
      }
      Serial.println();
    
      Serial.print("[");
      for(uint8_t i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
        float curr = pixels[i-1];
        float last = calibrated_pixels[i-1];
        if (curr - last > TEMP_BUFFER) {
          Serial.print(curr);
        } else
          Serial.print("-----");
        Serial.print(", ");
        if( i%8 == 0 ) Serial.println();
      }
      Serial.println("]");
      Serial.println();
    }
  #endif
}

