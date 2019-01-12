#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.1"
#define GRID_EXTENT             8    // size of grid (8x8)
#define TEMP_BUFFER             1    // min increase in temp needed to register person
#define MIN_DISTANCE            3    // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            5    // max distance that a point is allowed to move
#define MIN_HISTORY             4    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define AXIS                    y    // axis along which we expect points to move (x or y)
#define ALPHA                   0.01 // learning rate for background temp
#define LOWER_BOUND             0
#define UPPER_BOUND             (GRID_EXTENT+1)
#define BORDER_PADDING          (GRID_EXTENT/4)
#define STRICT_BORDER_PADDING   (BORDER_PADDING-1)
#define LENIENT_BORDER_PADDING  (BORDER_PADDING+1)

#define REED_PIN                3
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float cur_pixels[AMG88xx_PIXEL_ARRAY_SIZE];

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
uint8_t forgotten_count = 0;
uint8_t cycles_since_forgotten = 0;

uint8_t door_state;

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

#define x(p) ( pgm_read_byte_near(xcoordinates + (p)) )
#define y(p) ( pgm_read_byte_near(ycoordinates + (p)) )

uint8_t manhattan_distance(uint8_t p1, uint8_t p2) {
  int8_t yd = y(p2) - y(p1);
  int8_t xd = x(p2) - x(p1);
  return abs(yd) + abs(xd);
}

uint8_t distance(uint8_t p1, uint8_t p2) {
  uint8_t md = manhattan_distance(p1, p2);
  if (md > MAX_DISTANCE) return md;

  // calculate euclidean distance instead
  int8_t yd = y(p2) - y(p1);
  int8_t xd = x(p2) - x(p1);
  return floor(sqrt(sq(yd) + sq(xd)));
}

// used only by qsort in debugging section (compiled out otherwise)
int sort_asc(const void *cmp1, const void *cmp2) {
  uint8_t a = *((uint8_t *)cmp1);
  uint8_t b = *((uint8_t *)cmp2);
  return a - b;
}

#if AXIS == y
  #define NOT_AXIS      x
  #define SIDE(p)       ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) ? 1 : 2 )
#else
  #define NOT_AXIS      y
  #define SIDE(p)       ( (AXIS(p)) <= (GRID_EXTENT/2) ? 1 : 2 )
#endif
#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define SIDE1_PAD(i)    ( SIDE(i) == 1 ? TEMP_BUFFER : 0 )
#define CHECK_TEMP(i)   ( cur_pixels[(i)] > (avg_pixels[(i)] + TEMP_BUFFER + (SIDE1_PAD(i))) )
#define CHECK_DOOR(i)   ( door_state == HIGH || AXIS(i) <= (GRID_EXTENT/2) )
#define PIXEL_ACTIVE(i) ( (CHECK_TEMP(i)) && (CHECK_DOOR(i)) )

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
  SORT_ARRAY(cur_pixels, (PIXEL_ACTIVE(i)), AMG88xx_PIXEL_ARRAY_SIZE);
}

uint8_t sortPointsByHistory(uint8_t *ordered_indexes) {
  uint16_t ordered_values[MAX_PEOPLE];
  SORT_ARRAY(histories, (histories[i] > 0), MAX_PEOPLE);
}

void publishEvents() {
  if (door_state == LOW) return; // nothing happened if door is closed

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT && histories[i] > MIN_HISTORY) {
      // do not consider trajectories that end in border of middle 2 rows
      if ((NOT_AXIS(past_points[i]) > (LOWER_BOUND + STRICT_BORDER_PADDING) &&
           NOT_AXIS(past_points[i]) < (UPPER_BOUND - STRICT_BORDER_PADDING)) ||
          AXIS(past_points[i]) <= (LOWER_BOUND + LENIENT_BORDER_PADDING) ||
          AXIS(past_points[i]) >= (UPPER_BOUND - LENIENT_BORDER_PADDING)) {
        int diff = AXIS(starting_points[i]) - AXIS(past_points[i]);
        if (abs(diff) >= (GRID_EXTENT/2)) { // person traversed at least half the grid
          if (diff > 0) {
            publish("1");
            // artificially shift starting point ahead 1 row so that
            // if user turns around now, algorithm considers it an exit
            int s = past_points[i] - GRID_EXTENT;
            starting_points[i] = max(s, 0);
            histories[i] -= 2;
          } else {
            publish("2");
            int s = past_points[i] + GRID_EXTENT;
            starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
            histories[i] -= 2;
          }
        }
      }
    }
  }
}

bool readPixels() {
  float past_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(past_pixels, cur_pixels, AMG88xx_PIXEL_ARRAY_SIZE);

  amg.readPixels(cur_pixels);

  // return true if pixels changed
  return (memcmp(past_pixels, cur_pixels, AMG88xx_PIXEL_ARRAY_SIZE) != 0);
}

// if we see spikes in lots active pixels, massage readings by subtracting out
// the difference between the current average and historical average to hopefully
// eliminate noise but still keep peaks from people visible.
void equalizeValues() {
  float cur_avg = 0;
  float avg_avg = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    cur_avg += cur_pixels[i];
    avg_avg += avg_pixels[i];
  }
  float dif_avg = cur_avg - avg_avg;
  if (dif_avg > 0) {
    dif_avg /= AMG88xx_PIXEL_ARRAY_SIZE;
    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (CHECK_TEMP(i)) {
        float d = cur_pixels[i] - dif_avg;
        cur_pixels[i] = max(d, avg_pixels[i]);
      }
    }
  }
}

void updateAverages() {
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] += ALPHA * (cur_pixels[i] - avg_pixels[i]);
  }
}

void clearTrackers() {
  memset(histories, 0, MAX_PEOPLE);
  memset(past_points, UNDEF_POINT, MAX_PEOPLE);
  memset(forgotten_past_points, UNDEF_POINT, MAX_PEOPLE);
}

void processSensor() {
  if (!readPixels()) return;
  equalizeValues();

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
  uint8_t dropped_points = 0;

  for (uint8_t i=0; i<past_total_masses; i++) {
    uint8_t idx = ordered_past_points[i];
    uint8_t min_distance = MAX_DISTANCE;
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
    if (min_index != UNDEF_POINT && (total_masses + dropped_points) < past_total_masses &&
        ((AXIS(past_points[idx]) - LOWER_BOUND) <= min(min_distance, BORDER_PADDING) ||
         (UPPER_BOUND - AXIS(past_points[idx])) <= min(min_distance, BORDER_PADDING))) {
      // a point disappeared in this frame, looks like it could be this one
      min_index = UNDEF_POINT;
      dropped_points++;
    }
    if (min_index == UNDEF_POINT) {
      // point disappeared (no new point found), stop tracking it
      // but track it separately so that if a new point appears
      // later, we can pair them together below
      temp_forgotten_points[temp_forgotten_count] = past_points[idx];
      temp_forgotten_starting_points[temp_forgotten_count] = starting_points[idx];
      temp_forgotten_histories[temp_forgotten_count] = histories[idx];
      temp_forgotten_count++;
      past_points[idx] = UNDEF_POINT;
      histories[idx] = 0;
    } else {
      // closest point matched, update trackers
      if (past_points[idx] != points[min_index]) {
        if (SIDE(points[min_index]) != SIDE(past_points[idx])) {
          // point just crossed threshold, let's reduce its history to force
          // it to spend another cycle on this side before we count the event
          histories[idx] = min(histories[idx], MIN_HISTORY);
        } else {
          histories[idx]++;
        }
        past_points[idx] = points[min_index];
      }
      taken[min_index] = true;
    }
  }

  // look for any new points in this frame that did not match with past points

  for (uint8_t i=0; i<total_masses; i++) {
    if (!taken[i]) {
      uint16_t h = 1;
      uint8_t sp = points[i];
      // new point appeared (no past point found), start tracking it
      if ((AXIS(points[i]) > (LOWER_BOUND + BORDER_PADDING) &&
          AXIS(points[i]) < (UPPER_BOUND - BORDER_PADDING)) ||
          cycles_since_forgotten < 3) {
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
      bool row5 = false;
      if (h == 1 && AXIS(sp) == (GRID_EXTENT/2 + 1)) {
        // if point is starting in row 5, move it back to row 6
        sp += GRID_EXTENT;
        row5 = true;
      }
      // ignore new points that showed up in middle 2 rows of grid
      // unless we found a matching forgotten point or started in row 5
      if (h > 1 || row5 ||
          AXIS(points[i]) <= (LOWER_BOUND + LENIENT_BORDER_PADDING) ||
          AXIS(points[i]) >= (UPPER_BOUND - LENIENT_BORDER_PADDING)) {
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
  }

  // copy forgotten data points for this frame to global scope

  if (total_masses > 0 || past_total_masses > 0) {
    memcpy(forgotten_past_points, temp_forgotten_points, MAX_PEOPLE);
    memcpy(forgotten_starting_points, temp_forgotten_starting_points, MAX_PEOPLE);
    memcpy(forgotten_histories, temp_forgotten_histories, MAX_PEOPLE);
    forgotten_count = temp_forgotten_count;
    cycles_since_forgotten = 0;
  } else {
    if (cycles_since_forgotten == 1) {
      for (uint8_t i=0; i<forgotten_count; i++) {
        if (AXIS(forgotten_past_points[i]) <= (LOWER_BOUND + BORDER_PADDING) ||
            AXIS(forgotten_past_points[i]) >= (UPPER_BOUND - BORDER_PADDING) ||
            NOT_AXIS(forgotten_past_points[i]) <= (LOWER_BOUND + STRICT_BORDER_PADDING) ||
            NOT_AXIS(forgotten_past_points[i]) >= (UPPER_BOUND - STRICT_BORDER_PADDING)) {
          // this point exists on the outer edge of grid, forget it after 1 frame        
          forgotten_past_points[i] = UNDEF_POINT;
        }
      }
    }
    if (cycles_since_forgotten == 5) {
      memset(forgotten_past_points, UNDEF_POINT, MAX_PEOPLE);
    } else if (cycles_since_forgotten < 5) {
      cycles_since_forgotten++;
    }
  }

  // publish event if any people moved through doorway yet

  publishEvents();

  // wrap up with debugging output

  #if defined(ENABLE_SERIAL) && defined(PRINT_RAW_DATA)
    if (total_masses == 0 && past_total_masses > 0) {
      SERIAL_PRINTLN("cleared board");
    }
    if (total_masses > 0) {
      SERIAL_PRINT("Detected people at ");
      for (uint8_t i = 0; i<MAX_PEOPLE; i++) {
        if (past_points[i] != UNDEF_POINT) {
          SERIAL_PRINT(past_points[i]);
          SERIAL_PRINT(" (");
          SERIAL_PRINT(starting_points[i]);
          SERIAL_PRINT("->");
          SERIAL_PRINT(histories[i]);
          SERIAL_PRINT("), ");
        }
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
          SERIAL_PRINT(cur_pixels[idx]);
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
        } else {
          SERIAL_PRINT(" ----- ");
        }
        if (!(NOT_RIGHT_EDGE)) SERIAL_PRINTLN();
      }
      SERIAL_PRINTLN();
    }
  #endif

  updateAverages();
}

void checkDoorState() {
  if (door_state != digitalRead(REED_PIN)) {
    door_state = digitalRead(REED_PIN);
    if (door_state == HIGH) {
      publish("d1");
    } else {
      publish("d0");
    }
  }
}

void initialize() {
  amg.begin();

  blink(4);
  digitalWrite(LED, HIGH);

  pinMode(REED_PIN, INPUT_PULLUP);
  door_state = 3;

  clearTrackers();
  amg.readPixels(avg_pixels);
}

void loop() {
  checkDoorState();
  processSensor();
}

