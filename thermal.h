#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.1"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define TEMP_BUFFER             1.0  // min increase in temp needed to register person
#define MIN_DISTANCE            3.0  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            5.0  // max distance that a point is allowed to move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MIN_PIXEL_NEIGHBORS     2    // a cell must have at least 2 active neighbors
#define SIMILAR_TEMP_DIFF       1.0  // treat 2 cells within 1º of each other as the same
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
float raw_pixels[AMG88xx_PIXEL_ARRAY_SIZE];

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
bool crossed[MAX_PEOPLE];
uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
bool forgotten_crossed[MAX_PEOPLE];
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

float euclidean_distance(uint8_t p1, uint8_t p2) {
  uint8_t md = manhattan_distance(p1, p2);
  if (md > MAX_DISTANCE) return (float)md;

  // calculate euclidean distance instead
  int8_t yd = y(p2) - y(p1);
  int8_t xd = x(p2) - x(p1);
  return (float)sqrt(sq(yd) + sq(xd));
}

// used only by qsort in debugging section (compiled out otherwise)
int sort_asc(const void *cmp1, const void *cmp2) {
  uint8_t a = *((uint8_t *)cmp1);
  uint8_t b = *((uint8_t *)cmp2);
  return a - b;
}

#ifdef YAXIS
  #define AXIS          y
  #define NOT_AXIS      x
  #define SIDE(p)       ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) ? 1 : 2 )
#else
  #define AXIS          x
  #define NOT_AXIS      y
  #define SIDE(p)       ( (AXIS(p)) <= (GRID_EXTENT/2) ? 1 : 2 )
  #error Double check all your code, this is untested
#endif
#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define CHECK_TEMP(i)   ( cur_pixels[(i)] > (avg_pixels[(i)] + TEMP_BUFFER) )
#define CHECK_DOOR(i)   ( door_state == HIGH || AXIS(i) <= (GRID_EXTENT/2) )
#define PIXEL_ACTIVE(i) ( (CHECK_TEMP(i)) && (CHECK_DOOR(i)) )

// count how many neighboring cells are active for any given cell in the current frame
uint8_t neighborsForPixel(uint8_t idx) {
  #define TOP_LEFT        ( idx - (GRID_EXTENT+1) )
  #define TOP_MIDDLE      ( idx - GRID_EXTENT )
  #define TOP_RIGHT       ( idx - (GRID_EXTENT-1) )
  #define MIDDLE_LEFT     ( idx - 1 )
  #define MIDDLE_RIGHT    ( idx + 1 )
  #define BOTTOM_LEFT     ( idx + (GRID_EXTENT-1) )
  #define BOTTOM_MIDDLE   ( idx + GRID_EXTENT )
  #define BOTTOM_RIGHT    ( idx + (GRID_EXTENT+1) )
  #define NOT_LEFT_EDGE   ( x(idx) > 1 )
  #define NOT_RIGHT_EDGE  ( x(idx) < GRID_EXTENT )
  #define NOT_TOP_EDGE    ( idx >= GRID_EXTENT )
  #define NOT_BOTTOM_EDGE ( idx < (AMG88xx_PIXEL_ARRAY_SIZE - GRID_EXTENT) )

  // a cell has up to 8 neighbors. check each one to see if it's relevant and active.
  // xxx
  // xox
  // xxx
  uint8_t count = 0;
  if (PIXEL_ACTIVE(idx)) {
    if (NOT_LEFT_EDGE) {
      if (PIXEL_ACTIVE(MIDDLE_LEFT)) count++;
      if (NOT_TOP_EDGE && PIXEL_ACTIVE(TOP_LEFT)) count++;
      if (NOT_BOTTOM_EDGE && PIXEL_ACTIVE(BOTTOM_LEFT)) count++;
    }
    if (NOT_RIGHT_EDGE) {
      if (PIXEL_ACTIVE(MIDDLE_RIGHT)) count++;
      if (NOT_TOP_EDGE && PIXEL_ACTIVE(TOP_RIGHT)) count++;
      if (NOT_BOTTOM_EDGE && PIXEL_ACTIVE(BOTTOM_RIGHT)) count++;
    }
    if (NOT_TOP_EDGE && PIXEL_ACTIVE(TOP_MIDDLE)) count++;
    if (NOT_BOTTOM_EDGE && PIXEL_ACTIVE(BOTTOM_MIDDLE)) count++;
  }

  return count;
}

// This macro sorts array indexes based on their corresponding values
// in desc order. For example, given an array {2, 4, 1, 0},
//   SORT_ARRAY(a, (a[i] > 0), (a[i] > a[ordered_indexes[i]]), 4)
// will return 3 and ordered_indexes will be {1, 0, 2}. I'm sorry...
#define SORT_ARRAY(src, cnd, cmp, sze) ({                            \
  uint8_t count = 0;                                                 \
  for (uint8_t i=0; i<(sze); i++) {                                  \
    if ((cnd)) {                                                     \
      bool added = false;                                            \
      for (uint8_t j=0; j<count; j++) {                              \
        if ((cmp)) {                                                 \
          for (uint8_t x=count; x>j; x--) {                          \
            ordered_indexes[x] = ordered_indexes[x-1];               \
          }                                                          \
          ordered_indexes[j] = i;                                    \
          added = true;                                              \
          break;                                                     \
        }                                                            \
      }                                                              \
      if (!added) {                                                  \
        ordered_indexes[count] = i;                                  \
      }                                                              \
      count++;                                                       \
    }                                                                \
  }                                                                  \
  return count;                                                      \
})
//////////////////////////////////////////////////////////////////////

uint8_t sortPixelsByTemp(uint8_t *ordered_indexes) {
  uint8_t active[AMG88xx_PIXEL_ARRAY_SIZE];
  for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
    active[idx] = neighborsForPixel(idx);
  }

  SORT_ARRAY(cur_pixels, (active[i] >= MIN_PIXEL_NEIGHBORS),
    ((abs(cur_pixels[i] - cur_pixels[ordered_indexes[j]]) <= SIMILAR_TEMP_DIFF &&
      active[i] > active[ordered_indexes[j]]) ||
    (cur_pixels[i] - cur_pixels[ordered_indexes[j]]) > SIMILAR_TEMP_DIFF ||
    (cur_pixels[i] > cur_pixels[ordered_indexes[j]] &&
      active[i] == active[ordered_indexes[j]])),
    AMG88xx_PIXEL_ARRAY_SIZE);
}

uint8_t sortPastPointsByHistory(uint8_t *ordered_indexes) {
  SORT_ARRAY(histories, (histories[i] > 0),
    (histories[i] > histories[ordered_indexes[j]]), MAX_PEOPLE);
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
          } else {
            publish("2");
            int s = past_points[i] + GRID_EXTENT;
            starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
          }
          histories[i] -= 2;
          crossed[i] = true;
        }
      }
    }
  }
}

bool readPixels() {
  float past_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(past_pixels, raw_pixels, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float)));

  amg.readPixels(raw_pixels);

  // return true if pixels changed
  return (memcmp(past_pixels, raw_pixels, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float))) != 0);
}

// if we see spikes in lots active pixels, massage readings by subtracting out
// the difference between the current average and historical average to hopefully
// eliminate noise but still keep peaks from people visible.
void equalizeValues() {
  memcpy(cur_pixels, raw_pixels, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float)));
  float cur_avg1 = 0;
  float avg_avg1 = 0;
  float cur_avg2 = 0;
  float avg_avg2 = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (SIDE(i) == 1) {
      cur_avg1 += cur_pixels[i];
      avg_avg1 += avg_pixels[i];
    } else {
      cur_avg2 += cur_pixels[i];
      avg_avg2 += avg_pixels[i];
    }
  }
  float dif_avg1 = (cur_avg1 - avg_avg1)/(AMG88xx_PIXEL_ARRAY_SIZE/2);
  float dif_avg2 = (cur_avg2 - avg_avg2)/(AMG88xx_PIXEL_ARRAY_SIZE/2);
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    cur_pixels[i] -= (SIDE(i) == 1 ? dif_avg1 : dif_avg2);
  }
}

void updateAverages() {
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] += ALPHA * (raw_pixels[i] - avg_pixels[i]);
  }
}

void clearTrackers() {
  memset(histories, 0, (MAX_PEOPLE*sizeof(uint16_t)));
  memset(crossed, false, (MAX_PEOPLE*sizeof(bool)));
  memset(past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
  memset(forgotten_past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
}

uint8_t findCurrentPoints(uint8_t *points) {
  // sort pixels by temperature to find peaks
  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = sortPixelsByTemp(ordered_indexes);

  // determine which peaks are distinct people by making sure they are far enough apart
  uint8_t total_masses = 0;
  for (uint8_t i=0; i<active_pixel_count; i++) {
    uint8_t idx = ordered_indexes[i];
    bool distinct = true;
    for (uint8_t j=0; j<i; j++) {
      // make sure this point isn't close to another peak we already considered
      if (euclidean_distance(ordered_indexes[j], idx) <= MIN_DISTANCE) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      points[total_masses] = idx;
      total_masses++;
      if (total_masses == MAX_PEOPLE) break;  // we've hit our cap, stop looking
    }
  }

  return total_masses;
}

void processSensor() {
  if (!readPixels()) return;
  equalizeValues();

  // sort list of previously seen people by how many frames we've seen them to prioritize
  // finding people who have been in frame longest (reduces impact of noisy data)

  uint8_t ordered_past_points[MAX_PEOPLE];
  uint8_t past_total_masses = sortPastPointsByHistory(ordered_past_points);

  // find list of peaks in current frame

  uint8_t points[MAX_PEOPLE];
  uint8_t total_masses = findCurrentPoints(points);

  // pair previously seen points with new points to determine where people moved

  // "I don't know who you are or what you want, but you should know that I have a
  // very particular set of skills that make me a nightmare for people like you.
  // I will find you, I will track you, and I will turn the lights on for you."
  uint8_t taken[total_masses];
  memset(taken, 0, (total_masses*sizeof(uint8_t)));

  // "Good luck."
  uint8_t temp_forgotten_points[MAX_PEOPLE];
  memset(temp_forgotten_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
  uint8_t temp_forgotten_starting_points[MAX_PEOPLE];
  uint16_t temp_forgotten_histories[MAX_PEOPLE];
  bool temp_forgotten_crossed[MAX_PEOPLE];
  uint8_t temp_forgotten_count = 0;

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT ({                                                         \
    temp_forgotten_points[temp_forgotten_count] = past_points[idx];               \
    temp_forgotten_starting_points[temp_forgotten_count] = starting_points[idx];  \
    temp_forgotten_histories[temp_forgotten_count] = histories[idx];              \
    temp_forgotten_crossed[temp_forgotten_count] = crossed[idx];                  \
    temp_forgotten_count++;                                                       \
    past_points[idx] = UNDEF_POINT;                                               \
    pairs[idx] = UNDEF_POINT;                                                     \
    histories[idx] = 0;                                                           \
    crossed[idx] = false;                                                         \
  })

  uint8_t pairs[past_total_masses];
  for (uint8_t i=0; i<past_total_masses; i++) {
    uint8_t idx = ordered_past_points[i];
    float min_distance = MAX_DISTANCE;
    uint8_t min_index = UNDEF_POINT;
    for (uint8_t j=0; j<total_masses; j++) {
      // match with closest available point
      if (taken[j] == 0) {
        float d = euclidean_distance(past_points[idx], points[j]);
        if (d < min_distance) {
          min_distance = d;
          min_index = j;
        }
      }
    }
    if (min_index == UNDEF_POINT) {
      // try again, but include already matched points in second pass
      for (uint8_t j=0; j<total_masses; j++) {
        float d = euclidean_distance(past_points[idx], points[j]);
        if (d < min_distance) {
          min_distance = d;
          min_index = j;
        }
      }
    }
    if (min_index != UNDEF_POINT &&
        (total_masses + temp_forgotten_count) < past_total_masses && crossed[idx] &&
        ((AXIS(past_points[idx]) - LOWER_BOUND) <= min(min_distance, BORDER_PADDING) ||
         (UPPER_BOUND - AXIS(past_points[idx])) <= min(min_distance, BORDER_PADDING))) {
      // we know some point disappeared in this frame, and we know this point already
      // crossed the middle and was near the border on the other side. Most likely,
      // it's the one that left the grid, so let's drop it.
      min_index = UNDEF_POINT;
    }
    if (min_index == UNDEF_POINT) {
      // still not found...
      FORGET_POINT;
    } else {
      taken[min_index]++;
      pairs[idx] = min_index;
    }
  }

  for (uint8_t i=0; i<total_masses; i++) {
    if (taken[i] > 1) {
      // more than one past point is trying to match with this single current point...
      uint8_t max_score = 0;
      uint8_t max_idx = UNDEF_POINT;
      for (uint8_t idx=0; idx<past_total_masses; idx++) {
        if (pairs[idx] == i) {
          // chose the point that is a combination of most clustered and longest history.
          // we're just adding up the number of neighbors a cell has and its history
          // (capped at 4) to come up with a composite score to rank competing points
          uint8_t score = neighborsForPixel(past_points[idx]) + min(histories[idx], 4);
          if (score > max_score) {
            max_score = score;
            max_idx = idx;
          } else if (score == max_score) {
            // if 2 points have the same composite score, use the
            // one who's temp is more similar to the new point
            float d1 = abs(cur_pixels[past_points[idx]] - cur_pixels[points[i]]);
            float d2 = abs(cur_pixels[past_points[max_idx]] - cur_pixels[points[i]]);
            if (d1 < d2) {
              max_idx = idx;
            }
          }
        }
      }
      // once we've chosen our winning point, forget the rest...
      for (uint8_t idx=0; idx<past_total_masses; idx++) {
        if (pairs[idx] == i && idx != max_idx) {
          FORGET_POINT;
          taken[i]--;
        }
      }
    }

    if (taken[i] == 1) {
      for (uint8_t idx=0; idx<past_total_masses; idx++) {
        if (pairs[idx] == i) {
          // closest point matched, update trackers
          if (past_points[idx] != points[i]) {
            if (SIDE(points[i]) != SIDE(past_points[idx])) {
              // point just crossed threshold, let's reduce its history to force
              // it to spend another cycle on this side before we count the event
              histories[idx] = min(histories[idx], MIN_HISTORY);
            } else {
              histories[idx]++;
            }
            past_points[idx] = points[i];
          }
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      uint16_t h = 1;
      uint8_t sp = points[i];
      bool cross = false;
      if (cycles_since_forgotten < 5) {
        // first let's check forgotten points for a match
        for (uint8_t j=0; j<forgotten_count; j++) {
          if (forgotten_past_points[j] != UNDEF_POINT &&
              euclidean_distance(forgotten_past_points[j], points[i]) < MIN_DISTANCE) {
            sp = forgotten_starting_points[j];
            cross = forgotten_crossed[j];
            h = forgotten_histories[j];
            if (SIDE(forgotten_past_points[j]) != SIDE(points[i])) {
              h = min(h, MIN_HISTORY);
            }
            forgotten_past_points[j] = UNDEF_POINT;
            break;
          }
        }
      }
      bool row5 = false;
      if (h == 1 && AXIS(sp) == (GRID_EXTENT/2 + 1)) {
        // if point is starting in row 5, move it back to row 6
        // (giving benefit of doubt that this point appeared behind a closed door)
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
            crossed[j] = cross;
            break;
          }
        }
      }
    }
  }

  // copy forgotten data points for this frame to global scope

  if (total_masses > 0 || past_total_masses > 0) {
    memcpy(forgotten_past_points, temp_forgotten_points, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_starting_points, temp_forgotten_starting_points,
                                                         (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_histories, temp_forgotten_histories, (MAX_PEOPLE*sizeof(uint16_t)));
    memcpy(forgotten_crossed, temp_forgotten_crossed, (MAX_PEOPLE*sizeof(bool)));
    forgotten_count = temp_forgotten_count;
    cycles_since_forgotten = 0;
  } else {
    if (cycles_since_forgotten == 2) {
      for (uint8_t i=0; i<forgotten_count; i++) {
        if (AXIS(forgotten_past_points[i]) <= (LOWER_BOUND + BORDER_PADDING) ||
            AXIS(forgotten_past_points[i]) >= (UPPER_BOUND - BORDER_PADDING) ||
            NOT_AXIS(forgotten_past_points[i]) <= (LOWER_BOUND + STRICT_BORDER_PADDING) ||
            NOT_AXIS(forgotten_past_points[i]) >= (UPPER_BOUND - STRICT_BORDER_PADDING)) {
          // this point exists on the outer edge of grid, forget it after 1 frame        
          forgotten_past_points[i] = UNDEF_POINT;
        }
      }
    } else if (cycles_since_forgotten == 5) {
      memset(forgotten_past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
    }
    if (cycles_since_forgotten < 6) {
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
      memcpy(sorted_points, points, (total_masses*sizeof(uint8_t)));
      qsort(sorted_points, total_masses, sizeof(uint8_t), sort_asc);

      // print chart of what we saw in 8x8 grid
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
      SERIAL_FLUSH;
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

