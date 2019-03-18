#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.1"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE            1.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define DISTANCE_BONUS          2.0  // max extra distance a hot point can move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        10   // max empty cycles to remember forgotten points
#define ALPHA                   0.01 // learning rate for background temp
#define SLOW_ALPHA              0.0099
#define CONFIDENCE_THRESHOLD    0.1  // consider a point if we're 10% confident
#define AVG_CONF_THRESHOLD      0.3  // consider a set of points if we're 30% confident
#define GRADIENT_THRESHOLD      7    // 2.5º temp change gives us 100% confidence of person

#define REED_PIN                3
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float cur_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
bool crossed[MAX_PEOPLE];
float past_norms[MAX_PEOPLE];
float avg_norms[MAX_PEOPLE];
uint16_t count[MAX_PEOPLE];

uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
bool forgotten_crossed[MAX_PEOPLE];
uint8_t forgotten_num = 0;
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

float euclidean_distance(uint8_t p1, uint8_t p2) {
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
  #define SIDE1(p)      ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) )
#else
  #define AXIS          x
  #define NOT_AXIS      y
  #define SIDE1(p)      ( (AXIS(p)) <= (GRID_EXTENT/2) )
  #error Double check all your code, this is untested
#endif
#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define UPPER_BOUND     ( GRID_EXTENT+1 )
#define BORDER_PADDING  ( GRID_EXTENT/4 )
#define SIDE(p)         ( SIDE1(p) ? 1 : 2 )
#define CHECK_TEMP(i)   ( norm_pixels[(i)] > CONFIDENCE_THRESHOLD )
#define CHECK_DOOR(i)   ( door_state == HIGH || SIDE1(i) )
#define PIXEL_ACTIVE(i) ( (CHECK_TEMP(i)) && (CHECK_DOOR(i)) )
#define confidence(x)   ( avg_norms[(x)]/count[(x)] )
#define totalDistance(x)( euclidean_distance(starting_points[(x)], past_points[(x)]) )

// check if point is on the top or bottom edges
// xxxxxxxx
// xxxxxxxx
// xxxxxxxx
// oooooooo
// oooooooo
// xxxxxxxx
// xxxxxxxx
// xxxxxxxx
#define pointOnBorder(i) ( AXIS(i) < (GRID_EXTENT/2) || AXIS(i) > (GRID_EXTENT/2 + 1) )

#define pointOnStrictBorder(i) ( AXIS(i) <= BORDER_PADDING ||                 \
                                  AXIS(i) >= (UPPER_BOUND - BORDER_PADDING) )

// check if point is on left or right edges
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
#define pointOnLREdge(i) ( NOT_AXIS(i) == 1 || NOT_AXIS(i) == GRID_EXTENT )

#define pointOnOuterEdge(i) ( x(i) == 1 || y(i) == 1 ||                     \
                               x(i) == GRID_EXTENT || y(i) == GRID_EXTENT )

// This macro sorts array indexes based on their corresponding values
// in desc order. For example, given an array {2, 4, 1, 0},
//   SORT_ARRAY(a, (a[i] > 0), (a[i] > a[ordered_indexes[i]]), 4)
// will return 3 and ordered_indexes will be {1, 0, 2}. I'm sorry...
#define SORT_ARRAY(src, cnd, sze) ({                                 \
  uint8_t count = 0;                                                 \
  for (uint8_t i=0; i<(sze); i++) {                                  \
    if ((cnd)) {                                                     \
      bool added = false;                                            \
      for (uint8_t j=0; j<count; j++) {                              \
        if (src[i] > src[ordered_indexes[j]]) {                      \
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

uint8_t sortPixelsByConfidence(uint8_t *ordered_indexes) {
  SORT_ARRAY(norm_pixels, (PIXEL_ACTIVE(i)), AMG88xx_PIXEL_ARRAY_SIZE);
}

uint8_t sortPastPointsByHistory(uint8_t *ordered_indexes) {
  SORT_ARRAY(histories, (histories[i] > 0), MAX_PEOPLE);
}

void publishEvents() {
  if (door_state == LOW) return; // nothing happened if door is closed

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT && histories[i] > MIN_HISTORY &&
          confidence(i) > AVG_CONF_THRESHOLD) {
      int diff = AXIS(starting_points[i]) - AXIS(past_points[i]);
      // point cleanly crossed grid
      if ((abs(diff) >= (GRID_EXTENT/2) &&
          (!pointOnLREdge(past_points[i]) || pointOnBorder(past_points[i]))) ||
         // or point barely crossed threshold but we must be at least 60% confident
         (confidence(i) > (2*AVG_CONF_THRESHOLD) && abs(diff) >= (GRID_EXTENT/2 - 1) &&
          SIDE(starting_points[i]) != SIDE(past_points[i]))) {
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
        histories[i] = 1;
        crossed[i] = true;
      }
    }
  }
}

void publishMaybeEvents(uint8_t idx) {
  if (CHECK_DOOR(past_points[idx]) && !crossed[idx] && histories[idx] > MIN_HISTORY &&
      confidence(idx) > (1.2*AVG_CONF_THRESHOLD) && totalDistance(idx) >= MAX_DISTANCE) {
    if (SIDE1(past_points[idx])) {
      publish("m1");
    } else {
      publish("m2");
    }
  }
}

bool readPixels() {
  float past_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(past_pixels, cur_pixels, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float)));

  amg.readPixels(cur_pixels);

  // return true if pixels changed
  return (memcmp(past_pixels, cur_pixels, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float))) != 0);
}

void updateAverages() {
  float m1 = 0;
  float m2 = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (SIDE1(i)) {
      m1 = max(norm_pixels[i], m1);
    } else {
      m2 = max(norm_pixels[i], m2);
    }
  }

  float adjAlpha1 = ALPHA - SLOW_ALPHA*m1;
  float adjAlpha2 = ALPHA - SLOW_ALPHA*m2;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (SIDE1(i)) {
      avg_pixels[i] += adjAlpha1 * (cur_pixels[i] - avg_pixels[i]);
    } else {
      avg_pixels[i] += adjAlpha2 * (cur_pixels[i] - avg_pixels[i]);
    }
  }
}

void normalizePixels() {
  float base_avg = amg.readThermistor();
  float curr_avg = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    curr_avg += cur_pixels[i] - avg_pixels[i];
  }
  curr_avg = curr_avg/AMG88xx_PIXEL_ARRAY_SIZE + base_avg;

  float bgm1 = GRADIENT_THRESHOLD;
  float fgm1 = GRADIENT_THRESHOLD;
  float bgm2 = GRADIENT_THRESHOLD;
  float fgm2 = GRADIENT_THRESHOLD;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    norm_pixels[i] = cur_pixels[i] - avg_pixels[i] + base_avg;
    if (SIDE1(i)) {
      bgm1 = max(sq(norm_pixels[i] - base_avg), bgm1);
      fgm1 = max(sq(norm_pixels[i] - curr_avg), fgm1);
    } else {
      bgm2 = max(sq(norm_pixels[i] - base_avg), bgm2);
      fgm2 = max(sq(norm_pixels[i] - curr_avg), fgm2);
    }
  }

  #if GRADIENT_THRESHOLD == 0
    if (fgm1 == 0 || bgm1 == 0 || fgm2 == 0 || bgm2 == 0) {
      // avoid division by 0 error by pretending whole grid is empty
      memset(norm_pixels, 0, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float)));
      return;
    }
  #endif

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (SIDE1(i)) {
      norm_pixels[i] = sq(norm_pixels[i]-curr_avg)/fgm1 * sq(norm_pixels[i]-base_avg)/bgm1;
    } else {
      norm_pixels[i] = sq(norm_pixels[i]-curr_avg)/fgm2 * sq(norm_pixels[i]-base_avg)/bgm2;
    }
    if (pointOnOuterEdge(i)) norm_pixels[i] /= 3;
  }
}

void clearTrackers() {
  memset(histories, 0, (MAX_PEOPLE*sizeof(uint16_t)));
  memset(crossed, false, (MAX_PEOPLE*sizeof(bool)));
  memset(past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
}

uint8_t findCurrentPoints(uint8_t *points) {
  // sort pixels by confidence to find peaks
  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = sortPixelsByConfidence(ordered_indexes);

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
  normalizePixels();

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
  uint8_t pairs[MAX_PEOPLE];
  memset(pairs, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));

  // "Good luck."
  uint8_t temp_forgotten_points[MAX_PEOPLE];
  memset(temp_forgotten_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
  uint8_t temp_forgotten_starting_points[MAX_PEOPLE];
  uint16_t temp_forgotten_histories[MAX_PEOPLE];
  bool temp_forgotten_crossed[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;
  uint8_t forgotten_num_frd = 0;

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT ({                                                         \
    if (confidence(idx) > AVG_CONF_THRESHOLD) {                                   \
      publishMaybeEvents(idx);                                                    \
      temp_forgotten_points[temp_forgotten_num] = past_points[idx];               \
      temp_forgotten_starting_points[temp_forgotten_num] = starting_points[idx];  \
      temp_forgotten_histories[temp_forgotten_num] = histories[idx];              \
      temp_forgotten_crossed[temp_forgotten_num] = crossed[idx];                  \
      temp_forgotten_num++;                                                       \
    }                                                                             \
    forgotten_num_frd++;                                                          \
    pairs[idx] = UNDEF_POINT;                                                     \
    past_points[idx] = UNDEF_POINT;                                               \
    histories[idx] = 0;                                                           \
    crossed[idx] = false;                                                         \
  })

  for (uint8_t i=0; i<past_total_masses; i++) {
    uint8_t idx = ordered_past_points[i];
    float min_distance = MAX_DISTANCE + sq(past_norms[idx]) * DISTANCE_BONUS;
    uint8_t min_index = UNDEF_POINT;
    for (uint8_t j=0; j<total_masses; j++) {
      float d = euclidean_distance(past_points[idx], points[j]);
      if (min_index != UNDEF_POINT && abs(d - min_distance) < 1) {
        float c1 = abs(norm_pixels[points[j]] - past_norms[idx]);
        float c2 = abs(norm_pixels[points[min_index]] - past_norms[idx]);
        if (c1 < c2) {
          min_distance = d;
          min_index = j;
        } else if (c1 == c2) {
          float sd1 = euclidean_distance(starting_points[idx], points[j]);
          float sd2 = euclidean_distance(starting_points[idx], points[min_index]);
          if (sd1 > sd2) {
            min_distance = d;
            min_index = j;
          }
        }
      } else if (d < min_distance) {
        min_distance = d;
        min_index = j;
      }
    }

    if (min_index != UNDEF_POINT && crossed[idx] &&
        (total_masses + forgotten_num_frd) < past_total_masses &&
        (AXIS(past_points[idx]) <= min(min_distance, BORDER_PADDING) ||
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
      float max_score = 0;
      uint8_t max_idx = UNDEF_POINT;
      for (uint8_t j=0; j<past_total_masses; j++) {
        uint8_t idx = ordered_past_points[j];
        if (pairs[idx] == i) {
          float score = avg_norms[idx] * totalDistance(idx);
          score /= max(euclidean_distance(past_points[idx], points[i]), 0.5);
          if (score > max_score) {
            max_score = score;
            max_idx = idx;
          } else if (score == max_score) {
            if (max_idx == UNDEF_POINT) {
              max_idx = idx;
            } else {
              // if 2 competing points have the same score, pick the closer one
              float d1 = euclidean_distance(past_points[idx], points[i]);
              float d2 = euclidean_distance(past_points[max_idx], points[i]);
              if (d1 < d2 || (d1 == d2 && past_norms[idx] > past_norms[max_idx])) {
                max_idx = idx;
              }
            }
          }
        }
      }
      // once we've chosen our winning point, forget the rest...
      for (uint8_t j=0; j<past_total_masses; j++) {
        uint8_t idx = ordered_past_points[j];
        if (pairs[idx] == i && idx != max_idx) {
          FORGET_POINT;
          taken[i]--;
        }
      }
    }

    if (taken[i] == 1) {
      for (uint8_t j=0; j<past_total_masses; j++) {
        uint8_t idx = ordered_past_points[j];
        if (pairs[idx] == i) {
          // closest point matched, update trackers
          if (past_points[idx] != points[i]) {
            if (SIDE(points[i]) != SIDE(past_points[idx])) {
              if (confidence(idx) > 0.5) {
                histories[idx]++;
              } else {
                // point just crossed threshold, let's reduce its history to force
                // it to spend another cycle on this side before we count the event
                histories[idx] = min(histories[idx] + 1, MIN_HISTORY);
              }
            } else {
              if (points[i] == starting_points[idx]) {
                histories[idx] = 1;
              } else {
                histories[idx]++;
              }
            }
            past_points[idx] = points[i];
          }
          past_norms[idx] = norm_pixels[points[i]];
          avg_norms[idx] += past_norms[idx];
          count[idx]++;
          break;
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      uint16_t h = 1;
      uint8_t sp = points[i];
      bool cross = false;
      float an = norm_pixels[points[i]];
      bool addable = false;
      if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // first let's check forgotten points for a match
        for (uint8_t j=0; j<forgotten_num; j++) {
          if (forgotten_past_points[j] != UNDEF_POINT &&
              euclidean_distance(forgotten_past_points[j], points[i]) < MAX_DISTANCE) {
            h = forgotten_histories[j];
            sp = forgotten_starting_points[j];
            cross = forgotten_crossed[j];
            an = 0;
            if (points[i] == sp) {
              h = 1;
            } else if (SIDE(forgotten_past_points[j]) != SIDE(points[i])) {
              h = min(h, MIN_HISTORY);
            }
            forgotten_past_points[j] = UNDEF_POINT;
            addable = true;
            break;
          }
        }
      }

      bool valid_mid_point = true;
      if (!pointOnStrictBorder(points[i])) {
        for (uint8_t k=0; k<total_masses; k++) {
          if (SIDE(points[i]) != SIDE(points[k]) &&
              euclidean_distance(points[k], points[i]) < MAX_DISTANCE) {
            valid_mid_point = false;
            break;
          }
        }

        if (!addable && valid_mid_point && AXIS(sp) == (GRID_EXTENT/2 + 1)) {
          // if point is starting in row 5, move it back to row 6
          // (giving benefit of doubt that this point appeared behind a closed door)
          sp += GRID_EXTENT;
          addable = true;
        }
      }
      // ignore new points that showed up in middle 2 rows of grid
      if (valid_mid_point && (addable || pointOnBorder(sp))) {
        for (uint8_t j=0; j<MAX_PEOPLE; j++) {
          // look for first empty slot in past_points to use
          if (past_points[j] == UNDEF_POINT) {
            past_points[j] = points[i];
            histories[j] = h;
            starting_points[j] = sp;
            crossed[j] = cross;
            past_norms[j] = norm_pixels[points[i]];
            avg_norms[j] = an;
            count[j] = 1;
            break;
          }
        }
      }
    }
  }

  // copy forgotten data points for this frame to global scope

  if (temp_forgotten_num > 0) {
    memcpy(forgotten_past_points, temp_forgotten_points, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_starting_points, temp_forgotten_starting_points,
                                                         (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_histories, temp_forgotten_histories, (MAX_PEOPLE*sizeof(uint16_t)));
    memcpy(forgotten_crossed, temp_forgotten_crossed, (MAX_PEOPLE*sizeof(bool)));
    forgotten_num = temp_forgotten_num;
    cycles_since_forgotten = 0;
    SERIAL_PRINTLN("saved");
  } else if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
    cycles_since_forgotten++;
    if (cycles_since_forgotten == MAX_EMPTY_CYCLES && forgotten_num > 0) {
      // clear forgotten points list
      memset(forgotten_past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
      forgotten_num = 0;
      SERIAL_PRINTLN("forgot");
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
      SERIAL_PRINT("Detected ");
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
          SERIAL_PRINT(norm_pixels[idx]);
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
          SERIAL_PRINT(" ");
          SERIAL_PRINT(norm_pixels[idx]);
          SERIAL_PRINT(" ");
        }
        if (x(idx) == GRID_EXTENT) SERIAL_PRINTLN();
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

  pinMode(REED_PIN, INPUT_PULLUP);
  door_state = 3;

  // give sensor 13sec to stabilize
  blink(26);
  // let sensor calibrate with light off
  LOWPOWER_DELAY(SLEEP_2S);

  clearTrackers();
  amg.readPixels(avg_pixels);
}

void loop() {
  checkDoorState();
  processSensor();
}

