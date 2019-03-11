#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.1"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define TEMP_BUFFER             2.0  // min increase in temp needed to register person
#define HUMAN_THRESHOLD         3.0  // min degrees needed to confidently declare human
#define MIN_DISTANCE            3.0  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MIN_PIXEL_NEIGHBORS     0    // a cell must have at least 2 active neighbors
#define SIMILAR_TEMP_DIFF       1.0  // treat 2 cells within 1º of each other as the same
#define MAX_EMPTY_CYCLES        0    // max empty cycles to remember forgotten points
#define ALPHA                   0.0001 // learning rate for background temp
#define CONFIDENCE_THRESHOLD    0.5  // consider a point if we're 50% confident
#define GRADIENT_THRESHOLD      9    // 3º temp change gives us 100% confidence of person

#define REED_PIN                3
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float cur_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float avg1 = 0;
float avg2 = 0;

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
bool crossed[MAX_PEOPLE];
uint8_t neighbor_count[MAX_PEOPLE];
float past_temps[MAX_PEOPLE];

uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
bool forgotten_crossed[MAX_PEOPLE];
float forgotten_past_temps[MAX_PEOPLE];
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
  #define SIDE1(p)      ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) )
#else
  #define AXIS          x
  #define NOT_AXIS      y
  #define SIDE1(p)      ( (AXIS(p)) <= (GRID_EXTENT/2) )
  #error Double check all your code, this is untested
#endif
#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define SIDE(p)         ( SIDE1(p) ? 1 : 2 )
#define SIDE_AVG(p)     ( SIDE1(p) ? avg1 : avg2 )
#define SIDE_PAD(i)     ( pointOnLREdge(i) || pointOnBorder(i) ? (TEMP_BUFFER/2) : 0 )
#define AVG_TEMP(i)     ( max(SIDE_AVG(i), avg_pixels[(i)]) )
#define CHECK_TEMP(i)   ( norm_pixels[(i)] > CONFIDENCE_THRESHOLD )
#define CHECK_DOOR(i)   ( door_state == HIGH || AXIS(i) <= (GRID_EXTENT/2) )
#define PIXEL_ACTIVE(i) ( (CHECK_TEMP(i)) && (CHECK_DOOR(i)) )

#define LOWER_BOUND             0
#define UPPER_BOUND             (GRID_EXTENT+1)
#define BORDER_PADDING          (GRID_EXTENT/4)
#define LENIENT_BORDER_PADDING  (BORDER_PADDING+1)
#define STRICT_BORDER_PADDING   (BORDER_PADDING-1)

// check if point is on the top or bottom edges
// xxxxxxxx
// xxxxxxxx
// oooooooo
// oooooooo
// oooooooo
// oooooooo
// xxxxxxxx
// xxxxxxxx
#define pointOnBorder(i) ( AXIS(i) <= (LOWER_BOUND + BORDER_PADDING) ||             \
                            AXIS(i) >= (UPPER_BOUND - BORDER_PADDING) )
// check if point is in middle 2 rows
// oooooooo
// oooooooo
// oooooooo
// xxxxxxxx
// xxxxxxxx
// oooooooo
// oooooooo
// oooooooo
#define pointInMiddle(i) ( AXIS(i) > (LOWER_BOUND + LENIENT_BORDER_PADDING) &&      \
                            AXIS(i) < (UPPER_BOUND - LENIENT_BORDER_PADDING) )
// check if point is on left or right edges
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
// xoooooox
#define pointOnLREdge(i) ( NOT_AXIS(i) <= (LOWER_BOUND + STRICT_BORDER_PADDING) ||  \
                            NOT_AXIS(i) >= (UPPER_BOUND - STRICT_BORDER_PADDING) )

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

float maxPixelTemp(uint8_t idx) {
  float t = cur_pixels[idx];
  if (NOT_LEFT_EDGE) {
    if (cur_pixels[MIDDLE_LEFT] > t) t = cur_pixels[MIDDLE_LEFT];
    if (NOT_TOP_EDGE && cur_pixels[TOP_LEFT] > t) t = cur_pixels[TOP_LEFT];
    if (NOT_BOTTOM_EDGE && cur_pixels[BOTTOM_LEFT] > t) t = cur_pixels[BOTTOM_LEFT];
  }
  if (NOT_RIGHT_EDGE) {
    if (cur_pixels[MIDDLE_RIGHT] > t) t = cur_pixels[MIDDLE_RIGHT];
    if (NOT_TOP_EDGE && cur_pixels[TOP_RIGHT] > t) t = cur_pixels[TOP_RIGHT];
    if (NOT_BOTTOM_EDGE && cur_pixels[BOTTOM_RIGHT] > t) t = cur_pixels[BOTTOM_RIGHT];
  }
  if (NOT_TOP_EDGE && cur_pixels[TOP_MIDDLE] > t) t = cur_pixels[TOP_MIDDLE];
  if (NOT_BOTTOM_EDGE && cur_pixels[BOTTOM_MIDDLE] > t) t = cur_pixels[BOTTOM_MIDDLE];

  return t;
}

// count how many neighboring cells are active for any given cell in the current frame
uint8_t neighborsForPixel(uint8_t idx) {
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
#define SORT_ARRAY(src, cnd, sze) ({                                 \
  uint8_t count = 0;                                                 \
  for (uint8_t i=0; i<(sze); i++) {                                  \
    if ((cnd)) {                                                     \
      bool added = false;                                            \
      for (uint8_t j=0; j<count; j++) {                              \
        if (src[i] > src[ordered_indexes[j]]) {                                                 \
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
  SORT_ARRAY(norm_pixels, (PIXEL_ACTIVE(i)), AMG88xx_PIXEL_ARRAY_SIZE);
}

uint8_t sortPastPointsByHistory(uint8_t *ordered_indexes) {
  SORT_ARRAY(histories, (histories[i] > 0), MAX_PEOPLE);
}

void publishEvents() {
  if (door_state == LOW) return; // nothing happened if door is closed

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT) {
      int diff = AXIS(starting_points[i]) - AXIS(past_points[i]);
      // point cleanly crossed grid
      if ((histories[i] > MIN_HISTORY && abs(diff) >= (GRID_EXTENT/2) &&
          (!pointOnLREdge(past_points[i]) || !pointInMiddle(past_points[i]))) ||
         // or point barely crossed threshold but it must be human with 3º temp diff
         (SIDE(starting_points[i]) != SIDE(past_points[i]) &&
          histories[i] >= MIN_HISTORY && abs(diff) >= (GRID_EXTENT/2 - 1) &&
          maxPixelTemp(past_points[i]) >= (SIDE_AVG(past_points[i]) + HUMAN_THRESHOLD))) {
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

bool readPixels() {
  float past_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(past_pixels, cur_pixels, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float)));

  amg.readPixels(cur_pixels);

  // return true if pixels changed
  return (memcmp(past_pixels, cur_pixels, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float))) != 0);
}

// calculate average for each side of grid
void calculateAverages() {
  avg1 = 0;
  avg2 = 0;
  float avg_avg1 = 0;
  float avg_avg2 = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (SIDE1(i)) {
      avg1 += cur_pixels[i];
      avg_avg1 += avg_pixels[i];
    } else {
      avg2 += cur_pixels[i];
      avg_avg2 += avg_pixels[i];
    }
  }
  avg1 /= (AMG88xx_PIXEL_ARRAY_SIZE/2);
  avg2 /= (AMG88xx_PIXEL_ARRAY_SIZE/2);
  avg_avg1 /= (AMG88xx_PIXEL_ARRAY_SIZE/2);
  avg_avg2 /= (AMG88xx_PIXEL_ARRAY_SIZE/2);

  // decrease running average in case there was a sudden drop in temp
  float d1 = 0;
  float d2 = 0;
  if (avg1 < avg_avg1) {
    d1 = avg_avg1 - avg1;
  }
  if (avg2 < avg_avg2) {
    d2 = avg_avg2 - avg2;
  }
  if (d1 > 0 || d2 > 0) {
    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (SIDE1(i)) {
        avg_pixels[i] = max(avg_pixels[i] - d1, 0);
      } else {
        avg_pixels[i] = max(avg_pixels[i] - d2, 0);
      }
    }
  }
}

void updateAverages() {
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] += ALPHA * (cur_pixels[i] - avg_pixels[i]);
  }
}

void normalizePixels() {
  float base_avg = amg.readThermistor();
  float curr_avg = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    curr_avg += cur_pixels[i] - avg_pixels[i];
  }
  curr_avg = curr_avg/AMG88xx_PIXEL_ARRAY_SIZE + base_avg;

  float bgm = GRADIENT_THRESHOLD;
  float fgm = GRADIENT_THRESHOLD;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    norm_pixels[i] = cur_pixels[i] - avg_pixels[i] + base_avg;
    bgm = max(sq(norm_pixels[i] - base_avg), bgm);
    fgm = max(sq(norm_pixels[i] - curr_avg), fgm);
  }

  #if GRADIENT_THRESHOLD == 0
    if (fgm == 0 || bgm == 0) {
      // avoid division by 0 error by pretending whole grid is empty
      memset(norm_pixels, 0, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float)));
      return;
    }
  #endif

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    norm_pixels[i] = sq(norm_pixels[i] - curr_avg)/fgm * sq(norm_pixels[i] - base_avg)/bgm;
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
  normalizePixels();
  calculateAverages();

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
  float temp_forgotten_past_temps[MAX_PEOPLE];
  uint8_t temp_forgotten_count = 0;

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT ({                                                         \
    temp_forgotten_points[temp_forgotten_count] = past_points[idx];               \
    temp_forgotten_starting_points[temp_forgotten_count] = starting_points[idx];  \
    temp_forgotten_histories[temp_forgotten_count] = histories[idx];              \
    temp_forgotten_crossed[temp_forgotten_count] = crossed[idx];                  \
    temp_forgotten_past_temps[temp_forgotten_count] = past_temps[idx];            \
    temp_forgotten_count++;                                                       \
    past_points[idx] = UNDEF_POINT;                                               \
    pairs[idx] = UNDEF_POINT;                                                     \
    histories[idx] = 0;                                                           \
    crossed[idx] = false;                                                         \
  })

  #define MATCH_POINT ({                                                          \
    float d = euclidean_distance(past_points[idx], points[j]);                    \
    float t = maxPixelTemp(points[j]);                                            \
    if (d < min_distance && abs(past_temps[idx] - t) <= (2*SIMILAR_TEMP_DIFF)) {  \
      min_distance = d;                                                           \
      min_index = j;                                                              \
    }                                                                             \
  })

  for (uint8_t i=0; i<past_total_masses; i++) {
    uint8_t idx = ordered_past_points[i];
    float min_distance = MAX_DISTANCE;
    uint8_t min_index = UNDEF_POINT;
    for (uint8_t j=0; j<total_masses; j++) {
      // match with closest available point
      if (taken[j] == 0) {
        MATCH_POINT;
      }
    }
    if (min_index == UNDEF_POINT) {
      // try again, but include already matched points in second pass
      for (uint8_t j=0; j<total_masses; j++) {
        MATCH_POINT;
      }
    }
    if (min_index == UNDEF_POINT) {
      // STILL no match... if temps are high enough, let the point move faster
      min_distance += 2;
      for (uint8_t j=0; j<total_masses; j++) {
        if (cur_pixels[points[j]] > (AVG_TEMP(points[j]) + TEMP_BUFFER + SIDE_PAD(points[j]))) {
          MATCH_POINT;
        }
      }
    }

    if (min_index != UNDEF_POINT && crossed[idx] &&
        (total_masses + temp_forgotten_count) < past_total_masses &&
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
      for (uint8_t j=0; j<past_total_masses; j++) {
        uint8_t idx = ordered_past_points[j];
        if (pairs[idx] == i) {
          // chose the point that is a combination of most clustered and longest history.
          // we're just adding up the number of neighbors a cell has and its history
          // (capped at 4) to come up with a composite score to rank competing points
          uint8_t score = neighbor_count[idx] + min(histories[idx], 4);

          if (score > max_score) {
            max_score = score;
            max_idx = idx;
          } else if (score == max_score) {
            // if 2 points have the same composite score, pick the hotter one
            if (cur_pixels[past_points[idx]] > cur_pixels[past_points[max_idx]]) {
              max_idx = idx;
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
              // point just crossed threshold, let's reduce its history to force
              // it to spend another cycle on this side before we count the event
              histories[idx] = min(histories[idx] + 1, MIN_HISTORY);
            } else {
              histories[idx]++;
              if (points[i] == starting_points[idx]) {
                histories[idx] = 1;
              }
            }
            past_points[idx] = points[i];
          }
          past_temps[idx] = maxPixelTemp(points[i]);
          neighbor_count[idx] = neighborsForPixel(points[i]);
          break;
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      uint16_t h = 1;
      uint8_t sp = points[i];
      bool cross = false;
      float pt = maxPixelTemp(points[i]);
      bool addable = false;
      if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // first let's check forgotten points for a match
        for (uint8_t j=0; j<forgotten_count; j++) {
          if (forgotten_past_points[j] != UNDEF_POINT &&
              abs(forgotten_past_temps[j] - pt) <= (2*SIMILAR_TEMP_DIFF) &&
              euclidean_distance(forgotten_past_points[j], points[i]) < MIN_DISTANCE) {
            sp = forgotten_starting_points[j];
            cross = forgotten_crossed[j];
            pt = forgotten_past_temps[j];
            h = forgotten_histories[j];
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
      if (h == 1 && AXIS(sp) == (GRID_EXTENT/2 + 1)) {
        // if point is starting in row 5, move it back to row 6
        // (giving benefit of doubt that this point appeared behind a closed door)
        sp += GRID_EXTENT;
        addable = true;
      }
      // ignore new points that showed up in middle 2 rows of grid
      // unless we found a matching forgotten point or started in row 5
      if (addable || !pointInMiddle(points[i])) {
        for (uint8_t j=0; j<MAX_PEOPLE; j++) {
          // look for first empty slot in past_points to use
          if (past_points[j] == UNDEF_POINT) {
            past_points[j] = points[i];
            starting_points[j] = sp;
            histories[j] = h;
            crossed[j] = cross;
            past_temps[j] = pt;
            neighbor_count[j] = neighborsForPixel(points[i]);
            break;
          }
        }
      }
    }
  }

  // copy forgotten data points for this frame to global scope

  if (temp_forgotten_count > 0) {
    memcpy(forgotten_past_points, temp_forgotten_points, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_starting_points, temp_forgotten_starting_points,
                                                         (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_histories, temp_forgotten_histories, (MAX_PEOPLE*sizeof(uint16_t)));
    memcpy(forgotten_crossed, temp_forgotten_crossed, (MAX_PEOPLE*sizeof(bool)));
    memcpy(forgotten_past_temps, temp_forgotten_past_temps, (MAX_PEOPLE*sizeof(float)));
    forgotten_count = temp_forgotten_count;
    cycles_since_forgotten = 0;
    SERIAL_PRINTLN("saved forgotten points");
  } else if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
    cycles_since_forgotten++;
    if (cycles_since_forgotten == MAX_EMPTY_CYCLES && forgotten_count > 0) {
      // clear forgotten points list after 5 cycles
      memset(forgotten_past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
      forgotten_count = 0;
      SERIAL_PRINTLN("cleared forgotten points");
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
        if (!(NOT_RIGHT_EDGE)) SERIAL_PRINTLN();
      }
      SERIAL_PRINTLN();
//      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
//        SERIAL_PRINT(" ");
//        SERIAL_PRINT(cur_pixels[idx]);
//        SERIAL_PRINT(" ");
//        if (!(NOT_RIGHT_EDGE)) SERIAL_PRINTLN();
//      }
//      SERIAL_PRINTLN();
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

  SERIAL_PRINTLN("napping for 16s to calibrate AMG8833...");
//  LOWPOWER_DELAY(SLEEP_8S);
//  LOWPOWER_DELAY(SLEEP_8S);

  amg.readPixels(avg_pixels);
}

void loop() {
  checkDoorState();
  processSensor();
}

