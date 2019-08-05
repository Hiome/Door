#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.6.15"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE            2.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define DISTANCE_BONUS          2.5  // max extra distance a hot point can move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define CONFIDENCE_THRESHOLD    0.3  // consider a point if we're 30% confident
#define AVG_CONF_THRESHOLD      0.5  // consider a set of points if we're 50% confident
#define HIGH_CONF_THRESHOLD     0.8  // give points over 80% confidence extra benefits
#define BACKGROUND_GRADIENT     4.0
#define FOREGROUND_GRADIENT     3.0
#define MIN_TRAVEL_RATIO        0.1

#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

#ifdef R3
  #define AMG_ADDR              0x68
#else
  #define AMG_ADDR              0x69
#endif

float avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float cur_pixels_hash = 0;
float bgm = 0;
float fgm = 0;

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
uint8_t crossed[MAX_PEOPLE];
bool reverted[MAX_PEOPLE];
float past_norms[MAX_PEOPLE];
float avg_norms[MAX_PEOPLE];
float avg_bgms[MAX_PEOPLE];
float avg_fgms[MAX_PEOPLE];
uint16_t count[MAX_PEOPLE];

uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
uint8_t forgotten_crossed[MAX_PEOPLE];
bool forgotten_reverted[MAX_PEOPLE];
float forgotten_norms[MAX_PEOPLE];
float forgotten_bgms[MAX_PEOPLE];
float forgotten_fgms[MAX_PEOPLE];
uint8_t forgotten_num = 0;
uint8_t cycles_since_forgotten = 0;

#define DOOR_CLOSED 0
#define DOOR_AJAR   1
#define DOOR_OPEN   2
uint8_t door_state = DOOR_CLOSED;
uint8_t last_published_door_state = 9;
uint8_t frames_since_door_open = 0;

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

#ifdef YAXIS
  #define AXIS          y
  #define NOT_AXIS      x
  #define SIDE1(p)      ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) )
  #define SIDE2(p)      ( (p) >= (AMG88xx_PIXEL_ARRAY_SIZE/2) )
  #define SIDEL(p)      ( NOT_AXIS(p) <= (GRID_EXTENT/2) )
  #define SIDER(p)      ( NOT_AXIS(p) > (GRID_EXTENT/2) )
#else
  #define AXIS          x
  #define NOT_AXIS      y
  #define SIDE1(p)      ( (AXIS(p)) <= (GRID_EXTENT/2) )
  #define SIDE2(p)      ( (AXIS(p)) > (GRID_EXTENT/2) )
  #define SIDEL(p)      ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) )
  #define SIDER(p)      ( (p) >= (AMG88xx_PIXEL_ARRAY_SIZE/2)
  #error Double check all your code, this is untested
#endif
#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define UPPER_BOUND     ( GRID_EXTENT+1 )
#define BORDER_PADDING  ( GRID_EXTENT/4 )
#define SIDE(p)         ( SIDE1(p) ? 1 : 2 )
#define PIXEL_ACTIVE(i) ( norm_pixels[(i)] > CONFIDENCE_THRESHOLD )
#define confidence(x)   ( avg_norms[(x)]/((float)count[(x)]) )
#define avg_bgm(x)      ( avg_bgms[(x)]/((float)count[(x)]) )
#define avg_fgm(x)      ( avg_fgms[(x)]/((float)count[(x)]) )
#define totalDistance(x)( euclidean_distance(starting_points[(x)], past_points[(x)]) )
#define doorOpenedAgo(n)( frames_since_door_open < (n) && door_state == DOOR_OPEN )

// check if point is on the top or bottom edges
// xxxxxxxx
// xxxxxxxx
// xxxxxxxx
// oooooooo
// oooooooo
// xxxxxxxx
// xxxxxxxx
// xxxxxxxx
#ifdef YAXIS
  #define pointOnBorder(i)      ( (i) < (GRID_EXTENT * 3) || (i) >= (GRID_EXTENT * 5) )
  #define pointOnSmallBorder(i) ( (i) < (GRID_EXTENT * 2) || (i) >= (GRID_EXTENT * 6) )
  #define pointInMiddle(i)      ( (i) >= (GRID_EXTENT * 2) && (i) < (GRID_EXTENT * 6) )
  #define pointOnEdge(i)        ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
  #define pointOnLREdge(i)      ( NOT_AXIS(i) == 1 || NOT_AXIS(i) == GRID_EXTENT )
  #define pointOnLRBorder(i)    ( NOT_AXIS(i) <= 2 || NOT_AXIS(i) >= 7 )
#else
  #define pointOnBorder(i)      ( AXIS(i) <= 3 || AXIS(i) >= 6 )
  #define pointOnSmallBorder(i) ( AXIS(i) <= 2 || AXIS(i) >= 7 )
  #define pointInMiddle(i)      ( AXIS(i) > 2 && AXIS(i) < 7 )
  #define pointOnEdge(i)        ( AXIS(i) == 1 || AXIS(i) == 8 )
  #define pointOnLREdge(i)      ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
  #define pointOnLRBorder(i)    ( (i) < (GRID_EXTENT * 2) || (i) >= (GRID_EXTENT * 6) )
#endif

bool inEndZone(uint8_t p) {
  return pointOnBorder(p) && (pointOnSmallBorder(p) || pointOnLRBorder(p));
}

uint8_t readDoorState() {
  if (PIND & 0b00001000) {  // true if reed 3 is high (normal state)
    // door open if reed switch 4 is also high
    return PIND & 0b00010000 ? DOOR_OPEN : DOOR_AJAR;
  } else {
    return DOOR_CLOSED;
  }
}

bool checkDoorState() {
  uint8_t last_door_state = door_state;
  door_state = readDoorState();
  if (last_door_state != door_state) {
    frames_since_door_open = 0;
    SERIAL_PRINT(F("door "));
    SERIAL_PRINTLN(door_state);
  }
  if (((door_state == DOOR_CLOSED && last_published_door_state != DOOR_CLOSED) ||
       (door_state != DOOR_CLOSED && last_published_door_state != DOOR_OPEN)) &&
      publish(door_state == DOOR_CLOSED ? "d0" : "d1", "0", 0)) {
    last_published_door_state = door_state == DOOR_CLOSED ? DOOR_CLOSED : DOOR_OPEN;
  }

  return last_door_state != door_state;
}

bool checkForDoorClose(uint8_t idx) {
  // This could possibly fail if person's hand is on door knob opening door and sensor
  // detects that as a person. We'll see hand go from 1->2, and then get dropped as door
  // opens, and this if block will prevent it from reverting properly.
  if (SIDE2(past_points[idx]) && SIDE2(starting_points[idx]) && !reverted[idx] &&
      confidence(idx) > 0.6) {
    bool doorClosed = frames_since_door_open == 0;
    if (!doorClosed && door_state == DOOR_OPEN) {
      // door was previously open, did it change mid-frame?
      doorClosed = door_state != readDoorState();
      if (!doorClosed && confidence(idx) > HIGH_CONF_THRESHOLD) {
        // door might be transitioning between DOOR_AJAR -> DOOR_CLOSED, let's wait 30ms
        LOWPOWER_DELAY(SLEEP_30MS);
        doorClosed = door_state != readDoorState();
      }
    }
    return doorClosed;
  }
  return false;
}

void publishRevert(uint8_t idx) {
  char rBuf[3];
  sprintf(rBuf, "r%d", crossed[idx]);
  char wBuf[12];
  sprintf(wBuf, "%dx%dx%d", int(past_norms[idx]*100.0), int(bgm), int(fgm));
  publish(rBuf, wBuf, 20);
}

bool checkForRevert(uint8_t idx) {
  if (past_points[idx] == UNDEF_POINT || !crossed[idx] || checkForDoorClose(idx))
    return false;
  if (reverted[idx] && SIDE(past_points[idx]) == SIDE(starting_points[idx]) &&
      inEndZone(past_points[idx])) {
    // we had previously reverted this point, but it came back and made it through
    publishRevert(idx);
    reverted[idx] = false;
    return true;
  } else if (!reverted[idx] && (SIDE(past_points[idx]) != SIDE(starting_points[idx]) ||
              !inEndZone(past_points[idx]))) {
    // point disappeared in middle of grid, revert its crossing (probably noise or a hand)
    publishRevert(idx);
    reverted[idx] = true;
    return true;
  }
  return false;
}

void clearPointsAfterDoorClose() {
  if (checkDoorState()) {
    for (uint8_t i = 0; i<MAX_PEOPLE; i++) {
      checkForRevert(i);
      past_points[i] = UNDEF_POINT;
      forgotten_past_points[i] = UNDEF_POINT;
    }
    forgotten_num = 0;
    cycles_since_forgotten = MAX_EMPTY_CYCLES;
  }
}

void publishEvents() {
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT && SIDE(starting_points[i]) != SIDE(past_points[i]) &&
        (!crossed[i] || !reverted[i])) {
      float conf = confidence(i);
      if (conf <= AVG_CONF_THRESHOLD || histories[i] <= int(6.0 - conf*MIN_HISTORY))
        continue;
      // point cleanly crossed grid
      if (abs(AXIS(starting_points[i]) - AXIS(past_points[i])) >= 3) {
        uint8_t old_crossed = crossed[i];
        float abgm = avg_bgm(i);
        float afgm = avg_fgm(i);
        char meta[12];
        sprintf(meta, "%dx%dx%d", int(conf*100.0), int(abgm), int(afgm));
        if (SIDE1(past_points[i])) {
          crossed[i] = publish("1", meta, 20);
          // artificially shift starting point ahead 1 row so that
          // if user turns around now, algorithm considers it an exit
          int s = past_points[i] - GRID_EXTENT;
          starting_points[i] = max(s, 0);
        } else {
          crossed[i] = publish("2", meta, 20);
          int s = past_points[i] + GRID_EXTENT;
          starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
        }
        if (old_crossed) crossed[i] = 0;
        histories[i] = 1;
        reverted[i] = false;
        avg_norms[i] = conf + past_norms[i];
        avg_bgms[i] = abgm + bgm;
        avg_fgms[i] = afgm + fgm;
        count[i] = 2;
      }
    }
  }
}

bool pixelsChanged() {
  amg.readPixels(norm_pixels);

  float _hsh = 0.0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    _hsh += sq(norm_pixels[i] + i);
  }
  if (abs(_hsh - cur_pixels_hash) < 0.01) {
    // nothing changed, stop here
    return false;
  } else {
    cur_pixels_hash = _hsh;
    return true;
  }
}

bool normalizePixels() {
  if (!pixelsChanged()) return false;

  float cavg = 0.0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // reading is invalid if less than -20 or greater than 100,
    // but we use a smaller range than that to determine validity
    if (norm_pixels[i] < 0 || norm_pixels[i] > 80) {
      norm_pixels[i] = avg_pixels[i];
    }

    cavg += norm_pixels[i];
  }

  // calculate average
  cavg /= 64.0;

  // calculate CSM gradient
  bgm = BACKGROUND_GRADIENT;
  fgm = FOREGROUND_GRADIENT;
  float bgmt;
  float fgmt;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // difference in points among foreground
    fgmt = abs(norm_pixels[i] - cavg);
    // difference in points from background
    bgmt = abs(norm_pixels[i] - avg_pixels[i]);

    fgm = max(fgmt, fgm);
    bgm = max(bgmt, bgm);
  }

  float std;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    std = norm_pixels[i] - avg_pixels[i];

    // normalize points
    fgmt = abs(norm_pixels[i] - cavg)/fgm;
    bgmt = abs(std)/bgm;
    norm_pixels[i] = min(fgmt, bgmt);

    // update average baseline
    if (abs(std) > 4 && norm_pixels[i] > 0.6) {
      std *= 0.0001;
    } else {
      std *= 0.001;
    }
    avg_pixels[i] += std;
  }

  return true;
}

// insert element i into an array at position j, shift everything else over to make room
#define INSERT_POINT_HERE ({                            \
  if (norm_pixels[ordered_indexes[j]] > norm_pixels[i]) \
    norm_pixels[i] = norm_pixels[ordered_indexes[j]];   \
  for (int8_t x=active_pixel_count; x>j; x--) {         \
    ordered_indexes[x] = ordered_indexes[x-1];          \
  }                                                     \
  ordered_indexes[j] = i;                               \
  added = true;                                         \
  break;                                                \
})

uint8_t findCurrentPoints(uint8_t *points) {
  // sort pixels by confidence to find peaks
  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (PIXEL_ACTIVE(i)) {
      bool added = false;
      for (uint8_t j=0; j<active_pixel_count; j++) {
        float diff = norm_pixels[i] - norm_pixels[ordered_indexes[j]];
        if (diff >= 0.03) {
          INSERT_POINT_HERE;
        } else if (diff > -0.03) { // both points are similar...
          if (euclidean_distance(i, ordered_indexes[j]) <= MIN_DISTANCE) {
            // place the point that's closer to a peak in front of the other
            float d1 = 100;
            float d2 = 100;
            for (uint8_t x=0; x<j; x++) {
              d1 = euclidean_distance(i, ordered_indexes[x]);
              d2 = euclidean_distance(ordered_indexes[j], ordered_indexes[x]);
              if (d1 <= MIN_DISTANCE || d2 <= MIN_DISTANCE) break;
            }
            if (d1 <= MIN_DISTANCE || d2 <= MIN_DISTANCE) {
              // nearby peak found
              if (d1 <= d2) {
                // this point is closer to peak, so insert it first
                INSERT_POINT_HERE;
              }
            } else {
              // prefer point closer to middle
              bool sameAxis = AXIS(i) == AXIS(ordered_indexes[j]);
              if ((!sameAxis && SIDE1(i)) || (sameAxis && SIDEL(i))) {
                INSERT_POINT_HERE;
              }
            }
          } else {
            // identical points as part of a spectrum, prefer point closer to middle
            bool sameAxis = AXIS(i) == AXIS(ordered_indexes[j]);
            if ((!sameAxis && SIDE1(i)) || (sameAxis && SIDEL(i))) {
              INSERT_POINT_HERE;
            }
          }
        }
      }
      if (!added) {
        ordered_indexes[active_pixel_count] = i;
      }
      active_pixel_count++;
    }
  }

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
  if (!normalizePixels()) return;

  // find list of peaks in current frame
  uint8_t points[MAX_PEOPLE];
  uint8_t total_masses = findCurrentPoints(points);

  // pair previously seen points with new points to determine where people moved

  // "I don't know who you are or what you want, but you should know that I have a
  // very particular set of skills that make me a nightmare for people like you.
  // I will find you, I will track you, and I will turn the lights on for you."
  uint8_t taken[MAX_PEOPLE];
  memset(taken, 0, (MAX_PEOPLE*sizeof(uint8_t)));
  uint8_t pairs[MAX_PEOPLE];
  memset(pairs, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));

  // "Good luck."
  uint8_t temp_forgotten_points[MAX_PEOPLE];
  memset(temp_forgotten_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
  float temp_forgotten_norms[MAX_PEOPLE];
  uint8_t temp_forgotten_starting_points[MAX_PEOPLE];
  uint16_t temp_forgotten_histories[MAX_PEOPLE];
  uint8_t temp_forgotten_crossed[MAX_PEOPLE];
  bool temp_forgotten_reverted[MAX_PEOPLE];
  float temp_forgotten_bgms[MAX_PEOPLE];
  float temp_forgotten_fgms[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT ({                                                                   \
    if ((count[idx] > 1 && !crossed[idx]) || (crossed[idx] && checkForRevert(idx)) &&       \
          confidence(idx) > AVG_CONF_THRESHOLD) {                                           \
      temp_forgotten_points[temp_forgotten_num] = past_points[idx];                         \
      temp_forgotten_norms[temp_forgotten_num] = confidence(idx);                           \
      temp_forgotten_starting_points[temp_forgotten_num] = starting_points[idx];            \
      temp_forgotten_histories[temp_forgotten_num] = histories[idx];                        \
      temp_forgotten_crossed[temp_forgotten_num] = crossed[idx];                            \
      temp_forgotten_reverted[temp_forgotten_num] = reverted[idx];                          \
      temp_forgotten_bgms[temp_forgotten_num] = avg_bgm(idx);                               \
      temp_forgotten_fgms[temp_forgotten_num] = avg_fgm(idx);                               \
      temp_forgotten_num++;                                                                 \
    }                                                                                       \
    pairs[idx] = UNDEF_POINT;                                                               \
    past_points[idx] = UNDEF_POINT;                                                         \
  })

  uint8_t past_total_masses = 0;
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT) past_total_masses++;
  }

  if (past_total_masses > 0) {
    for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
      if (past_points[idx] != UNDEF_POINT) {
        float conf = confidence(idx);
        float max_distance = MAX_DISTANCE + conf * DISTANCE_BONUS;
        float min_score = 100;
        uint8_t min_index = UNDEF_POINT;
        for (uint8_t j=0; j<total_masses; j++) {
          // if more than a 3x difference between these points, don't pair them
          if ((norm_pixels[points[j]] < conf && norm_pixels[points[j]] * 3.0 + 0.1 < conf) ||
              (conf < norm_pixels[points[j]] && conf * 3.0 + 0.1 < norm_pixels[points[j]])) {
            continue;
          }

          float d = euclidean_distance(past_points[idx], points[j]);

          // if switching sides with low confidence, don't pair
          if (SIDE(points[j]) != SIDE(past_points[idx]) &&
              (conf < AVG_CONF_THRESHOLD || norm_pixels[points[j]]/d < MIN_TRAVEL_RATIO)) {
            continue;
          }

          if (d < max_distance) {
            float ratioP = min(norm_pixels[points[j]]/conf, conf/norm_pixels[points[j]]);
            if (crossed[idx]) ratioP /= 2.0; // ratio matters less once point is crossed
            float directionBonus = 0;
            uint8_t sp_axis = AXIS(past_points[idx]);
            uint8_t np_axis = AXIS(points[j]);
            if (SIDE1(starting_points[idx])) {
              if (crossed[idx]) {
                if (np_axis <= sp_axis) directionBonus = 0.1;
              } else if (np_axis >= sp_axis) directionBonus = 0.1;
            } else {
              if (crossed[idx]) {
                if (np_axis >= sp_axis) directionBonus = 0.1;
              } else if (np_axis <= sp_axis) directionBonus = 0.1;
            }
            float score = (d/max_distance) - ratioP - directionBonus +
                              max(AVG_CONF_THRESHOLD - norm_pixels[points[j]], 0.0);
            if (min_score - score > 0.05) {
              min_score = score;
              min_index = j;
            } else if (min_index != UNDEF_POINT && score - min_score < 0.05) {
              // score is the same, pick the point that lets this one move farthest
              float sd1 = euclidean_distance(starting_points[idx], points[j]);
              float sd2 = euclidean_distance(starting_points[idx], points[min_index]);
              if (crossed[idx] ? sd1 < sd2 : sd1 > sd2) {
                min_score = score;
                min_index = j;
              }
            }
          }
        }

        if (min_index == UNDEF_POINT) {
          // still not found...
          FORGET_POINT;
        } else {
          taken[min_index]++;
          pairs[idx] = min_index;
        }
      }
    }
  }

  for (uint8_t i=0; i<total_masses; i++) {
    if (taken[i] > 1) {
      // more than one past point is trying to match with this single current point...
      float max_score = 0.0;
      uint8_t max_idx = UNDEF_POINT;
      for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
        if (past_points[idx] != UNDEF_POINT && pairs[idx] == i) {
          float conf = confidence(idx);
          float score = conf;
          float d = euclidean_distance(past_points[idx], points[i]);
          if (score + 0.05 < AVG_CONF_THRESHOLD || (crossed[idx] &&
              (AXIS(past_points[idx]) <= min(d, 2) ||
            ((GRID_EXTENT+1) - AXIS(past_points[idx])) <= min(d, 2)))) {
            score = 0.0;
          } else {
            float directionBonus = 0;
            if (crossed[idx]) {
              if (SIDE1(starting_points[idx])) {
                if (AXIS(points[i]) <= AXIS(past_points[idx])) directionBonus = 3.0;
              } else if (AXIS(points[i]) >= AXIS(past_points[idx])) directionBonus = 3.0;
            }
            score *= (max(totalDistance(idx), 0.2) + directionBonus);
            score *= (1.0 - abs(norm_pixels[points[i]] - conf));
            score /= max(d, 0.9);
          }
          if (score - max_score > 0.05) {
            max_score = score;
            max_idx = idx;
          } else if (max_score - score < 0.05 ) {
            if (max_idx == UNDEF_POINT) {
              max_score = score;
              max_idx = idx;
            } else {
              // if 2 competing points have the same score, pick the closer one
              float d2 = euclidean_distance(past_points[max_idx], points[i]);
              if (d+0.05 < d2 || (d-d2 < 0.05 && conf > confidence(max_idx))) {
                max_score = score;
                max_idx = idx;
              }
            }
          }
        }
      }
      // once we've chosen our winning point, forget the rest...
      for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
        if (past_points[idx] != UNDEF_POINT && pairs[idx] == i && idx != max_idx) {
          FORGET_POINT;
          taken[i]--;
        }
      }
    }

    if (taken[i] == 1) {
      for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
        if (past_points[idx] != UNDEF_POINT && pairs[idx] == i) {
          // closest point matched, update trackers
          if ((pointOnEdge(points[i]) && SIDE(starting_points[idx]) == SIDE(points[i])) ||
              (crossed[idx] && inEndZone(points[i]))) {
            // always consider a point on the outer edge as just starting off
            // this will fail if a person appears right when noise is lost in middle of grid,
            // causing it to assume that's the same person and revert the revert.
            past_points[idx] = points[i];
            checkForRevert(idx);
            starting_points[idx] = points[i];
            past_norms[idx] = norm_pixels[points[i]];
            histories[idx] = 1;
            crossed[idx] = 0;
            reverted[idx] = false;
            if (pointOnEdge(points[i]) && pointOnEdge(past_points[idx])) {
              avg_norms[idx] = norm_pixels[points[i]];
              avg_bgms[idx] = bgm;
              avg_fgms[idx] = fgm;
              count[idx] = 1;
            } else {
              avg_norms[idx] = confidence(idx) + norm_pixels[points[i]];
              avg_bgms[idx] = avg_bgm(idx) + bgm;
              avg_fgms[idx] = avg_fgm(idx) + fgm;
              count[idx] = 2;
            }
          } else {
            if (past_points[idx] != points[i]) {
              if (SIDE(points[i]) != SIDE(past_points[idx])) {
                if (confidence(idx) > HIGH_CONF_THRESHOLD &&
                    norm_pixels[points[i]] > HIGH_CONF_THRESHOLD) {
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
            avg_bgms[idx] += bgm;
            avg_fgms[idx] += fgm;
            count[idx]++;
          }
          break;
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      uint16_t h = 1;
      uint8_t sp = points[i];
      uint8_t cross = 0;
      bool revert = false;
      float an = norm_pixels[points[i]];
      float b = bgm;
      float f = fgm;
      uint8_t c = 1;
      bool retroMatched = false;
      bool nobodyInFront = true;

      if (an <= AVG_CONF_THRESHOLD) continue;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        for (uint8_t j=0; j<temp_forgotten_num; j++) {
          if (temp_forgotten_points[j] != UNDEF_POINT &&
              // point cannot be more than 3x warmer than forgotten point
              ((temp_forgotten_norms[j] <= an && temp_forgotten_norms[j]*3.0 + 0.1 > an) ||
                (an < temp_forgotten_norms[j] && an*3.0 + 0.1 > temp_forgotten_norms[j]))) {
            // if switching sides with low confidence or moving too far, don't pair
            float d = euclidean_distance(temp_forgotten_points[j], points[i]);
            if (d >= MAX_DISTANCE || (SIDE(points[i]) != SIDE(temp_forgotten_points[j]) &&
                (temp_forgotten_norms[j] < AVG_CONF_THRESHOLD || an/d < MIN_TRAVEL_RATIO))) {
              continue;
            }

            sp = temp_forgotten_starting_points[j];
            h = points[i] == sp ? 1 : min(temp_forgotten_histories[j], MIN_HISTORY);
            cross = temp_forgotten_crossed[j];
            revert = temp_forgotten_reverted[j];
            an += temp_forgotten_norms[j];
            b += temp_forgotten_bgms[j];
            f += temp_forgotten_fgms[j];
            c++;
            temp_forgotten_points[j] = UNDEF_POINT;
            retroMatched = true;
            break;
          }
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        for (uint8_t j=0; j<forgotten_num; j++) {
          if (forgotten_past_points[j] != UNDEF_POINT &&
              // point cannot be more than 3x warmer than forgotten point
              ((forgotten_norms[j] <= an && forgotten_norms[j]*3.0 + 0.1 > an) ||
                (an < forgotten_norms[j] && an*3.0 + 0.1 > forgotten_norms[j]))) {
            // if switching sides with low confidence or moving too far, don't pair
            float d = euclidean_distance(forgotten_past_points[j], points[i]);
            if (d >= MAX_DISTANCE || (SIDE(points[i]) != SIDE(forgotten_past_points[j]) &&
                (forgotten_norms[j] < AVG_CONF_THRESHOLD || an/d < MIN_TRAVEL_RATIO))) {
              continue;
            }

            sp = forgotten_starting_points[j];
            h = points[i] == sp ? 1 : min(forgotten_histories[j], MIN_HISTORY);
            cross = forgotten_crossed[j];
            revert = forgotten_reverted[j];
            an += forgotten_norms[j]/((float)cycles_since_forgotten + 2.0);
            b += forgotten_bgms[j];
            f += forgotten_fgms[j];
            c++;
            forgotten_past_points[j] = UNDEF_POINT;
            retroMatched = true;
            break;
          }
        }
      }

      if (!retroMatched && pointInMiddle(sp)) {
        bool nobodyOnBoard = false;
        if (past_total_masses > 0) {
          for (uint8_t j=0; j<MAX_PEOPLE; j++) {
            if (past_points[j] != UNDEF_POINT && count[j] > 1 &&
                confidence(j) > AVG_CONF_THRESHOLD &&
                euclidean_distance(past_points[j], sp) < 5.0) {
              // there's already a person in the middle of the grid
              // so it's unlikely a new valid person just appeared in the middle
              // (person can't be running and door wasn't closed)
              nobodyOnBoard = false;
              if (SIDE1(sp)) {
                if (AXIS(past_points[j]) >= AXIS(sp)) {
                  nobodyInFront = false;
                  break;
                }
              } else if (AXIS(past_points[j]) <= AXIS(sp)) {
                nobodyInFront = false;
                break;
              }
            }
          }
        }

        // if point has mid confidence with nobody ahead...
        if (nobodyInFront && an > 0.6 && doorOpenedAgo(4) &&
            // and it is in row 5, allow it (door just opened)
            (AXIS(sp) == (GRID_EXTENT/2 + 1) || (nobodyOnBoard && an > HIGH_CONF_THRESHOLD &&
              // or row 4 if person was already through door by the sensor registered it
              AXIS(sp) == (GRID_EXTENT/2) && PIXEL_ACTIVE(sp+GRID_EXTENT)))) {
          retroMatched = true;
          sp += GRID_EXTENT;
        }
      }

      // ignore new points on side 1 immediately after door opens/closes
      if ((frames_since_door_open < 3 && SIDE1(sp)) ||
          (frames_since_door_open < 5 && door_state != DOOR_OPEN)) continue;

      // ignore new points that showed up in middle 2 rows of grid
      if (retroMatched || (pointOnBorder(sp) && (nobodyInFront || pointOnSmallBorder(sp) ||
            pointOnLRBorder(sp)))) {
        for (uint8_t j=0; j<MAX_PEOPLE; j++) {
          // look for first empty slot in past_points to use
          if (past_points[j] == UNDEF_POINT) {
            past_points[j] = points[i];
            histories[j] = h;
            starting_points[j] = sp;
            crossed[j] = cross;
            reverted[j] = revert;
            past_norms[j] = norm_pixels[points[i]];
            avg_norms[j] = an;
            avg_bgms[j] = b;
            avg_fgms[j] = f;
            count[j] = c;
            break;
          }
        }
      }
    }
  }

  // copy forgotten data points for this frame to global scope

  if (temp_forgotten_num > 0) {
    memcpy(forgotten_norms, temp_forgotten_norms, (MAX_PEOPLE*sizeof(float)));
    memcpy(forgotten_past_points, temp_forgotten_points, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_starting_points, temp_forgotten_starting_points,
                                                         (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_histories, temp_forgotten_histories, (MAX_PEOPLE*sizeof(uint16_t)));
    memcpy(forgotten_crossed, temp_forgotten_crossed, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_reverted, temp_forgotten_reverted, (MAX_PEOPLE*sizeof(bool)));
    memcpy(forgotten_bgms, temp_forgotten_bgms, (MAX_PEOPLE*sizeof(float)));
    memcpy(forgotten_fgms, temp_forgotten_fgms, (MAX_PEOPLE*sizeof(float)));
    forgotten_num = temp_forgotten_num;
    cycles_since_forgotten = 0;
    SERIAL_PRINTLN(F("saved"));
  } else if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
    cycles_since_forgotten++;
    if (cycles_since_forgotten == MAX_EMPTY_CYCLES && forgotten_num > 0) {
      // clear forgotten points list
      memset(forgotten_past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
      forgotten_num = 0;
      SERIAL_PRINTLN(F("forgot"));
    }
  }

  // publish event if any people moved through doorway yet

  publishEvents();

  if (frames_since_door_open < 5) {
    frames_since_door_open++;
  }

  // wrap up with debugging output

  #if defined(ENABLE_SERIAL) && defined(PRINT_RAW_DATA)
    if (total_masses == 0 && past_total_masses > 0) {
      SERIAL_PRINTLN(F("cleared board"));
    }
    if (total_masses > 0) {
      for (uint8_t i = 0; i<MAX_PEOPLE; i++) {
        if (past_points[i] != UNDEF_POINT) {
          SERIAL_PRINT(past_points[i]);
          SERIAL_PRINT(F(" ("));
          SERIAL_PRINT(starting_points[i]);
          SERIAL_PRINT(F("-"));
          SERIAL_PRINT(histories[i]);
          SERIAL_PRINT(F("), "));
        }
      }
      SERIAL_PRINTLN();
      SERIAL_PRINTLN(bgm);
      SERIAL_PRINTLN(fgm);

      // print chart of what we saw in 8x8 grid
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        SERIAL_PRINT(F(" "));
        if (norm_pixels[idx] < CONFIDENCE_THRESHOLD)
          SERIAL_PRINT(F("----"));
        else
          SERIAL_PRINT(norm_pixels[idx]);
        SERIAL_PRINT(F(" "));
        if (x(idx) == GRID_EXTENT) SERIAL_PRINTLN();
      }
      SERIAL_PRINTLN();
      SERIAL_FLUSH;
    }
  #endif
}

void initialize() {
  amg.begin(AMG_ADDR);

  // setup reed switches
   DDRD =  DDRD & B11100111;  // set pins 3 and 4 as inputs
  PORTD = PORTD | B00011000;  // pull pins 3 and 4 high

  LOWPOWER_DELAY(SLEEP_1S);
  publish(FIRMWARE_VERSION, "0", 20);

  // give sensor 12sec to stabilize
  LOWPOWER_DELAY(SLEEP_8S);
  LOWPOWER_DELAY(SLEEP_4S);

  memset(past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));

  amg.readPixels(avg_pixels);

  for (uint8_t k=0; k < 11; k++) {
    while (!pixelsChanged()) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (norm_pixels[i] < 0 || norm_pixels[i] > 65) continue;
      avg_pixels[i] += 0.1*(norm_pixels[i] - avg_pixels[i]);
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  processSensor();
}
