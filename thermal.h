#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
  #define TEST_PCBA           // uncomment to print raw amg sensor data
#endif

#define FIRMWARE_VERSION        "V0.6.24"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE            2.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define DISTANCE_BONUS          2.5  // max extra distance a hot point can move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define CONFIDENCE_THRESHOLD    0.2  // consider a point if we're 20% confident
#define AVG_CONF_THRESHOLD      0.3  // consider a set of points if we're 30% confident
#define BACKGROUND_GRADIENT     2.0
#define FOREGROUND_GRADIENT     2.0
#define T_THRESHOLD             3    // min squared standard deviations of change for a pixel
#define MIN_NEIGHBORS           3    // min size of halo effect to consider a point legit
#define NUM_STD_DEV             2.0  // max num of std dev to include in trimmed average
#define MIN_TRAVEL_RATIO        0.2

#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;

#ifdef R3
  #define AMG_ADDR              0x68
#else
  #define AMG_ADDR              0x69
#endif

uint16_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint16_t std_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
bool pos_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float cur_pixels_hash = 0;
float global_bgm = 0;
float global_fgm = 0;

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
#define PIXEL_ACTIVE(i) ( norm_pixels[(i)] > CONFIDENCE_THRESHOLD )
#define bgPixel(x)      ( ((float)avg_pixels[(x)])/1000.0 )
#define stdPixel(x)     ( ((float)std_pixels[(x)])/1000.0 )
#define doorOpenedAgo(n)( frames_since_door_open < (n) && door_state == DOOR_OPEN )

uint8_t SIDE(uint8_t p) {
  return SIDE1(p) ? 1 : 2;
}

bool MAHALANBOIS(uint8_t x) {
  float d = norm_pixels[(x)]-bgPixel(x);
  return sq(d)/stdPixel(x) >= T_THRESHOLD;
}

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

uint8_t forgotten_num = 0;
uint8_t cycles_since_forgotten = MAX_EMPTY_CYCLES;

#define DOOR_CLOSED 0
#define DOOR_AJAR   1
#define DOOR_OPEN   2
uint8_t door_state = DOOR_OPEN;
uint8_t last_published_door_state = 9;
uint8_t frames_since_door_open = 0;

#define FRD_EVENT         0
#define MAYBE_EVENT       1
#define DOOR_CLOSE_EVENT  2

uint8_t readDoorState() {
  if (PIND & 0b00001000) {  // true if reed 3 is high (normal state)
    // door open if reed switch 4 is also high
    return PIND & 0b00010000 ? DOOR_OPEN : DOOR_AJAR;
  } else {
    return DOOR_CLOSED;
  }
}

typedef struct Person {
  uint8_t   past_position;
  uint8_t   starting_position;
  uint16_t  history;
  uint8_t   crossed;
  bool      reverted;
  bool      positive;
  float     past_conf;
  float     confidence;
  float     total_conf;
  float     total_bgm;
  float     total_fgm;
  uint16_t  count;

  bool      real() { return past_position != UNDEF_POINT; };
  void      updateConfidence() { confidence = total_conf/((float)count); };
  float     bgm() { return total_bgm/((float)count); };
  float     fgm() { return total_fgm/((float)count); };

  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };

  float     totalDistance() { return euclidean_distance(starting_position, past_position); };

  void revert() {
    char rBuf[3];
    sprintf(rBuf, "r%d", crossed);
    char wBuf[17];
    sprintf(wBuf, "%dx%dx%d",
      int(past_conf*100.0),
      int(global_bgm*100.0),
      int(global_fgm*100.0)
    );
    publish(rBuf, wBuf, RETRY_COUNT);
  };

  bool checkForDoorClose() {
    // This could possibly fail if person's hand is on door knob opening door and sensor
    // detects that as a person. We'll see hand go from 1->2, and then get dropped as door
    // opens, and this if block will prevent it from reverting properly.
    if (SIDE2(past_position) && confidence > 0.6) {
      bool doorClosed = frames_since_door_open == 0;
      if (!doorClosed && door_state == DOOR_OPEN) {
        // door was previously open, did it change mid-frame?
        doorClosed = door_state != readDoorState();
      }
      return doorClosed;
    }
    return false;
  };

  bool checkForRevert() {
    if (!real() || !crossed) return false;
  
    if (reverted && side() == starting_side() &&
        (inEndZone(past_position) || checkForDoorClose())) {
      // we had previously reverted this point, but it came back and made it through
      revert();
      reverted = false;
      return true;
    } else if (!reverted && (side() != starting_side() ||
                (!inEndZone(past_position) && !checkForDoorClose()))) {
      // point disappeared in middle of grid, revert its crossing (probably noise or a hand)
      revert();
      reverted = true;
      return true;
    }
    return false;
  };

  bool publishPacket(uint8_t eventType) {
    if (abs(AXIS(starting_position) - AXIS(past_position)) >= 3) {
      uint8_t old_crossed = crossed;
      float abgm = bgm();
      float afgm = fgm();
      char meta[19];
      sprintf(meta, "%s%dx%dx%dx%d",
        positive ? "+" : "-",
        int(confidence*100.0),
        int(abgm*100.0),
        int(afgm*100.0),
        history
      );
      if (SIDE1(past_position)) {
        if (eventType == FRD_EVENT) {
          crossed = publish(door_state == DOOR_OPEN ? "1" : "a1", meta, RETRY_COUNT);
          // artificially shift starting point ahead 1 row so that
          // if user turns around now, algorithm considers it an exit
          int s = past_position - GRID_EXTENT;
          starting_position = max(s, 0);
        } else if (eventType == MAYBE_EVENT) {
          publish("m1", meta, RETRY_COUNT);
          return true;
        }
      } else {
        if (eventType == FRD_EVENT) {
          crossed = publish(door_state == DOOR_OPEN ? "2" : "a2", meta, RETRY_COUNT);
          int s = past_position + GRID_EXTENT;
          starting_position = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
        } else if (eventType == MAYBE_EVENT) {
          publish("m2", meta, RETRY_COUNT);
          return true;
        } else if (eventType == DOOR_CLOSE_EVENT) {
          publish("2", meta, RETRY_COUNT);
          return true;
        }
      }
      if (old_crossed) crossed = 0;
      history = 1;
      reverted = false;
      total_conf = confidence + past_conf;
      total_bgm = abgm + global_bgm;
      total_fgm = afgm + global_fgm;
      count = 2;
      updateConfidence();
      return true;
    }
    return false;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue.
  // Hopefully it is not and this can be deleted!
  bool publishMaybeEvent() {
    if (real() && starting_side() != side() && (!crossed || !reverted) &&
          history >= MIN_HISTORY && confidence > AVG_CONF_THRESHOLD) {
      return publishPacket(MAYBE_EVENT);
    }
    return false;
  };
  
  // same as publishEvent, but only called when door was shut to ensure we don't cut off
  // a point that just didn't hit its min history requirement going from 1->2
  bool checkForCrossing() {
    // if this is called, we know door just changed
    if (door_state != DOOR_OPEN && real() &&
        SIDE1(starting_position) && SIDE2(past_position) &&
        (!crossed || !reverted) && history >= MIN_HISTORY && confidence > 0.8) {
      return publishPacket(DOOR_CLOSE_EVENT);
    }
    return false;
  };

  void forget() {
    checkForRevert();
    checkForCrossing() || publishMaybeEvent();
  };
};

Person UNDEF_PERSON = {UNDEF_POINT};

Person known_people[MAX_PEOPLE];
Person forgotten_people[MAX_PEOPLE];

void publishEvents() {
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    Person p = known_people[i];
    if (p.real() && p.starting_side() != p.side() && (!p.crossed || !p.reverted) &&
        p.confidence > AVG_CONF_THRESHOLD &&
        p.history > int(roundf(6.0 - p.confidence*MIN_HISTORY)) &&
        p.publishPacket(FRD_EVENT)) {
      known_people[i] = p; // update known_people array
    }
  }
}

bool checkDoorState() {
  uint8_t last_door_state = door_state;
  door_state = readDoorState();

  if (last_door_state != door_state) {
    frames_since_door_open = 0;
    SERIAL_PRINT(F("d "));
    SERIAL_PRINTLN(door_state);
  }

  if (((door_state == DOOR_CLOSED && last_published_door_state != DOOR_CLOSED) ||
       (door_state != DOOR_CLOSED && last_published_door_state != DOOR_OPEN)) &&
      publish(door_state == DOOR_CLOSED ? "d0" : "d1", "0", 0)) {
    last_published_door_state = door_state == DOOR_CLOSED ? DOOR_CLOSED : DOOR_OPEN;
  }

  return last_door_state != door_state;
}

void clearPointsAfterDoorClose() {
  if (checkDoorState()) {
    for (uint8_t i = 0; i<MAX_PEOPLE; i++) {
      known_people[i].forget();
      known_people[i] = UNDEF_PERSON;
      forgotten_people[i] = UNDEF_PERSON;
    }
    forgotten_num = 0;
    cycles_since_forgotten = MAX_EMPTY_CYCLES;
  }
}

float trimMean(float sum, float sq_sum, uint8_t side) {
  float mean = sum/32.0;
  float variance = sq_sum - sq(sum)/32.0;
  variance = NUM_STD_DEV * sqrt(variance);
  float cavg = 0.0;
  float total = 0.0;
  for (uint8_t i=(side==1 ? 0 : 32); i<(side==1 ? 32 : AMG88xx_PIXEL_ARRAY_SIZE); i++) {
    if (norm_pixels[i] > (mean - variance) && norm_pixels[i] < (mean + variance)) {
      cavg += norm_pixels[i];
      total++;
    }
  }
  if (total > 0) {
    return cavg/(float)total;
  } else {
    // somehow variance was perfectly 0, pretty much impossible to get here
    return mean;
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

  // ignore points that don't have enough neighbors
  bool ignorable[AMG88xx_PIXEL_ARRAY_SIZE];
  float x_sum1 = 0.0;
  float sq_sum1 = 0.0;
  float x_sum2 = 0.0;
  float sq_sum2 = 0.0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // reading is invalid if less than -20 or greater than 100,
    // but we use a smaller range than that to determine validity
    if (norm_pixels[i] < 0 || norm_pixels[i] > 80) {
      norm_pixels[i] = bgPixel(i);
    }

    if (SIDE1(i)) {
      x_sum1 += norm_pixels[i];
      sq_sum1 += sq(norm_pixels[i]);
    } else {
      x_sum2 += norm_pixels[i];
      sq_sum2 += sq(norm_pixels[i]);
    }

    uint8_t neighbors = 0;
    if (MAHALANBOIS(i)) {
      if (AXIS(i) > 1) {
        // not top row
        if (MAHALANBOIS(i - GRID_EXTENT)) neighbors++;
        if (NOT_AXIS(i) > 1 && MAHALANBOIS(i-(GRID_EXTENT+1))) neighbors++;
        if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i-(GRID_EXTENT-1))) neighbors++;
      }
      if (AXIS(i) < GRID_EXTENT) {
        // not bottom row
        if (MAHALANBOIS(i + GRID_EXTENT)) neighbors++;
        if (NOT_AXIS(i) > 1 && MAHALANBOIS(i+(GRID_EXTENT-1))) neighbors++;
        if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i+(GRID_EXTENT+1))) neighbors++;
      }
      if (NOT_AXIS(i) > 1 && MAHALANBOIS(i-1)) neighbors++;
      if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i+1)) neighbors++;
    }

    ignorable[i] = neighbors < MIN_NEIGHBORS;
  }

  // calculate trimmed average
  float cavg1 = trimMean(x_sum1, sq_sum1, 1);
  float cavg2 = trimMean(x_sum2, sq_sum2, 2);

  #ifdef TEST_PCBA
    for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
      SERIAL_PRINT(F(" "));
      SERIAL_PRINT(norm_pixels[idx]);
      SERIAL_PRINT(F(" "));
      if (x(idx) == GRID_EXTENT) SERIAL_PRINTLN();
    }
    SERIAL_PRINTLN();
    SERIAL_PRINT("thermistor ");
    SERIAL_PRINTLN(amg.readThermistor());
    SERIAL_PRINT("mean ");
    SERIAL_PRINTLN((cavg1 + cavg2)/2);
  #endif

  // calculate CSM gradient
  float fgm1 = FOREGROUND_GRADIENT;
  float fgm2 = FOREGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (!ignorable[i]) {
      // difference in points from foreground
      float fgmt1 = abs(norm_pixels[i] - cavg1);
      float fgmt2 = abs(norm_pixels[i] - cavg2);
      if (fgmt1 < fgmt2) {
        fgm1 = max(fgmt1, fgm1);
      } else {
        fgm2 = max(fgmt2, fgm2);
      }
    }
  }

  global_fgm = max(fgm1, fgm2);

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (!ignorable[i]) {
      float fgmt1 = (norm_pixels[i] - cavg1)/fgm1;
      float fgmt2 = (norm_pixels[i] - cavg2)/fgm2;
      // pick the smaller of the 2 foreground gradients
      if (abs(fgmt1) > abs(fgmt2)) fgmt1 = fgmt2;
      if (fgmt1 < CONFIDENCE_THRESHOLD) ignorable[i] = true;
    }
  }

  global_bgm = BACKGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (!ignorable[i]) {
      // difference in points from background
      float bgmt = abs(norm_pixels[i] - bgPixel(i));
      global_bgm = max(bgmt, global_bgm);
    }
  }

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float std = norm_pixels[i] - bgPixel(i);

    // normalize points
    if (ignorable[i]) {
      norm_pixels[i] = 0.0;
      pos_pixels[i] = false;
    } else {
      float fgmt1 = (norm_pixels[i] - cavg1)/fgm1;
      float fgmt2 = (norm_pixels[i] - cavg2)/fgm2;
      // pick the smaller of the 2 foreground gradients
      if (abs(fgmt1) > abs(fgmt2)) fgmt1 = fgmt2;
      float bgmt = std/global_bgm;

      if (abs(fgmt1) < abs(bgmt)) {
        norm_pixels[i] = min(abs(fgmt1), 1);
        pos_pixels[i] = fgmt1 >= 0;
      } else {
        norm_pixels[i] = abs(bgmt);
        pos_pixels[i] = bgmt >= 0;
      }
    }

    // update average baseline
    float var = 0.1*(sq(std) - stdPixel(i));
    // implicit alpha of 0.001
    if (frames_since_door_open < 5 && norm_pixels[i] < AVG_CONF_THRESHOLD) {
      // door just changed, increase alpha to 0.2 to adjust quickly to new background
      std *= 200.0;
      // but don't let this unfairly impact the point's variance, lower alpha to 0.0001
      var *= 0.1;
    } else if (norm_pixels[i] > 0.6) {
      // looks like a person, lower alpha to 0.0001
      std *= 0.1;
      var *= 0.1;
    } else if (global_fgm < 2.01 && global_bgm < 2.01 &&
        norm_pixels[i] < AVG_CONF_THRESHOLD) {
      // nothing going on, increase alpha to 0.01
      std *= 10.0;
    }
    avg_pixels[i] += ((int)roundf(std));
    int16_t s = ((int)roundf(var));
    if (s < 0 && -s > std_pixels[i] - 100)
      std_pixels[i] = 100;
    else
      std_pixels[i] += s;
  }

  return true;
}

// insert element i into an array at position j, shift everything else over to make room
#define INSERT_POINT_HERE ({                            \
  if (norm_pixels[ordered_indexes[j]] > norm_pixels[i]) \
    norm_pixels[i] = norm_pixels[ordered_indexes[j]];   \
  for (int8_t x=k; x>j; x--) {                          \
    ordered_indexes[x] = ordered_indexes[x-1];          \
  }                                                     \
  ordered_indexes[j] = i;                               \
  added = true;                                         \
  break;                                                \
})

uint8_t findCurrentPoints(uint8_t *points) {
  // sort pixels by confidence to find peaks
  uint8_t ordered_indexes_temp[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
  // sort by confidence
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (PIXEL_ACTIVE(i)) {
      bool added = false;
      for (uint8_t j=0; j<active_pixel_count; j++) {
        if (norm_pixels[i] > norm_pixels[ordered_indexes_temp[j]]) {
          for (int8_t x=active_pixel_count; x>j; x--) {
            ordered_indexes_temp[x] = ordered_indexes_temp[x-1];
          }
          ordered_indexes_temp[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        ordered_indexes_temp[active_pixel_count] = i;
      }
      active_pixel_count++;
    }
  }

  // reorder based on peaks
  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  for (uint8_t k=0; k<active_pixel_count; k++) {
    uint8_t i = ordered_indexes_temp[k];
    bool added = false;
    for (uint8_t j=0; j<k; j++) {
      float diff = norm_pixels[i] - norm_pixels[ordered_indexes[j]];
      if (abs(diff) < 0.11) { // both points are similar...
        if (j > 0 && euclidean_distance(i, ordered_indexes[j]) <= MIN_DISTANCE) {
          // place the point that's closer to a peak in front of the other
          float d1 = 100;
          float d2 = 100;
          for (uint8_t x=0; x<j; x++) {
            d1 = euclidean_distance(i, ordered_indexes[x]);
            d2 = euclidean_distance(ordered_indexes[j], ordered_indexes[x]);
            if (d1 <= MIN_DISTANCE || d2 <= MIN_DISTANCE) break;
          }
          // nearby peak found
          if (d1 < d2) {
            // this point is closer to peak, so insert it first
            INSERT_POINT_HERE;
          }
        } else {
          // identical points as part of a spectrum, prefer point closer to middle
          bool sameAxis = AXIS(i) == AXIS(ordered_indexes[j]);
          if ((sameAxis && ((SIDEL(i) && i > ordered_indexes[j]) ||
                            (SIDER(i) && i < ordered_indexes[j]))) ||
              (!sameAxis && ((SIDE1(i) && i > ordered_indexes[j]) ||
                             (SIDE2(i) && i < ordered_indexes[j])))) {
            INSERT_POINT_HERE;
          }
        }
      }
    }
    if (!added) {
      ordered_indexes[k] = i;
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

void forget_person(uint8_t idx, Person *temp_forgotten_people, uint8_t *pairs,
                    uint8_t &temp_forgotten_num) {
  Person p = known_people[idx];
  if (((p.count > 1 && !p.crossed) || (p.crossed && p.checkForRevert())) &&
        p.confidence > AVG_CONF_THRESHOLD) {
    p.publishMaybeEvent();
    temp_forgotten_people[temp_forgotten_num] = p;
    temp_forgotten_num++;
  }
  pairs[idx] = UNDEF_POINT;
  known_people[idx] = UNDEF_PERSON;
}

bool remember_person(Person p, uint8_t point, uint16_t &h, uint8_t &sp, uint8_t &cross,
                      bool &revert, float &an, bool &pos, float &b, float &f, uint8_t &c) {
  if (p.real() && pos == p.positive) {
    float scale_factor = 1.0;
    float past_bgm = p.bgm();
    if (global_bgm < past_bgm) {
      scale_factor = global_bgm/past_bgm;
    }
    if (p.crossed && scale_factor < 0.7 && inEndZone(p.past_position) &&
        p.starting_side() == p.side()) {
      // significant drop in gradient and point made it to end zone,
      // most likely point left
      return false;
    }
    float adjusted_conf = an * scale_factor;
    // point cannot be more than 3x warmer than forgotten point
    if ((p.confidence < adjusted_conf && p.confidence * 3.0 < adjusted_conf) ||
        (adjusted_conf < p.confidence && adjusted_conf * 3.0 < p.confidence)) {
      return false;
    }
    // if switching sides with low confidence or moving too far, don't pair
    float d = euclidean_distance(p.past_position, point);
    if (d >= MAX_DISTANCE || (SIDE(point) != p.side() &&
        (p.confidence < AVG_CONF_THRESHOLD || adjusted_conf/d < MIN_TRAVEL_RATIO))) {
      return false;
    }

    sp = p.starting_position;
    h = point == sp ? 1 : min(p.history, MIN_HISTORY);
    cross = p.crossed;
    revert = p.reverted;
    an += p.confidence;
    b += past_bgm;
    f += p.fgm();
    c++;
    return true;
  }
  return false;
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
  uint8_t pairs[MAX_PEOPLE];

  // "Good luck."
  Person temp_forgotten_people[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;

  uint8_t past_total_masses = 0;

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    taken[i] = 0;
    pairs[i] = UNDEF_POINT;
    temp_forgotten_people[i] = UNDEF_PERSON;

    if (known_people[i].real()) past_total_masses++;
  }

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT (forget_person(idx, temp_forgotten_people, pairs, temp_forgotten_num))

  if (past_total_masses > 0) {
    for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
      Person p = known_people[idx];
      if (p.real()) {
        uint8_t min_index = UNDEF_POINT;
        float scale_factor = 1.0;
        float past_bgm = p.bgm();
        if (global_bgm < past_bgm) {
          scale_factor = global_bgm/past_bgm;
        }
        if (!p.crossed || scale_factor > 0.7 || !inEndZone(p.past_position) ||
            p.starting_side() != p.side()) {
          // gradient is still roughly similar or point never made it to end zone anyway,
          // so we'll tolerate a significant drop
          float max_distance = MAX_DISTANCE + p.confidence * DISTANCE_BONUS;
          float min_score = 100;
          for (uint8_t j=0; j<total_masses; j++) {
            float c = norm_pixels[points[j]] * scale_factor;
            // if confidence doesn't share same sign
            if (pos_pixels[points[j]] != p.positive ||
            // or if more than a 3x difference between these points, don't pair them
                (c < p.confidence && c * 3.0 < p.confidence) ||
                (p.confidence < c && p.confidence * 3.0 < c)) {
              continue;
            }
  
            float d = euclidean_distance(p.past_position, points[j]);
  
            // if switching sides with low confidence, don't pair
            if (SIDE(points[j]) != p.side() &&
                (p.confidence < AVG_CONF_THRESHOLD || c/d < MIN_TRAVEL_RATIO)) {
              continue;
            }
  
            if (d < max_distance) {
              float ratioP = min(c/p.confidence, p.confidence/c);
              if (p.crossed) ratioP /= 2.0; // ratio matters less once point is crossed
              float directionBonus = 0;
              uint8_t sp_axis = AXIS(p.past_position);
              uint8_t np_axis = AXIS(points[j]);
              if (SIDE1(p.starting_position)) {
                if (p.crossed) {
                  if (np_axis <= sp_axis) directionBonus = 0.1;
                } else if (np_axis >= sp_axis) directionBonus = 0.1;
              } else {
                if (p.crossed) {
                  if (np_axis >= sp_axis) directionBonus = 0.1;
                } else if (np_axis <= sp_axis) directionBonus = 0.1;
              }
              float score = (d/max_distance) - ratioP - directionBonus +
                                max(AVG_CONF_THRESHOLD - c, 0.0);
              if (min_score - score > 0.05) {
                min_score = score;
                min_index = j;
              } else if (min_index != UNDEF_POINT && score - min_score < 0.05) {
                // score is the same, pick the point that lets this one move farthest
                float sd1 = euclidean_distance(p.starting_position, points[j]);
                float sd2 = euclidean_distance(p.starting_position, points[min_index]);
                if (p.crossed ? sd1 < sd2 : sd1 > sd2) {
                  min_score = score;
                  min_index = j;
                }
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
        Person p = known_people[idx];
        if (p.real() && pairs[idx] == i) {
          float score = p.confidence;
          float d = euclidean_distance(p.past_position, points[i]);
          if (score + 0.05 < AVG_CONF_THRESHOLD || (p.crossed &&
              (AXIS(p.past_position) <= min(d, 2) ||
            ((GRID_EXTENT+1) - AXIS(p.past_position)) <= min(d, 2)))) {
            score = 0.0;
          } else {
            score = sq(score);
            float directionBonus = 0;
            if (p.crossed) {
              if (SIDE1(p.starting_position)) {
                if (AXIS(points[i]) <= AXIS(p.past_position)) directionBonus = 3.0;
              } else if (AXIS(points[i]) >= AXIS(p.past_position)) directionBonus = 3.0;
            }
            float td = p.totalDistance();
            if (td < 1) {
              if (p.count == 1) td = 1.0;
              else td = 1.0/((float)(p.count - 1));
            }
            score *= td + directionBonus;
            score *= (1.0 - abs(norm_pixels[points[i]] - p.past_conf));
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
              float d2 = euclidean_distance(known_people[max_idx].past_position, points[i]);
              if (d+0.05 < d2 ||
                  (d-d2 < 0.05 && p.confidence > known_people[max_idx].confidence)) {
                max_score = score;
                max_idx = idx;
              }
            }
          }
        }
      }
      // once we've chosen our winning point, forget the rest...
      for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
        if (known_people[idx].real() && pairs[idx] == i && idx != max_idx) {
          FORGET_POINT;
          taken[i]--;
        }
      }
    }

    if (taken[i] == 1) {
      for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
        Person p = known_people[idx];
        if (p.real() && pairs[idx] == i) {
          // closest point matched, update trackers
          if (pointOnEdge(points[i]) && p.starting_side() == SIDE(points[i])) {
            // always consider a point on the outer edge as just starting off
            p.past_position = points[i];
            p.past_conf = norm_pixels[points[i]];
            p.checkForRevert();
            // reset everything
            p.starting_position = points[i];
            p.history = 1;
            p.crossed = 0;
            p.reverted = false;
            if (pointOnEdge(points[i]) && pointOnEdge(p.past_position)) {
              p.total_conf = p.past_conf;
              p.total_bgm = global_bgm;
              p.total_fgm = global_fgm;
              p.count = 1;
            } else {
              p.total_conf = p.confidence + p.past_conf;
              p.total_bgm = p.bgm() + global_bgm;
              p.total_fgm = p.fgm() + global_fgm;
              p.count = 2;
            }
          } else {
            if (p.past_position != points[i]) {
              if (SIDE(points[i]) != p.side()) {
                p.history = min(p.history + 1, MIN_HISTORY);
              } else if (points[i] == p.starting_position) {
                p.history = 1;
              } else {
                p.history++;
              }
              p.past_position = points[i];
            }
            p.past_conf = norm_pixels[points[i]];
            p.total_conf += p.past_conf;
            p.total_bgm += global_bgm;
            p.total_fgm += global_fgm;
            p.count++;
          }
          p.updateConfidence();
          known_people[idx] = p;
          break;
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      uint16_t h = 1;
      uint8_t sp = points[i];
      uint8_t cross = 0;
      bool revert = false;
      float an = norm_pixels[sp];
      bool pos = pos_pixels[sp];
      float b = global_bgm;
      float f = global_fgm;
      uint8_t c = 1;
      bool retroMatched = false;
      bool nobodyInFront = true;

      if (an <= AVG_CONF_THRESHOLD) continue;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        for (uint8_t j=0; j<temp_forgotten_num; j++) {
          if (remember_person(temp_forgotten_people[j], points[i],
                              h, sp, cross, revert, an, pos, b, f, c)) {
            retroMatched = true;
            temp_forgotten_people[j] = UNDEF_PERSON;
            break;
          }
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        for (uint8_t j=0; j<forgotten_num; j++) {
          if (remember_person(forgotten_people[j], points[i],
                              h, sp, cross, revert, an, pos, b, f, c)) {
            retroMatched = true;
            forgotten_people[j] = UNDEF_PERSON;
            break;
          }
        }
      }

      if (!retroMatched && pointInMiddle(sp)) {
        bool nobodyOnBoard = false;
        if (past_total_masses > 0) {
          for (uint8_t j=0; j<MAX_PEOPLE; j++) {
            Person p = known_people[j];
            if (p.real() && p.past_conf > 0.8 &&
                (p.count > 1 || pointOnEdge(p.past_position)) &&
                p.confidence > 0.8 && p.fgm() > (FOREGROUND_GRADIENT + 0.1)) {
              // there's already a person in the middle of the grid
              // so it's unlikely a new valid person just appeared in the middle
              // (person can't be running and door wasn't closed)
              nobodyOnBoard = false;
              if (SIDE1(sp)) {
                if (AXIS(p.past_position) >= AXIS(sp)) {
                  nobodyInFront = false;
                  break;
                }
              } else if (AXIS(p.past_position) <= AXIS(sp)) {
                nobodyInFront = false;
                break;
              }
            }
          }
        }

        // if point has mid confidence with nobody ahead...
        if (nobodyInFront && an > 0.8 && doorOpenedAgo(4) &&
            // and it is in row 5, allow it (door just opened)
            (AXIS(sp) == (GRID_EXTENT/2 + 1) || (nobodyOnBoard && an > 0.9 &&
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
          if (!known_people[j].real()) {
            Person p;
            p.past_position = points[i];
            p.history = h;
            p.starting_position = sp;
            p.crossed = cross;
            p.reverted = revert;
            p.past_conf = norm_pixels[points[i]];
            p.positive = pos;
            p.total_conf = an;
            p.total_bgm = b;
            p.total_fgm = f;
            p.count = c;
            p.updateConfidence();
            known_people[j] = p;
            break;
          }
        }
      }
    }
  }

  // copy forgotten data points for this frame to global scope

  if (temp_forgotten_num > 0) {
    for (uint8_t i=0; i<MAX_PEOPLE; i++) {
      forgotten_people[i] = temp_forgotten_people[i];
    }
    forgotten_num = temp_forgotten_num;
    cycles_since_forgotten = 0;
    SERIAL_PRINTLN(F("s"));
  } else if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
    cycles_since_forgotten++;
    if (cycles_since_forgotten == MAX_EMPTY_CYCLES && forgotten_num > 0) {
      // clear forgotten points list
      for (uint8_t i=0; i<MAX_PEOPLE; i++) {
        forgotten_people[i] = UNDEF_PERSON;
      }
      forgotten_num = 0;
      SERIAL_PRINTLN(F("f"));
    }
  }

  // publish event if any people moved through doorway yet

  publishEvents();

  if (frames_since_door_open < 5) {
    frames_since_door_open++;
  }

  // wrap up with debugging output

  #ifdef PRINT_RAW_DATA
    if (total_masses == 0 && past_total_masses > 0) {
      SERIAL_PRINTLN(F("cleared board"));
    }
    if (total_masses > 0) {
      for (uint8_t i = 0; i<MAX_PEOPLE; i++) {
        Person p = known_people[i];
        if (p.real()) {
          SERIAL_PRINT(p.past_position);
          SERIAL_PRINT(F(" ("));
          SERIAL_PRINT(p.starting_position);
          SERIAL_PRINT(F("-"));
          SERIAL_PRINT(p.history);
          SERIAL_PRINT(F("),"));
        }
      }
      SERIAL_PRINTLN();

      SERIAL_PRINTLN(global_bgm);
      SERIAL_PRINTLN(global_fgm);
      float avg_avg = 0;
      float avg_std = 0;
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        avg_avg += bgPixel(idx);
        avg_std += stdPixel(idx);
      }
      avg_avg /= 64.0;
      avg_std /= 64.0;
      SERIAL_PRINTLN(avg_avg);
      SERIAL_PRINTLN(avg_std);

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
  publish(FIRMWARE_VERSION, "0", RETRY_COUNT*2);

  // give sensor 16sec to stabilize
  LOWPOWER_DELAY(SLEEP_8S);
  LOWPOWER_DELAY(SLEEP_8S);

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    known_people[i] = UNDEF_PERSON;
    forgotten_people[i] = UNDEF_PERSON;
  }

  pixelsChanged();

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (norm_pixels[i] < 0) norm_pixels[i] = 0;
    if (norm_pixels[i] > 65) norm_pixels[i] = 65;
    avg_pixels[i] = ((int)roundf(norm_pixels[i] * 1000.0));
    std_pixels[i] = 100;
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!pixelsChanged()) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (norm_pixels[i] < 0 || norm_pixels[i] > 65) continue;
      float std = norm_pixels[i] - bgPixel(i);
      // implicit alpha of 0.1
      avg_pixels[i] += ((int)roundf(100.0 * std));
      int16_t var = ((int)roundf(100.0 * (sq(std) - stdPixel(i))));
      if (var < 0 && -var > std_pixels[i] - 100)
        std_pixels[i] = 100;
      else
        std_pixels[i] += var;
    }
  }
}

float avgStd() {
  float avg_std = 0;
  for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
    avg_std += stdPixel(idx);
  }
  return avg_std/64.0;
}

unsigned long lastNoiseCheck = 0;
void checkForNoise() {
  long diff = millis() - lastNoiseCheck;
  if (abs(diff) > 1800000) { // only poll once every 30 min
    float astd = avgStd();
    if (astd > 0.8) {
      char rBuf[3];
      sprintf(rBuf, "std%d", int(astd*100.0));
      char wBuf[17];
      sprintf(wBuf, "%dx%d",
        int(global_bgm*100.0),
        int(global_fgm*100.0)
      );
      publish(rBuf, wBuf, 5);
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  //checkForNoise();
  processSensor();
}
