#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
//  #define TEST_PCBA           // uncomment to print raw amg sensor data
#endif

#define FIRMWARE_VERSION        "V0.7.4"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE_FRD        1.5  // absolute min distance between 2 points (neighbors)
#define MIN_DISTANCE            2.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define DISTANCE_BONUS          2.5  // max extra distance a hot point can move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define MAX_CLEARED_CYCLES      10   // max number of cycles before we assume frame is empty
#define CONFIDENCE_THRESHOLD    0.2  // consider a point if we're 20% confident
#define AVG_CONF_THRESHOLD      0.3  // consider a set of points if we're 30% confident
#define BACKGROUND_GRADIENT     2.0
#define FOREGROUND_GRADIENT     2.0
#define T_THRESHOLD             0.5  // min change for a pixel to be considered
#define MIN_NEIGHBORS           2    // min size of halo effect to consider a point legit
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
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
bool pos_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint8_t neighbors_count[AMG88xx_PIXEL_ARRAY_SIZE];
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
#define doorOpenedAgo(n)( frames_since_door_open < (n) && door_state == DOOR_OPEN )

uint8_t SIDE(uint8_t p) {
  return SIDE1(p) ? 1 : 2;
}

bool MAHALANBOIS(uint8_t x) {
  return norm_pixels[(x)]-bgPixel(x) >= T_THRESHOLD;
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
  return pointOnBorder(p) || pointOnLRBorder(p);
}

uint8_t cycles_since_person = MAX_CLEARED_CYCLES;
uint8_t forgotten_num = 0;
uint8_t cycles_since_forgotten = MAX_EMPTY_CYCLES;

#define DOOR_CLOSED 0
#define DOOR_AJAR   1
#define DOOR_OPEN   2
uint8_t door_state = DOOR_OPEN;
uint8_t last_published_door_state = 9;
uint8_t frames_since_door_open = 0;
uint8_t door_side = 1;

#define FRD_EVENT         0
#define MAYBE_EVENT       1
#define DOOR_CLOSE_EVENT  2
#define REVERT_FRD        3
#define MAYBE_REVERT      4

uint8_t readDoorState() {
  #ifdef RECESSED
    bool reed3High = PIND & 0b00001000; // true if reed 3 is high (normal state)
    bool reed4High = PIND & 0b00010000; // true if reed 4 is high (normal state)
    if (reed3High && reed4High) return DOOR_OPEN;
    if (!reed3High && !reed4High) return DOOR_CLOSED;
    if (reed4High) door_side = 2;
    return DOOR_AJAR;
  #else
    if (PIND & 0b00001000) {  // true if reed 3 is high (normal state)
      // door open if reed switch 4 is also high
      return PIND & 0b00010000 ? DOOR_OPEN : DOOR_AJAR;
    } else {
      return DOOR_CLOSED;
    }
  #endif
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
  uint16_t  total_neighbors;
  uint16_t  count;
  float     max_distance_covered;

  bool      real() { return past_position != UNDEF_POINT; };
  void      updateConfidence() { confidence = total_conf/((float)count); };
  float     bgm() { return total_bgm/((float)count); };
  float     fgm() { return total_fgm/((float)count); };
  float     avg_neighbors() { return (float)total_neighbors/(float)count; };

  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };

  float     totalDistance() { return euclidean_distance(starting_position, past_position); };

  void updateMaxDistance() {
    if (max_distance_covered < MAX_DISTANCE) {
      float d = totalDistance();
      max_distance_covered = max(d, max_distance_covered);
    }
  };

  void revert(uint8_t eventType) {
    char rBuf[3];
    if (eventType == REVERT_FRD)
      sprintf(rBuf, "r%d", crossed);
    else if (eventType == MAYBE_REVERT)
      sprintf(rBuf, "e%d", crossed);
    char wBuf[30];
    sprintf(wBuf, "%dx%dx%dx%dx%dx%dx%dx%d",
      int(past_conf*100.0),
      int(global_bgm*100.0),
      int(global_fgm*100.0),
      starting_position,
      past_position,
      history,
      int(avg_neighbors()*10.0),
      int(max_distance_covered*10.0)
    );
    publish(rBuf, wBuf, RETRY_COUNT);
  };

  bool checkForDoorClose() {
    // This could possibly fail if person's hand is on door knob opening door and sensor
    // detects that as a person. We'll see hand go from 1->2, and then get dropped as door
    // opens, and this if block will prevent it from reverting properly.
    if (SIDE(past_position) != door_side && confidence > 0.6) {
      if (frames_since_door_open == 0) {
        return door_state != DOOR_OPEN;
      } else if (door_state == DOOR_OPEN) {
        // door is still open, but did it change mid-frame?
        return door_state != readDoorState();
      }
    }
    return false;
  };

  bool checkForRevert() {
    if (!real() || !crossed) return false;
  
    if (reverted && side() == starting_side() &&
        (inEndZone(past_position) || checkForDoorClose())) {
      // we had previously reverted this point, but it came back and made it through
      revert(REVERT_FRD);
      reverted = false;
      return true;
    } else if (!reverted && !checkForDoorClose() && (side() != starting_side() ||
                !inEndZone(past_position))) {
      // point disappeared in middle of grid, revert its crossing (probably noise or a hand)
      revert(REVERT_FRD);
      reverted = true;
      return true;
    }
    return false;
  };

  void publishPacket(uint8_t eventType) {
    uint8_t old_crossed = crossed;
    float abgm = bgm();
    float afgm = fgm();
    float anei = avg_neighbors();
    char meta[30];
    sprintf(meta, "%s%dx%dx%dx%dx%dx%dx%dx%d",
      positive ? "+" : "-",
      int(confidence*100.0),
      int(abgm*100.0),
      int(afgm*100.0),
      starting_position,
      past_position,
      history,
      int(anei*10.0),
      int(max_distance_covered*10.0)
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
        return;
      } else if (eventType == DOOR_CLOSE_EVENT && door_side == 2) {
        publish("1", meta, RETRY_COUNT);
        return;
      }
    } else {
      if (eventType == FRD_EVENT) {
        crossed = publish(door_state == DOOR_OPEN ? "2" : "a2", meta, RETRY_COUNT);
        int s = past_position + GRID_EXTENT;
        starting_position = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
      } else if (eventType == MAYBE_EVENT) {
        publish("m2", meta, RETRY_COUNT);
        return;
      } else if (eventType == DOOR_CLOSE_EVENT && door_side == 1) {
        publish("2", meta, RETRY_COUNT);
        return;
      }
    }
    if (old_crossed) crossed = 0;
    history = 1;
    reverted = false;
    total_conf = confidence + past_conf;
    total_bgm = abgm + global_bgm;
    total_fgm = afgm + global_fgm;
    total_neighbors = int(anei) + neighbors_count[past_position];
    count = 2;
    updateConfidence();
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  bool publishMaybeEvent() {
    if (!real() || confidence < AVG_CONF_THRESHOLD || history < 2) return false;
    
    if (starting_side() != side() && (!crossed || !reverted) && history >= MIN_HISTORY
        && avg_neighbors() > MIN_NEIGHBORS) {
      if (SIDE(starting_position) == door_side && checkForDoorClose()) {
        publishPacket(DOOR_CLOSE_EVENT);
        return true;
      }
      publishPacket(MAYBE_EVENT);
      return true;
    } else {
      revert(MAYBE_REVERT);
    }
    return false;
  };

  void forget() {
    checkForRevert() || publishMaybeEvent();
  };
};

Person UNDEF_PERSON = {UNDEF_POINT};

Person known_people[MAX_PEOPLE];
Person forgotten_people[MAX_PEOPLE];

void publishEvents() {
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    Person p = known_people[i];
    if (p.real() && p.starting_side() != p.side() && (!p.crossed || !p.reverted) &&
        p.confidence > AVG_CONF_THRESHOLD && p.max_distance_covered >= MIN_DISTANCE &&
        p.history >= int(roundf(6.3 - p.confidence*MIN_HISTORY)) &&
        p.avg_neighbors() > MIN_NEIGHBORS) {
      p.publishPacket(FRD_EVENT);
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
  uint8_t total = 0.0;
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
  float x_sum1 = 0.0;
  float sq_sum1 = 0.0;
  float x_sum2 = 0.0;
  float sq_sum2 = 0.0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // reading is invalid if less than -20 or greater than 100,
    // but we use a smaller range than that to determine validity
    if (norm_pixels[i] < 0 || norm_pixels[i] > 65) {
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
      if (i >= GRID_EXTENT) {
        // not top row
        if (MAHALANBOIS(i - GRID_EXTENT)) neighbors++;
        if (NOT_AXIS(i) > 1 && MAHALANBOIS(i-(GRID_EXTENT+1))) neighbors++;
        if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i-(GRID_EXTENT-1))) neighbors++;
      }
      if (i < GRID_EXTENT*7) {
        // not bottom row
        if (MAHALANBOIS(i + GRID_EXTENT)) neighbors++;
        if (NOT_AXIS(i) > 1 && MAHALANBOIS(i+(GRID_EXTENT-1))) neighbors++;
        if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i+(GRID_EXTENT+1))) neighbors++;
      }
      if (NOT_AXIS(i) > 1 && MAHALANBOIS(i-1)) neighbors++;
      if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i+1)) neighbors++;
    }

    neighbors_count[i] = neighbors;
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
    if (neighbors_count[i] > 1) {
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
  global_bgm = BACKGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (neighbors_count[i] > 1) {
      float fgmt1 = (norm_pixels[i] - cavg1)/fgm1;
      fgmt1 = abs(fgmt1);
      float fgmt2 = (norm_pixels[i] - cavg2)/fgm2;
      fgmt2 = abs(fgmt2);
      // pick the smaller of the 2 foreground gradients
      if (min(fgmt1, fgmt2) < CONFIDENCE_THRESHOLD) {
        neighbors_count[i] = 0;
      } else {
        // difference in points from background
        float bgmt = abs(norm_pixels[i] - bgPixel(i));
        global_bgm = max(bgmt, global_bgm);
      }
    }
  }

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float std = norm_pixels[i] - bgPixel(i);

    // normalize points
    if (neighbors_count[i] <= 1) {
      norm_pixels[i] = 0.0;
      pos_pixels[i] = false;
    } else {
      float fgmt1 = (norm_pixels[i] - cavg1)/fgm1;
      float fgmta1 = abs(fgmt1);
      float fgmt2 = (norm_pixels[i] - cavg2)/fgm2;
      float fgmta2 = abs(fgmt2);
      // pick the smaller of the 2 foreground gradients
      if (fgmta1 > fgmta2) fgmt1 = fgmt2;
      float bgmt = std/global_bgm;

      if (fgmta1 < abs(bgmt)) {
        norm_pixels[i] = min(fgmta1, 1);
        pos_pixels[i] = fgmt1 >= 0;
      } else {
        norm_pixels[i] = abs(bgmt);
        pos_pixels[i] = bgmt >= 0;
      }
    }

    // update average baseline
    // implicit alpha of 0.001
    if (frames_since_door_open < 5 &&
        ((door_state == DOOR_OPEN && norm_pixels[i] < CONFIDENCE_THRESHOLD) ||
         (door_state != DOOR_OPEN && norm_pixels[i] < 0.6))) {
      // door just changed, increase alpha to 0.2 to adjust quickly to new background
      std *= 200.0;
    } else if (norm_pixels[i] > 0.5) {
      // looks like a person, lower alpha to 0.0001
      std *= 0.1;
    } else if (cycles_since_person < 2) {
      for (uint8_t x=0; x<MAX_PEOPLE; x++) {
        if (known_people[x].real() && known_people[x].confidence > 0.5 &&
            known_people[x].positive == pos_pixels[i] &&
            euclidean_distance(known_people[x].past_position, i) < 4) {
          std *= 0.1;
          break;
        }
      }
    } else if (cycles_since_person == MAX_CLEARED_CYCLES &&
                norm_pixels[i] < CONFIDENCE_THRESHOLD) {
      // nothing going on, increase alpha to 0.005
      std *= 5.0;
    }
    avg_pixels[i] += ((int)roundf(std));
  }

  return true;
}

uint8_t findCurrentPoints(uint8_t *points) {
  // sort pixels by confidence
  uint8_t ordered_indexes_temp[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
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
        // append i to end of array
        ordered_indexes_temp[active_pixel_count] = i;
      }
      active_pixel_count++;
    }
  }

  // reorder based on position
  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  for (uint8_t z=0; z<active_pixel_count; z++) {
    uint8_t i = ordered_indexes_temp[z];
    bool added = false;
    for (uint8_t j=0; j<z; j++) {
      if (pos_pixels[i] == pos_pixels[ordered_indexes[j]] &&
          abs(norm_pixels[i] - norm_pixels[ordered_indexes[j]]) < 0.15) {
        float d1 = 100;
        float d2 = 100;
        for (uint8_t x=0; x<j; x++) {
          if (pos_pixels[i] != pos_pixels[ordered_indexes[x]]) continue;
          d1 = euclidean_distance(i, ordered_indexes[x]);
          d2 = euclidean_distance(ordered_indexes[j], ordered_indexes[x]);
          if (d1 < MIN_DISTANCE || d2 < MIN_DISTANCE) break;
        }
        if (d1 < MIN_DISTANCE || d2 < MIN_DISTANCE) {
          // pick point closer to a previous peak
          added = d1 < d2;
        } else {
          // prefer point closer to middle of grid
          uint8_t edge1 = AXIS(i);
          uint8_t edge2 = AXIS(ordered_indexes[j]);

          // use columns instead of rows if same row
          if (edge1 == edge2) {
            edge1 = NOT_AXIS(i);
            edge2 = NOT_AXIS(ordered_indexes[j]);
          }

          // calculate row # from opposite edge
          if (edge1 > 4) edge1 = GRID_EXTENT+1 - edge1;
          if (edge2 > 4) edge2 = GRID_EXTENT+1 - edge2;

          if (edge1 == edge2 && edge1 == 4 && SIDE(i) != SIDE(ordered_indexes[j])) {
            // we're debating between 2 points on either side of border. Normally
            // we'd just prefer the one on side1, but this can cause a messy flip-flopping
            // between sides, so let's prefer the side that the point was already on
            for (uint8_t x=0; x<MAX_PEOPLE; x++) {
              if (known_people[x].real() && known_people[x].positive == pos_pixels[i] &&
                    euclidean_distance(i, known_people[x].past_position) < MIN_DISTANCE) {
                if (SIDE(i) == known_people[x].side()) edge1++;
                break;
              }
            }
          }

          added = edge1 > edge2;
        }
        if (added && norm_pixels[i] < norm_pixels[ordered_indexes[j]]) {
          norm_pixels[i] = norm_pixels[ordered_indexes[j]];
        }
      }

      if (added) {
        // insert point i in front of j
        for (int8_t x=z; x>j; x--) {
          ordered_indexes[x] = ordered_indexes[x-1];
        }
        ordered_indexes[j] = i;
        break;
      }
    }
    if (!added) {
      // append i to end of array
      ordered_indexes[z] = i;
    }
  }

  // reorder based on blob boundaries
  uint8_t sorted_size = 0;
  uint8_t total_masses = 0;
  for (uint8_t y=0; y<active_pixel_count; y++) {
    uint8_t current_point = ordered_indexes[y];
    if (current_point == UNDEF_POINT) continue;

    bool addMe = true;
    for (uint8_t x=0; x<total_masses; x++) {
      if (euclidean_distance(points[x], current_point) < MIN_DISTANCE) {
        // point too close to a known peak
        addMe = false;
        break;
      }
    }
    if (addMe) {
      for (uint8_t x=1; x<sorted_size; x++) {
        if (euclidean_distance(ordered_indexes_temp[x], current_point) < MIN_DISTANCE_FRD) {
          // point too close to edge of known blob
          addMe = false;
          break;
        }
      }
    }
    if (addMe) {
      points[total_masses] = current_point;
      total_masses++;
      if (total_masses == MAX_PEOPLE) break;
    }

    ordered_indexes_temp[sorted_size] = current_point;
    sorted_size++;
    ordered_indexes[y] = UNDEF_POINT;
    for (uint8_t x=sorted_size-1; x<sorted_size; x++) {
      // since MIN_DISTANCE_FRD == 1.5, we know we can only be adding neighbors
      // so we can stop looking once we've hit this point's max possible neighbors
      uint8_t added = 0;
      uint8_t max_added = 8;
      bool onYEdge = pointOnEdge(ordered_indexes_temp[x]);
      bool onXEdge = pointOnLREdge(ordered_indexes_temp[x]);
      if (onYEdge || onXEdge) {
        max_added = (onYEdge && onXEdge) ? 3 : 5;
      }

      bool blobEdge = norm_pixels[current_point] - norm_pixels[ordered_indexes_temp[x]] >0.5;

      for (uint8_t k=y+1; k<active_pixel_count; k++) {
        uint8_t i = ordered_indexes[k];
        if (i != UNDEF_POINT && pos_pixels[i] == pos_pixels[current_point] &&
              euclidean_distance(i, ordered_indexes_temp[x]) < MIN_DISTANCE_FRD) {
          if (blobEdge && norm_pixels[i] - norm_pixels[ordered_indexes_temp[x]] > 0.1)
            continue;
          ordered_indexes_temp[sorted_size] = i;
          ordered_indexes[k] = UNDEF_POINT;
          sorted_size++;
          added++;
          if (added == max_added) break;
        }
      }
    }
  }

  return total_masses;
}

void forget_person(uint8_t idx, Person *temp_forgotten_people, uint8_t *pairs,
                    uint8_t &temp_forgotten_num) {
  Person p = known_people[idx];
  if (p.confidence > 0.5 && ((p.count > 1 && !p.crossed) ||
      (p.crossed && p.checkForRevert()))) {
    p.publishMaybeEvent();
    temp_forgotten_people[temp_forgotten_num] = p;
    temp_forgotten_num++;
  }
  pairs[idx] = UNDEF_POINT;
  known_people[idx] = UNDEF_PERSON;
}

bool remember_person(Person p, uint8_t point, uint16_t &h, uint8_t &sp, uint8_t &cross,
                      bool &revert, float &an, bool &pos, float &b, float &f, uint16_t &n,
                      uint8_t &c) {
  if (p.real() && pos == p.positive) {
    float scale_factor = 1.0;
    float past_bgm = p.bgm();
    if (global_bgm < past_bgm) {
      scale_factor = global_bgm/past_bgm;
    }
    if (p.crossed && scale_factor < 0.7 && inEndZone(p.past_position) &&
        p.starting_side() == p.side()) {
      // significant drop in gradient and point made it to end zone, point likely left
      return false;
    }
    float adjusted_conf = an * scale_factor;
    // point cannot be more than 2x warmer than forgotten point
    if ((p.confidence < adjusted_conf && p.confidence * 2.0 < adjusted_conf) ||
        (adjusted_conf < p.confidence && adjusted_conf * 2.0 < p.confidence)) {
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
    n += int(p.avg_neighbors());
    c++;
    return true;
  }
  return false;
}

uint8_t pointsAbove(uint8_t i) {
  uint8_t height = 0;
  for (uint8_t x = i - GRID_EXTENT; x > 0; x -= GRID_EXTENT) {
    if (norm_pixels[x] > CONFIDENCE_THRESHOLD) height++;
    else break;
  }
  return height;
}

uint8_t pointsBelow(uint8_t i) {
  uint8_t height = 0;
  for (uint8_t x = i + GRID_EXTENT; x < AMG88xx_PIXEL_ARRAY_SIZE; x += GRID_EXTENT) {
    if (norm_pixels[x] > CONFIDENCE_THRESHOLD) height++;
    else break;
  }
  return height;
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
            // or if more than a 2x difference between these points, don't pair them
                (c < p.confidence && c * 2.0 < p.confidence) ||
                (p.confidence < c && p.confidence * 2.0 < c)) {
              continue;
            }
  
            float d = euclidean_distance(p.past_position, points[j]);
  
            // if switching sides with low confidence, don't pair
            if (SIDE(points[j]) != p.side() &&
                (p.confidence < AVG_CONF_THRESHOLD ||
                  norm_pixels[points[j]]/d < MIN_TRAVEL_RATIO)) {
              continue;
            }
  
            if (d < max_distance) {
              float ratioP = min(norm_pixels[points[j]]/p.confidence,
                                 p.confidence/norm_pixels[points[j]]);
              if (p.crossed) ratioP /= 2.0; // ratio matters less once point is crossed
              float directionBonus = 0;
              uint8_t sp_axis = AXIS(p.past_position);
              uint8_t np_axis = AXIS(points[j]);
              if (np_axis == sp_axis) directionBonus = 0.05;
              else if (SIDE1(p.starting_position)) {
                if (p.crossed) {
                  if (np_axis < sp_axis) directionBonus = 0.1;
                } else if (np_axis > sp_axis) directionBonus = 0.1;
              } else { // side 2
                if (p.crossed) {
                  if (np_axis > sp_axis) directionBonus = 0.1;
                } else if (np_axis < sp_axis) directionBonus = 0.1;
              }
              float score = sq(d/max_distance) - ratioP - directionBonus +
                                max(AVG_CONF_THRESHOLD - c, 0.0);
              if (min_score - score > 0.05) {
                min_score = score;
                min_index = j;
              } else if (min_index != UNDEF_POINT && score - min_score < 0.05) {
                // score is the same, pick the point that lets this one move farthest
                bool useJ = false;
                uint8_t axis1 = AXIS(points[j]);
                uint8_t axis2 = AXIS(points[min_index]);
                if (axis1 == axis2) {
                  // points on same axis, choose the one with higher confidence
                  useJ = norm_pixels[points[j]] > norm_pixels[points[min_index]];
                } else if (p.crossed) {
                  useJ = SIDE1(p.starting_position) ? axis1 < axis2 : axis1 > axis2;
                } else {
                  useJ = SIDE1(p.starting_position) ? axis1 > axis2 : axis1 < axis2;
                }
                if (useJ) {
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
              if (AXIS(points[i]) == AXIS(p.past_position)) directionBonus = 2.5;
              else if (SIDE1(p.starting_position)) {
                if (AXIS(points[i]) < AXIS(p.past_position)) directionBonus = 3.0;
              } else if (AXIS(points[i]) > AXIS(p.past_position)) directionBonus = 3.0;
            } else {
              if (AXIS(points[i]) == AXIS(p.past_position)) directionBonus = 0.5;
              else if (SIDE1(p.starting_position)) {
                if (AXIS(points[i]) > AXIS(p.past_position)) directionBonus = 1.0;
              } else if (AXIS(points[i]) < AXIS(p.past_position)) directionBonus = 1.0;
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
            p.max_distance_covered = 0;
            p.history = 1;
            p.crossed = 0;
            p.reverted = false;
            if (pointOnEdge(points[i]) && pointOnEdge(p.past_position)) {
              p.total_conf = p.past_conf;
              p.total_bgm = global_bgm;
              p.total_fgm = global_fgm;
              p.total_neighbors = neighbors_count[points[i]];
              p.count = 1;
            } else {
              p.total_conf = p.confidence + p.past_conf;
              p.total_bgm = p.bgm() + global_bgm;
              p.total_fgm = p.fgm() + global_fgm;
              p.total_neighbors = int(p.avg_neighbors()) + neighbors_count[points[i]];
              p.count = 2;
            }
          } else {
            if (p.past_position != points[i]) {
              if (SIDE(points[i]) != p.side()) {
                if (p.confidence > 0.8 && norm_pixels[points[i]] > 0.8) {
                  p.history++;
                } else {
                  // point just crossed threshold, let's reduce its history to force
                  // it to spend another cycle on this side before we count the event
                  p.history = min(p.history + 1, MIN_HISTORY);
                }
              } else if (points[i] == p.starting_position) {
                p.history = 1;
              } else {
                p.history++;
              }
              p.past_position = points[i];
              p.updateMaxDistance();
            }
            p.past_conf = norm_pixels[points[i]];
            p.total_conf += p.past_conf;
            p.total_bgm += global_bgm;
            p.total_fgm += global_fgm;
            p.total_neighbors += neighbors_count[points[i]];
            p.count++;
          }
          cycles_since_person = 0;
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
      uint16_t n = neighbors_count[sp];
      uint8_t c = 1;
      bool retroMatched = false;

      if (an <= AVG_CONF_THRESHOLD || n < MIN_NEIGHBORS) continue;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        for (uint8_t j=0; j<temp_forgotten_num; j++) {
          if (remember_person(temp_forgotten_people[j], points[i],
                              h, sp, cross, revert, an, pos, b, f, n, c)) {
            retroMatched = true;
            temp_forgotten_people[j] = UNDEF_PERSON;
            SERIAL_PRINTLN(F("almost f'd"));
            break;
          }
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        for (uint8_t j=0; j<forgotten_num; j++) {
          if (remember_person(forgotten_people[j], points[i],
                              h, sp, cross, revert, an, pos, b, f, n, c)) {
            retroMatched = true;
            forgotten_people[j] = UNDEF_PERSON;
            break;
          }
        }
      }

      // ignore new points on side 1 immediately after door opens/closes
      if ((frames_since_door_open < 3 && SIDE(sp) == door_side) ||
          (frames_since_door_open < 5 && door_state != DOOR_OPEN) ||
          (!retroMatched &&
            ((SIDE1(sp) && pointsBelow(sp) >= 4) || (SIDE2(sp) && pointsAbove(sp) >= 4))))
        continue;

      // ignore new points that showed up in middle 2 rows of grid
      if (retroMatched || norm_pixels[points[i]] > 0.6 || pointOnSmallBorder(sp) ||
          pointOnLRBorder(sp)) {
        for (uint8_t j=0; j<MAX_PEOPLE; j++) {
          // look for first empty slot in past_points to use
          if (!known_people[j].real()) {
            Person p;
            p.past_position = points[i];
            p.history = h;
            p.starting_position = sp;
            p.max_distance_covered = 0;
            if (sp != points[i]) p.updateMaxDistance();
            p.crossed = cross;
            p.reverted = revert;
            p.past_conf = norm_pixels[points[i]];
            p.positive = pos;
            p.total_conf = an;
            p.total_bgm = b;
            p.total_fgm = f;
            p.total_neighbors = n;
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
  if (cycles_since_person < MAX_CLEARED_CYCLES) {
    cycles_since_person++;
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
          SERIAL_PRINT(F("-"));
          SERIAL_PRINT(neighbors_count[p.past_position]);
          SERIAL_PRINT(F("),"));
        }
      }
      SERIAL_PRINTLN();

      SERIAL_PRINTLN(global_bgm);
      SERIAL_PRINTLN(global_fgm);
      float avg_avg = 0;
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        avg_avg += bgPixel(idx);
      }
      avg_avg /= 64.0;
      SERIAL_PRINTLN(avg_avg);

      // print chart of what we saw in 8x8 grid
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        SERIAL_PRINT(F(" "));
        if (norm_pixels[idx] < CONFIDENCE_THRESHOLD)
          SERIAL_PRINT(F("-----"));
        else {
          SERIAL_PRINT(pos_pixels[idx] ? '+' : '-');
          SERIAL_PRINT(norm_pixels[idx]);
        }
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
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  processSensor();
}
