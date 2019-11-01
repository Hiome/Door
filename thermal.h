#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
//  #define TEST_PCBA           // uncomment to print raw amg sensor data
#endif

#define FIRMWARE_VERSION        "V0.7.24"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE_FRD        1.5  // absolute min distance between 2 points (neighbors)
#define MIN_DISTANCE            2.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define MAX_CLEARED_CYCLES      10   // max number of cycles before we assume frame is empty
#define CONFIDENCE_THRESHOLD    0.2  // consider a point if we're 30% confident
#define AVG_CONF_THRESHOLD      0.3  // consider a set of points if we're 40% confident
#define ABSOLUTE_THRESHOLD      1.1  // min number of degrees temp change to be considered
#define BACKGROUND_GRADIENT     2.0
#define FOREGROUND_GRADIENT     2.0
#define NUM_STD_DEV             3.0  // max num of std dev to include in trimmed average
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
uint8_t conf_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
bool pos_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int8_t neighbors_count[AMG88xx_PIXEL_ARRAY_SIZE];
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
#else
  #define AXIS          x
  #define NOT_AXIS      y
  #define SIDE1(p)      ( (AXIS(p)) <= (GRID_EXTENT/2) )
  #define SIDE2(p)      ( (AXIS(p)) > (GRID_EXTENT/2) )
  #error Double check all your code, this is untested
#endif
#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define bgPixel(x)      ( ((float)avg_pixels[(x)])/1000.0 )

uint8_t SIDE(uint8_t p) {
  return SIDE1(p) ? 1 : 2;
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
  #define pointOnEdge(i)        ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
  #define pointOnLREdge(i)      ( NOT_AXIS(i) == 1 || NOT_AXIS(i) == GRID_EXTENT )
#else
  #define pointOnBorder(i)      ( AXIS(i) <= 3 || AXIS(i) >= 6 )
  #define pointOnSmallBorder(i) ( AXIS(i) <= 2 || AXIS(i) >= 7 )
  #define pointOnEdge(i)        ( AXIS(i) == 1 || AXIS(i) == 8 )
  #define pointOnLREdge(i)      ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
#endif

bool pointOnAnyEdge(uint8_t p) {
  return pointOnEdge(p) || pointOnLREdge(p);
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

bool doorJustOpened() {
  if (door_state == DOOR_OPEN) return frames_since_door_open == 0;
  else return readDoorState() == DOOR_OPEN;
}

uint8_t pointsAbove(uint8_t i) {
  uint8_t height = 0;
  bool pos = pos_pixels[i];
  for (int8_t x = i - GRID_EXTENT; x >= 0; x -= GRID_EXTENT) {
    if (norm_pixels[x] > CONFIDENCE_THRESHOLD && pos_pixels[x] == pos) height++;
    else break;
  }
  return height;
}

uint8_t pointsBelow(uint8_t i) {
  uint8_t height = 0;
  bool pos = pos_pixels[i];
  for (uint8_t x = i + GRID_EXTENT; x < AMG88xx_PIXEL_ARRAY_SIZE; x += GRID_EXTENT) {
    if (norm_pixels[x] > CONFIDENCE_THRESHOLD && pos_pixels[x] == pos) height++;
    else break;
  }
  return height;
}

uint8_t calcHeight(uint8_t i) {
  return pointsAbove(i) + pointsBelow(i);
}

uint8_t calcWidth(uint8_t i) {
  uint8_t width = 0;
  uint8_t axis = AXIS(i);
  bool pos = pos_pixels[i];
  for (uint8_t x = i+1; x < AMG88xx_PIXEL_ARRAY_SIZE && AXIS(x) == axis; x++) {
    if (norm_pixels[x] > CONFIDENCE_THRESHOLD && pos_pixels[x] == pos) width++;
    else break;
  }
  for (int8_t x = i-1; x >= 0 && AXIS(x) == axis; x--) {
    if (norm_pixels[x] > CONFIDENCE_THRESHOLD && pos_pixels[x] == pos) width++;
    else break;
  }
  return width;
}

typedef struct Person {
  uint8_t   past_position;          // 0-63 + UNDEF_POINT
  uint8_t   starting_position :6;   // 0-63
  uint8_t   max_position      :6;   // 0-63
  uint8_t   max_jump          :6;   // max possible jump of 55
  uint8_t   history           :4;   // 1-10
  uint8_t   crossed           :4;   // 0-9
  bool      reverted          :1;   // 0-1
  bool      positive          :1;   // 0-1
  float     total_bgm;
  float     total_fgm;
  uint8_t   total_neighbors;        // 0-80
  uint8_t   total_height;           // 0-80
  uint8_t   total_width;            // 0-80
  uint16_t  total_bgm_conf    :10;  // 0-1000
  uint16_t  total_fgm_conf    :10;  // 0-1000
  uint8_t   count             :4;   // 1-7
  uint16_t  count_frd;

  bool resetIfNecessary() {
    if (count > 5) {
      resetTotals();
      if (count_frd > 60000) count_frd = 1000;
      return true;
    }
    return false;
  };

  void resetTotals() {
    history = min(history, MIN_HISTORY);
    total_bgm = bgm();
    total_fgm = fgm();
    total_bgm_conf = int(bgm_confidence());
    total_fgm_conf = int(fgm_confidence());
    total_neighbors = int(neighbors());
    total_height = int(height());
    total_width = int(width());
    count = 1;
  };

  bool      real() { return past_position != UNDEF_POINT; };
  float     bgm() { return total_bgm/((float)count); };
  float     fgm() { return total_fgm/((float)count); };
  float     neighbors() { return (float)total_neighbors/(float)count; };
  float     height() { return (float)total_height/(float)count; };
  float     width()  { return (float)total_width/(float)count; };

  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };

  float     bgm_confidence() { return ((float)total_bgm_conf)/((float)count); };
  float     fgm_confidence() { return ((float)total_fgm_conf)/((float)count); };
  float     scaled_bgm() { return bgm_confidence()/100.0 * bgm(); };
  float     scaled_fgm() { return fgm_confidence()/100.0 * fgm(); };
  float     confidence() {
    uint16_t c = min(total_bgm_conf, total_fgm_conf);
    float conf = ((float)c)/100.0;
    return conf/((float)count);
  };

  #define METALENGTH  49
  void generateMeta(char *meta) {
    sprintf(meta, "%s%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%d",
      positive ? "+" : "-",               // 1  +
      int(bgm_confidence()),              // 3  100
      int(fgm_confidence()),              // 3  100
      int(bgm()*100.0),                   // 4  1020
      int(fgm()*100.0),                   // 4  1020
      starting_position,                  // 2  23
      past_position,                      // 2  37
      history,                            // 5  65536
      count_frd,                          // 5  65536
      int(neighbors()*10.0),              // 2  80
      int(height()*10.0),                 // 2  70
      int(width()*10.0),                  // 2  70
      max_jump                            // 2  99
    );                                    // + 11 'x' + 1 null => 49 total
  };

  void revert(uint8_t eventType) {
    char rBuf[3];
    char meta[METALENGTH];
    generateMeta(meta);
    sprintf(rBuf, "%s%d", (eventType == REVERT_FRD ? "r" : "e"), crossed);
    publish(rBuf, meta, eventType == REVERT_FRD ? RETRY_COUNT : 0);
  };

  bool checkForDoorClose() {
    // This could possibly fail if person's hand is on door knob opening door and sensor
    // detects that as a person. We'll see hand go from 1->2, and then get dropped as door
    // opens, and this if block will prevent it from reverting properly.
    if (starting_side() == door_side && confidence() > 0.6) {
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
  
    if (reverted && side() == starting_side()) {
      // we had previously reverted this point, but it came back and made it through
      revert(REVERT_FRD);
      reverted = false;
      return true;
    } else if (!reverted && side() != starting_side()) {
      // point disappeared in middle of grid, revert its crossing (probably noise or a hand)
      revert(REVERT_FRD);
      reverted = true;
      return true;
    }
    return false;
  };

  void publishPacket(uint8_t eventType) {
    char meta[METALENGTH];
    generateMeta(meta);
    uint8_t old_crossed = crossed;
    if (SIDE1(past_position)) {
      if (eventType == FRD_EVENT) {
        crossed = publish(door_state == DOOR_OPEN ? "1" : "a1", meta, RETRY_COUNT);
        // artificially shift starting point ahead 1 row so that
        // if user turns around now, algorithm considers it an exit
        int s = past_position - GRID_EXTENT;
        starting_position = max(s, 0);
      } else if (eventType == MAYBE_EVENT) {
        publish("m1", meta, 5);
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
        publish("m2", meta, 5);
        return;
      } else if (eventType == DOOR_CLOSE_EVENT && door_side == 1) {
        publish("2", meta, RETRY_COUNT);
        return;
      }
    }
    if (old_crossed) crossed = 0;
    history = 1;
    max_jump = 0;
    reverted = false;
    max_position = past_position;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  bool publishMaybeEvent() {
    if (!real() || confidence() < AVG_CONF_THRESHOLD) return false;

    if (history >= MIN_HISTORY && (!crossed || !reverted)) {
      if (starting_side() != side()) {
        if (checkForDoorClose()) // publish full event (not a2) even if door is closed
          publishPacket(DOOR_CLOSE_EVENT);
        else
          publishPacket(FRD_EVENT);
      } else if (starting_side() != SIDE(max_position) && !pointOnBorder(past_position) &&
                  checkForDoorClose()) {
        // door just closed and point made it across but then died on border.
        // this might be somebody leaning in to close the door
        publishPacket(MAYBE_EVENT);
      } else if (euclidean_distance(starting_position, max_position) > MAX_DISTANCE) {
        revert(MAYBE_REVERT);
        return false;
      } else return false;
      return true;
    } else if (crossed || reverted || (history >= 2 &&
                euclidean_distance(starting_position, max_position) > MAX_DISTANCE)) {
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
        p.confidence() > AVG_CONF_THRESHOLD && p.history > MIN_HISTORY &&
        pointOnBorder(p.past_position) &&
        euclidean_distance(p.starting_position, p.past_position) > MIN_DISTANCE) {
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

void recheckPoint(uint8_t x, uint8_t i) {
  if (norm_pixels[(x)] <= CONFIDENCE_THRESHOLD) return;

  if (pos_pixels[x] == pos_pixels[i]) {
    neighbors_count[i]++;
  } else {
    neighbors_count[i]--;
  }
}

void calculateFgm(float cavg1, float cavg2) {
  global_fgm = FOREGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (!neighbors_count[i]) continue;
    if (global_bgm > 1) {
      float bgmt = norm_pixels[i] - bgPixel(i);
      bgmt = abs(bgmt)/global_bgm;
      if (bgmt <= CONFIDENCE_THRESHOLD) {
        neighbors_count[i] = 0;
        continue;
      }
    }
    float fgmt1 = abs(norm_pixels[i] - cavg1);
    float fgmt2 = abs(norm_pixels[i] - cavg2);
    fgmt1 = min(fgmt1, fgmt2);
    global_fgm = max(fgmt1, global_fgm);
  }
}

void calculateBgm(float cavg1, float cavg2) {
  global_bgm = BACKGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (!neighbors_count[i]) continue;
    float fgmt1 = abs(norm_pixels[i] - cavg1);
    float fgmt2 = abs(norm_pixels[i] - cavg2);
    fgmt1 = min(fgmt1, fgmt2);
    fgmt1 /= global_fgm;
    if (fgmt1 <= CONFIDENCE_THRESHOLD) {
      neighbors_count[i] = 0;
    } else {
      // difference in points from background
      float bgmt = norm_pixels[i] - bgPixel(i);
      bgmt = abs(bgmt);
      global_bgm = max(bgmt, global_bgm);
    }
  }
}

bool normalizePixels() {
  if (!pixelsChanged()) return false;

  float x_sum1 = 0.0;
  float sq_sum1 = 0.0;
  float x_sum2 = 0.0;
  float sq_sum2 = 0.0;
  float bg_avg = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // reading is invalid if less than -20 or greater than 100,
    // but we use a smaller range than that to determine validity
    if (norm_pixels[i] < 0 || norm_pixels[i] > 65) {
      norm_pixels[i] = bgPixel(i);
    }

    bg_avg += bgPixel(i);

    if (SIDE1(i)) {
      x_sum1 += norm_pixels[i];
      sq_sum1 += sq(norm_pixels[i]);
    } else {
      x_sum2 += norm_pixels[i];
      sq_sum2 += sq(norm_pixels[i]);
    }

    neighbors_count[i] = 1;
  }

  bg_avg /= 64.0;

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
    SERIAL_PRINT(F("thermistor "));
    SERIAL_PRINTLN(amg.readThermistor());
    SERIAL_PRINT(F("mean "));
    SERIAL_PRINTLN((cavg1 + cavg2)/2);
  #endif

  // calculate CSM gradients
  global_bgm = 0; // needed to skip bgm check in first calculateFgm
  calculateFgm(cavg1, cavg2);
  // run 2 passes to amplify real points over noise
  calculateBgm(cavg1, cavg2);
  calculateFgm(cavg1, cavg2);
  calculateBgm(cavg1, cavg2);
  calculateFgm(cavg1, cavg2);

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float std = norm_pixels[i] - bgPixel(i);

    // normalize points
    pos_pixels[i] = norm_pixels[i] >= bg_avg;
    float bgmt = abs(std/global_bgm);
    float fgmt1 = abs(norm_pixels[i] - cavg1);
    float fgmt2 = abs(norm_pixels[i] - cavg2);
    fgmt1 = min(fgmt1, fgmt2);
    fgmt2 = fgmt1/global_fgm;
    if (fgmt2 < bgmt) {
      norm_pixels[i] = fgmt2;
      conf_pixels[i] = int(bgmt*100.0);
    } else {
      norm_pixels[i] = bgmt;
      conf_pixels[i] = int(fgmt2*100.0) + 100;
    }

    if (norm_pixels[i] < CONFIDENCE_THRESHOLD || !neighbors_count[i] ||
          std < 0.6 || fgmt1 < 0.6) {
      norm_pixels[i] = 0.0;
    }

    // update average baseline
    // implicit alpha of 0.001
    if (frames_since_door_open < 5 &&
        ((door_state == DOOR_OPEN && norm_pixels[i] < CONFIDENCE_THRESHOLD) ||
         (door_state != DOOR_OPEN && norm_pixels[i] < 0.6))) {
      // door just changed, increase alpha to 0.1 to adjust quickly to new background
      std *= 100.0;
    } else if (norm_pixels[i] > 0.5) {
      // looks like a person, lower alpha to 0.0001
      std *= 0.1;
    } else if (cycles_since_person < 2) {
      for (uint8_t x=0; x<MAX_PEOPLE; x++) {
        if (!known_people[x].real() || known_people[x].positive != pos_pixels[i]) continue;
        float b = known_people[x].scaled_bgm();
        if (abs(b - std) < ABSOLUTE_THRESHOLD &&
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

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (norm_pixels[i] < CONFIDENCE_THRESHOLD) {
      continue;
    }

    neighbors_count[i] = 0;

    if (i >= GRID_EXTENT) {
      // not top row
      recheckPoint(i - GRID_EXTENT, i);
      if (NOT_AXIS(i) > 1) {
        recheckPoint(i-(GRID_EXTENT+1), i);
      }
      if (NOT_AXIS(i) < GRID_EXTENT) {
        recheckPoint(i-(GRID_EXTENT-1), i);
      }
    }
    if (i < GRID_EXTENT*7) {
      // not bottom row
      recheckPoint(i + GRID_EXTENT, i);
      if (NOT_AXIS(i) > 1) {
        recheckPoint(i+(GRID_EXTENT-1), i);
      }
      if (NOT_AXIS(i) < GRID_EXTENT) {
        recheckPoint(i+(GRID_EXTENT+1), i);
      }
    }
    if (NOT_AXIS(i) > 1) {
      recheckPoint(i-1, i);
    }
    if (NOT_AXIS(i) < GRID_EXTENT) {
      recheckPoint(i+1, i);
    }

    neighbors_count[i] = max(neighbors_count[i], 0);
  }

  return true;
}

uint8_t findCurrentPoints(uint8_t *points) {
  // sort pixels by confidence
  uint8_t ordered_indexes_temp[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (norm_pixels[i] > CONFIDENCE_THRESHOLD) {
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
        if (neighbors_count[i] != neighbors_count[ordered_indexes[j]]) {
          // prefer the point that's more in middle of blob
          added = neighbors_count[i] > neighbors_count[ordered_indexes[j]];
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

    points[total_masses] = current_point;
    total_masses++;
    if (total_masses == MAX_PEOPLE) break;

    ordered_indexes_temp[sorted_size] = current_point;
    sorted_size++;
    ordered_indexes[y] = UNDEF_POINT;

    for (uint8_t x=sorted_size-1; x<sorted_size; x++) {
      // scan all points added after current_point, since they must be part of same blob
      uint8_t added = 0;
      uint8_t max_added = 8;
      bool lrEdge = pointOnLREdge(ordered_indexes_temp[x]);
      bool tbEdge = pointOnEdge(ordered_indexes_temp[x]);
      if (lrEdge || tbEdge) {
        if (lrEdge && tbEdge) max_added = 3;
        else max_added = 5;
      }
      bool blobEdge = norm_pixels[current_point] - norm_pixels[ordered_indexes_temp[x]] >0.6;

      for (uint8_t k=y+1; k<active_pixel_count; k++) {
        // scan all known points after current_point to find neighbors to point x
        uint8_t i = ordered_indexes[k];
        if (i != UNDEF_POINT && pos_pixels[i] == pos_pixels[current_point] &&
              euclidean_distance(i, ordered_indexes_temp[x]) < MIN_DISTANCE_FRD) {
          if (blobEdge && norm_pixels[i] - norm_pixels[ordered_indexes_temp[x]] > 0.15) {
            // do nothing
          } else {
            ordered_indexes_temp[sorted_size] = i;
            ordered_indexes[k] = UNDEF_POINT;
            sorted_size++;
          }
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
  if (p.confidence() > AVG_CONF_THRESHOLD && (p.checkForRevert() || p.history > 1)) {
    p.publishMaybeEvent();
    temp_forgotten_people[temp_forgotten_num] = p;
    temp_forgotten_num++;
  }
  pairs[idx] = UNDEF_POINT;
  known_people[idx] = UNDEF_PERSON;
}

bool remember_person(Person p, uint8_t point, uint8_t &h, uint8_t &sp, uint8_t &mp,
                      uint8_t &mj, uint8_t &cross, bool &revert, float an, uint16_t &bn,
                      uint16_t &fn, bool &pos, float &b, float &f, uint8_t &n,
                      uint8_t &height, uint8_t &width, uint8_t &c, uint16_t &cfrd) {
  if (p.real() && p.positive == pos_pixels[point]) {
    float conf = p.confidence();
    // point cannot be more than 2x warmer than forgotten point
    if ((conf < an && conf * 2.0 < an) || (an < conf && an * 2.0 < conf)) {
      return false;
    }
    // if switching sides with low confidence or moving too far, don't pair
    float d = euclidean_distance(p.past_position, point);
    if (d >= MAX_DISTANCE || (SIDE(point) != p.side() &&
        (conf < AVG_CONF_THRESHOLD || an/d < MIN_TRAVEL_RATIO))) {
      return false;
    }

    if ((SIDE1(p.starting_position) && AXIS(p.starting_position) >= AXIS(mp)) ||
        (SIDE2(p.starting_position) && AXIS(p.starting_position) <= AXIS(mp))) {
      // this point is moved behind previous starting point, just start over
      return false;
    }

    if (p.count > 1) p.resetTotals();

    // point is ahead of starting point at least
    sp = p.starting_position;

    if ((SIDE1(sp) && AXIS(p.max_position) > AXIS(mp)) ||
        (SIDE2(sp) && AXIS(p.max_position) < AXIS(mp))) {
      // point moved backwards from past position
      mp = p.max_position;
      h = 1;
    } else {
      // point is moving forward
      h = min(p.history, MIN_HISTORY);
    }

    uint8_t newJump = int(d*10.0);
    mj = max(newJump, p.max_jump);

    cross = p.crossed;
    revert = p.reverted;
    bn += p.total_bgm_conf;
    fn += p.total_fgm_conf;
    b += p.total_bgm;
    f += p.total_fgm;
    n += p.total_neighbors;
    height += p.total_height;
    width += p.total_width;
    c += p.count;
    cfrd += p.count_frd;
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
        if (p.resetIfNecessary()) known_people[idx] = p;
        uint8_t min_index = UNDEF_POINT;
        float conf = p.confidence();
        float anei = p.neighbors();
        float max_distance = MIN_DISTANCE + (conf * min(anei, MIN_DISTANCE));
        float min_score = 100;
        for (uint8_t j=0; j<total_masses; j++) {
          // if confidence doesn't share same sign
          if (p.positive != pos_pixels[points[j]] ||
          // or if more than a 2x difference between these points, don't pair them
              (norm_pixels[points[j]] < conf && norm_pixels[points[j]] * 2.0 < conf) ||
              (conf < norm_pixels[points[j]] && conf * 2.0 < norm_pixels[points[j]])) {
            continue;
          }

          float d = euclidean_distance(p.past_position, points[j]);

          // if switching sides with low confidence, don't pair
          if (SIDE(points[j]) != p.side() &&
              (conf < AVG_CONF_THRESHOLD || norm_pixels[points[j]]/d < MIN_TRAVEL_RATIO)) {
            continue;
          }

          if (d < max_distance) {
            float ratioP = min(norm_pixels[points[j]]/conf, conf/norm_pixels[points[j]]);
            if (p.crossed) ratioP /= 2.0; // ratio matters less once point is crossed
            float directionBonus = 0;
            uint8_t sp_axis = AXIS(p.past_position);
            uint8_t np_axis = AXIS(points[j]);
            if (np_axis == sp_axis) directionBonus = 0.05;
            else if (SIDE1(p.starting_position)) {
              if (p.crossed && !pointOnSmallBorder(p.starting_position)) {
                if (np_axis < sp_axis) directionBonus = 0.1;
              } else if (np_axis > sp_axis) {
                directionBonus = 0.1;
              }
            } else { // side 2
              if (p.crossed && !pointOnSmallBorder(p.starting_position)) {
                if (np_axis > sp_axis) directionBonus = 0.1;
              } else if (np_axis < sp_axis) {
                directionBonus = 0.1;
              }
            }
            if (!p.crossed || pointOnSmallBorder(p.starting_position)) {
              directionBonus += (0.02*neighbors_count[points[j]]);
            }

            if (norm_pixels[points[j]] < AVG_CONF_THRESHOLD) {
              directionBonus -= (AVG_CONF_THRESHOLD - norm_pixels[points[j]]);
            }

            float score = sq(d/max_distance) - ratioP - directionBonus;
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
                useJ = SIDE1(p.starting_position) ? (axis1 < axis2) : (axis1 > axis2);
              } else {
                useJ = SIDE1(p.starting_position) ? (axis1 > axis2) : (axis1 < axis2);
              }
              if (useJ) {
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
        Person p = known_people[idx];
        if (p.real() && pairs[idx] == i) {
          float conf = p.confidence();
          float score = conf;
          float d = euclidean_distance(p.past_position, points[i]);
          uint8_t axis = AXIS(p.past_position);
          uint8_t naxis = NOT_AXIS(p.past_position);
          if (score + 0.05 < AVG_CONF_THRESHOLD || (p.crossed &&
              (axis <= min(d, 2) || ((GRID_EXTENT+1) - axis) <= min(d, 2) ||
              naxis <= min(d, 2) || ((GRID_EXTENT+1) - naxis) <= min(d, 2)))) {
            score = 0.0;
          } else {
            score = sq(score);
            float directionBonus = 0;
            uint8_t paxis = AXIS(points[i]);
            if (paxis == axis) { // and allies
              if (p.crossed && SIDE(points[i]) == p.starting_side()) {
                directionBonus = 2.5;
              } else directionBonus = 0.5;
            } else if (SIDE1(p.starting_position)) {
              if (p.crossed && !pointOnSmallBorder(p.starting_position)) {
                if (paxis < axis) directionBonus = 3.0;
              } else if (paxis > axis) {
                directionBonus = 1.0;
              }
            } else { // side 2
              if (p.crossed && !pointOnSmallBorder(p.starting_position)) {
                if (paxis > axis) directionBonus = 3.0;
              } else if (paxis < axis) {
                directionBonus = 1.0;
              }
            }
            directionBonus += (0.1*p.neighbors());
            float td = euclidean_distance(p.starting_position, p.past_position);
            if (td < 1) {
              if (p.count == 1 || p.crossed) td = 1.0;
              else td = 1.0/((float)(p.count - 1));
            }
            score *= (td + directionBonus);
            score *= p.history;
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
            } else { // score is identical
              // if 2 competing points have the same score, pick the closer one
              float d2 = euclidean_distance(known_people[max_idx].past_position, points[i]);
              if (d+0.05 < d2 ||
                  (d-d2 < 0.05 && conf > known_people[max_idx].confidence())) {
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
          if (p.past_position != points[i]) {
            uint8_t newJump = int(euclidean_distance(p.past_position, points[i])*10.0);
            p.max_jump = max(newJump, p.max_jump);

            if ((SIDE1(p.starting_position) && AXIS(points[i]) <= AXIS(p.max_position)) ||
                (SIDE2(p.starting_position) && AXIS(points[i]) >= AXIS(p.max_position))) {
              // don't increase history if point is not moving forward
              if (p.count > 1) p.resetTotals();
              if ((SIDE1(p.starting_position) &&
                    AXIS(points[i]) <= AXIS(p.starting_position)) ||
                  (SIDE2(p.starting_position) &&
                    AXIS(points[i]) >= AXIS(p.starting_position))) {
                // reset history if point is further back than where it started
                if (!p.crossed || pointOnEdge(points[i])) {
                  // reset everything, unless point is crossed and could still move back
                  p.past_position = points[i]; // needs to be set before checkForRevert
                  p.checkForRevert();
                  p.crossed = 0;
                  p.reverted = false;
                  p.total_bgm_conf = 0;
                  p.total_fgm_conf = 0;
                  p.total_bgm = 0;
                  p.total_fgm = 0;
                  p.total_neighbors = 0;
                  p.total_height = 0;
                  p.total_width = 0;
                  p.count = 0;
                }
                // reset start position, unless point is in a revert crisis
                if (!p.count || !p.reverted) {
                  p.history = 1;
                  p.starting_position = points[i];
                  p.max_position = points[i];
                  p.max_jump = 0;
                }
              } else if (p.reverted) {
                // pull back max position if person is waffling in middle
                p.max_position = points[i];
              }
            } else {
              // "always forward, forward always" - Luke Cage
              p.history++;
              if (SIDE(points[i]) != p.side() || p.history > 9) {
                // point just crossed threshold, let's reduce its history to force
                // it to spend another cycle on this side before we count the event
                p.history = min(p.history, MIN_HISTORY);
              }
              p.max_position = points[i];
            }
            p.past_position = points[i];
          }
          if (conf_pixels[points[i]] > 100) {
            p.total_bgm_conf += int(norm_pixels[points[i]] * 100.0);
            p.total_fgm_conf += conf_pixels[points[i]] - 100;
          } else {
            p.total_bgm_conf += conf_pixels[points[i]];
            p.total_fgm_conf += int(norm_pixels[points[i]] * 100.0);
          }
          p.total_bgm += global_bgm;
          p.total_fgm += global_fgm;
          p.total_neighbors += neighbors_count[points[i]];
          p.total_height += calcHeight(points[i]);
          p.total_width += calcWidth(points[i]);
          p.count++;
          p.count_frd++;
          cycles_since_person = 0;
          known_people[idx] = p;
          break;
        }
      }
    } else if (taken[i] == 0 && norm_pixels[points[i]] > AVG_CONF_THRESHOLD) {
      // new point appeared (no past point found), start tracking it
      uint8_t sp = points[i];
      uint8_t mp = sp;
      uint8_t mj = 0;
      uint8_t h = 1;
      uint8_t cross = 0;
      bool revert = false;
      uint16_t bn;
      uint16_t fn;
      if (conf_pixels[sp] > 100) {
        bn = int(norm_pixels[sp] * 100.0);
        fn = conf_pixels[sp] - 100;
      } else {
        bn = conf_pixels[sp];
        fn = int(norm_pixels[sp] * 100.0);
      }
      float an = (float)min(bn,fn)/100.0;
      bool pos = pos_pixels[sp];
      float b = global_bgm;
      float f = global_fgm;
      uint8_t n = neighbors_count[sp];
      uint8_t height = calcHeight(sp);
      uint8_t width = calcWidth(sp);
      uint8_t c = 1;
      uint16_t cfrd = 1;
      bool retroMatched = false;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        for (uint8_t j=0; j<temp_forgotten_num; j++) {
          if (remember_person(temp_forgotten_people[j], points[i], h, sp, mp, mj, cross,
                revert, an, bn, fn, pos, b, f, n, height, width, c, cfrd)) {
            retroMatched = true;
            temp_forgotten_people[j] = UNDEF_PERSON;
            SERIAL_PRINTLN(F("af"));
            break;
          }
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        for (uint8_t j=0; j<forgotten_num; j++) {
          if (remember_person(forgotten_people[j], points[i], h, sp, mp, mj, cross,
                revert, an, bn, fn, pos, b, f, n, height, width, c, cfrd)) {
            retroMatched = true;
            forgotten_people[j] = UNDEF_PERSON;
            break;
          }
        }
      }

      if (!retroMatched && !pointOnBorder(sp)) {
        // if point is right in middle, drag it to the side it appears to be coming from
        uint8_t a = pointsAbove(sp);
        uint8_t b = pointsBelow(sp);
        if (an > 0.6 && doorJustOpened()) {
          if (SIDE1(sp)) {
            if (door_side == 1 && b > a) sp += GRID_EXTENT;
          } else if (door_side == 2 && a > b) sp -= GRID_EXTENT;
        } else {
          uint8_t personSide = 0;
          for (uint8_t x=0; x<MAX_PEOPLE; x++) {
            if (!known_people[x].real() || known_people[x].positive != pos) continue;
            float bc = known_people[x].scaled_bgm();
            float sb = bn/100.0 * b;
            if (abs(bc - sb) < ABSOLUTE_THRESHOLD && known_people[x].height() >= 1 &&
                euclidean_distance(known_people[x].past_position, sp) < 4) {
              personSide = known_people[x].side();
              break;
            }
          }
          if (personSide) {
            // there's another person in the frame, assume this is a split of that person
            if (SIDE(sp) != personSide) {
              if (personSide == 2) sp += GRID_EXTENT;
              else sp -= GRID_EXTENT;
            }
          } else {
            // nobody else in frame, choose the side that has more of the blob
            if (SIDE1(sp)) {
              if (b >= max(2*a, 2)) sp += GRID_EXTENT;
            } else if (a >= max(2*b, 2)) sp -= GRID_EXTENT;
          }
        }
      }

      // ignore new points on side 1 immediately after door opens/closes
      if ((frames_since_door_open < 3 && SIDE(sp) == door_side) ||
          (frames_since_door_open < 5 && door_state != DOOR_OPEN))
        continue;

      for (uint8_t j=0; j<MAX_PEOPLE; j++) {
        // look for first empty slot in past_points to use
        if (!known_people[j].real()) {
          Person p;
          p.past_position = points[i];
          p.max_position = mp;
          p.max_jump = mj;
          p.history = h;
          p.starting_position = sp;
          p.crossed = cross;
          p.reverted = revert;
          p.positive = pos;
          p.total_bgm_conf = bn;
          p.total_fgm_conf = fn;
          p.total_bgm = b;
          p.total_fgm = f;
          p.total_neighbors = n;
          p.total_height = height;
          p.total_width = width;
          p.count = c;
          p.count_frd = cfrd;
          known_people[j] = p;
          break;
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
      SERIAL_PRINTLN(F("c"));
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
//      float avg_avg = 0;
//      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
//        avg_avg += bgPixel(idx);
//      }
//      avg_avg /= 64.0;
//      SERIAL_PRINTLN(avg_avg);

      // print chart of what we saw in 8x8 grid
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        SERIAL_PRINT(F(" "));
        SERIAL_PRINT(pos_pixels[idx] ? F("+") : F("-"));
        if (norm_pixels[idx] < CONFIDENCE_THRESHOLD) {
          SERIAL_PRINT(F("----"));
        } else {
          SERIAL_PRINT(norm_pixels[idx]);
        }
        SERIAL_PRINT(F(" "));
        if (x(idx) == GRID_EXTENT) SERIAL_PRINTLN();
      }
//      SERIAL_PRINTLN(F("avg"));
//      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
//        SERIAL_PRINT(F(" "));
//        SERIAL_PRINT(avg_pixels[idx]);
//        SERIAL_PRINT(F(" "));
//        if (x(idx) == GRID_EXTENT) SERIAL_PRINTLN();
//      }
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
      // implicit alpha of 0.2
      avg_pixels[i] += ((int)roundf(200.0 * std));
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  processSensor();
}
