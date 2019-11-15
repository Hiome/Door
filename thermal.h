#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
//  #define TEST_PCBA           // uncomment to print raw amg sensor data
#endif

#define FIRMWARE_VERSION        "V0.8.8"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE_FRD        1.5  // absolute min distance between 2 points (neighbors)
#define MIN_DISTANCE            2.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define MAX_CLEARED_CYCLES      10   // max number of cycles before we assume frame is empty
#define CONFIDENCE_THRESHOLD    20   // consider a point if we're 30% confident
#define AVG_CONF_THRESHOLD      30   // consider a set of points if we're 40% confident
#define HIGH_CONF_THRESHOLD     60   // consider 60% confidence as very high
#define BACKGROUND_GRADIENT     2.0
#define FOREGROUND_GRADIENT     2.0
#define NUM_STD_DEV             2.0  // max num of std dev to include in trimmed average
#define MIN_TRAVEL_RATIO        20
#define NORMAL_TEMP_DIFFERENCE  2.0  // temp difference between 2 points within same frame
#define MAX_TEMP_DIFFERENCE     4.0  // max temp difference between 2 matchable people

#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;

#ifdef R3
  #define AMG_ADDR              0x68
#else
  #define AMG_ADDR              0x69
#endif

float raw_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint16_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint8_t norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int8_t neighbors_count[AMG88xx_PIXEL_ARRAY_SIZE];
float global_bgm = 0;
float global_fgm = 0;
float cavg1 = 0.0;
float cavg2 = 0.0;
float var1 = 0.0;
float var2 = 0.0;

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

float euclidean_distance(uint8_t p1, uint8_t p2) {
  // calculate euclidean distance instead
  int8_t yd = y(p2) - y(p1);
  int8_t xd = x(p2) - x(p1);
  return (float)sqrt(sq(yd) + sq(xd));
}

uint8_t axis_distance(uint8_t p1, uint8_t p2) {
  int8_t axisJump = AXIS(p1) - AXIS(p2);
  return abs(axisJump);
}

uint8_t possible_neighbors(uint8_t x) {
  bool lrEdge = pointOnLREdge(x);
  bool tbEdge = pointOnEdge(x);
  if (lrEdge || tbEdge) {
    if (lrEdge && tbEdge) return 3;
    return 5;
  }
  return 8;
}

uint8_t cycles_since_person = MAX_CLEARED_CYCLES;
uint8_t cycles_since_forgotten = MAX_EMPTY_CYCLES;

#define DOOR_CLOSED 0
#define DOOR_OPEN   1
#define DOOR_AJAR   2
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

typedef struct Person {
  uint8_t   past_position;          // 0-63 + UNDEF_POINT
  uint8_t   starting_position :6;   // 0-63
  uint8_t   max_position      :6;   // 0-63
  uint8_t   max_jump          :3;   // 1-7
  uint8_t   history           :4;   // 1-10
  uint8_t   crossed           :4;   // 0-9
  bool      reverted          :1;   // 0-1
  uint8_t   max_temp_jump     :3;   // 0-7
  uint8_t   starting_temp     :6;   // 0-63
  uint8_t   past_temp         :6;   // 0-63
  float     total_raw_temp;
  uint8_t   total_variance;
  float     total_bgm;
  float     total_fgm;
  uint8_t   total_neighbors;        // 0-80
  uint8_t   total_height;           // 0-80
  uint8_t   total_width;            // 0-80
  uint16_t  total_conf        :10;  // 0-1000
  uint8_t   count             :4;   // 1-7
  uint16_t  count_start;
  uint8_t   count_end;
  uint8_t   forgotten_count   :2;   // 0-3

  bool resetIfNecessary() {
    if (count > 5) {
      resetTotals();
      if (count_start > 60000) count_start = 10000;
      if (count_end > 240) count_end = 100;
      return true;
    }
    return false;
  };

  void resetTotals() {
    history = min(history, MIN_HISTORY);
    total_bgm = bgm();
    total_fgm = fgm();
    total_raw_temp = raw_temp();
    total_variance = int(roundf(variance()));
    total_conf = int(confidence());
    total_neighbors = int(neighbors());
    total_height = int(height());
    total_width = int(width());
    count = 1;
  };

  bool      real() { return past_position != UNDEF_POINT; };
  uint16_t  total_count() { return count_start + count_end; };
  float     confidence() { return ((float)total_conf)/((float)count); };
  float     bgm() { return total_bgm/((float)count); };
  float     fgm() { return total_fgm/((float)count); };
  float     raw_temp()  { return total_raw_temp/((float)count); };
  float     variance()  { return (float)total_variance/((float)count); };
  float     neighbors() { return (float)total_neighbors/(float)count; };
  float     height() { return (float)total_height/(float)count; };
  float     width()  { return (float)total_width/(float)count; };
  float     total_distance() {
    return euclidean_distance(starting_position, past_position);
  };

  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };
  uint8_t   temp_drift() { return abs(starting_temp - past_temp); };

  #define METALENGTH  52
  void generateMeta(char *meta) {
    sprintf(meta, "%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%d",
      int(confidence()),                  // 3  100
      int(bgm()*100.0),                   // 4  1020
      int(fgm()*100.0),                   // 4  1020
      starting_position,                  // 2  23
      past_position,                      // 2  37
      history,                            // 1  8
      count_start,                        // 5  65536
      count_end,                          // 3  240
      int(neighbors()*10.0),              // 2  80
      int(height()*10.0),                 // 2  70
      int(width()*10.0),                  // 2  70
      max_jump,                           // 1  5
      temp_drift(),                       // 1  4
      max_temp_jump,                      // 1  9
      int(roundf(variance())),            // 2  10
      forgotten_count                     // 1  3
    );                                    // + 15 'x' + 1 null => 52 total
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
    if (starting_side() == door_side && confidence() > HIGH_CONF_THRESHOLD) {
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
    reverted = false;
    history = 1;
    max_jump = 0;
    max_temp_jump = 0;
    starting_temp = past_temp;
    max_position = past_position;
    count_start = count_end;
    count_end = 0;
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
      } else if (euclidean_distance(starting_position, past_position) > MAX_DISTANCE) {
        revert(MAYBE_REVERT);
        return false;
      } else return false;
      return true;
    } else if (crossed || reverted) {
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

float diffFromPerson(uint8_t a, Person b) {
  float d = raw_pixels[(a)] - b.raw_temp();
  return abs(d);
}

bool samePoints(uint8_t a, uint8_t b) {
  return norm_pixels[(a)] > CONFIDENCE_THRESHOLD &&
          abs(raw_pixels[(a)] - raw_pixels[(b)]) < NORMAL_TEMP_DIFFERENCE;
}

uint8_t findClosestPerson(Person *arr, uint8_t i, float maxDistance) {
  uint8_t p = UNDEF_POINT;
  float minTemp = 2.0;
  for (uint8_t x=0; x<MAX_PEOPLE; x++) {
    if (arr[x].real()) {
      float dist = euclidean_distance(arr[x].past_position, i);
      if (dist > maxDistance) continue;
      float tempDiff = diffFromPerson(i, arr[x]);
      float tempRatio = tempDiff/NORMAL_TEMP_DIFFERENCE;
      tempRatio = max(tempRatio, 0.1);
      float distRatio = dist/maxDistance;
      distRatio = max(distRatio, 0.1);
      tempDiff = tempRatio * distRatio;
      if (tempDiff < minTemp) {
        p = x;
        minTemp = tempDiff;
      }
    }
  }
  return p;
}

Person findLargestPerson(uint8_t i) {
  Person a = UNDEF_PERSON;
  float minScore = 100.0;
  for (uint8_t x=0; x<MAX_PEOPLE; x++) {
    Person p = known_people[x]; 
    if (p.real() && p.history > 1 && p.height() >= 1) {
      float dist = euclidean_distance(p.past_position, i);
      if (dist > 4.0) continue;
      float score = dist - (p.height() * p.width());
      if (score < minScore) {
        a = p;
        minScore = score;
      }
    }
  }
  return a;
}

uint8_t pointsAbove(uint8_t i) {
  uint8_t height = 0;
  for (int8_t x = i - GRID_EXTENT; x >= 0; x -= GRID_EXTENT) {
    if (samePoints(x, i)) height++;
    else break;
  }
  return height;
}

uint8_t pointsBelow(uint8_t i) {
  uint8_t height = 0;
  for (uint8_t x = i + GRID_EXTENT; x < AMG88xx_PIXEL_ARRAY_SIZE; x += GRID_EXTENT) {
    if (samePoints(x, i)) height++;
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
  for (uint8_t x = i+1; x < AMG88xx_PIXEL_ARRAY_SIZE && AXIS(x) == axis; x++) {
    if (samePoints(x, i)) width++;
    else break;
  }
  for (int8_t x = i-1; x >= 0 && AXIS(x) == axis; x--) {
    if (samePoints(x, i)) width++;
    else break;
  }
  return width;
}

void publishEvents() {
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    Person p = known_people[i];
    if (p.real() && p.starting_side() != p.side() && (!p.crossed || !p.reverted) &&
        p.confidence() > AVG_CONF_THRESHOLD && p.history > MIN_HISTORY &&
        pointOnBorder(p.past_position) && p.total_distance() > MIN_DISTANCE) {
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

  if (door_state != last_published_door_state) {
    publish(
      (door_state == DOOR_CLOSED ? "d0" : (door_state == DOOR_OPEN ? "d1" : "d2")), "0", 0);
    last_published_door_state = door_state;
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
    cycles_since_forgotten = MAX_EMPTY_CYCLES;
  }
}

float trimMean(float sum, float sq_sum, uint8_t side) {
  float mean = sum/32.0;
  float variance = (sq_sum - (sq(sum)/32.0))/31.0;
  variance = sqrt(variance);
  if (side == 1) var1 = variance;
  else var2 = variance;
  float lowerBound = mean - (NUM_STD_DEV * variance);
  float upperBound = mean + (NUM_STD_DEV * variance);
  float cavg = 0.0;
  uint8_t total = 0;
  for (uint8_t i=(side==1 ? 0 : 32); i<(side==1 ? 32 : AMG88xx_PIXEL_ARRAY_SIZE); i++) {
    if (raw_pixels[i] > lowerBound && raw_pixels[i] < upperBound) {
      cavg += raw_pixels[i];
      total++;
    }
  }
  return total > 0 ? cavg/(float)total : mean;
}

void recheckPoint(uint8_t x, uint8_t i) {
  if (samePoints(x, i)) neighbors_count[i]++;
}

// calculate difference from foreground
float fgDiff(uint8_t i) {
  float fgmt1 = abs(raw_pixels[i] - cavg1);
  float fgmt2 = abs(raw_pixels[i] - cavg2);
  return min(fgmt1, fgmt2);
}

// calculate difference from background
float bgDiff(uint8_t i) {
  float std = raw_pixels[i] - bgPixel(i);
  return abs(std);
}

uint8_t calcGradient(uint8_t i, float diff, float scale) {
  if (diff < 0.6) {
    neighbors_count[i] = 0;
    return 0;
  }
  diff /= scale;
  diff = min(diff, 1.0);
  return ((int)roundf(diff*100.0));
}

// calculate foreground gradient percent
uint8_t calcFgm(uint8_t i) {
  if (neighbors_count[i] < 1) return 0;
  return calcGradient(i, fgDiff(i), global_fgm);
}

// calculate background gradient percent
uint8_t calcBgm(uint8_t i) {
  if (neighbors_count[i] < 1) return 0;
  return calcGradient(i, bgDiff(i), global_bgm);
}

// calculate foreground gradient scale
void calculateFgm() {
  global_fgm = FOREGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (neighbors_count[i] < 1) continue;
    float fgmt = fgDiff(i);
    if (global_bgm > 1.0)
      fgmt *= (calcBgm(i)/100.0);
    global_fgm = max(fgmt, global_fgm);
  }
}

// calculate background gradient scale
void calculateBgm() {
  global_bgm = BACKGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (neighbors_count[i] < 1) continue;
    float bgmt = bgDiff(i);
    bgmt *= (calcFgm(i)/100.0);
    global_bgm = max(bgmt, global_bgm);
  }
}

bool normalizePixels() {
  if (!amg.readPixels(raw_pixels)) return false;

  float x_sum1 = 0.0;
  float sq_sum1 = 0.0;
  float x_sum2 = 0.0;
  float sq_sum2 = 0.0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (SIDE1(i)) {
      x_sum1 += raw_pixels[i];
      sq_sum1 += sq(raw_pixels[i]);
    } else {
      x_sum2 += raw_pixels[i];
      sq_sum2 += sq(raw_pixels[i]);
    }

    neighbors_count[i] = 1;
  }

  // calculate trimmed average
  cavg1 = trimMean(x_sum1, sq_sum1, 1);
  cavg2 = trimMean(x_sum2, sq_sum2, 2);

  #ifdef TEST_PCBA
    for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
      SERIAL_PRINT(F(" "));
      SERIAL_PRINT(raw_pixels[idx]);
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
  calculateFgm();
  // run 2 passes to amplify real points over noise
  calculateBgm();
  calculateFgm();
  calculateBgm();
  calculateFgm();

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    uint8_t bgm = calcBgm(i);
    uint8_t fgm = calcFgm(i);
    uint8_t conf = min(bgm, fgm);

    if (conf < CONFIDENCE_THRESHOLD) {
      neighbors_count[i] = 0;
      norm_pixels[i] = 0;
    } else {
      norm_pixels[i] = conf;
    }
  }

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (neighbors_count[i] < 1) continue;

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
  }

  return true;
}

void updateBgAverage() {
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // update average baseline
    // implicit alpha of 0.001
    float std = raw_pixels[i] - bgPixel(i);
    float fgm = calcFgm(i);
    if (fgm < CONFIDENCE_THRESHOLD) std *= 900.0; // fuck it
    else if (frames_since_door_open < 5 && fgm < HIGH_CONF_THRESHOLD) {
      if (door_state != DOOR_OPEN || (frames_since_door_open < 2 && SIDE(i) == door_side)) {
        // door just opened, imprint everything on side 1
        std *= 900.0;
      } else {
        // door is open, either for more than 2 frames or we're on side 2
        std *= 200.0;
      }
    } else if (cycles_since_person == 0) {
      for (uint8_t x=0; x<MAX_PEOPLE; x++) {
        if (known_people[x].real() && known_people[x].total_count() > 5 &&
              known_people[x].confidence() > 50 &&
              known_people[x].total_distance() > MIN_DISTANCE &&
              diffFromPerson(i, known_people[x]) < NORMAL_TEMP_DIFFERENCE &&
              euclidean_distance(known_people[x].past_position, i) < MAX_DISTANCE) {
          std *= 0.1;
          break;
        }
      }
    } else if (cycles_since_person == MAX_CLEARED_CYCLES) {
      // nothing going on, increase alpha to 0.01
      std *= 10.0;
    }
    avg_pixels[i] += ((int)roundf(std));
  }
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
      if (abs(norm_pixels[i] - norm_pixels[ordered_indexes[j]]) < 15) {
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
            uint8_t x = findClosestPerson(known_people, i, MIN_DISTANCE);
            if (x != UNDEF_POINT && SIDE(i) == known_people[x].side()) edge1++;
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
      uint8_t max_added = possible_neighbors(ordered_indexes_temp[x]);
      bool blobEdge = norm_pixels[current_point] - norm_pixels[ordered_indexes_temp[x]] >60;

      for (uint8_t k=y+1; k<active_pixel_count; k++) {
        // scan all known points after current_point to find neighbors to point x
        uint8_t i = ordered_indexes[k];
        if (i != UNDEF_POINT &&
              euclidean_distance(i, ordered_indexes_temp[x]) < MIN_DISTANCE_FRD) {
          if (blobEdge && norm_pixels[i] - norm_pixels[ordered_indexes_temp[x]] > 15) {
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
  if (p.forgotten_count < 3 && p.confidence() > AVG_CONF_THRESHOLD &&
        (p.checkForRevert() || p.history > 1)) {
    p.publishMaybeEvent();
    p.forgotten_count++;
    temp_forgotten_people[temp_forgotten_num] = p;
    temp_forgotten_num++;
  }
  pairs[idx] = UNDEF_POINT;
  known_people[idx] = UNDEF_PERSON;
}

bool remember_person(Person *arr, uint8_t point, uint8_t &h, uint8_t &sp, uint8_t &mp,
                      uint8_t &mj, uint8_t &tj, uint8_t &st, uint8_t &cross, bool &revert,
                      uint16_t &conf, float &b, float &f, float &rt, uint8_t &n,
                      uint8_t &height, uint8_t &width, uint8_t &c, uint16_t &cstart,
                      uint8_t &cend, uint8_t &v, uint8_t &fc) {
  uint8_t pi = findClosestPerson(arr, point, MAX_DISTANCE);
  if (pi != UNDEF_POINT) {
    Person p = arr[pi];
    // if switching sides with low confidence or moving too far, don't pair
    float d = euclidean_distance(p.past_position, point);
    if (SIDE(point) != p.side() && (p.confidence() < AVG_CONF_THRESHOLD ||
          ((float)norm_pixels[sp])/d < MIN_TRAVEL_RATIO)) {
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

    uint8_t axisJump = axis_distance(p.past_position, point);
    mj = max(axisJump, p.max_jump);

    uint8_t tempJump = abs(p.past_temp - ((int)roundf(raw_pixels[point])));
    tj = max(tempJump, p.max_temp_jump);

    cross = p.crossed;
    revert = p.reverted;
    st = p.starting_temp;
    rt += p.total_raw_temp;
    conf += p.total_conf;
    b += p.total_bgm;
    f += p.total_fgm;
    v += p.total_variance;
    n += p.total_neighbors;
    height += p.total_height;
    width += p.total_width;
    c += p.count;
    cstart += p.count_start;
    cend += p.count_end;
    fc += p.forgotten_count;
    arr[pi] = UNDEF_PERSON;
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
        float max_distance = MIN_DISTANCE + (conf/100.0 * min(anei, MIN_DISTANCE));
        float min_score = 100;
        for (uint8_t j=0; j<total_masses; j++) {
          float d = euclidean_distance(p.past_position, points[j]);
          // if switching sides with low confidence, don't pair
          if (SIDE(points[j]) != p.side() && (conf < AVG_CONF_THRESHOLD ||
                ((float)norm_pixels[points[j]])/d < MIN_TRAVEL_RATIO)) {
            continue;
          }

          if (d < max_distance) {
            float ratioP = min(((float)norm_pixels[points[j]])/conf,
                               conf/((float)norm_pixels[points[j]]));
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

            if (norm_pixels[points[j]] < AVG_CONF_THRESHOLD) {
              directionBonus -= (((float)(AVG_CONF_THRESHOLD-norm_pixels[points[j]]))/100.0);
            }

            float tempDiff = diffFromPerson(points[j], p);
            if (tempDiff < NORMAL_TEMP_DIFFERENCE) {
              directionBonus += (0.1*neighbors_count[points[j]]);
            }

            float score = sq(d/max_distance) + sq(tempDiff/MAX_TEMP_DIFFERENCE) - ratioP -
                            directionBonus;
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
          float score = conf/100.0;
          float d = euclidean_distance(p.past_position, points[i]);
          uint8_t axis = AXIS(p.past_position);
          uint8_t naxis = NOT_AXIS(p.past_position);
          if (conf + 5 < AVG_CONF_THRESHOLD || (p.crossed &&
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
            float tempDiff = diffFromPerson(points[i], p);
            directionBonus -= sq(tempDiff/MAX_TEMP_DIFFERENCE);
            if (tempDiff < NORMAL_TEMP_DIFFERENCE)
              directionBonus += (0.1*p.neighbors());
            if (p.max_jump > 1)
              directionBonus -= sq((p.max_jump - 1)/3.0);
            directionBonus -= sq(p.max_temp_jump/MAX_TEMP_DIFFERENCE);
            float td = p.total_distance();
            if (td < 1) {
              if (p.total_count() == 1 || p.crossed) td = 1.0;
              else td = 1.0/((float)(p.total_count() - 1));
            }
            score *= (td + directionBonus);
            score *= p.history;
            float newConf = (float)norm_pixels[points[i]];
            newConf = abs(newConf - conf);
            score *= (1.0 - newConf/100.0);
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
                  (d-d2 < 0.05 && p.history > known_people[max_idx].history)) {
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
            uint8_t axisJump = axis_distance(p.past_position, points[i]);
            p.max_jump = max(axisJump, p.max_jump);

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
                  p.total_raw_temp = 0;
                  p.total_variance = 0;
                  p.total_conf = 0;
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
          uint8_t tempJump = abs(p.past_temp - ((int)roundf(raw_pixels[points[i]])));
          p.max_temp_jump = max(tempJump, p.max_temp_jump);
          p.past_temp = ((int)roundf(raw_pixels[points[i]]));
          p.total_raw_temp += raw_pixels[points[i]];
          p.total_conf += norm_pixels[points[i]];
          p.total_bgm += bgDiff(points[i]);
          p.total_fgm += fgDiff(points[i]);
          p.total_variance += ((int)roundf(max(var1, var2)));
          p.total_neighbors += neighbors_count[points[i]];
          p.total_height += calcHeight(points[i]);
          p.total_width += calcWidth(points[i]);
          p.count++;
          if (SIDE(points[i]) == p.starting_side()) {
            p.count_start++;
            p.count_end = 0;
          } else
            p.count_end++;
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
      uint8_t tj = 0;
      uint8_t st = ((int)roundf(raw_pixels[sp]));
      uint8_t h = 1;
      uint8_t cross = 0;
      bool revert = false;
      uint16_t conf = norm_pixels[sp];
      float rt = raw_pixels[sp];
      float b = global_bgm;
      float f = global_fgm;
      uint8_t v = ((int)roundf(max(var1, var2)));
      uint8_t n = neighbors_count[sp];
      uint8_t height = calcHeight(sp);
      uint8_t width = calcWidth(sp);
      uint8_t c = 1;
      uint16_t cstart = 1;
      uint8_t cend = 0;
      uint8_t fc = 0;
      bool retroMatched = false;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        if (remember_person(temp_forgotten_people, points[i], h, sp, mp, mj, tj, st, cross,
              revert, conf, b, f, rt, n, height, width, c, cstart, cend, v, fc)) {
          retroMatched = true;
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        if (remember_person(forgotten_people, points[i], h, sp, mp, mj, tj, st, cross,
              revert, conf, b, f, rt, n, height, width, c, cstart, cend, v, fc)) {
          retroMatched = true;
        }
      }

      if (!retroMatched && !pointOnBorder(sp)) {
        // if point is right in middle, drag it to the side it appears to be coming from
        uint8_t a = pointsAbove(sp);
        uint8_t b = pointsBelow(sp);
        if (norm_pixels[sp] > HIGH_CONF_THRESHOLD && doorJustOpened()) {
          if (SIDE1(sp)) {
            if (door_side == 1 && b > a) sp += GRID_EXTENT;
          } else if (door_side == 2 && a > b) sp -= GRID_EXTENT;
        } else {
          Person x = findLargestPerson(sp);
          if (x.real()) {
            // there's another person in the frame, assume this is a split of that person
            if (SIDE(sp) != x.starting_side()) {
              if (x.starting_side() == 2) sp += GRID_EXTENT;
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
      if ((frames_since_door_open < 2 && SIDE(sp) == door_side) ||
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
          p.starting_temp = st;
          p.past_temp = ((int)roundf(raw_pixels[points[i]]));
          p.max_temp_jump = tj;
          p.crossed = cross;
          p.reverted = revert;
          p.total_raw_temp = rt;
          p.total_conf = conf;
          p.total_bgm = b;
          p.total_fgm = f;
          p.total_variance = v;
          p.total_neighbors = n;
          p.total_height = height;
          p.total_width = width;
          p.count = c;
          p.count_start = cstart;
          p.count_end = cend;
          p.forgotten_count = fc;
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
    cycles_since_forgotten = 0;
    SERIAL_PRINTLN(F("s"));
  } else if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
    cycles_since_forgotten++;
    if (cycles_since_forgotten == MAX_EMPTY_CYCLES) {
      // clear forgotten points list
      for (uint8_t i=0; i<MAX_PEOPLE; i++) {
        forgotten_people[i] = UNDEF_PERSON;
      }
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

  updateBgAverage();

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
//          SERIAL_PRINT(F("-"));
//          SERIAL_PRINT(neighbors_count[p.past_position]);
          SERIAL_PRINT(F("),"));
        }
      }
      SERIAL_PRINTLN();

      SERIAL_PRINTLN(global_bgm);
      SERIAL_PRINTLN(global_fgm);
      SERIAL_PRINTLN(var1);
      SERIAL_PRINTLN(var2);
//      float avg_avg = 0;
//      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
//        avg_avg += bgPixel(idx);
//      }
//      avg_avg /= 64.0;
//      SERIAL_PRINTLN(avg_avg);

      // print chart of what we saw in 8x8 grid
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        SERIAL_PRINT(F(" "));
        if (norm_pixels[idx] < CONFIDENCE_THRESHOLD) {
          SERIAL_PRINT(F("---"));
        } else {
          if (norm_pixels[idx] < 100) SERIAL_PRINT(F(" "));
          SERIAL_PRINT(norm_pixels[idx]);
        }
//        SERIAL_PRINT(raw_pixels[idx]);
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

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    raw_pixels[i] = 0.0;
  }

  amg.readPixels(raw_pixels);

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] = ((int)roundf(raw_pixels[i] * 1000.0));
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!amg.readPixels(raw_pixels)) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      float std = raw_pixels[i] - bgPixel(i);
      // alpha of 0.3
      avg_pixels[i] += ((int)roundf(300.0 * std));
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  processSensor();
}
