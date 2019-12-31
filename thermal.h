#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
//  #define TEST_PCBA           // uncomment to print raw amg sensor data
#endif

#define FIRMWARE_VERSION        "V0.8.28"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE_FRD        1.5  // absolute min distance between 2 points (neighbors)
#define MIN_DISTANCE            2.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              3    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define MAX_DOOR_CHANGE_FRAMES  5    // max number of cycles we count after door changes
#define MAX_CLEARED_CYCLES      10   // max number of cycles before we assume frame is empty
#define CONFIDENCE_THRESHOLD    5    // consider a point if we're 5% confident
#define AVG_CONF_THRESHOLD      30   // consider a set of points if we're 30% confident
#define HIGH_CONF_THRESHOLD     60   // consider 60% confidence as very high
#define BACKGROUND_GRADIENT     2.0
#define FOREGROUND_GRADIENT     2.0
#define NUM_STD_DEV             2.0  // max num of std dev to include in trimmed average
#define NORMAL_TEMP_DIFFERENCE  2.0  // temp difference between 2 points within same frame
#define MAX_TEMP_DIFFERENCE     4.0  // max temp difference between 2 matchable people
#define MIN_TEMP                2.0  // ignore all points colder than 2º C
#define MAX_TEMP                45.0 // ignore all points hotter than 45ºC

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
uint8_t neighbors_count[AMG88xx_PIXEL_ARRAY_SIZE];
float global_bgm = 0;
float global_fgm = 0;
float global_variance = 0;
float cavg1 = 0.0;
float cavg2 = 0.0;

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

uint8_t not_axis_distance(uint8_t p1, uint8_t p2) {
  int8_t axisJump = NOT_AXIS(p1) - NOT_AXIS(p2);
  return abs(axisJump);
}

uint8_t max_axis_jump(uint8_t p1, uint8_t p2) {
  uint8_t axisJump = axis_distance(p1, p2);
  uint8_t notAxisJump = not_axis_distance(p1, p2);
  return max(axisJump, notAxisJump);
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
#define DOOR_CLOSE_EVENT  1

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
  if (door_state == DOOR_OPEN) return frames_since_door_open <= 1;
  else return readDoorState() == DOOR_OPEN;
}

typedef struct {
  uint8_t   past_position;        //:7 0-63 + UNDEF_POINT
  uint8_t   starting_position;    //:6 0-63
  uint8_t   history;              //:4 1-10
  uint8_t   max_temp_drift;       //:6 0-60
  uint8_t   total_neighbors;      //:7 0-80
  uint8_t   total_height;         //:7 0-80
  uint8_t   total_width;          //:7 0-80

  uint16_t  total_conf        :10;  // 0-1000
  uint8_t   forgotten_count   :2;   // 0-3
  uint8_t   count             :4;   // 1-7

  uint8_t   crossed           :4;   // 0-9
  uint8_t   max_jump          :3;   // 0-7
  bool      reverted          :1;   // 0-1

  float     total_raw_temp;
  float     total_variance;
  float     total_bgm;
  float     total_fgm;

  uint8_t   count_end;
  uint16_t  count_start;

  bool resetIfNecessary() {
    if (count > MIN_HISTORY) {
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
    total_variance = variance();
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
  float     variance()  { return total_variance/((float)count); };
  float     neighbors() { return (float)total_neighbors/(float)count; };
  float     height() { return (float)total_height/(float)count; };
  float     width()  { return (float)total_width/(float)count; };
  float     total_distance() {
    return euclidean_distance(starting_position, past_position);
  };

  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };

  #define METALENGTH  53
  void generateMeta(char *meta) {
    sprintf(meta, "%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%dx%d",
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
      int(variance()*100.0),              // 4  1000
      max_temp_drift,                     // 2  99
      forgotten_count                     // 1  3
    );                                    // + 14 'x' + 1 null => 53 total
  };

  void revert() {
    char rBuf[3];
    char meta[METALENGTH];
    generateMeta(meta);
    sprintf(rBuf, "r%d", crossed);
    publish(rBuf, meta, RETRY_COUNT);
  };

  bool checkForDoorClose() {
    // This could possibly fail if person's hand is on door knob opening door and sensor
    // detects that as a person. We'll see hand go from 1->2, and then get dropped as door
    // opens, and this if block will prevent it from reverting properly.
    if (int(confidence()) > HIGH_CONF_THRESHOLD) {
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

    if (checkForDoorClose()) {
      if ((side() != door_side || !pointOnBorder(past_position)) &&
          ((reverted && starting_side() != door_side) ||
          (!reverted && starting_side() == door_side))) {
        // door just closed and point is on side 2 or right in the middle,
        // revert if it was previously reverted from side 2 or was a non-reverted fake entry
        revert();
        reverted = !reverted;
        return true;
      }
    } else if ((reverted && side() == starting_side()) ||
              (!reverted && side() != starting_side())) {
      // we had previously reverted this point, but it came back and made it through,
      // or point never reverted but is back on the OG starting side
      revert();
      reverted = !reverted;
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
      } else if (eventType == DOOR_CLOSE_EVENT && door_side == 2) {
        publish("1", meta, RETRY_COUNT);
        return;
      }
    } else {
      if (eventType == FRD_EVENT) {
        crossed = publish(door_state == DOOR_OPEN ? "2" : "a2", meta, RETRY_COUNT);
        int s = past_position + GRID_EXTENT;
        starting_position = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
      } else if (eventType == DOOR_CLOSE_EVENT && door_side == 1) {
        publish("2", meta, RETRY_COUNT);
        return;
      }
    }
    if (old_crossed) crossed = 0;
    reverted = false;
    history = 1;
    max_jump = 0;
    max_temp_drift = 0;
    count_start = 1;
    count_end = 0;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  bool publishMaybeEvent() {
    if (!real() || int(confidence()) < CONFIDENCE_THRESHOLD) return false;

    if (history >= MIN_HISTORY && (!crossed || !reverted)) {
      if (starting_side() != side()) {
        if (door_state != DOOR_OPEN && checkForDoorClose()) {
          // publish full event (not a2) even if door is closed
          publishPacket(DOOR_CLOSE_EVENT);
        } else {
          publishPacket(FRD_EVENT);
        }
        return true;
      }
    }

    return false;
  };

  void forget() {
    checkForRevert() || publishMaybeEvent();
  };
} Person;

Person UNDEF_PERSON;
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

void recheckPoint(uint8_t x, uint8_t i) {
  if (samePoints(x, i)) neighbors_count[i]++;
}

uint8_t findClosestPerson(Person *arr, uint8_t i, float maxDistance, float maxTemp) {
  uint8_t p = UNDEF_POINT;
  float minTemp = 1.0;
  for (uint8_t x=0; x<MAX_PEOPLE; x++) {
    if (arr[x].real()) {
      float dist = euclidean_distance(arr[x].past_position, i);
      if (dist > maxDistance) continue;

      float tempDiff = diffFromPerson(i, arr[x]);
      if (tempDiff > maxTemp) continue;

      float tempRatio = tempDiff/maxTemp;
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
  float maxScore = 1.0;
  for (uint8_t x=0; x<MAX_PEOPLE; x++) {
    Person p = known_people[x];
    if (p.real() && p.history > 1) {
      if (euclidean_distance(p.past_position, i) > 4.0) continue;
      float score = p.height() * p.width();
      if (score > maxScore) {
        a = p;
        maxScore = score;
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
        p.history > MIN_HISTORY && pointOnBorder(p.past_position) &&
        int(p.confidence()) > CONFIDENCE_THRESHOLD) {
      p.publishPacket(FRD_EVENT);
      known_people[i] = p; // update known_people array
    }
  }
}

bool checkDoorState() {
  uint8_t last_door_state = door_state;
  door_state = readDoorState();

  if (door_state != last_published_door_state && publish(
    (door_state == DOOR_CLOSED ? "d0" : (door_state == DOOR_OPEN ? "d1" : "d2")), "0", 0)) {
    last_published_door_state = door_state;
  }

  if (last_door_state != door_state) {
    frames_since_door_open = 0;
    return true;
  }

  return false;
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

float trimMean(float sum, float sq_sum, uint8_t cnt, uint8_t side) {
  float mean = sum/((float)cnt);
  float variance = (sq_sum - (sq(sum)/((float)cnt)))/((float)(cnt - 1));
  variance = sqrt(variance);
  global_variance = max(global_variance, variance);
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

// calculate difference from foreground
float fgDiff(uint8_t i) {
  if (raw_pixels[i] < MIN_TEMP || raw_pixels[i] > MAX_TEMP) return 0.0;
  float fgmt1 = abs(raw_pixels[i] - cavg1);
  float fgmt2 = abs(raw_pixels[i] - cavg2);
  return min(fgmt1, fgmt2);
}

// calculate difference from background
float bgDiff(uint8_t i) {
  if (raw_pixels[i] < MIN_TEMP || raw_pixels[i] > MAX_TEMP) return 0.0;
  float std = raw_pixels[i] - bgPixel(i);
  return abs(std);
}

uint8_t calcGradient(float diff, float scale) {
  if (diff < 0.9 || diff > 10.0) return 0;
  diff /= scale;
  if (diff > 0.995) return 100;
  return int(diff*100.0);
}

// calculate foreground gradient percent
uint8_t calcFgm(uint8_t i) {
  return calcGradient(fgDiff(i), global_fgm);
}

// calculate background gradient percent
uint8_t calcBgm(uint8_t i) {
  return calcGradient(bgDiff(i), global_bgm);
}

// calculate foreground gradient scale
void calculateFgm() {
  global_fgm = FOREGROUND_GRADIENT;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
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
  uint8_t total1 = 0;
  uint8_t total2 = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (raw_pixels[i] < MIN_TEMP || raw_pixels[i] > MAX_TEMP) continue;
    if (SIDE1(i)) {
      x_sum1 += raw_pixels[i];
      sq_sum1 += sq(raw_pixels[i]);
      total1++;
    } else {
      x_sum2 += raw_pixels[i];
      sq_sum2 += sq(raw_pixels[i]);
      total2++;
    }
  }

  if (total1 <= 1 || total2 <= 1) return false;

  // calculate trimmed average
  global_variance = 0.0;
  cavg1 = trimMean(x_sum1, sq_sum1, total1, 1);
  cavg2 = trimMean(x_sum2, sq_sum2, total2, 2);

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
    norm_pixels[i] = min(bgm, fgm);
  }

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    neighbors_count[i] = 0;

    if (norm_pixels[i] < CONFIDENCE_THRESHOLD) continue;

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

float calculateNewBackground(uint8_t i) {
  // implicit alpha of 0.001
  float std = raw_pixels[i] - bgPixel(i);

  if (raw_pixels[i] < MIN_TEMP || raw_pixels[i] > MAX_TEMP) return std;

  if (fgDiff(i) < global_variance) {
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES && norm_pixels[i] < 50)
      return std * 50.0; // alpha = 0.05
    if (cycles_since_person == MAX_CLEARED_CYCLES)
      return std * 10.0; // alpha = 0.01
  }

  if (cycles_since_person == 0) {
    for (uint8_t x=0; x<MAX_PEOPLE; x++) {
      if (known_people[x].real() && known_people[x].total_count() > 5 &&
            samePoints(known_people[x].past_position, i) &&
            int(known_people[x].confidence()) > 50 &&
            euclidean_distance(known_people[x].past_position, i) < MAX_DISTANCE &&
            (known_people[x].total_distance() > MIN_DISTANCE_FRD ||
                    known_people[x].fgm() > 1.5*known_people[x].variance())) {
        // point has moved or is significantly higher than variance
        // decrease alpha to 0.00001
        return std * 0.01;
        break;
      }
    }
  }

  return std;
}

void updateBgAverage() {
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    int32_t temp = ((int)avg_pixels[i]) + ((int)roundf(calculateNewBackground(i)));
    avg_pixels[i] = temp;
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
      int8_t td = norm_pixels[ordered_indexes[j]] - norm_pixels[i];
      if (td < 15) {
        if (abs(neighbors_count[i] - neighbors_count[ordered_indexes[j]]) > 1) {
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
            // we're debating between 2 points on either side of border.
            // first try to find which side this person was previously on to avoid
            // flip-flopping. If none found, prefer the side that's on door side
            uint8_t x = findClosestPerson(known_people, i, MIN_DISTANCE,
                          NORMAL_TEMP_DIFFERENCE);
            if (x == UNDEF_POINT || known_people[x].history < 2) {
              if (td < 5 && SIDE(i) == door_side) edge1++;
            } else if (SIDE(i) == known_people[x].side()) edge1++;
          }

          added = edge1 > edge2;
        }
        if (added) {
          norm_pixels[i] = norm_pixels[ordered_indexes[j]];
          // insert point i in front of j
          for (int8_t x=z; x>j; x--) {
            ordered_indexes[x] = ordered_indexes[x-1];
          }
          ordered_indexes[j] = i;
          break;
        }
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
      for (uint8_t k=y+1; k<active_pixel_count; k++) {
        // scan all known points after current_point to find neighbors to point x
        if (ordered_indexes[k] != UNDEF_POINT &&
              samePoints(ordered_indexes[k], ordered_indexes_temp[x]) &&
              euclidean_distance(ordered_indexes[k], ordered_indexes_temp[x]) <
                  MIN_DISTANCE_FRD) {
          ordered_indexes_temp[sorted_size] = ordered_indexes[k];
          ordered_indexes[k] = UNDEF_POINT;
          sorted_size++;
        }
      }
    }
  }

  return total_masses;
}

void forget_person(uint8_t idx, Person *temp_forgotten_people, uint8_t *pairs,
                    uint8_t &temp_forgotten_num) {
  Person p = known_people[idx];
  if (p.forgotten_count < 3 && int(p.confidence()) > AVG_CONF_THRESHOLD &&
        (p.checkForRevert() || axis_distance(p.starting_position, p.past_position) > 1)) {
    p.publishMaybeEvent();
    p.forgotten_count++;
    temp_forgotten_people[temp_forgotten_num] = p;
    temp_forgotten_num++;
  }
  pairs[idx] = UNDEF_POINT;
  known_people[idx] = UNDEF_PERSON;
}

bool remember_person(Person *arr, uint8_t point, uint8_t &h, uint8_t &sp,
                      uint8_t &mj, uint8_t &md, uint8_t &cross, bool &revert,
                      uint16_t &conf, float &b, float &f, float &rt, uint8_t &n,
                      uint8_t &height, uint8_t &width, uint8_t &c, uint16_t &cstart,
                      uint8_t &cend, float &v, uint8_t &fc) {
  uint8_t pi = findClosestPerson(arr, point, MAX_DISTANCE, MAX_TEMP_DIFFERENCE);
  if (pi != UNDEF_POINT) {
    Person p = arr[pi];

    uint8_t ppaxis = AXIS(p.past_position);
    if ((SIDE1(p.starting_position) && ppaxis-1 > AXIS(point)) ||
        (SIDE2(p.starting_position) && ppaxis+1 < AXIS(point))) {
      // this point is moved behind previous position, just start over
      return false;
    }

    if (p.history < MIN_HISTORY && p.side() != p.starting_side() &&
          pointOnSmallBorder(p.past_position)) {
      uint8_t normalized_axis = ppaxis > 4 ? GRID_EXTENT+1 - ppaxis : ppaxis;
      if (p.history <= (MIN_HISTORY - normalized_axis)) {
        // impossible for this person to ever do anything useful with its life, kill it
        return false;
      }
    }

    // can't move if the old point is closer to raw temp than the new point
    float tempDiff = diffFromPerson(point, p);
    if ((norm_pixels[p.past_position] < CONFIDENCE_THRESHOLD ||
        abs(norm_pixels[p.past_position] - norm_pixels[point]) > 15) &&
        diffFromPerson(p.past_position, p) < tempDiff) {
      return false;
    }

    if (p.count > 1) p.resetTotals();

    // point is ahead of starting point at least
    sp = p.starting_position;
    h = min(p.history, MIN_HISTORY);

    uint8_t axisJump = max_axis_jump(p.past_position, point);
    mj = max(axisJump, p.max_jump);

    uint8_t tempDrift = (int)roundf(tempDiff * 10.0);
    md = max(p.max_temp_drift, tempDrift);

    cross = p.crossed;
    revert = p.reverted;
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

void createNewPerson(uint8_t pp, uint8_t mj, uint8_t md, uint8_t h, uint8_t sp,
                    uint8_t cross, bool revert, float rt, uint16_t conf, float b, float f,
                    float v, uint8_t n, uint8_t height, uint8_t width, uint8_t c,
                    uint16_t cstart, uint8_t cend, uint8_t fc, uint8_t j) {
  Person p;
  p.past_position = pp;
  p.starting_position = sp;
  p.max_jump = mj;
  p.max_temp_drift = md;
  p.history = h;
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

  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    taken[i] = 0;
    pairs[i] = UNDEF_POINT;
    temp_forgotten_people[i] = UNDEF_PERSON;
  }

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT (forget_person(idx, temp_forgotten_people, pairs, temp_forgotten_num))

  for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
    Person p = known_people[idx];

    if (!p.real()) continue;
    if (p.resetIfNecessary()) known_people[idx] = p;

    uint8_t min_index = UNDEF_POINT;
    float min_score = 100;
    float conf = p.confidence();
    uint8_t sp_axis = AXIS(p.past_position);

    if (p.history < MIN_HISTORY && p.side() != p.starting_side() &&
          pointOnSmallBorder(p.past_position)) {
      uint8_t normalized_axis = sp_axis > 4 ? GRID_EXTENT+1 - sp_axis : sp_axis;
      if (p.history <= (MIN_HISTORY - normalized_axis)) {
        // impossible for this person to ever do anything useful with its life, kill it
        FORGET_POINT;
        continue;
      }
    }

    for (uint8_t j=0; j<total_masses; j++) {
      if (max_axis_jump(points[j], p.past_position) > 4) continue;

      // can't shift more than 5º at once
      float tempDiff = diffFromPerson(points[j], p);
      if (tempDiff > MAX_TEMP_DIFFERENCE) continue;

      // can't shift more than 2º if bigger than 30% conf gap or both points are on edge
      int8_t confDiff = int(conf) - norm_pixels[points[j]];
      bool confThresholdMet = abs(confDiff) > AVG_CONF_THRESHOLD;
      if (tempDiff > NORMAL_TEMP_DIFFERENCE && (confThresholdMet ||
            (pointOnSmallBorder(p.past_position) && pointOnSmallBorder(points[j])))) {
        continue;
      }

      // can't move if the old point is closer to raw temp than the new point
      if (norm_pixels[p.past_position] < CONFIDENCE_THRESHOLD ||
          abs(norm_pixels[p.past_position] - norm_pixels[points[j]]) > 15) {
        float tempDiffOld = diffFromPerson(p.past_position, p);
        if (tempDiffOld < tempDiff && (confThresholdMet ||
              tempDiffOld + NORMAL_TEMP_DIFFERENCE < tempDiff)) {
          continue;
        }
      }

      float ratioP = min(((float)norm_pixels[points[j]])/conf,
                         conf/((float)norm_pixels[points[j]]));
      if (p.crossed) ratioP /= 2.0; // ratio matters less once point is crossed
      float directionBonus = 0;
      bool crossedInMiddle = p.crossed && !pointOnSmallBorder(p.starting_position);
      uint8_t np_axis = AXIS(points[j]);
      if (np_axis == sp_axis) directionBonus = 0.05;
      else if (SIDE1(p.starting_position)) {
        if (crossedInMiddle) {
          if (np_axis < sp_axis) directionBonus = 0.1;
        } else if (np_axis > sp_axis) {
          directionBonus = 0.1;
        }
      } else { // side 2
        if (crossedInMiddle) {
          if (np_axis > sp_axis) directionBonus = 0.1;
        } else if (np_axis < sp_axis) {
          directionBonus = 0.1;
        }
      }

      if (!crossedInMiddle) {
        directionBonus += (0.05*neighbors_count[points[j]]);
      }

      float d = euclidean_distance(p.past_position, points[j]);
      float score = sq(d/5.0) + sq(tempDiff/MAX_TEMP_DIFFERENCE) -
                      sq(ratioP) - directionBonus;
      if (min_score - score > 0.05 || (score - min_score < 0.05 &&
            tempDiff < diffFromPerson(points[min_index], p))) {
        // either score is less than min score, or if it's similar,
        // choose the point with more similar raw temp
        min_score = score;
        min_index = j;
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

  for (uint8_t i=0; i<total_masses; i++) {
    if (taken[i] > 1) {
      // more than one past point is trying to match with this single current point...
      float max_score = 0.0;
      uint8_t max_idx = UNDEF_POINT;
      for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
        Person p = known_people[idx];
        if (p.real() && pairs[idx] == i) {
          float score = 0.0;
          float d = euclidean_distance(p.past_position, points[i]);
          uint8_t axis = AXIS(p.past_position);
          uint8_t naxis = NOT_AXIS(p.past_position);
          uint8_t maxis = min(int(d), 2);
          if (p.crossed &&
              (axis <= maxis || ((GRID_EXTENT+1) - axis) <= maxis ||
              naxis <= maxis || ((GRID_EXTENT+1) - naxis) <= maxis)) {
            // do nothing
          } else {
            float conf = p.confidence();
            score = conf/100.0;
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
            float tempDiff = diffFromPerson(points[i], p);
            directionBonus -= sq(tempDiff/MAX_TEMP_DIFFERENCE);
            if (p.max_jump > 1) {
              directionBonus -= sq(float(p.max_jump - 1)/MAX_DISTANCE);
            }
            float td = axis_distance(p.starting_position, p.past_position);
            if (td <= MIN_DISTANCE_FRD) {
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
            uint8_t axisJump = max_axis_jump(p.past_position, points[i]);
            p.max_jump = max(axisJump, p.max_jump);

            if ((SIDE1(p.starting_position) && AXIS(points[i]) < AXIS(p.past_position)) ||
                (SIDE2(p.starting_position) && AXIS(points[i]) > AXIS(p.past_position))) {
              // point moved backwards
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
                  p.count_start = 0;
                }
                // reset start position, unless point is in a revert crisis
                if (!p.reverted) {
                  p.starting_position = points[i];
                }
                p.history = 1;
                p.max_jump = 0;
                p.max_temp_drift = 0;
              } else if (p.history > 1) {
                // point moved backwards a little bit, decrement history
                p.history--;
              }
            } else if (AXIS(points[i]) != AXIS(p.past_position)) {
              // "always forward, forward always" - Luke Cage
              p.history++;
              if (SIDE(points[i]) != p.side() || p.history > 9) {
                // point just crossed threshold, let's reduce its history to force
                // it to spend another cycle on this side before we count the event
                p.history = min(p.history, MIN_HISTORY);
              }
            }
            p.past_position = points[i];
          }
          if (p.count) {
            uint8_t td = (int)roundf(diffFromPerson(points[i], p) * 10.0);
            p.max_temp_drift = max(p.max_temp_drift, td);
          }
          p.total_raw_temp += raw_pixels[points[i]];
          p.total_conf += norm_pixels[points[i]];
          p.total_bgm += bgDiff(points[i]);
          p.total_fgm += fgDiff(points[i]);
          p.total_variance += global_variance;
          p.total_neighbors += neighbors_count[points[i]];
          p.total_height += calcHeight(points[i]);
          p.total_width += calcWidth(points[i]);
          p.count++;
          if (p.side() == p.starting_side()) {
            p.count_start++;
            p.count_end = 0;
          } else {
            p.count_end++;
          }
          cycles_since_person = 0;
          known_people[idx] = p;
          break;
        }
      }
    } else if (taken[i] == 0 && norm_pixels[points[i]] > CONFIDENCE_THRESHOLD) {
      // new point appeared (no past point found), start tracking it
      uint8_t sp = points[i];
      uint8_t mj = 0;
      uint8_t md = 0;
      uint8_t h = 1;
      uint8_t cross = 0;
      bool revert = false;
      uint16_t conf = norm_pixels[sp];
      float rt = raw_pixels[sp];
      float b = bgDiff(sp);
      float f = fgDiff(sp);
      float v = global_variance;
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
        if (remember_person(temp_forgotten_people, points[i], h, sp, mj, md, cross,
              revert, conf, b, f, rt, n, height, width, c, cstart, cend, v, fc)) {
          retroMatched = true;
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        if (remember_person(forgotten_people, points[i], h, sp, mj, md, cross,
              revert, conf, b, f, rt, n, height, width, c, cstart, cend, v, fc)) {
          retroMatched = true;
        }
      }

      if (!retroMatched && !pointOnBorder(sp)) {
        // if point is right in middle, drag it to the side it appears to be coming from
        if (doorJustOpened()) {
          if (norm_pixels[sp] > HIGH_CONF_THRESHOLD) {
            uint8_t na = max(norm_pixels[sp - GRID_EXTENT], HIGH_CONF_THRESHOLD);
            uint8_t nb = max(norm_pixels[sp + GRID_EXTENT], HIGH_CONF_THRESHOLD);
            if (SIDE1(sp)) {
              if (door_side == 1 && nb > na) sp += GRID_EXTENT;
            } else if (door_side == 2 && na > nb) sp -= GRID_EXTENT;
          }
        } else {
          Person x = findLargestPerson(sp);
          if (x.real()) {
            // there's another person in the frame, assume this is a split of that person
            if (!x.crossed || (x.count_start > 5 && !x.count_end)) {
              // either first person hasn't crossed yet or they've been on other side for
              // > 0.5 seconds. Either way, make sure new point is on the same side
              if (SIDE(sp) != x.starting_side()) {
                if (x.starting_side() == 2) sp += GRID_EXTENT;
                else sp -= GRID_EXTENT;
              }
            } else if (SIDE(sp) == x.starting_side()) {
              // point just crossed, put new point on opposite side so it counts on its own
              if (x.starting_side() == 1) sp += GRID_EXTENT;
              else sp -= GRID_EXTENT;
            }
          }
        }
      }

      // ignore new points on side 1 immediately after door opens/closes
      if ((frames_since_door_open < 2 && SIDE(sp) == door_side) ||
          (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES && door_state == DOOR_CLOSED)) {
        continue;
      }

      float minConf = 200.0;
      uint8_t minIndex = UNDEF_POINT;
      for (uint8_t j=0; j<MAX_PEOPLE; j++) {
        // look for first empty slot in past_points to use
        if (!known_people[j].real()) {
          createNewPerson(points[i], mj, md, h, sp, cross, revert, rt, conf, b, f, v, n,
                          height, width, c, cstart, cend, fc, j);
          minIndex = UNDEF_POINT;
          break;
        } else {
          float pConf = known_people[j].confidence();
          if (pConf < minConf) {
            minConf = pConf;
            minIndex = j;
          }
        }
      }
      if (minIndex != UNDEF_POINT && (((float)conf)/(float)c) > minConf) {
        // replace lower conf slot with this new point
        createNewPerson(points[i], mj, md, h, sp, cross, revert, rt, conf, b, f, v, n,
                        height, width, c, cstart, cend, fc, minIndex);
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

  updateBgAverage();

  beatHeart();

  if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) {
    frames_since_door_open++;
  }
  if (cycles_since_person < MAX_CLEARED_CYCLES) {
    cycles_since_person++;
  }

  // wrap up with debugging output

  #ifdef PRINT_RAW_DATA
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
      SERIAL_PRINTLN(global_variance);
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
  UNDEF_PERSON.past_position = UNDEF_POINT;

  amg.begin(AMG_ADDR);

  // setup reed switches
  DDRD  = DDRD  & B11100111;  // set pins 3 and 4 as inputs
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
      int32_t temp = ((int)avg_pixels[i]) + ((int)roundf(300.0 * std));
      avg_pixels[i] = temp;
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  processSensor();
}
