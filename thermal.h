#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
//  #define TEST_PCBA           // uncomment to print raw amg sensor data
//  #define TIME_CYCLES
#endif

#define FIRMWARE_VERSION        "V20.2.13b"
#define YAXIS                        // axis along which we expect points to move (x or y)

#include "thermal_types.h"
#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;

#ifdef R3
  #define AMG_ADDR              0x68
#else
  #define AMG_ADDR              0x69
#endif

const uint8_t GRID_EXTENT            = 8;    // size of grid (8x8)
const uint8_t MIN_HISTORY            = 3;    // min number of times a point needs to be seen
const uint8_t MAX_PEOPLE             = 5;    // most people we support in a single frame
const uint8_t MAX_EMPTY_CYCLES       = 2;    // cycles to remember forgotten points
const uint8_t MAX_FORGOTTEN_COUNT    = 2;    // max number of times allowed to forget someone
const uint8_t MAX_DOOR_CHANGE_FRAMES = 5;    // cycles we keep counting after door changes
const uint8_t MAX_CLEARED_CYCLES     = 10;   // cycles before we assume frame is empty
const uint8_t CONFIDENCE_THRESHOLD   = 5;    // min 5% confidence required
const uint8_t AVG_CONF_THRESHOLD     = 30;   // avg 30% confidence required
const uint8_t MIN_TEMP               = 2;    // ignore all points colder than 2º C
const uint8_t MAX_TEMP               = 45;   // ignore all points hotter than 45ºC
const uint8_t MIN_TEMP_FRD           = 13;
const uint8_t MAX_TEMP_FRD           = 37;
const uint8_t NUM_BUCKETS            = 10;   // 3 + log2(n) * log(n) where n = distinct rows
const float   MAX_DISTANCE           = 2.0;  // max distance that a point is allowed to move
const float   BACKGROUND_GRADIENT    = 2.0;
const float   FOREGROUND_GRADIENT    = 2.0;
const coord_t UNDEF_POINT            = AMG88xx_PIXEL_ARRAY_SIZE + 10;
const idx_t   UNDEF_INDEX            = UNDEF_POINT;

float   raw_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
fint3_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint8_t norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint8_t side1Point = 0;
uint8_t side2Point = 0;
float global_bgm = 0;
float global_fgm = 0;
float global_variance = 0;
float cavg1 = 0;
float cavg2 = 0;

// store in-memory so we don't have to do math every time
const axis_t xcoordinates[AMG88xx_PIXEL_ARRAY_SIZE] PROGMEM = {
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8
};
const axis_t ycoordinates[AMG88xx_PIXEL_ARRAY_SIZE] PROGMEM = {
  1,  1,  1,  1,  1,  1,  1,  1,
  2,  2,  2,  2,  2,  2,  2,  2,
  3,  3,  3,  3,  3,  3,  3,  3,
  4,  4,  4,  4,  4,  4,  4,  4,
  5,  5,  5,  5,  5,  5,  5,  5,
  6,  6,  6,  6,  6,  6,  6,  6,
  7,  7,  7,  7,  7,  7,  7,  7,
  8,  8,  8,  8,  8,  8,  8,  8
};

#define xCoord(p) ( (axis_t)pgm_read_byte_near(xcoordinates + (p)) )
#define yCoord(p) ( (axis_t)pgm_read_byte_near(ycoordinates + (p)) )

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
  #define AXIS                  yCoord
  #define NOT_AXIS              xCoord
  #define SIDE1(p)              ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) )
  #define SIDE2(p)              ( (p) >= (AMG88xx_PIXEL_ARRAY_SIZE/2) )
  #define pointOnBorder(i)      ( (i) < (GRID_EXTENT * 3) || (i) >= (GRID_EXTENT * 5) )
  #define pointOnSmallBorder(i) ( (i) < (GRID_EXTENT * 2) || (i) >= (GRID_EXTENT * 6) )
  #define pointOnEdge(i)        ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
  #define pointOnLREdge(i)      ( NOT_AXIS(i) == 1 || NOT_AXIS(i) == GRID_EXTENT )
#else
  #define AXIS                  xCoord
  #define NOT_AXIS              yCoord
  #define SIDE1(p)              ( (AXIS(p) <= (GRID_EXTENT/2) )
  #define SIDE2(p)              ( (AXIS(p) > (GRID_EXTENT/2) )
  #define pointOnBorder(i)      ( AXIS(i) <= 3 || AXIS(i) >= 6 )
  #define pointOnSmallBorder(i) ( AXIS(i) <= 2 || AXIS(i) >= 7 )
  #define pointOnEdge(i)        ( AXIS(i) == 1 || AXIS(i) == GRID_EXTENT )
  #define pointOnLREdge(i)      ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
  #error Double check all your code, this is untested
#endif

uint8_t SIDE(coord_t p) {
  return SIDE1(p) ? 1 : 2;
}

axis_t normalizeAxis(axis_t p) {
  return (p > 4 ? (GRID_EXTENT+1 - p) : p);
}

float euclidean_distance(coord_t p1, coord_t p2) {
  // calculate euclidean distance instead
  int8_t yd = ((int8_t)yCoord(p2)) - ((int8_t)yCoord(p1));
  int8_t xd = ((int8_t)xCoord(p2)) - ((int8_t)xCoord(p1));
  return (float)sqrt(sq(yd) + sq(xd)) + 0.05;
}

uint8_t axis_distance(coord_t p1, coord_t p2) {
  int8_t axisJump = ((int8_t)AXIS(p1)) - ((int8_t)AXIS(p2));
  return abs(axisJump);
}

uint8_t not_axis_distance(coord_t p1, coord_t p2) {
  int8_t axisJump = ((int8_t)NOT_AXIS(p1)) - ((int8_t)NOT_AXIS(p2));
  return abs(axisJump);
}

uint8_t max_axis_jump(coord_t p1, coord_t p2) {
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
uint8_t last_published_door_state = 9; // initialize to something invalid to force first loop
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

//typedef struct {
//  coord_t   current_position;
//  uint8_t   confidence;
//  uint8_t   neighbors;
//  uint8_t   height;
//  uint8_t   width;
//  float     raw_temp;
//  float     bgm;
//  float     fgm;
//
//  bool      real() { return current_position != UNDEF_POINT; };
//  uint8_t   side() { return SIDE(current_position); };
//  float     max_distance() {
//    return MAX_DISTANCE + (height+width+neighbors)/4.0 + (confidence/100.0);
//  };
//  float     max_allowed_temp_drift() {
//    float maxT = fgm * 0.85;
//    return min(maxT, 5.0);
//  };
//  float     difference_from_point(coord_t a) {
//    return abs(raw_pixels[(a)] - raw_temp);
//  };
//} PossiblePerson;

typedef struct {
  coord_t   past_position;        //:7 0-63 + UNDEF_POINT
  coord_t   starting_position;    //:6 0-63
  coord_t   max_position;         //:6 0-63
  uint8_t   confidence;           //:7 0-100
  fint1_t   max_temp_drift;       //:6 0-60
  uint16_t  count;

  uint8_t   history           :4; // 1-10 (could be 1-7)
  uint8_t   neighbors         :4; // 0-8

  uint8_t   height            :3; // 0-7
  uint8_t   forgotten_count   :2; // 0-2
  uint8_t   width             :3; // 0-7

  uint8_t   crossed           :4; // 0-9
  uint8_t   max_jump          :3; // 0-7
  bool      reverted          :1; // 0-1

  float     raw_temp;
  float     bgm;
  float     fgm;

  bool      real() { return past_position != UNDEF_POINT; };
  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };
  float     total_distance() {
    return euclidean_distance(starting_position, past_position);
  };
  float     max_distance() {
    return MAX_DISTANCE + (height+width+neighbors)/4.0 + (confidence/100.0);
  };
  float     max_allowed_temp_drift() {
    float maxT = fgm * 0.85;
    return constrain(maxT, 2.0, 5.0);
  };
  float     difference_from_point(coord_t a) {
    return abs(raw_pixels[(a)] - raw_temp);
  };

  #define METALENGTH  49 // TODO drop extra 0's from neighbors/height/width
  void generateMeta(char *meta) {
    sprintf(meta, "%ux%ux%ux%ux%ux%ux%ux%u0x%u0x%u0x%ux%ux%ux%u",
      confidence,                         // 3  100
      floatToFint2(bgm),                  // 4  1020
      floatToFint2(fgm),                  // 4  1020
      starting_position,                  // 2  64
      past_position,                      // 2  64
      history,                            // 1  8
      count,                              // 5  60000
      neighbors,                          // 1  8
      height,                             // 1  7
      width,                              // 1  7
      max_jump,                           // 1  5
      floatToFint2(global_variance),      // 4  1000
      max_temp_drift,                     // 2  99
      forgotten_count                     // 1  3
    );                                    // + 13 'x' + 1 null => 46 total
  };

  void revert() {
    char rBuf[3];
    char meta[METALENGTH];
    generateMeta(meta);
    sprintf(rBuf, "r%u", crossed);
    publish(rBuf, meta, RETRY_COUNT);
  };

  bool checkForDoorClose() {
    // This could possibly fail if person's hand is on door knob opening door and sensor
    // detects that as a person. We'll see hand go from 1->2, and then get dropped as door
    // opens, and this if block will prevent it from reverting properly.
    if (confidence > 60) {
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
        if (!crossed) return;
        // artificially shift starting point ahead 1 row so that
        // if user turns around now, algorithm considers it an exit
        int8_t s = ((int8_t)past_position) - ((int8_t)GRID_EXTENT);
        starting_position = max(s, 0);
      } else if (eventType == DOOR_CLOSE_EVENT && door_side == 2) {
        publish("1", meta, RETRY_COUNT);
        return;
      }
    } else {
      if (eventType == FRD_EVENT) {
        crossed = publish(door_state == DOOR_OPEN ? "2" : "a2", meta, RETRY_COUNT);
        if (!crossed) return;
        uint8_t s = past_position + GRID_EXTENT;
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
    count = 1;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  bool publishMaybeEvent() {
    if (!real() || confidence < CONFIDENCE_THRESHOLD) {
      return false;
    }

    if (history >= MIN_HISTORY && (!crossed || !reverted) && starting_side() != side()) {
      if (door_state != DOOR_OPEN && checkForDoorClose()) {
        // publish full event (not a2) even if door is closed
        publishPacket(DOOR_CLOSE_EVENT);
      } else {
        publishPacket(FRD_EVENT);
      }
      return true;
    }

    return false;
  };

  void forget() {
    checkForRevert() || publishMaybeEvent();
  };
} Person;

Person known_people[MAX_PEOPLE];
Person forgotten_people[MAX_PEOPLE];
const Person UNDEF_PERSON = {
  .past_position=UNDEF_POINT,
  // (╯°□°）╯︵ ┻━┻  avr-gcc doesn't implement non-trivial designated initializers...
  // comment out rest of this to save some RAM in exchange for lots of compiler warnings
  .starting_position=UNDEF_POINT,
  .max_position=UNDEF_POINT,
  .confidence=0,
  .max_temp_drift=0,
  .count=1,
  .history=1,
  .neighbors=0,
  .height=0,
  .forgotten_count=MAX_FORGOTTEN_COUNT,
  .width=0,
  .crossed=0,
  .max_jump=0,
  .reverted=false,
  .raw_temp=0,
  .bgm=0,
  .fgm=0
};

float bgPixel(coord_t x) {
  return fint3ToFloat(avg_pixels[(x)]);
}

float diffFromPoint(coord_t a, coord_t b) {
  return abs(raw_pixels[(a)] - raw_pixels[(b)]);
}

// calculate difference from foreground
float fgDiff(coord_t i) {
  if (((uint8_t)raw_pixels[(i)])<=MIN_TEMP_FRD || ((uint8_t)raw_pixels[(i)])>=MAX_TEMP_FRD) {
    return 0.0;
  }
  float fgmt1 = abs(raw_pixels[(i)] - cavg1);
  float fgmt2 = abs(raw_pixels[(i)] - cavg2);
  fgmt1 = min(fgmt1, fgmt2);

  if (((uint8_t)raw_pixels[(i)]) <= MIN_TEMP_FRD+2) {
    // raw_pixels must be between 14-15 if MIN_TEMP_FRD is 13
    fgmt2 = raw_pixels[i] - MIN_TEMP_FRD;
    fgmt1 = min(fgmt1, fgmt2);
  } else if (((uint8_t)raw_pixels[(i)]) >= MAX_TEMP_FRD-2) {
    // raw_pixels must be between 35-36 if MAX_TEMP_FRD is 37
    fgmt2 = MAX_TEMP_FRD - raw_pixels[i];
    fgmt1 = min(fgmt1, fgmt2);
  }

  return fgmt1;
}

// calculate difference from background
float bgDiff(coord_t i) {
  if (((uint8_t)raw_pixels[(i)])<=MIN_TEMP_FRD || ((uint8_t)raw_pixels[(i)])>=MAX_TEMP_FRD) {
    return 0.0;
  }
  float std = raw_pixels[(i)] - bgPixel(i);
  return abs(std);
}

uint8_t calcGradient(float diff, float scale) {
  if (diff < 0.5 || ((uint8_t)diff) > 20) return 0;
  diff /= scale;
  if (diff > 0.995) return 100;
  return (uint8_t)(diff*100.0);
}

// calculate foreground gradient percent
uint8_t calcFgm(coord_t i) {
  return calcGradient(fgDiff(i), global_fgm);
}

// calculate background gradient percent
uint8_t calcBgm(coord_t i) {
  return calcGradient(bgDiff(i), global_bgm);
}

float maxTempDiffForPoint(coord_t x) {
  float maxT = fgDiff(x) * 0.85;
  return min(maxT, 5.0);
}

bool samePoints(coord_t a, coord_t b) {
  if (norm_pixels[(a)] < CONFIDENCE_THRESHOLD) return false;
  return diffFromPoint(a, b) < maxTempDiffForPoint(b);
}

bool samePerson(coord_t a, Person b) {
  if (norm_pixels[(a)] < CONFIDENCE_THRESHOLD) return false;
  float mtdPerson = b.max_allowed_temp_drift();
  float mtdPoint = maxTempDiffForPoint(a);
  return b.difference_from_point(a) < max(mtdPerson, mtdPoint);
}

uint8_t maxPossibleNeighbors(coord_t i) {
  bool onTBEdge = pointOnEdge(i);
  bool onLREdge = pointOnLREdge(i);
  if (onTBEdge || onLREdge) {
    if (onTBEdge && onLREdge) return 3;
    return 5;
  }
  return 8;
}

uint8_t neighborsCount(coord_t i) {
  if (norm_pixels[(i)] < CONFIDENCE_THRESHOLD) return 0;
  uint8_t nc = 0;
  if (i >= GRID_EXTENT) { // not top row
    // top
    if (samePoints(i - GRID_EXTENT, i)) nc++;
    // top left
    if (NOT_AXIS(i) > 1 && samePoints(i-(GRID_EXTENT+1), i)) nc++;
    // top right
    if (NOT_AXIS(i) < GRID_EXTENT && samePoints(i-(GRID_EXTENT-1), i)) nc++;
  }
  if (i < GRID_EXTENT*7) { // not bottom row
    // bottom
    if (samePoints(i + GRID_EXTENT, i)) nc++;
    // bottom left
    if (NOT_AXIS(i) > 1 && samePoints(i+(GRID_EXTENT-1), i)) nc++;
    // bottom right
    if (NOT_AXIS(i) < GRID_EXTENT && samePoints(i+(GRID_EXTENT+1), i)) nc++;
  }
  // left
  if (NOT_AXIS(i) > 1 && samePoints(i-1, i)) nc++;
  // right
  if (NOT_AXIS(i) < GRID_EXTENT && samePoints(i+1, i)) nc++;
  return nc;
}

idx_t findClosestPerson(Person *arr, coord_t i, float maxDistance) {
  idx_t pidx = UNDEF_INDEX;
  float minTemp = 1.0;
  for (idx_t x=0; x<MAX_PEOPLE; x++) {
    Person p = arr[x];
    if (p.real()) {
      float maxD = p.max_distance();
      maxDistance = min(maxDistance, maxD);
      float dist = euclidean_distance(p.past_position, i);
      if (dist > maxDistance) continue;

      float tempDiff = p.difference_from_point(i);
      float maxT = p.max_allowed_temp_drift() * 0.6;
      if (tempDiff > maxT) continue;

      float tempRatio = tempDiff/maxT;
      tempRatio = max(tempRatio, 0.1);
      float distRatio = dist/maxDistance;
      distRatio = max(distRatio, 0.1);
      tempDiff = tempRatio * distRatio;
      if (tempDiff < 0.5 && tempDiff < minTemp) {
        pidx = x;
        minTemp = tempDiff;
      }
    }
  }
  return pidx;
}

bool otherPersonExists(coord_t i) {
  for (idx_t x=0; x<MAX_PEOPLE; x++) {
    Person p = known_people[x];
    if (p.real() && ((p.count > 3 && p.history > 1) || p.crossed) && p.confidence > 60 &&
          p.neighbors > 2 && ((uint8_t)euclidean_distance(i, p.past_position)) < 5) {
      return true;
    }
  }
  return false;
}

bool compareNeighboringPixels(coord_t x, coord_t y, coord_t i) {
  return (diffFromPoint(x, i) + 0.5) < diffFromPoint(y, i) && samePoints(x, i);
}

uint8_t calcHeight(coord_t i) {
  uint8_t height = 0;
  for (coord_t x = i + GRID_EXTENT; x < AMG88xx_PIXEL_ARRAY_SIZE; x += GRID_EXTENT) {
    if (samePoints(x, i)) height++;
    else break;
  }
  for (int8_t x = ((int8_t)i) - ((int8_t)GRID_EXTENT); x >= 0; x -= ((int8_t)GRID_EXTENT)) {
    if (samePoints(x, i)) height++;
    else break;
  }
  return height;
}

uint8_t calcWidth(coord_t i) {
  uint8_t width = 0;
  uint8_t axis = AXIS(i);
  for (coord_t x = i+1; x < AMG88xx_PIXEL_ARRAY_SIZE && AXIS(x) == axis; x++) {
    if (samePoints(x, i)) width++;
    else break;
  }
  for (int8_t x = ((int8_t)i)-1; x >= 0 && AXIS(x) == axis; x--) {
    if (samePoints(x, i)) width++;
    else break;
  }
  return width;
}

void publishEvents() {
  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    Person p = known_people[i];
    if (p.real() && p.starting_side() != p.side() && (!p.crossed || !p.reverted) &&
        p.history > MIN_HISTORY && p.confidence > CONFIDENCE_THRESHOLD) {
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
    for (idx_t i = 0; i<MAX_PEOPLE; i++) {
      known_people[i].forget();
      known_people[i] = UNDEF_PERSON;
      forgotten_people[i].publishMaybeEvent();
      forgotten_people[i] = UNDEF_PERSON;
    }
    cycles_since_forgotten = MAX_EMPTY_CYCLES;
    side1Point = 0;
    side2Point = 0;
  }
}

float trimMean(float sum, float sq_sum, uint8_t cnt, uint8_t side) {
  float variance = (sq_sum - (sq(sum)/((float)cnt)))/((float)cnt);
  variance = sqrt(abs(variance));
  global_variance = max(global_variance, variance);

  float mean = sum/((float)cnt);
  float lowerBound = mean - variance;
  float upperBound = mean + variance;
  float cavg = 0.0;
  uint8_t total = 0;
  for (coord_t i=(side==1 ? 0 : 32); i<(side==1 ? 32 : AMG88xx_PIXEL_ARRAY_SIZE); i++) {
    if (raw_pixels[i] > lowerBound && raw_pixels[i] < upperBound) {
      cavg += raw_pixels[i];
      total++;
    }
  }

  return total ? (cavg/(float)total) : mean;
}

// calculate foreground gradient scale
void calculateFgm() {
  global_fgm = FOREGROUND_GRADIENT;
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float fgmt = fgDiff(i);
    if (fgmt < global_fgm || fgmt > 10.0) continue;
    if (global_bgm > 1.0) fgmt *= (calcBgm(i)/100.0);
    global_fgm = max(fgmt, global_fgm);
  }
}

// calculate background gradient scale
void calculateBgm() {
  global_bgm = BACKGROUND_GRADIENT;
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float bgmt = bgDiff(i);
    if (bgmt < global_bgm || bgmt > 10.0) continue;
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
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (((uint8_t)raw_pixels[i]) < MIN_TEMP || ((uint8_t)raw_pixels[i]) > MAX_TEMP) continue;

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

  if (!total1 || !total2) return false;

  // calculate trimmed average
  global_variance = 0.0;
  cavg1 = trimMean(x_sum1, sq_sum1, total1, 1);
  cavg2 = trimMean(x_sum2, sq_sum2, total2, 2);

  #ifdef TEST_PCBA
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

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    uint8_t bgm = calcBgm(i);
    uint8_t fgm = calcFgm(i);
    norm_pixels[i] = min(bgm, fgm);
  }

  return true;
}

float calculateNewBackground(coord_t i) {
  // implicit alpha of 0.001 because avg_pixels is raw_pixels*1000.0
  float std = raw_pixels[(i)] - bgPixel(i);
  float fgd = fgDiff(i);

  if (door_state != DOOR_CLOSED && frames_since_door_open < 3 && fgd < 0.6) {
    // door just opened, learn new bg very rapidly
    return std * 300.0; // alpha = 0.3
  }

  if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES && fgd < 0.8) {
    // Learn new bg quickly but cautiously
    return std * 50.0; // alpha = 0.05
  }

  if (cycles_since_person == 0 && abs(std) > 0.5) {
    for (idx_t x=0; x<MAX_PEOPLE; x++) {
      if (known_people[x].real() && known_people[x].count > 5 &&
            known_people[x].count < 8000 && known_people[x].confidence > 50 &&
            samePoints(known_people[x].past_position, (i)) &&
            ((uint8_t)euclidean_distance(known_people[x].past_position, (i))) < 3 &&
            ((uint8_t)known_people[x].total_distance()) > 1) {
        // point has moved or is significantly higher than variance, decrease alpha.
        // due to rounding, this means std needs to be at least 5 for bg to update.
        return std * 0.1; // alpha = 0.0001
      }
    }
  }

  if (fgd < 0.5) {
    if (global_variance < 0.5) return std * 100.0; // alpha = 0.1
    // foreground is uniform, learn it as the background
    return std * 50.0; // alpha = 0.05
  }

  return std; // alpha = 0.001
}

void updateBgAverage() {
  #ifdef RECESSED
    // if door is covering sensor, don't update background averages
    if (door_state == DOOR_CLOSED) return;
  #endif

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // ignore sensor staring at back of door for wired sensors
    if (door_state == DOOR_CLOSED && SIDE(i) != door_side) continue;
    // ignore points where door is blocking sensor
    if (door_state == DOOR_AJAR && SIDE(i) == door_side) continue;
    // ignore extreme raw pixels
    if (((uint8_t)raw_pixels[(i)]) < MIN_TEMP || ((uint8_t)raw_pixels[(i)]) > MAX_TEMP) {
      continue;
    }

    int32_t temp = ((int32_t)avg_pixels[i]) + ((int32_t)round(calculateNewBackground(i)));
    if (temp < (((int32_t)MIN_TEMP)*1000) || temp > (((int32_t)MAX_TEMP)*1000)) continue;
    avg_pixels[i] = temp;
  }
}

// find how many of this points neighbors have already been counted
uint8_t findKnownNeighbors(coord_t *arr, uint8_t arrSize, coord_t p, coord_t &lastNeighbor) {
  uint8_t knownNeighbors = 0;
  for (uint8_t f=0; f<arrSize; f++) {
    if (arr[f] == p) continue;
    if (samePoints(arr[f], p) && ((uint8_t)euclidean_distance(arr[f], p)) == 1) {
      lastNeighbor = arr[f];
      knownNeighbors++;
      if (knownNeighbors > 1) return knownNeighbors;
    }
  }
  return knownNeighbors;
}

uint8_t bucketNum(float r, float minVal, float maxVal) {
  if (r <= minVal + 0.1) return 0;
  if (r >= maxVal - 0.1) return NUM_BUCKETS-1;
  float width = (maxVal - minVal)/NUM_BUCKETS;
  return (uint8_t)((r - minVal)/width);
}

uint8_t findCurrentPoints(coord_t *points) {
  // sort pixels by confidence
  coord_t ordered_indexes_temp[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t neighbors_cache[AMG88xx_PIXEL_ARRAY_SIZE] = { 0 };
  uint8_t active_pixel_count = 0;
  float minVal = MAX_TEMP_FRD;
  float maxVal = MIN_TEMP_FRD;
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (norm_pixels[i] > CONFIDENCE_THRESHOLD) {
      bool added = false;
      for (uint8_t j=0; j<active_pixel_count; j++) {
        if (norm_pixels[i] > norm_pixels[(ordered_indexes_temp[j])]) {
          for (int8_t x=active_pixel_count; x>j; x--) {
            ordered_indexes_temp[x] = ordered_indexes_temp[(x-1)];
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
      neighbors_cache[i] = neighborsCount(i);
    }

    if (((uint8_t)raw_pixels[i]) < MIN_TEMP_FRD || ((uint8_t)raw_pixels[i]) > MAX_TEMP_FRD) {
      continue;
    }
    if (raw_pixels[i] < minVal) minVal = raw_pixels[i];
    else if (raw_pixels[i] > maxVal) maxVal = raw_pixels[i];
  }

  uint8_t bin_counts[NUM_BUCKETS] = { 0 };
  coord_t sibling_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    uint8_t bidx = bucketNum(raw_pixels[i], minVal, maxVal);
    bin_counts[bidx]++;
    sibling_indexes[i] = UNDEF_POINT;
  }

  // reorder based on position
  coord_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  float fgThreshold = min(global_variance, 0.9)*1.5;
  for (uint8_t z=0; z<active_pixel_count; z++) {
    coord_t i = ordered_indexes_temp[z];
    bool added = false;
    if (fgDiff(i) > fgThreshold && bgDiff(i) > 0.8) {
      for (uint8_t j=0; j<z; j++) {
        coord_t oj = sibling_indexes[j] == UNDEF_POINT ? ordered_indexes[j] :
                        sibling_indexes[j];
        if (diffFromPoint(oj, i) < 0.7) {
          uint8_t nci = neighbors_cache[i];
          uint8_t ncj = neighbors_cache[ordered_indexes[j]];
          if (nci > (ncj + 1)) {
            // prefer the point that's more in middle of blob
            added = true;
          } else if (nci >= ncj) {
            // prefer point closer to middle of grid
            axis_t edge1 = AXIS(i);
            axis_t edge2 = AXIS(ordered_indexes[j]);
  
            // use columns instead of rows if same row
            if (edge1 == edge2) {
              edge1 = NOT_AXIS(i);
              edge2 = NOT_AXIS(ordered_indexes[j]);
            }
  
            // calculate row # from opposite edge
            edge1 = normalizeAxis(edge1);
            edge2 = normalizeAxis(edge2);
  
            if (edge1 == edge2 && edge1 == 4 && SIDE(i) != SIDE(ordered_indexes[j])) {
              // we're debating between 2 points on either side of border.
              // find which side this person was previously on to avoid flip-flopping.
              if (SIDE1(i)) {
                if (diffFromPoint(i-GRID_EXTENT, i) + 0.5 <
                      diffFromPoint(ordered_indexes[j]+GRID_EXTENT, ordered_indexes[j]) &&
                        samePoints(i-GRID_EXTENT, i)) {
                  edge1++;
                }
              } else if (diffFromPoint(i+GRID_EXTENT, i) + 0.5 <
                      diffFromPoint(ordered_indexes[j]-GRID_EXTENT, ordered_indexes[j]) &&
                        samePoints(i+GRID_EXTENT, i)) {
                edge1++;
              }
            }
  
            added = edge1 > edge2;
          }
          if (added) {
            // insert point i in front of j
            for (int8_t x=z; x>j; x--) {
              sibling_indexes[x] = sibling_indexes[(x-1)];
              ordered_indexes[x] = ordered_indexes[(x-1)];
            }
            sibling_indexes[j] = oj;
            ordered_indexes[j] = i;
            break;
          }
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
  uint8_t bin_clusters[NUM_BUCKETS] = { 0 };
  float bucketWidth = (maxVal - minVal)/NUM_BUCKETS;
  for (uint8_t y=0; y<active_pixel_count; y++) {
    coord_t current_point = ordered_indexes[y];
    if (current_point == UNDEF_POINT) continue;

    uint8_t bidx = bucketNum(raw_pixels[current_point], minVal, maxVal);
    bin_clusters[bidx]++;

    bool addable = total_masses < MAX_PEOPLE;
    // check if point is too weak to consider for a peak
    float fgD = addable ? fgDiff(current_point) : 0;
    if (addable && (fgD < fgThreshold || bgDiff(current_point) < 0.8)) {
      addable = false;
    }

    // check if point is too close to existing cluster
    if (addable) {
      for (idx_t x=0; x<total_masses; x++) {
        if ((uint8_t)euclidean_distance(points[x], current_point) == 1 &&
              diffFromPoint(points[x], current_point) < 5) {
          addable = false;
          break;
        }
      }
    }

    ordered_indexes_temp[sorted_size] = current_point;
    sorted_size++;
    ordered_indexes[y] = UNDEF_POINT;

    uint8_t bucketPointsInCluster = 1;
    float bucketLowerLimit = minVal + bucketWidth*bidx - 0.1;
    float bucketUpperLimit = bucketLowerLimit + bucketWidth + 0.1;

    // scan all points added after current_point, since they must be part of same blob
    for (uint8_t x=sorted_size-1; x<sorted_size; x++) {
      coord_t blobPoint = ordered_indexes_temp[x];
      coord_t lastFoundNeighbor = UNDEF_POINT;
      uint8_t knownNeighbors = findKnownNeighbors(ordered_indexes_temp, sorted_size,
          blobPoint, lastFoundNeighbor);

      if (addable && knownNeighbors > 0 && blobPoint == current_point) {
        // this is a peak that is touching another blob, don't let it peak
        if (knownNeighbors > 1) {
          addable = false;
        } else { // knownNeighbors == 1
          coord_t lFN2 = UNDEF_POINT;
          if (findKnownNeighbors(ordered_indexes_temp, x, lastFoundNeighbor, lFN2) > 1) {
            addable = false;
          }
        }
      }
      if (knownNeighbors > 1 || blobPoint == current_point) {
        // this is touching an existing blob from at least 2 points or is a peak,
        // so find rest of blob
        for (uint8_t k=y+1; k<active_pixel_count; k++) {
          // scan all known points after current_point to find neighbors to point x
          if (ordered_indexes[k] != UNDEF_POINT &&
              samePoints(ordered_indexes[k], blobPoint) &&
              ((uint8_t)euclidean_distance(ordered_indexes[k], blobPoint)) == 1) {
            ordered_indexes_temp[sorted_size] = ordered_indexes[k];
            sorted_size++;
            if (raw_pixels[ordered_indexes[k]] > bucketLowerLimit &&
                raw_pixels[ordered_indexes[k]] < bucketUpperLimit) {
              bucketPointsInCluster++;
            }
            ordered_indexes[k] = UNDEF_POINT;
          }
        }
      }
    }

    // check if point is too small/large to be a valid person
    if (addable) {
      uint8_t maxSize = fgD < 1 ? 10 : 15;
      maxSize += ((uint8_t)(bucketWidth + 0.5));
      float minSizeRatio = 0.25 - (min(fgD, 2)/10.0) - (bucketWidth/10.0);
      minSizeRatio = max(minSizeRatio, 0.05);
      uint8_t maxClusters = (((uint8_t)fgD) > 2 ? 4 : 3) + ((uint8_t)(bucketWidth + 0.5));
      if (bucketPointsInCluster < maxSize && bin_clusters[bidx] <= maxClusters &&
            ((float)bucketPointsInCluster)/((float)bin_counts[bidx]) > minSizeRatio) {
        points[total_masses] = current_point;
        total_masses++;
      }
    }
  }

  // if points are from the same bucket, drop them
  for (uint8_t i = 0; i < total_masses; i++) {
    uint8_t idx = points[i];
    uint8_t bidx = bucketNum(raw_pixels[idx], minVal, maxVal);
    uint8_t maxClusters = (((uint8_t)fgDiff(idx))>2 ? 4 : 3) + ((uint8_t)(bucketWidth+0.5));
    if (bin_clusters[bidx] > maxClusters) {
      points[i] = UNDEF_POINT;
    }
  }

  // compact points array to remove UNDEF_POINTs
  uint8_t final_total_masses = 0;
  for (uint8_t i = 0; i < total_masses; i++) {
    if (points[i] == UNDEF_POINT) {
      for (uint8_t y = i+1; y < total_masses; y++) {
        if (points[y] != UNDEF_POINT) {
          points[i] = points[y];
          points[y] = UNDEF_POINT;
          break;
        }
      }
    }

    if (points[i] != UNDEF_POINT) final_total_masses++;
    else break;
  }

  #ifdef PRINT_RAW_DATA
    if (final_total_masses >= 0) {
      SERIAL_PRINT(maxVal);
      SERIAL_PRINT(F(", "));
      SERIAL_PRINT(minVal);
      SERIAL_PRINT(F(", "));
      SERIAL_PRINTLN(bucketWidth);
      for (uint8_t i=0; i<NUM_BUCKETS; i++) {
        SERIAL_PRINT(i);
        SERIAL_PRINT(F(" ("));
        SERIAL_PRINT((minVal + i*bucketWidth));
        SERIAL_PRINT(F("-"));
        SERIAL_PRINT((minVal + (i+1)*bucketWidth));
        SERIAL_PRINT(F("): "));
        SERIAL_PRINTLN(bin_counts[i]);
      }
    }
  #endif

  return final_total_masses;
}

void forget_person(idx_t idx, Person *temp_forgotten_people, idx_t *pairs,
                    uint8_t &temp_forgotten_num) {
  Person p = known_people[(idx)];
  if (p.forgotten_count < MAX_FORGOTTEN_COUNT && p.confidence > AVG_CONF_THRESHOLD &&
        (p.checkForRevert() || axis_distance(p.starting_position, p.past_position) > 1) &&
        p.fgm > 1.5 && p.history <= p.count) {
    temp_forgotten_people[(temp_forgotten_num)] = p;
    temp_forgotten_num++;
  }
  pairs[(idx)] = UNDEF_INDEX;
  known_people[(idx)] = UNDEF_PERSON;
}

bool remember_person(Person *arr, coord_t point, uint8_t &h, coord_t &sp, uint8_t &mj,
                      fint1_t &md, uint8_t &cross, bool &revert, uint16_t &c, uint8_t &fc,
                      uint8_t height, uint8_t width, uint8_t neighbors, uint8_t conf) {
  float maxD = MAX_DISTANCE + (height+width+neighbors)/4.0 + (conf/100.0);
  idx_t pi = findClosestPerson(arr, point, min(maxD, 5.0));
  if (pi != UNDEF_INDEX) {
    Person p = arr[pi];

    axis_t ppaxis = AXIS(p.past_position);
    if ((SIDE1(p.starting_position) && ppaxis-1 > AXIS(point)) ||
        (SIDE2(p.starting_position) && ppaxis+1 < AXIS(point))) {
      // this point is moved behind previous position, just start over
      return false;
    }

    if (p.history <= MIN_HISTORY && p.side() != p.starting_side() &&
          pointOnSmallBorder(p.past_position) &&
          p.history <= (MIN_HISTORY+1 - normalizeAxis(ppaxis))) {
      // impossible for this person to ever do anything useful with its life, kill it
      return false;
    }

    // point is ahead of starting point at least
    sp = p.starting_position;
    h = min(p.history, MIN_HISTORY);

    fint1_t tempDrift = floatToFint1(p.difference_from_point(point));
    md = max(tempDrift, p.max_temp_drift);

    uint8_t axisJump = max_axis_jump(p.past_position, point);
    mj = max(axisJump, p.max_jump);

    cross = p.crossed;
    revert = p.reverted;
    c = p.count + 1;
    fc = p.forgotten_count + 1;
    arr[pi] = UNDEF_PERSON;
    return true;
  }
  return false;
}

void createNewPerson(coord_t pp, uint8_t mj, fint1_t md, uint8_t h, coord_t sp,
                    uint8_t cross, bool revert, float rt, uint8_t conf, float b,
                    float f, uint8_t n, uint8_t height, uint8_t width,
                    uint16_t c, uint8_t fc, idx_t j) {
  Person p = {
    .past_position=pp,
    .starting_position=sp,
    .max_position=pp,
    .confidence=conf,
    .max_temp_drift=md,
    .count=c,
    .history=h,
    .neighbors=n,
    .height=height,
    .forgotten_count=fc,
    .width=width,
    .crossed=cross,
    .max_jump=mj,
    .reverted=revert,
    .raw_temp=rt,
    .bgm=b,
    .fgm=f
  };
  known_people[j] = p;
}

bool processSensor() {
  if (!normalizePixels()) return false;

  // find list of peaks in current frame
  coord_t points[MAX_PEOPLE];
  uint8_t total_masses = findCurrentPoints(points);

  // pair previously seen points with new points to determine where people moved

  // "I don't know who you are or what you want, but you should know that I have a
  // very particular set of skills that make me a nightmare for people like you.
  // I will find you, I will track you, and I will turn the lights on for you."
  uint8_t taken[MAX_PEOPLE];
  idx_t pairs[MAX_PEOPLE];

  // "Good luck."
  Person temp_forgotten_people[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;

  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    taken[i] = 0;
    pairs[i] = UNDEF_INDEX;
    temp_forgotten_people[i] = UNDEF_PERSON;
  }

  if (side1Point || side2Point) {
    bool conjoinedBlobExists = false;
    for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
      Person p = known_people[idx];
      if (p.real() && p.neighbors >= 3 && p.confidence > 50) {
        conjoinedBlobExists = true;
        break;
      }
    }
    if (!conjoinedBlobExists) {
      side1Point = 0;
      side2Point = 0;
      SERIAL_PRINTLN(F("csp"));
    }
  }

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT (forget_person(idx, temp_forgotten_people, pairs, temp_forgotten_num))

  for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
    Person p = known_people[idx];
    if (!p.real()) continue;

    if (p.history <= MIN_HISTORY && p.side() != p.starting_side() &&
          pointOnSmallBorder(p.past_position) &&
          p.history <= (MIN_HISTORY+1 - normalizeAxis(AXIS(p.past_position)))) {
      // impossible for this person to ever do anything useful with its life, kill it
      FORGET_POINT;
      continue;
    }

    idx_t min_index = UNDEF_INDEX;
    float min_score = 100;
    float maxT = p.max_allowed_temp_drift();
    float maxD2 = p.max_distance();

    // pair this person with a point in current frame
    for (idx_t j=0; j<total_masses; j++) {
      uint8_t h = calcHeight(points[j]);
      uint8_t w = calcWidth(points[j]);
      uint8_t n = neighborsCount(points[j]);
      float maxD = MAX_DISTANCE + ((h + w + n)/4.0) + (norm_pixels[points[j]]/100.0);
      maxD = min(maxD, maxD2); // choose smaller range of 2 points as max range
      maxD = min(maxD, 6.0);   // don't let the D grow too big

      float d = euclidean_distance(p.past_position, points[j]);
      if (d > maxD) continue;

      // can't shift more than 2-5º at once
      float tempDiff = p.difference_from_point(points[j]);
      float maxTPoint = maxTempDiffForPoint(points[j]);
      float maxTfrd = max(maxT, maxTPoint);
      if (tempDiff > maxTfrd) continue;

      float score = sq(d/maxD) + sq(tempDiff/maxTfrd);
      if (!p.crossed || pointOnSmallBorder(p.starting_position)) {
        score -= (0.02*((float)n));
      }

      if (score > 1.8) continue; // distance is high AND temp diff is high AND conf is low

      if (score <= (min_score - 0.05) || (score <= (min_score + 0.05) &&
            tempDiff < p.difference_from_point(points[min_index]))) {
        // either score is less than min score, or if it's similar,
        // choose the point with more similar raw temp
        min_score = score;
        min_index = j;
      }
    }

    if (min_index == UNDEF_INDEX) {
      // still not found...
      FORGET_POINT;
    } else {
      taken[min_index]++;
      pairs[idx] = min_index;
    }
  }

  for (idx_t i=0; i<total_masses; i++) {
    if (taken[i] > 1) {
      // more than one person is trying to match with this single point, pick the best one...
      idx_t max_idx = UNDEF_INDEX;
      float max_score = -100.0;
      float score;
      for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
        Person p = known_people[idx];
        if (p.real() && pairs[idx] == i) {
          // prefer people with more neighbors
          score = (0.03*((float)p.neighbors));

          // prefer people with more similar temps
          float tempDiff = p.difference_from_point(points[i]);
          float maxT = p.max_allowed_temp_drift();
          score -= sq(tempDiff/maxT);

          // prefer people who didn't take crazy leaps to get here
          float d = euclidean_distance(p.past_position, points[i]);
          float maxD = p.max_distance();
          score -= sq(d/maxD);

          // prefer people who have been around longer
          score += ((p.total_distance() + p.history)/10.0);

          if (score >= max_score + 0.05) {
            max_score = score;
            max_idx = idx;
          } else if (score >= max_score - 0.05 ) {
            // if 2 competing points have the same score, pick the closer one
            float d2 = euclidean_distance(known_people[max_idx].past_position, points[i]);
            if (d+0.05 < d2 || (d-d2 < 0.05 && p.history>known_people[max_idx].history)) {
              max_score = score;
              max_idx = idx;
            }
          }
        }
      }
      // once we've chosen our winning point, forget the rest...
      bool holyMatrimony = norm_pixels[(points[i])] > 50 && !pointOnSmallBorder(points[i]) &&
                              neighborsCount(points[i]) >= 4;
      for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
        if (known_people[idx].real() && pairs[idx] == i && idx != max_idx) {
          // does this look like two blobs combining into one?
          if (holyMatrimony && known_people[idx].neighbors &&
                known_people[idx].count > 1 && known_people[idx].confidence > 50) {
            if (SIDE1(known_people[idx].starting_position)) {
              side1Point = max(known_people[idx].confidence, side1Point);
              SERIAL_PRINTLN(F("side1Point"));
            } else {
              side2Point = max(known_people[idx].confidence, side2Point);
              SERIAL_PRINTLN(F("side2Point"));
            }
          }
          FORGET_POINT;
          taken[i]--;
          if (taken[i] == 1) break;
        }
      }
    }

    if (taken[i] == 1) {
      for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
        Person p = known_people[idx];
        if (p.real() && pairs[idx] == i) {
          // closest point matched, update trackers
          if (p.past_position != points[i]) {
            uint8_t axisJump = max_axis_jump(p.past_position, points[i]);
            p.max_jump = max(axisJump, p.max_jump);

            if (AXIS(points[i]) == AXIS(p.starting_position) ||
                (SIDE1(p.starting_position) && AXIS(points[i]) < AXIS(p.past_position)) ||
                (SIDE2(p.starting_position) && AXIS(points[i]) > AXIS(p.past_position))) {
              // point moved backwards
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
                  p.count = 0;
                }
                // reset start position, unless point is in a revert crisis
                if (!p.reverted) {
                  p.starting_position = points[i];
                  p.max_position = points[i];
                }
                p.history = 1;
                p.max_jump = 0;
                p.max_temp_drift = 0;
              } else if (p.history > 1) {
                // point moved backwards a little bit, decrement history
                uint8_t sad = axis_distance(p.starting_position, points[i]) + 1;
                // Starting Axis Distance must be greater than 0, or else it would be in
                // earlier if condition. Make sure history is never higher than 1 x row.
                uint8_t newHistory = p.history;
                // only subtract history if we move back more than 1 row
                if (axis_distance(p.max_position, points[i]) > 1) newHistory--;
                p.history = min(newHistory, sad);
              }
            } else if (AXIS(points[i]) != AXIS(p.past_position) &&
                      // don't give credit to a point that's just coming back to its max pos
                      (AXIS(points[i]) != AXIS(p.max_position) ||
                      // unless it had fallen back more than 1 row and been punished before
                        axis_distance(p.max_position, p.past_position) > 1)) {
              // "always forward, forward always" - Luke Cage
              p.history++;
              p.max_position = points[i];
              if (SIDE(points[i]) != p.side() || p.history > 9) {
                // point just crossed threshold, let's reduce its history to force
                // it to spend another cycle on this side before we count the event
                p.history = min(p.history, MIN_HISTORY);
              }
            }
            p.past_position = points[i];
          }
          if (p.count) {
            fint1_t td = floatToFint1(p.difference_from_point(points[i]));
            p.max_temp_drift = max(p.max_temp_drift, td);
          }
          p.confidence = norm_pixels[(points[i])];
          p.raw_temp = raw_pixels[(points[i])];
          p.bgm = bgDiff(points[i]);
          p.fgm = fgDiff(points[i]);
          p.neighbors = neighborsCount(points[i]);
          p.height = calcHeight(points[i]);
          p.width = calcWidth(points[i]);
          p.count = min(p.count + 1, 60000);
          cycles_since_person = 0;
          known_people[idx] = p;
          break;
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      coord_t sp = points[i];
      uint8_t mj = 0;
      fint1_t md = 0;
      uint8_t cross = 0;
      bool revert = false;
      uint8_t conf = norm_pixels[sp];
      float rt = raw_pixels[sp];
      float b = bgDiff(sp);
      float f = fgDiff(sp);
      uint8_t n = neighborsCount(sp);
      uint8_t height = calcHeight(sp);
      uint8_t width = calcWidth(sp);
      uint8_t h = 1;
      uint16_t c = 1;
      uint8_t fc = 0;
      bool retroMatched = false;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        if (remember_person(temp_forgotten_people, points[i], h, sp, mj, md, cross,
              revert, c, fc, height, width, n, conf)) {
          retroMatched = true;
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        if (remember_person(forgotten_people, points[i], h, sp, mj, md, cross,
              revert, c, fc, height, width, n, conf)) {
          retroMatched = true;
        }
      }

      axis_t spAxis = normalizeAxis(AXIS(sp));
      if (!retroMatched && spAxis >= 3 && (door_side == SIDE(sp) || !doorJustOpened()) &&
            (side1Point || side2Point) && otherPersonExists(sp)) {
        // if point is right in middle, drag it to the side it appears to be coming from
        coord_t a = sp - GRID_EXTENT;
        coord_t b = sp + GRID_EXTENT;
        // there's another person in the frame, assume this is a split of that person
        if (side1Point > (side2Point+10) && (spAxis == 4 ||
              compareNeighboringPixels(a,b,sp))) {
          if (SIDE2(sp)) {
            h++;
            while (SIDE2(sp)) {
              sp -= GRID_EXTENT;
            }
          }
          side1Point = 0;
          side2Point = 0;
        } else if (side2Point > (side1Point+10) && (spAxis == 4 ||
              compareNeighboringPixels(b,a,sp))) {
          if (SIDE1(sp)) {
            h++;
            while (SIDE1(sp)) {
              sp += GRID_EXTENT;
            }
          }
          side1Point = 0;
          side2Point = 0;
        }
      }

      // ignore new points if door is not open or immediately after door opens
      if (door_state != DOOR_OPEN || (frames_since_door_open < 2 && SIDE(sp)==door_side)) {
        continue;
      }

      uint8_t minConf = 100;
      idx_t minIndex = UNDEF_INDEX;
      for (idx_t j=0; j<MAX_PEOPLE; j++) {
        // look for first empty slot in past_points to use
        if (!known_people[j].real()) {
          createNewPerson(points[i], mj, md, h, sp, cross, revert, rt, conf, b, f, n,
                          height, width, c, fc, j);
          minIndex = UNDEF_INDEX;
          break;
        } else if (known_people[j].confidence < minConf) {
          minConf = known_people[j].confidence;
          minIndex = j;
        }
      }
      if (minIndex != UNDEF_INDEX && conf > minConf) {
        // replace lower conf slot with this new point
        createNewPerson(points[i], mj, md, h, sp, cross, revert, rt, conf, b, f, n,
                        height, width, c, fc, minIndex);
      }
    }
  }

  // copy forgotten data points for this frame to global scope

  if (temp_forgotten_num > 0) {
    for (idx_t i=0; i<MAX_PEOPLE; i++) {
      forgotten_people[i].publishMaybeEvent();
      forgotten_people[i] = temp_forgotten_people[i];
    }
    cycles_since_forgotten = 0;
    SERIAL_PRINTLN(F("s"));
  } else if (cycles_since_forgotten < MAX_EMPTY_CYCLES) {
    cycles_since_forgotten++;
    if (cycles_since_forgotten == MAX_EMPTY_CYCLES) {
      // clear forgotten points list
      for (idx_t i=0; i<MAX_PEOPLE; i++) {
        forgotten_people[i].publishMaybeEvent();
        forgotten_people[i] = UNDEF_PERSON;
      }
      SERIAL_PRINTLN(F("f"));
    }
  }

  // wrap up with debugging output

  #ifdef PRINT_RAW_DATA
    if (total_masses >= 0) {
      for (idx_t i = 0; i<MAX_PEOPLE; i++) {
        Person p = known_people[i];
        if (p.real()) {
          SERIAL_PRINT(p.past_position);
          SERIAL_PRINT(F(" ("));
          SERIAL_PRINT(p.starting_position);
          SERIAL_PRINT(F("-"));
          SERIAL_PRINT(p.history);
          SERIAL_PRINT(F("-"));
          SERIAL_PRINT(p.neighbors);
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
      for (coord_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        SERIAL_PRINT(raw_pixels[idx]);
        if (norm_pixels[idx] < CONFIDENCE_THRESHOLD) {
          SERIAL_PRINT(F("---"));
        } else {
          if (norm_pixels[idx] < 100) SERIAL_PRINT(F(" "));
          SERIAL_PRINT(norm_pixels[idx]);
        }
        if (xCoord(idx) == GRID_EXTENT)
          SERIAL_PRINTLN();
        else
          SERIAL_PRINT(F("  "));
      }
//      SERIAL_PRINTLN(F("avg"));
//      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
//        SERIAL_PRINT(F(" "));
//        SERIAL_PRINT(avg_pixels[idx]);
//        SERIAL_PRINT(F(" "));
//        if (xCoord(idx) == GRID_EXTENT) SERIAL_PRINTLN();
//      }
      SERIAL_PRINTLN();
      SERIAL_FLUSH;
    }
  #endif

  return true;
}

void runThermalLoop() {
  if (processSensor()) {
    // publish event if any people moved through doorway yet
    publishEvents();
    // update avg_pixels
    updateBgAverage();
    // send heartbeat event if necessary
    beatHeart();
    // increment counter for how long door has been open
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) {
      frames_since_door_open++;
    }
    // increment counter for how long it's been since we saw a live body
    if (cycles_since_person < MAX_CLEARED_CYCLES) {
      cycles_since_person++;
    }

    #ifdef TIME_CYCLES
      SERIAL_PRINTLN(millis());
    #endif
  }
}

void initialize() {
  amg.begin(AMG_ADDR);

  // setup reed switches
  DDRD  = DDRD  & B11100111;  // set pins 3 and 4 as inputs
  PORTD = PORTD | B00011000;  // pull pins 3 and 4 high

  LOWPOWER_DELAY(SLEEP_1S);
  publish(FIRMWARE_VERSION, "0", RETRY_COUNT*2);

  // give sensor 16sec to stabilize
  LOWPOWER_DELAY(SLEEP_8S);
  LOWPOWER_DELAY(SLEEP_8S);

  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    known_people[i] = UNDEF_PERSON;
    forgotten_people[i] = UNDEF_PERSON;
  }

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    raw_pixels[i] = 0.0;
  }

  amg.readPixels(raw_pixels);

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] = floatToFint3(constrain(raw_pixels[i], MIN_TEMP, MAX_TEMP));
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!amg.readPixels(raw_pixels)) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (((uint8_t)raw_pixels[i]) < MIN_TEMP || ((uint8_t)raw_pixels[i]) > MAX_TEMP) {
        continue;
      }
      float std = raw_pixels[i] - bgPixel(i);
      // alpha of 0.3
      int32_t temp = ((int32_t)avg_pixels[i]) + ((int32_t)(300.0 * std));
      if (temp < (((int32_t)MIN_TEMP)*1000) || temp > (((int32_t)MAX_TEMP)*1000)) continue;
      avg_pixels[i] = temp;
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  runThermalLoop();
}
