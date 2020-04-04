#ifdef ENABLE_SERIAL
  #define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing
  #define PRINT_CLUSTERS
//  #define TIME_CYCLES
#endif

#define FIRMWARE_VERSION        "V20.4.4"
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
const uint8_t MIN_TEMP               = 2;    // ignore all points colder than 2º C
const uint8_t MAX_TEMP               = 45;   // ignore all points hotter than 45ºC
const uint8_t NUM_BUCKETS            = 10;   // 3 + log2(n) * log(n) where n = distinct rows
const float   MAX_DISTANCE           = 2.0;  // max distance that a point is allowed to move
const float   BACKGROUND_GRADIENT    = 2.0;
const float   FOREGROUND_GRADIENT    = 2.0;
const coord_t UNDEF_POINT            = AMG88xx_PIXEL_ARRAY_SIZE + 10;
const idx_t   UNDEF_INDEX            = UNDEF_POINT;

float   raw_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
fint3_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint8_t side1Point = 0;
uint8_t side2Point = 0;
float global_bgm = 0;
float global_fgm = 0;
float cavg1 = 0;
float cavg2 = 0;

#ifdef ENABLE_SERIAL

// Replace above progmem with following functions to save 146 bytes of storage space
axis_t xCoord(coord_t p) {
  return (p % GRID_EXTENT) + 1;
}
axis_t yCoord(coord_t p) {
  return (p/GRID_EXTENT) + 1;
}

#else

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

#endif

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

float calcMaxDistance(uint8_t height, uint8_t width, uint8_t neighbors, uint8_t confidence) {
  return MAX_DISTANCE + (height+width+neighbors)/4.0 + (confidence/100.0);
}
#define MAX_DIST_FORMULA ( calcMaxDistance(height, width, neighbors, confidence) )

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

uint8_t int_distance(coord_t a, coord_t b) {
  return ((uint8_t)(euclidean_distance((a), (b)) + 0.1));
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
#define SUSPICIOUS_EVENT  1

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
  if (door_state != DOOR_CLOSED) return frames_since_door_open < MAX_DOOR_CHANGE_FRAMES;
  else return readDoorState() != DOOR_CLOSED;
}

bool doorJustClosed() {
  if (frames_since_door_open == 0) return door_state != DOOR_OPEN;
  else if (door_state == DOOR_OPEN) return readDoorState() != DOOR_OPEN;
  return false;
}

float bgPixel(coord_t x) {
  return fint3ToFloat(avg_pixels[(x)]);
}

float diffFromPoint(coord_t a, coord_t b) {
  return abs(raw_pixels[(a)] - raw_pixels[(b)]);
}

// calculate difference from foreground
float fgDiff(coord_t i) {
  if (((uint8_t)raw_pixels[(i)]) <= MIN_TEMP || ((uint8_t)raw_pixels[(i)]) >= MAX_TEMP) {
    return 0.0;
  }
  float fgmt1 = abs(raw_pixels[(i)] - cavg1);
  float fgmt2 = abs(raw_pixels[(i)] - cavg2);
  return min(fgmt1, fgmt2);
}

// calculate difference from background
float bgDiff(coord_t i) {
  if (((uint8_t)raw_pixels[(i)]) <= MIN_TEMP || ((uint8_t)raw_pixels[(i)]) >= MAX_TEMP) {
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

float maxTempDiffForFgd(float f) {
  f *= 0.85;
  return min(f, 5.0);
}

float maxTempDiffForPoint(coord_t x) {
  return maxTempDiffForFgd(fgDiff(x));
}

typedef struct {
  coord_t   current_position;   //:6
  uint8_t   confidence;         //:7

  uint16_t  blobSize            :6;
  uint8_t   noiseSize           :6;
  uint8_t   neighbors           :4;

  uint8_t   height              :4;
  uint8_t   width               :4;

  uint8_t   side() { return SIDE(current_position); };
  float     raw_temp() { return raw_pixels[current_position]; };
  float     bgm() { return bgDiff(current_position); };
  float     fgm() { return fgDiff(current_position); };
  float     max_distance() {
    return MAX_DIST_FORMULA;
  };
  float     max_allowed_temp_drift() {
    return maxTempDiffForFgd(fgm());
  };
} PossiblePerson;

PossiblePerson points[MAX_PEOPLE];

typedef struct {
  coord_t   past_position;        //:7 0-63 + UNDEF_POINT
  coord_t   starting_position;    //:6 0-63
  coord_t   suspicious_position;  //:7 0-63 + UNDEF_POINT
  uint8_t   confidence;           //:7 0-100
  fint1_t   max_temp_drift;       //:6 0-60
  uint16_t  count;

  uint8_t   history           :3; // 1-7
  bool      retreating        :1; // 0-1
  uint8_t   neighbors         :4; // 0-8

  uint8_t   height            :3; // 0-7
  uint8_t   forgotten_count   :2; // 0-2
  uint8_t   width             :3; // 0-7

  uint8_t   crossed           :4; // 0-9
  uint8_t   max_jump          :3; // 0-7
  bool      reverted          :1; // 0-1

  uint32_t  avg_bgm           :11;
  uint16_t  avg_fgm           :11;
  uint8_t   avg_height        :3;
  uint8_t   avg_width         :3;
  uint8_t   avg_neighbors     :4;

  uint8_t   avg_confidence;
  uint8_t   blobSize;
  uint8_t   noiseSize;

  float     raw_temp;
  float     fgm;

  bool      real() { return past_position != UNDEF_POINT; };
  bool      suspicious() { return suspicious_position != UNDEF_POINT; };
  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };
  float     total_distance() {
    return euclidean_distance(starting_position, past_position);
  };
  float     max_distance() {
    return MAX_DIST_FORMULA;
  };
  float     max_allowed_temp_drift() {
    return maxTempDiffForFgd(fgm);
  };
  float     difference_from_point(coord_t a) {
    return abs(raw_pixels[(a)] - raw_temp);
  };

  #define METALENGTH  47
  void generateMeta(char *meta) {
    sprintf(meta, "%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%u",
      avg_confidence,                     // 3  100
      (uint16_t)avg_bgm,                  // 4  1020
      avg_fgm,                            // 4  1020
      starting_position,                  // 2  64
      past_position,                      // 2  64
      history,                            // 1  8
      count,                              // 5  60000
      blobSize,                           // 2  60
      avg_neighbors,                      // 1  8
      avg_height,                         // 1  7
      avg_width,                          // 1  7
      max_jump,                           // 1  5
      noiseSize,                          // 2  60
      max_temp_drift,                     // 2  99
      forgotten_count                     // 1  3
    );                                    // + 14 'x' + 1 null => 47 total
  };

  uint8_t _publishFrd(const char* msg, uint8_t retries) {
    char meta[METALENGTH];
    generateMeta(meta);
    return publish(msg, meta, retries);
  };

  void revert() {
    char rBuf[3];
    sprintf(rBuf, "r%u", crossed);
    _publishFrd(rBuf, RETRY_COUNT);
  };

  bool checkForRevert() {
    if (!real() || !crossed || suspicious()) return false;

    if (confidence > 60 && doorJustClosed()) {
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
    if (eventType == SUSPICIOUS_EVENT) {
      _publishFrd("s", 3);
      return;
    }

    uint8_t old_crossed = crossed;
    if (SIDE1(past_position)) {
      crossed = _publishFrd("1", RETRY_COUNT);
      if (!crossed) return;
      // artificially shift starting point ahead 1 row so that
      // if user turns around now, algorithm considers it an exit
      int8_t s = ((int8_t)past_position) - ((int8_t)GRID_EXTENT);
      starting_position = max(s, 0);
    } else {
      crossed = _publishFrd("2", RETRY_COUNT);
      if (!crossed) return;
      uint8_t s = past_position + GRID_EXTENT;
      starting_position = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
    }
    max_temp_drift = 0;
    forgotten_count = 0;
    max_jump = 0;
    count = 1;
    history = 1;
    retreating = false;
    reverted = false;
    suspicious_position = UNDEF_POINT;
    if (old_crossed) crossed = 0;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  bool publishMaybeEvent() {
    if (!real()) return false;

    if (door_state != DOOR_OPEN && frames_since_door_open == 0 && door_side == side()) {
      return false;
    }

    if ((history >= MIN_HISTORY || (history == 2 && avg_fgm > 200 && avg_bgm > 200)) &&
        (!crossed || !reverted) && starting_side() != side()) {
      publishPacket(FRD_EVENT);
      return true;
    } else if (suspicious() && starting_side() == side()) {
      starting_position = suspicious_position;
      publishPacket(FRD_EVENT);
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
  .starting_position=UNDEF_POINT,
  .suspicious_position=UNDEF_POINT,
  .confidence=0,
  .max_temp_drift=0,
  .count=1,
  .history=1,
  .retreating=false,
  .neighbors=0,
  .height=0,
  .forgotten_count=MAX_FORGOTTEN_COUNT,
  .width=0,
  .crossed=0,
  .max_jump=0,
  .reverted=false,
  .avg_bgm=0,
  .avg_fgm=0,
  .avg_height=0,
  .avg_width=0,
  .avg_neighbors=0,
  .avg_confidence=0,
  .blobSize=0,
  .noiseSize=0,
  .raw_temp=0,
  .fgm=0
};

uint8_t loadNeighbors(coord_t i, coord_t (&nArray)[8]) {
  uint8_t nc = 0;
  axis_t nai = NOT_AXIS(i);

  if (i >= GRID_EXTENT) { // not top row
    // top
    nArray[nc] = i-GRID_EXTENT;
    nc++;
    // top left
    if (nai > 1) {
      nArray[nc] = i-(GRID_EXTENT+1);
      nc++;
    }
    // top right
    if (nai < GRID_EXTENT) {
      nArray[nc] = i-(GRID_EXTENT-1);
      nc++;
    }
  }
  if (i < GRID_EXTENT*7) { // not bottom row
    // bottom
    nArray[nc] = i + GRID_EXTENT;
    nc++;
    // bottom left
    if (nai > 1) {
      nArray[nc] = i+(GRID_EXTENT-1);
      nc++;
    }
    // bottom right
    if (nai < GRID_EXTENT) {
      nArray[nc] = i+(GRID_EXTENT+1);
      nc++;
    }
  }
  // left
  if (nai > 1) {
    nArray[nc] = i-1;
    nc++;
  }
  // right
  if (nai < GRID_EXTENT) {
    nArray[nc] = i+1;
    nc++;
  }

  return nc;
}

uint8_t neighborsCount(coord_t i,float mt,uint8_t (&norm_pixels)[AMG88xx_PIXEL_ARRAY_SIZE]) {
  uint8_t nc = 0;
  coord_t neighbors[8];
  uint8_t totalNc = loadNeighbors(i, neighbors);
  for (uint8_t x = 0; x < totalNc; x++) {
    coord_t n = neighbors[x];
    if (norm_pixels[n] > CONFIDENCE_THRESHOLD && diffFromPoint(n, i) < mt) nc++;
  }
  return nc;
}

void publishEvents() {
  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    Person p = known_people[i];
    if (p.real() && p.history > MIN_HISTORY && (!p.crossed || !p.reverted) &&
        p.starting_side() != p.side()) {
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
//  #ifdef RECESSED
//    uint8_t old_door = door_state;
//  #endif

  if (checkDoorState()) {
    cycles_since_forgotten = MAX_EMPTY_CYCLES;
    side1Point = 0;
    side2Point = 0;

//    #ifdef RECESSED
//      if (door_state == DOOR_OPEN && old_door == DOOR_AJAR) return;
//    #endif

    for (idx_t i = 0; i<MAX_PEOPLE; i++) {
      known_people[i].forget();
      known_people[i] = UNDEF_PERSON;
      forgotten_people[i].publishMaybeEvent();
      forgotten_people[i] = UNDEF_PERSON;
    }
  }
}

float trimMean(uint8_t side) {
  coord_t sortedPixels[AMG88xx_PIXEL_ARRAY_SIZE/2];
  uint8_t total = 0;
  uint8_t baseline = side==1 ? 0 : 32;
  for (coord_t i=baseline; i<(32 + baseline); i++) {
    // sort clusters by raw temp
    if (((uint8_t)raw_pixels[i]) > MIN_TEMP && ((uint8_t)raw_pixels[i]) < MAX_TEMP) {
      uint8_t adjI = i - baseline;
      bool added = false;
      for (uint8_t j=0; j<adjI; j++) {
        if (raw_pixels[i] > raw_pixels[(sortedPixels[j])]) {
          for (int8_t x=adjI; x>j; x--) {
            sortedPixels[x] = sortedPixels[(x-1)];
          }
          sortedPixels[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        // append i to end of array
        sortedPixels[adjI] = i;
      }
      total++;
    }
  }

  SERIAL_PRINTLN(total);

  float avg = 0;
  uint8_t newTotal = 0;
  for (idx_t i = 3; i < total-3; i++) {
    // only take mean of middle 80% of pixels
    avg += raw_pixels[(sortedPixels[i])];
    newTotal++;
    if ((uint8_t)bgDiff(sortedPixels[i]) == 0) {
      // double weight of points with low background diff
      avg += raw_pixels[(sortedPixels[i])];
      newTotal++;
    }
  }

  SERIAL_PRINTLN(newTotal);

  if (!newTotal) {
    SERIAL_PRINTLN(F("xxx"));
    return 0; // no chosen points, skip this frame (should be impossible)
  }

  return avg/newTotal;
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

  // calculate trimmed average
  cavg1 = trimMean(1);
  cavg2 = trimMean(2);

  if (((int8_t)cavg1) == 0 || ((int8_t)cavg2) == 0) return false;

  // calculate CSM gradients
  global_bgm = 0; // needed to skip bgm check in first calculateFgm
  calculateFgm();
  // run 2 passes to amplify real points over noise
  calculateBgm();
  calculateFgm();
  calculateBgm();
  calculateFgm();

  return true;
}

float calculateNewBackground(coord_t i) {
  // implicit alpha of 0.001 because avg_pixels is raw_pixels*1000.0
  float std = raw_pixels[(i)] - bgPixel(i);
  float bgd = abs(std);
  if (bgd < 0.5) return 10*std; // alpha = 0.01

  float cavg = SIDE1(i) ? cavg1 : cavg2;
  float fgd = abs(raw_pixels[(i)] - cavg);

  if ((uint8_t)bgd > 1 && (fgd < 0.8 || bgd > max(3*fgd, 4))) {
    // rapidly update when background changes quickly
    uint16_t alpha = 20 * (uint8_t)bgd;
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) alpha *= 2;
    return std * min(alpha, 300);
  }

  // increment/decrement average by 0.001. Every 1º change will take 100 sec to learn.
  // This scales linearly so something that's 5º warmer will require ~8 min to learn.
  return std < 0 ? -1 : 1;
}

void updateBgAverage() {
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // ignore extreme raw pixels
    if (((uint8_t)raw_pixels[(i)]) <= MIN_TEMP || ((uint8_t)raw_pixels[(i)]) >= MAX_TEMP) {
      continue;
    }

    // yes we can use += here and rely on type promotion, but I want to be absolutely
    // explicit that we need to use int32_t and not int16_t to avoid overflow
    avg_pixels[i] = ((int32_t)avg_pixels[i]) + ((int32_t)round(calculateNewBackground(i)));
  }
}

bool isNeighborly(coord_t a, coord_t b) {
  return int_distance(a, b) == 1;
}

uint8_t findCurrentPoints() {
  // sort pixels by confidence
  coord_t ordered_indexes_temp[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    uint8_t bgm = calcBgm(i);
    uint8_t fgm = calcFgm(i);
    norm_pixels[i] = min(bgm, fgm);

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
    }
  }

  // adjust sorted pixels based on neighbor count and position
  coord_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  coord_t sibling_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  for (uint8_t z=0; z<active_pixel_count; z++) {
    coord_t i = ordered_indexes_temp[z];
    bool added = false;
    float fgd = fgDiff(i);
    float mt = maxTempDiffForFgd(fgd);
    uint8_t nci = neighborsCount(i, mt, norm_pixels);
    for (uint8_t j=0; j<z; j++) {
      coord_t oj = sibling_indexes[j];
      if ((norm_pixels[i]*2) > norm_pixels[oj] && diffFromPoint(oj, i) < min(fgd-0.1, 0.6)) {
        float mt2 = maxTempDiffForPoint(ordered_indexes[j]);
        uint8_t ncj = neighborsCount(ordered_indexes[j], mt2, norm_pixels);
        if (nci > (ncj + 1)) {
          // prefer the point that's more in middle of blob
          added = true;
        } else if (nci >= ncj) {
          // prefer point closer to middle of grid
          // it is tempting to limit this to only when temp doesn't change, but beware!
          // that forces the blob to sometimes make large leaps that get blocked
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
    if (!added) {
      // append i to end of array
      ordered_indexes[z] = i;
      sibling_indexes[z] = i;
    }
  }

  // DBSCAN algorithm will identify core points and label every point to a cluster number
  uint8_t sorted_size = 0;
  uint8_t total_masses = 0;
  // 0 means no cluster, first cluster starts at 1
  uint8_t clusterNum[AMG88xx_PIXEL_ARRAY_SIZE] = { 0 };
  uint8_t clusterIdx = 0;
  for (uint8_t y=0; y<active_pixel_count; y++) {
    coord_t current_point = ordered_indexes[y];
    if (current_point == UNDEF_POINT) continue;

    clusterIdx++;
    clusterNum[current_point] = clusterIdx;
    ordered_indexes_temp[sorted_size] = current_point;
    sorted_size++;
    ordered_indexes[y] = UNDEF_POINT;
    float fgd = fgDiff(current_point);
    float mt = maxTempDiffForFgd(fgd);

    // scan all points added after current_point, since they must be part of same blob
    for (uint8_t x=sorted_size-1; x<sorted_size; x++) {
      coord_t blobPoint = ordered_indexes_temp[x];
      float mtb;
      if (blobPoint == current_point) {
        mtb = mt;
      } else {
        mtb = maxTempDiffForPoint(blobPoint);
        // find how many neighbors this point has
        uint8_t fnc = 0;
        coord_t foundNeighbor = UNDEF_POINT;
        coord_t blobNeighbors[8];
        uint8_t nc = loadNeighbors(blobPoint, blobNeighbors);
        for (uint8_t bn = 0; bn < nc; bn++) {
          if (clusterNum[blobNeighbors[bn]] == clusterIdx) {
            fnc++;
            if (fnc == 2) break;
            foundNeighbor = blobNeighbors[bn];
          }
        }
        if (fnc == 1) {
          // if point only has 1 connection to this blob, maybe it's time to stop expanding
          bool skippable = true;
          for (uint8_t bn = 0; bn < nc; bn++) {
            if (clusterNum[blobNeighbors[bn]] == 0 &&
                  diffFromPoint(blobNeighbors[bn], blobPoint) < mtb) {
              // keep expanding if there are actually more connections to the blob
              if (isNeighborly(blobNeighbors[bn], foundNeighbor)) {
                skippable = false;
                break;
              }
//              else {
//                // scan all neighbors of new point to see if it's touching this blob
//                coord_t blobNeighbors2[8];
//                uint8_t nc2 = loadNeighbors(blobNeighbors[bn], blobNeighbors2);
//                for (uint8_t bn2 = 0; bn2 < nc2; bn2++) {
//                  // don't double count the current point
//                  if (blobNeighbors2[bn2] == blobPoint) continue;
//                  // check the other neighbors for a connection
//                  if (clusterNum[blobNeighbors2[bn2]] == clusterIdx) {
//                    // it is! We must keep expanding this blob then
//                    skippable = false;
//                    break;
//                  }
//                }
//                if (!skippable) break;
//              }
            }
          }
          if (skippable) continue;
        }
      }

      for (uint8_t k=y+1; k<active_pixel_count; k++) {
        // scan all known points after current_point to find neighbors to point x
        if (ordered_indexes[k] != UNDEF_POINT &&
            isNeighborly(ordered_indexes[k], blobPoint) &&
            (diffFromPoint(ordered_indexes[k], blobPoint) < mtb ||
              diffFromPoint(ordered_indexes[k], current_point) < mt)) {
          clusterNum[ordered_indexes[k]] = clusterIdx;
          ordered_indexes_temp[sorted_size] = ordered_indexes[k];
          sorted_size++;
          ordered_indexes[k] = UNDEF_POINT;
        }
      }
    }

    // check if point is too small/large to be a valid person
    uint8_t blobSize = 1;
    uint8_t totalBlobSize = 1;
    axis_t minAxis = AXIS(current_point);
    axis_t maxAxis = minAxis;
    axis_t minNAxis = NOT_AXIS(current_point);
    axis_t maxNAxis = minNAxis;
    uint8_t neighbors = 0;
    for (coord_t n = 0; n < AMG88xx_PIXEL_ARRAY_SIZE; n++) {
      if (clusterNum[n] == clusterIdx && n != current_point) {
        totalBlobSize++;

        float dp = diffFromPoint(n, current_point);
        if (dp > mt) continue;

        blobSize++;
        axis_t axisn = AXIS(n);
        minAxis = min(minAxis, axisn);
        maxAxis = max(maxAxis, axisn);
        axis_t naxisn = NOT_AXIS(n);
        minNAxis = min(minNAxis, naxisn);
        maxNAxis = max(maxNAxis, naxisn);
        if (isNeighborly(n, current_point)) neighbors++;
      }
    }

    if (totalBlobSize > 60) {
      SERIAL_PRINT(F("x "));
      SERIAL_PRINT(current_point);
      SERIAL_PRINTLN(F(" toobig"));
      continue;
    }

    uint8_t height = maxAxis - minAxis;
    uint8_t width = maxNAxis - minNAxis;
    uint8_t dimension = max(height, width) + 1;
    uint8_t boundingBox = min(dimension, 5);
    uint8_t bgd = (uint8_t)bgDiff(current_point);
    // ignore a blob that fills less than 1/3 of its bounding box
    // a blob with 9 points will always pass density test
    if ((totalBlobSize + min((uint8_t)fgd, bgd))*3 >= sq(boundingBox)) {
      uint8_t noiseSize = 0;
      float mt_constrained = mt*0.7;
      mt_constrained = constrain(mt_constrained, 0.51, 1.51);
      for (coord_t n = 0; n < AMG88xx_PIXEL_ARRAY_SIZE; n++) {
        if (clusterNum[n] == clusterIdx) continue;

        float dp = diffFromPoint(n, current_point);
        if (dp > mt_constrained) continue;

        if (norm_pixels[n] < CONFIDENCE_THRESHOLD) {
          // only count 0 confidence points within 0.3º (regardless of distance)
          if (dp > 0.3) continue;
        } else if (dimension < 5 && int_distance(n, current_point) >= dimension+2) {
          // only count points with confidence within dimension+2 distance
          continue;
        }

        noiseSize++;
      }

      if (noiseSize <= blobSize) {
        SERIAL_PRINT(F("+ "));
        SERIAL_PRINTLN(current_point);

        PossiblePerson pp = {
          .current_position=current_point,
          .confidence=norm_pixels[current_point],
          .blobSize=blobSize,
          .noiseSize=noiseSize,
          .neighbors=neighbors,
          .height=height,
          .width=width
        };
        points[total_masses] = pp;
        total_masses++;
        if (total_masses == MAX_PEOPLE) break;
      } else {
        SERIAL_PRINT(F("x "));
        SERIAL_PRINT(current_point);
        SERIAL_PRINT(F(" noise="));
        SERIAL_PRINTLN(noiseSize);
      }
    } else {
      SERIAL_PRINT(F("x "));
      SERIAL_PRINT(current_point);
      SERIAL_PRINTLN(F(" nodensity"));
    }
  }

  return total_masses;
}

void forget_person(idx_t idx, Person (&temp_forgotten_people)[MAX_PEOPLE],
                    idx_t (&pairs)[MAX_PEOPLE], uint8_t &temp_forgotten_num) {
  Person p = known_people[(idx)];
  if (p.confidence > 30 &&
      (p.checkForRevert() || axis_distance(p.starting_position, p.past_position) > 1)) {
    if (p.forgotten_count < MAX_FORGOTTEN_COUNT) {
      temp_forgotten_people[(temp_forgotten_num)] = p;
      temp_forgotten_num++;
    } else {
      // we're giving up on this point, but at least publish what we have
      p.publishMaybeEvent();
    }
  }
  pairs[(idx)] = UNDEF_INDEX;
  known_people[(idx)] = UNDEF_PERSON;
}

bool otherPersonExists(coord_t i) {
  for (idx_t x=0; x<MAX_PEOPLE; x++) {
    Person p = known_people[x];
    if (p.real() && ((p.count > 3 && p.history > 1) || p.crossed) && p.confidence > 60 &&
          p.neighbors > 2 && int_distance(i, p.past_position) < 5) {
      return true;
    }
  }
  return false;
}

bool compareNeighboringPixels(coord_t x, coord_t y, coord_t i, float f) {
  float d = diffFromPoint(x, i);
  return (d + 0.5) < diffFromPoint(y, i) && d < maxTempDiffForFgd(f);
}

idx_t findClosestPerson(Person (&arr)[MAX_PEOPLE], coord_t i, float maxDistance) {
  idx_t pidx = UNDEF_INDEX;
  float minTemp = 1.0;
  for (idx_t x=0; x<MAX_PEOPLE; x++) {
    Person p = arr[x];
    if (p.real()) {
      float maxD = p.max_distance();
      maxDistance = max(maxDistance, maxD);
      maxDistance = min(maxDistance, 4.0);
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

bool remember_person(Person (&arr)[MAX_PEOPLE], coord_t point, uint8_t &h, coord_t &sp,
        uint8_t &mj, fint1_t &md, uint8_t &cross, bool &revert, uint16_t &c, uint8_t &fc,
        uint8_t height, uint8_t width, uint8_t neighbors, uint8_t conf) {
  float maxD = calcMaxDistance(height, width, neighbors, conf);
  idx_t pi = findClosestPerson(arr, point, maxD);
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

bool processSensor() {
  if (!normalizePixels()) return false;

  // find list of peaks in current frame
  uint8_t total_masses = findCurrentPoints();

  // pair known points with new possible points to determine where people moved

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
    float maxTperson = p.max_allowed_temp_drift();
    float maxDperson = p.max_distance();

    // pair this person with a point in current frame
    for (idx_t j=0; j<total_masses; j++) {
      PossiblePerson pp = points[j];

      float maxDpoint = pp.max_distance();
      // choose larger range of 2 points as max distance
      maxDpoint = max(maxDpoint, maxDperson);
      maxDpoint = min(maxDpoint, 5.5); // don't let the D grow too big

      float d = euclidean_distance(p.past_position, pp.current_position);
      if (d > maxDpoint) continue;

      // can't shift more than 2-5º at once
      float maxTpoint = pp.max_allowed_temp_drift();
      float tempDiff = p.difference_from_point(pp.current_position);
      if (tempDiff > max(maxTperson, maxTpoint)) continue;

      float score = sq(d/maxDperson) + sq(max(tempDiff, 1)/maxTperson);
      if (!p.crossed || pointOnSmallBorder(p.starting_position)) {
        score -= (0.02*((float)pp.neighbors));
      }

      // distance is high AND temp diff is high
      if (d > 2 && score > 1.8) continue;

      if (score <= (min_score - 0.1) || (score <= (min_score + 0.1) &&
            tempDiff < p.difference_from_point(points[min_index].current_position))) {
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
      float maxT = points[i].max_allowed_temp_drift();
      float maxD = points[i].max_distance();
      for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
        Person p = known_people[idx];
        if (p.real() && pairs[idx] == i) {
          // prefer people with more neighbors
          score = (0.03*((float)p.neighbors));

          // prefer people with more similar temps
          float tempDiff = p.difference_from_point(points[i].current_position);
          tempDiff = max(tempDiff, 1);
          score -= sq(tempDiff/maxT);

          // prefer people who didn't take crazy leaps to get here
          float d = euclidean_distance(p.past_position, points[i].current_position);
          score -= sq(d/maxD);

          if (score >= max_score + 0.1) {
            max_score = score;
            max_idx = idx;
          } else if (score >= max_score - 0.1 &&
              (p.total_distance() + p.history) >
                (known_people[max_idx].total_distance() + known_people[max_idx].history)) {
            // if 2 competing points have the same score, pick the one with more history
            max_score = score;
            max_idx = idx;
          }
        }
      }
      // once we've chosen our winning point, forget the rest...
      bool holyMatrimony = points[i].confidence > 50 && points[i].neighbors >= 4 &&
                            !pointOnSmallBorder(points[i].current_position);
      for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
        if (known_people[idx].real() && pairs[idx] == i && idx != max_idx) {
          // does this look like two blobs combining into one?
          if (holyMatrimony && known_people[idx].neighbors &&
                known_people[idx].count > 1 && known_people[idx].confidence > 50) {
            if (SIDE1(known_people[idx].starting_position)) {
              side1Point = max(known_people[idx].confidence, side1Point);
              SERIAL_PRINTLN(F("s1Pt"));
            } else {
              side2Point = max(known_people[idx].confidence, side2Point);
              SERIAL_PRINTLN(F("s2Pt"));
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
          PossiblePerson pp = points[i];

          float tempDiff = p.difference_from_point(pp.current_position);
          fint1_t td = floatToFint1(tempDiff);
          p.max_temp_drift = max(p.max_temp_drift, td);

          uint8_t axisJump = max_axis_jump(p.past_position, pp.current_position);
          if (((!p.suspicious() && pp.side() != p.starting_side()) ||
                (p.suspicious() && pp.side() == p.starting_side())) &&
                (axisJump > 3 || tempDiff >= (p.max_allowed_temp_drift() + 1)) &&
                (uint8_t)pp.fgm() >= 2 && (uint8_t)pp.bgm() >= 2) {
            p.suspicious_position = p.suspicious() ? UNDEF_POINT : pp.current_position;
            p.publishPacket(SUSPICIOUS_EVENT);
          }

          if (p.past_position != pp.current_position) {
            p.max_jump = max(axisJump, p.max_jump);

            if (AXIS(pp.current_position) == AXIS(p.starting_position) ||
                (SIDE1(p.starting_position) &&
                  AXIS(pp.current_position) < AXIS(p.past_position)) ||
                (SIDE2(p.starting_position) &&
                  AXIS(pp.current_position) > AXIS(p.past_position))) {
              // point moved backwards
              if (!p.suspicious() && ((SIDE1(p.starting_position) &&
                    AXIS(pp.current_position) <= AXIS(p.starting_position)) ||
                  (SIDE2(p.starting_position) &&
                    AXIS(pp.current_position) >= AXIS(p.starting_position)))) {
                // reset history if point is further back than where it started
                if (!p.crossed || pointOnEdge(pp.current_position)) {
                  // reset everything, unless point is crossed and could still move back.
                  // past_position needs to be set before checkForRevert
                  p.past_position = pp.current_position;
                  p.checkForRevert();
                  p.count = 0;
                  p.crossed = 0;
                  p.forgotten_count = 0;
                  p.reverted = false;
                }
                // reset start position, unless point is in a revert crisis
                if (!p.reverted) {
                  p.starting_position = pp.current_position;
                  p.retreating = false;
                }
                p.history = 1;
                p.max_jump = 0;
                p.max_temp_drift = 0;
              } else if (AXIS(p.past_position) != AXIS(pp.current_position)) {
                if (p.suspicious() && pp.side() == p.starting_side() &&
                    axis_distance(p.suspicious_position, pp.current_position) >= 3) {
                  p.max_jump = 0;
                  p.max_temp_drift = 0;
                  p.forgotten_count = 0;
                  p.crossed = 0;
                  p.history = 3;
                  p.reverted = false;
                  p.retreating = false;
                  p.starting_position = p.suspicious_position;
                  p.suspicious_position = UNDEF_POINT;
                } else if (p.history > 1) {
                  // point moved backwards a little bit, decrement history
                  uint8_t sad = axis_distance(p.starting_position, pp.current_position) + 1;
                  // Starting Axis Distance must be greater than 0, or else it would be in
                  // earlier if condition. Make sure history is never higher than 1 x row.
                  uint8_t newHistory = p.history;
                  // only subtract history if we move back more than 1 row
                  if ((p.history > 2 || pp.side() == p.starting_side()) && (p.retreating ||
                        axis_distance(p.past_position, pp.current_position) > 1)) {
                    newHistory--;
                  }
                  p.retreating = true;
                  p.history = min(newHistory, sad);
                }
              }
            } else if (AXIS(p.past_position) != AXIS(pp.current_position)) {
              // "always forward, forward always" - Luke Cage
              if (!p.retreating || axis_distance(p.past_position, pp.current_position) > 1) {
                p.history++;
              }
              p.retreating = false;
              if (pp.side() != p.side() || p.history > 6) {
                // point just crossed threshold, let's reduce its history to force
                // it to spend another cycle on this side before we count the event
                p.history = min(p.history, MIN_HISTORY);
              }
            }
            p.past_position = pp.current_position;
          }
          // update current state
          p.confidence = pp.confidence;
          p.raw_temp = pp.raw_temp();
          p.fgm = pp.fgm();
          p.neighbors = pp.neighbors;
          p.height = pp.height;
          p.width = pp.width;
          if (p.count) {
            // update running averages
            #define UPDATE_RUNNING_AVG(o,n) ( o = ((n) + (o*2))/3 )
            UPDATE_RUNNING_AVG(p.avg_bgm, floatToFint2(pp.bgm()));
            UPDATE_RUNNING_AVG(p.avg_fgm, floatToFint2(p.fgm));
            UPDATE_RUNNING_AVG(p.avg_height, p.height);
            UPDATE_RUNNING_AVG(p.avg_width, p.width);
            UPDATE_RUNNING_AVG(p.avg_neighbors, p.neighbors);
            UPDATE_RUNNING_AVG(p.avg_confidence, p.confidence);
            UPDATE_RUNNING_AVG(p.blobSize, (uint8_t)pp.blobSize);
            UPDATE_RUNNING_AVG(p.noiseSize, pp.noiseSize);
            p.count = min(p.count + 1, 60000);
          } else {
            // reset all averages, this is a brand new point
            p.avg_bgm = floatToFint2(pp.bgm());
            p.avg_fgm = floatToFint2(p.fgm);
            p.avg_height = p.height;
            p.avg_width = p.width;
            p.avg_neighbors = p.neighbors;
            p.avg_confidence = p.confidence;
            p.blobSize = (uint8_t)pp.blobSize;
            p.noiseSize = pp.noiseSize;
            p.count = 1;
          }
          cycles_since_person = 0;
          known_people[idx] = p;
          break;
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      coord_t sp = points[i].current_position;
      uint8_t mj = 0;
      fint1_t md = 0;
      uint8_t cross = 0;
      bool revert = false;
      uint8_t conf = points[i].confidence;
      float rt = points[i].raw_temp();
      float b = points[i].bgm();
      float f = points[i].fgm();
      uint8_t n = points[i].neighbors;
      uint8_t height = points[i].height;
      uint8_t width = points[i].width;
      uint8_t h = 1;
      uint16_t c = 1;
      uint8_t fc = 0;
      bool retroMatched = false;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i].current_position)) {
        // first let's check points on death row from this frame for a match
        if (remember_person(temp_forgotten_people, points[i].current_position, h, sp, mj,
              md, cross, revert, c, fc, height, width, n, conf)) {
          retroMatched = true;
        }
      }

      if (!retroMatched && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
        // second let's check past forgotten points for a match
        if (remember_person(forgotten_people, points[i].current_position, h, sp, mj,
              md, cross, revert, c, fc, height, width, n, conf)) {
          retroMatched = true;
        }
      }

      axis_t spAxis = normalizeAxis(AXIS(sp));
      if (!retroMatched && spAxis >= 3) {
        // if point is right in middle, drag it to the side it appears to be coming from
        coord_t a = sp - GRID_EXTENT;
        coord_t b = sp + GRID_EXTENT;
        bool djo = doorJustOpened();
        if (djo && spAxis == 4 && height > 0 && door_side == SIDE(sp)) {
          if (door_side == 1) {
            if (compareNeighboringPixels(b,a,sp,f)) {
              sp += GRID_EXTENT;
              h++;
            }
          } else if (compareNeighboringPixels(a,b,sp,f)) {
            sp -= GRID_EXTENT;
            h++;
          }
        } else if (!djo && (side1Point || side2Point) && otherPersonExists(sp)) {
          // there's another person in the frame, assume this is a split of that person
          if (side1Point > (side2Point+10) && (spAxis == 4 ||
                compareNeighboringPixels(a,b,sp,f))) {
            if (SIDE2(sp)) {
              h++;
              while (SIDE2(sp)) {
                sp -= GRID_EXTENT;
              }
            }
            side1Point = 0;
            side2Point = 0;
          } else if (side2Point > (side1Point+10) && (spAxis == 4 ||
                compareNeighboringPixels(b,a,sp,f))) {
            if (SIDE1(sp)) {
              h++;
              while (SIDE1(sp)) {
                sp += GRID_EXTENT;
              }
            }
            side1Point = 0;
            side2Point = 0;
          }
        } else if (!djo && (uint8_t)b > 1 && (uint8_t)f > 1 && spAxis == 4 && height > 0) {
          // catch entries on door open for people who did not setup the door contact magnet
          if (SIDE1(sp)) {
            if (compareNeighboringPixels(b,a,sp,f)) {
              sp += GRID_EXTENT;
              h++;
            }
          } else if (compareNeighboringPixels(a,b,sp,f)) {
            sp -= GRID_EXTENT;
            h++;
          }
        }
      }

      // ignore new points if door is not open or immediately after door opens
      if (door_state == DOOR_CLOSED || ((frames_since_door_open < 2 ||
            door_state == DOOR_AJAR) && SIDE(sp)==door_side)) {
        continue;
      }

      for (idx_t j=0; j<MAX_PEOPLE; j++) {
        // look for first empty slot in known_people to use
        if (!known_people[j].real()) {
          Person p = {
            .past_position=points[i].current_position,
            .starting_position=sp,
            .suspicious_position=UNDEF_POINT,
            .confidence=conf,
            .max_temp_drift=md,
            .count=c,
            .history=h,
            .retreating=false,
            .neighbors=n,
            .height=height,
            .forgotten_count=fc,
            .width=width,
            .crossed=cross,
            .max_jump=mj,
            .reverted=revert,
            .avg_bgm=floatToFint2(b),
            .avg_fgm=floatToFint2(f),
            .avg_height=height,
            .avg_width=width,
            .avg_neighbors=n,
            .avg_confidence=conf,
            .blobSize=(uint8_t)points[i].blobSize,
            .noiseSize=(uint8_t)points[i].noiseSize,
            .raw_temp=rt,
            .fgm=f
          };
          known_people[j] = p;
          break;
        }
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
    //if (total_masses > 0) { // ignore frames where nothing happened
    if (true) {
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

      SERIAL_PRINT(cavg1);
      SERIAL_PRINT(F(", "));
      SERIAL_PRINTLN(cavg2);
//      SERIAL_PRINTLN(amg.readThermistor());
      SERIAL_PRINTLN(global_bgm);
      SERIAL_PRINTLN(global_fgm);
//      float avg_avg = 0;
//      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
//        avg_avg += bgPixel(idx);
//      }
//      avg_avg /= 64.0;
//      SERIAL_PRINTLN(avg_avg);

      // print chart of what we saw in 8x8 grid
      for (coord_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        SERIAL_PRINT(raw_pixels[idx]);
        uint8_t bgm = calcBgm(idx);
        uint8_t fgm = calcFgm(idx);
        uint8_t norm = min(bgm, fgm);
        if (norm < CONFIDENCE_THRESHOLD) {
          SERIAL_PRINT(F("---"));
        } else {
          if (norm < 100) SERIAL_PRINT(F(" "));
          SERIAL_PRINT(norm);
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
//      SERIAL_FLUSH;
    }
  #endif

  return true;
}

void clearSideXPoints() {
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
}

void runThermalLoop() {
  if (processSensor()) {
    // publish event if any people moved through doorway yet
    publishEvents();
    // update avg_pixels
    updateBgAverage();
    // clear sideXPoints used to track which side a merged from (naming things is hard, ok)
    clearSideXPoints();
    // send heartbeat event if necessary
    // 108000 = 10 (frames/sec) * 60 (sec/min) * 60 (min/hr) * 3 (hrs)
    beatHeart(108000);
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
  checkForUpdates();
}

void initialize() {
  amg.begin(AMG_ADDR);

  // setup reed switches
  DDRD  = DDRD  & B11100111;  // set pins 3 and 4 as inputs
  PORTD = PORTD | B00011000;  // pull pins 3 and 4 high

  LOWPOWER_DELAY(SLEEP_1S);
  publish(FIRMWARE_VERSION, "0", RETRY_COUNT*2);

  // check right on boot just in case this is a recovery attempt for a bricked sensor
  checkForUpdates();

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
      if (((uint8_t)raw_pixels[i]) <= MIN_TEMP || ((uint8_t)raw_pixels[i]) >= MAX_TEMP) {
        continue;
      }
      float std = raw_pixels[i] - bgPixel(i);
      // alpha of 0.3
      avg_pixels[i] = ((int32_t)avg_pixels[i]) + ((int32_t)(300.0 * std));
    }
  }
}

void loop_frd() {
  clearPointsAfterDoorClose();
  runThermalLoop();
}
