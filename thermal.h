#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.5.0"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE            1.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define DISTANCE_BONUS          2.5  // max extra distance a hot point can move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              4    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define CONFIDENCE_THRESHOLD    0.3  // consider a point if we're 30% confident
#define AVG_CONF_THRESHOLD      0.4  // consider a set of points if we're 40% confident
#define HIGH_CONF_THRESHOLD     0.8  // give points over 80% confidence extra benefits
#define GRADIENT_THRESHOLD      3.0  // 3º temp change gives us 100% confidence of person
#define MIN_TRAVEL_RATIO        0.15 // ratio of norm/distance that a point must pass

#define FAST_ALPHA              0.1
#define ALPHA                   0.001
#define SLOW_ALPHA              0.0001

#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float cur_pixels_hash = 0;

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
uint8_t crossed[MAX_PEOPLE];
bool reverted[MAX_PEOPLE];
float avg_norms[MAX_PEOPLE];
uint16_t avg_heights[MAX_PEOPLE];
uint16_t avg_widths[MAX_PEOPLE];
uint16_t count[MAX_PEOPLE];
uint16_t zombieCount[MAX_PEOPLE];

uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
uint8_t forgotten_crossed[MAX_PEOPLE];
bool forgotten_reverted[MAX_PEOPLE];
float forgotten_norms[MAX_PEOPLE];
uint8_t forgotten_heights[MAX_PEOPLE];
uint8_t forgotten_widths[MAX_PEOPLE];
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

bool sameSide(uint8_t a, uint8_t b) {
  return SIDE1(a) ? SIDE1(b) : SIDE2(b);
}

bool pixelActive(uint8_t i) {
  return norm_pixels[(i)] > CONFIDENCE_THRESHOLD;
}

float confidence(uint8_t x) {
  return avg_norms[(x)]/((float)count[(x)]);
}

float avgHeight(uint8_t x) {
  return (float)avg_heights[(x)]/((float)count[x]);
}

float avgWidth(uint8_t x) {
  return (float)avg_widths[(x)]/((float)count[x]);
}

uint8_t iavgHeight(uint8_t x) {
  return (int)round(avgHeight(x));
}

uint8_t iavgWidth(uint8_t x) {
  return (int)round(avgWidth(x));
}

float totalDistance(uint8_t x) {
  return euclidean_distance(starting_points[(x)], past_points[(x)]);
}

bool doorOpenedAgo(uint8_t x) {
  return frames_since_door_open < (x) && door_state != DOOR_CLOSED;
}

bool doorClosedAgo(uint8_t x) {
  return frames_since_door_open < (x) && door_state == DOOR_CLOSED;
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
  #define pointOnLREdge(i)      ( NOT_AXIS(i) == 1 || NOT_AXIS(1) == GRID_EXTENT )
#else
  #define pointOnBorder(i)      ( AXIS(i) <= 3 || AXIS(i) >= 6 )
  #define pointOnSmallBorder(i) ( AXIS(i) <= 2 || AXIS(i) >= 7 )
  #define pointInMiddle(i)      ( AXIS(i) > 2 && AXIS(i) < 7 )
  #define pointOnEdge(i)        ( AXIS(i) == 1 || AXIS(i) == 8 )
  #define pointOnLREdge(i)      ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
#endif

uint8_t massHeight(uint8_t i) {
  uint8_t height = 1;
  for (uint8_t x = i+GRID_EXTENT;
        x < AMG88xx_PIXEL_ARRAY_SIZE && norm_pixels[x] > 0.001;
        x += GRID_EXTENT) {
    height++;
  }
  for (int8_t x = i-GRID_EXTENT; x >= 0 && norm_pixels[x] > 0.001; x -= GRID_EXTENT) {
    height++;
  }

  return height;
}

uint8_t massWidth(uint8_t i) {
  uint8_t startAxis = AXIS(i);
  uint8_t width = 1;
  for (uint8_t x = i+1;
        x < AMG88xx_PIXEL_ARRAY_SIZE && norm_pixels[x] > 0.001 && AXIS(x) == startAxis;
        x++) {
    width++;
  }
  for (int8_t x = i-1; x >= 0 && norm_pixels[x] > 0.001 && AXIS(x) == startAxis; x--) {
    width++;
  }

  return width;
}

void checkDoorState() {
  uint8_t last_door_state = door_state;
  if (PIND & 0b00001000) {  // true if reed 3 is high (normal state)
    // door open if reed switch 4 is also high
    door_state = PIND & 0b00010000 ? DOOR_OPEN : DOOR_AJAR;
  } else {
    door_state = DOOR_CLOSED;
  }
  if (last_door_state != door_state) {
    frames_since_door_open = 0;
    memset(past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
  }
  if (((door_state == DOOR_CLOSED && last_published_door_state != DOOR_CLOSED) ||
       (door_state != DOOR_CLOSED && last_published_door_state != DOOR_OPEN)) &&
      publish(door_state == DOOR_CLOSED ? "d0" : "d1", 0, 0)) {
    last_published_door_state = door_state == DOOR_CLOSED ? DOOR_CLOSED : DOOR_OPEN;
  }
}

void publishRevert(uint8_t idx) {
  checkDoorState();
  if (frames_since_door_open == 0) return;
  char rBuf[3];
  sprintf(rBuf, "r%d", crossed[idx]);
  publish(rBuf, floor(confidence(idx)*100.0), 10);
}

void checkForRevert(uint8_t idx) {
  if (!crossed[idx] || frames_since_door_open == 0 || zombieCount[idx] == 1) return;
  if (reverted[idx] && (pointOnSmallBorder(past_points[idx]) ||
        pointOnLREdge(past_points[idx]))) {
    // we had previously reverted this point, but it came back and made it through
    publishRevert(idx);
    reverted[idx] = false;
  } else if (!reverted[idx] && confidence(idx) < HIGH_CONF_THRESHOLD &&
      pointInMiddle(past_points[idx]) && !pointOnLREdge(past_points[idx])) {
    // point disappeared in middle of grid, revert its crossing (probably noise or a hand)
    publishRevert(idx);
    reverted[idx] = true;
  }
}

void publishEvents() {
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT && histories[i] > MIN_HISTORY && zombieCount[i] != 1 &&
          !sameSide(starting_points[i], past_points[i]) &&
          confidence(i) > AVG_CONF_THRESHOLD) {
      int diff = AXIS(starting_points[i]) - AXIS(past_points[i]);
      // point cleanly crossed grid
      if (abs(diff) >= 3 || totalDistance(i) >= 6) {
        uint8_t width = iavgWidth(i);
        uint8_t height = iavgHeight(i);
        uint8_t conf = (int)floor(confidence(i) * 100.0);
        uint16_t props = width*1000 + height*100 + conf;
        if (SIDE1(past_points[i])) {
          crossed[i] = publish("1", props, 10);
          // artificially shift starting point ahead 1 row so that
          // if user turns around now, algorithm considers it an exit
          int s = past_points[i] - GRID_EXTENT;
          starting_points[i] = max(s, 0);
        } else {
          crossed[i] = publish("2", props, 10);
          int s = past_points[i] + GRID_EXTENT;
          starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
        }
        histories[i] = 1;
        reverted[i] = false;
        avg_norms[i] = confidence(i);
        avg_heights[i] = height;
        avg_widths[i] = width;
        count[i] = 1;
      }
    }
  }
}

void scanSegment(float *arr, uint8_t base, uint8_t inc, float edgePoint) {
  float pos = 0.0;
  float neg = 0.0;
  uint8_t maxi = UNDEF_POINT;
  uint8_t mini = UNDEF_POINT;
  bool positive = true;

  uint8_t maxr = (GRID_EXTENT - 1)*inc;
  for (uint8_t r=0; r<maxr; r+=inc) {
    uint8_t i = base + r;
    if (abs(arr[i]) <= CONFIDENCE_THRESHOLD) {
      arr[i] = 0;
      continue;
    }
    bool new_positive = arr[i] > 0;
    if (new_positive == positive) {
      if (positive) {
        if (arr[i] >= pos) {
          if (maxi != UNDEF_POINT) arr[maxi] = 0;
          pos = arr[i];
          maxi = i;
        } else {
          arr[i] = 0;
        }
      } else {
        if (arr[i] <= neg) {
          if (mini != UNDEF_POINT) arr[mini] = 0;
          neg = arr[i];
          mini = i;
        } else {
          arr[i] = 0;
        }
      }
    } else {
      if (maxi != UNDEF_POINT && mini != UNDEF_POINT) {
        // we found a pair, ship it
        uint8_t upper_pos = max(maxi, mini);
        uint8_t lower_pos = min(maxi, mini);
        // this depends on int math to round answer, otherwise we'd need to floor the eq
        uint8_t new_pos = upper_pos - ((upper_pos - lower_pos)/(2*inc))*inc;
        float new_val = max(pos, -neg);
        float sub_val = new_val - 0.05;
        for (uint8_t x = lower_pos+inc; x <= upper_pos; x+=inc) {
          arr[x] = x == new_pos ? new_val + 1 : sub_val;
        }
        arr[lower_pos] = 0;

        // reset trackers for rest of row
        pos = 0.0;
        neg = 0.0;
        maxi = UNDEF_POINT;
        mini = UNDEF_POINT;
      }
  
      positive = new_positive;
      if (positive) {
        pos = arr[i];
        maxi = i;
      } else {
        neg = arr[i];
        mini = i;
      }
    }
  }

  bool outerPoint = false;
  uint8_t upper_bound = base + (GRID_EXTENT-1)*inc;
  if (maxi != UNDEF_POINT && mini == UNDEF_POINT) {
    if (-edgePoint > 0.1) {
      neg = edgePoint;
      mini = base;
      outerPoint = true;
    }
    edgePoint = arr[upper_bound];
    if (-edgePoint > 0.1 && (mini == UNDEF_POINT || edgePoint < neg)) {
      neg = edgePoint;
      mini = upper_bound;
      outerPoint = true;
    }
  } else if (maxi == UNDEF_POINT && mini != UNDEF_POINT) {
    if (edgePoint > 0.1) {
      pos = edgePoint;
      maxi = base;
      outerPoint = true;
    }
    edgePoint = arr[upper_bound];
    if (edgePoint > 0.1 && (maxi == UNDEF_POINT || edgePoint > pos)) {
      pos = edgePoint;
      maxi = upper_bound;
      outerPoint = true;
    }
  }

  if (maxi != UNDEF_POINT && mini != UNDEF_POINT) {
    // we found a pair, ship it
    uint8_t upper_pos = max(maxi, mini);
    uint8_t lower_pos = min(maxi, mini);

    if (outerPoint) {
      if ((upper_pos - lower_pos)/inc > 4) {
        if (maxi != UNDEF_POINT) arr[maxi] = 0;
        if (mini != UNDEF_POINT) arr[mini] = 0;
        arr[upper_bound] = 0;
        return;
      }
      float new_val = (pos - neg)/2.0;
      float sub_val = new_val - 0.05;
      uint8_t new_pos = upper_pos == upper_bound ? upper_pos : lower_pos;
      for (uint8_t x = lower_pos; x <= upper_pos; x+=inc) {
        arr[x] = x == new_pos ? new_val + 1 : sub_val;
      }
      if (new_pos == lower_pos) {
        arr[upper_bound] = 0;
      } else {
        arr[lower_pos] = 0;
      }
    } else {
      float new_val = max(pos, -neg);
      float sub_val = new_val - 0.05;
      // this depends on int math to round answer, otherwise we'd need to floor the eq
      uint8_t new_pos = upper_pos - ((upper_pos - lower_pos)/(2*inc))*inc;
      for (uint8_t x = lower_pos+inc; x <= upper_pos; x+=inc) {
        arr[x] = x == new_pos ? new_val + 1 : sub_val;
      }
      arr[lower_pos] = 0;
      arr[upper_bound] = 0;
    }
  } else {
    if (maxi != UNDEF_POINT) arr[maxi] = 0;
    if (mini != UNDEF_POINT) arr[mini] = 0;
    arr[upper_bound] = 0;
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

  // calculate CSM gradient
  float bgm = GRADIENT_THRESHOLD;
  float fgm = GRADIENT_THRESHOLD;
  float vfgm = GRADIENT_THRESHOLD;
  float bgmt;
  float fgmt;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // reading is invalid if less than -20 or greater than 100,
    // but we use a smaller range than that to determine validity
    if (norm_pixels[i] < 0 || norm_pixels[i] > 65) {
      norm_pixels[i] = avg_pixels[i];
      continue;
    }

    if (NOT_AXIS(i) < GRID_EXTENT) {
      fgmt = norm_pixels[i+1] - norm_pixels[i];
      fgmt = abs(fgmt);
      bgmt = abs(norm_pixels[i+1] - avg_pixels[i+1]) - abs(norm_pixels[i] - avg_pixels[i]);
      bgmt = abs(bgmt);

      if (fgmt > fgm) fgm = fgmt;
      if (bgmt > bgm) bgm = bgmt;
    }

    if (i < GRID_EXTENT*7) {
      fgmt = norm_pixels[i+GRID_EXTENT] - norm_pixels[i];
      fgmt = abs(fgmt);
      if (fgmt > vfgm) vfgm = fgmt;
    }
  }

  float normv_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  memcpy(normv_pixels, norm_pixels, AMG88xx_PIXEL_ARRAY_SIZE*sizeof(float));

  float offGrid;
  float rowAvg;
  for (uint8_t r=0; r<AMG88xx_PIXEL_ARRAY_SIZE; r+=GRID_EXTENT) {
    rowAvg = 0.0;
    for (uint8_t ri=r; ri<r+GRID_EXTENT; ri++) {
      rowAvg += norm_pixels[ri];
    }
    rowAvg /= (float)GRID_EXTENT;

    fgmt = (norm_pixels[r] - avg_pixels[r])/fgm;
    bgmt = (norm_pixels[r] - rowAvg)/bgm;

    offGrid = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
    if (abs(offGrid) > 1) {
      offGrid = offGrid > 0 ? 1.0 : -1.0;
    }

    for (uint8_t i=r; i<r+GRID_EXTENT-1; i++) {
      fgmt = (norm_pixels[i+1] - norm_pixels[i])/fgm;
      bgmt = (abs(norm_pixels[i+1] - avg_pixels[i+1]) -
              abs(norm_pixels[i] - avg_pixels[i]))/bgm;

      norm_pixels[i] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
    }

    uint8_t i = r+GRID_EXTENT-1;
    fgmt = (avg_pixels[i] - norm_pixels[i])/fgm;
    bgmt = (rowAvg - norm_pixels[i])/bgm;

    norm_pixels[i] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
    if (abs(norm_pixels[i]) > 1) {
      norm_pixels[i] = norm_pixels[i] > 0 ? 1.0 : -1.0;
    }

    scanSegment(norm_pixels, r, 1, offGrid);
  }

  float std;
  for (uint8_t c=0; c<GRID_EXTENT; c++) {
    offGrid = (normv_pixels[c] - avg_pixels[c])/vfgm;
    if (abs(offGrid) > 1) {
      offGrid = offGrid > 0 ? 1.0 : -1.0;
    }

    for (uint8_t i=c; i<AMG88xx_PIXEL_ARRAY_SIZE; i+=GRID_EXTENT) {
      std = normv_pixels[i] - avg_pixels[i];

      if (i >= GRID_EXTENT*7) {
        normv_pixels[i] = -std/vfgm;
        if (abs(normv_pixels[i]) > 1) {
          normv_pixels[i] = normv_pixels[i] > 0 ? 1.0 : -1.0;
        }
      } else {
        normv_pixels[i] = (normv_pixels[i+GRID_EXTENT] - normv_pixels[i])/vfgm;
      }

      // update average baseline
      if (abs(std) > 2 && norm_pixels[i] >= 0.6) {
        // lower alpha to 0.0001
        std *= SLOW_ALPHA;
      } else {
        // implicit alpha of 0.001
        std *= ALPHA;
      }
      avg_pixels[i] += std;
    }

    scanSegment(normv_pixels, c, GRID_EXTENT, offGrid);
  }

  for (uint8_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (normv_pixels[i] < 0.001) {
      norm_pixels[i] = 0;
      continue;
    }

    if (norm_pixels[i] > 0.6) {
      float bonus = 0;
      if (norm_pixels[i] > 1.05) {
        norm_pixels[i] -= 1.05;
        bonus = 0.05;
      }
      uint8_t new_pos = UNDEF_POINT;
      for (uint8_t x = i + GRID_EXTENT;
          x < AMG88xx_PIXEL_ARRAY_SIZE && norm_pixels[x] > 0.001 && normv_pixels[x] > 0.001;
          x += GRID_EXTENT) {
        if (normv_pixels[x] > 1.05) {
          new_pos = x;
          break;
        }
      }
      if (new_pos == UNDEF_POINT) {
        for (int8_t x = i;
            x >= 0 && norm_pixels[x] > 0.001 && normv_pixels[x] > 0.001; x -= GRID_EXTENT) {
          if (normv_pixels[x] > 1.05) {
            new_pos = x;
            break;
          }
        }
      }

      if (new_pos == UNDEF_POINT || new_pos == i) {
        // either we couldn't find vertical center or this is a perfect point
        // that's already vertically centered. Either way, reset it and move along.
        norm_pixels[i] += bonus;
        continue;
      }

      float t;
      float v;
      int8_t inc = new_pos > i ? -GRID_EXTENT : GRID_EXTENT;
      for (int8_t y = new_pos; y != i; y += inc) {
        t = norm_pixels[y];
        if (t > 1.05) t -= 1;
        v = norm_pixels[i];
        if (y == new_pos) v += bonus;
        if (v > t) {
          norm_pixels[y] = v;
        }
      }
    }
  }

  return true;
}

bool connectedPoints(uint8_t p1, uint8_t p2) {
  uint8_t a = min(p1, p2);
  uint8_t b = max(p1, p2);
  uint8_t lowerb = b;
  uint8_t upperb = b;

  for (int8_t x = b-1; x >= 0 && norm_pixels[x] > 0.001 && AXIS(x) == AXIS(b); x--) {
    lowerb = x;
  }
  for (uint8_t x = b+1;
        x < AMG88xx_PIXEL_ARRAY_SIZE && norm_pixels[x] > 0.001 && AXIS(x) == AXIS(b);
        x++) {
    upperb = x;
  }

  // waterfall over left edge of blob until we get to the same axis as point b
  while (AXIS(a) < AXIS(b)) {
    // find left-most edge of this row
    for (int8_t x = a-1; x >= 0 && AXIS(x) == AXIS(a); x--) {
      a = x;
      if (norm_pixels[a] < 0.001) break;
    }
    // find the row below
    bool matched = false;
    for (uint8_t x = a; AXIS(x) == AXIS(a); x++) {
      if (norm_pixels[x + GRID_EXTENT] > 0.001) {
        // a point from the row below was found!
        a = x + GRID_EXTENT;
        matched = true;
        break;
      }
      // we reached other side of blob without finding a row below, must be bottom of blob
      if (x > a && norm_pixels[x] < 0.001) return false;
    }
    // we ran out space on this row, must be bottom of blob
    if (!matched) return false;
  }

  // a and b must be on same axis by the time we get here
  if (a >= lowerb && a <= upperb) return true;
  if (a > upperb) return false;

  // did not find connection, try again from right side of blob in case we have a U shape
  a = min(p1, p2);
  while (AXIS(a) < AXIS(b)) {
    // find right-most edge of this row
    for (uint8_t x = a+1; AXIS(x) == AXIS(a); x++) {
      a = x;
      if (norm_pixels[a] < 0.001) break;
    }
    // find the row below
    bool matched = false;
    for (uint8_t x = a; x >= 0 && AXIS(x) == AXIS(a); x--) {
      if (norm_pixels[x + GRID_EXTENT] > 0.001) {
        // a point from the row below was found!
        a = x + GRID_EXTENT;
        matched = true;
        break;
      }
      // we reached other side of blob without finding a row below, must be bottom of blob
      if (x < a && norm_pixels[x] < 0.001) return false;
    }
    // we ran out space on this row, must be bottom of blob
    if (!matched) return false;
  }

  return a >= lowerb && a <= upperb;
}

void insertPointHere(uint8_t *arr, uint8_t active_pixel_count, uint8_t j, uint8_t i) {
  if (norm_pixels[arr[j]] > norm_pixels[i]) norm_pixels[i] = norm_pixels[arr[j]];
  for (int8_t x=active_pixel_count; x>j; x--) {
    arr[x] = arr[x-1];
  }
  arr[j] = i;
}

// insert element i into an array at position j, shift everything else over to make room
#define INSERT_POINT_HERE ({                                  \
  insertPointHere(ordered_indexes, active_pixel_count, j, i); \
  added = true;                                               \
  break;                                                      \
})

uint8_t findCurrentPoints(uint8_t *points) {
  // sort pixels by confidence to find peaks
  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (pixelActive(i)) {
      bool added = false;
      for (uint8_t j=0; j<active_pixel_count; j++) {
        float diff = norm_pixels[i] - norm_pixels[ordered_indexes[j]];
        if (diff > 0.03) {
          // point i has higher confidence, place it in front of existing point j
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
            } else if (abs(diff) < 0.0001) {
              // identical points as part of a spectrum, prefer point closer to middle
              bool sameAxis = AXIS(i) == AXIS(ordered_indexes[j]);
              if ((!sameAxis && SIDE1(i)) || (sameAxis && SIDEL(i))) {
                INSERT_POINT_HERE;
              }
            }
          } else if (abs(diff) < 0.0001) {
            // identical points as part of a spectrum, prefer point closer to middle
            bool sameAxis = AXIS(i) == AXIS(ordered_indexes[j]);
            if ((!sameAxis && SIDE1(i)) || (sameAxis && SIDEL(i))) {
              INSERT_POINT_HERE;
            }
          } else {
            // points aren't nearby, prefer larger masses
            uint8_t h1 = massHeight(i);
            uint8_t h2 = massHeight(ordered_indexes[j]);
            if (h1 > h2) {
              INSERT_POINT_HERE;
            } else if (h1 == h2) {
              uint8_t w1 = massWidth(i);
              uint8_t w2 = massWidth(ordered_indexes[j]);
              if (w1 > w2) {
                INSERT_POINT_HERE;
              }
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
      // and that it isn't part of a blob we already registered
      if (euclidean_distance(ordered_indexes[j], idx) <= MIN_DISTANCE ||
          connectedPoints(ordered_indexes[j], idx)) {
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

  // cache width and height for each point
  uint8_t heightCache[MAX_PEOPLE];
  uint8_t widthCache[MAX_PEOPLE];
  for (uint8_t i = 0; i<total_masses; i++) {
    heightCache[i] = massHeight(points[i]);
    widthCache[i] = massWidth(points[i]);
  }

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
  uint8_t temp_forgotten_heights[MAX_PEOPLE];
  uint8_t temp_forgotten_widths[MAX_PEOPLE];
  uint8_t temp_forgotten_crossed[MAX_PEOPLE];
  bool temp_forgotten_reverted[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;
  uint8_t forgotten_num_frd = 0;

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT ({                                                                   \
    checkForRevert(idx);                                                                    \
    if (((count[idx] > 1 && !crossed[idx]) ||                                               \
          (crossed[idx] && pointInMiddle(past_points[idx]))) &&                             \
          confidence(idx) > AVG_CONF_THRESHOLD) {                                           \
      temp_forgotten_points[temp_forgotten_num] = past_points[idx];                         \
      temp_forgotten_norms[temp_forgotten_num] = confidence(idx);                           \
      temp_forgotten_starting_points[temp_forgotten_num] = starting_points[idx];            \
      temp_forgotten_histories[temp_forgotten_num] = histories[idx];                        \
      temp_forgotten_heights[temp_forgotten_num] = iavgHeight(idx);                         \
      temp_forgotten_widths[temp_forgotten_num] = iavgWidth(idx);                           \
      temp_forgotten_crossed[temp_forgotten_num] = crossed[idx];                            \
      temp_forgotten_reverted[temp_forgotten_num] = reverted[idx];                          \
      temp_forgotten_num++;                                                                 \
    }                                                                                       \
    forgotten_num_frd++;                                                                    \
    pairs[idx] = UNDEF_POINT;                                                               \
    past_points[idx] = UNDEF_POINT;                                                         \
  })

  uint8_t past_total_masses = 0;
  for (uint8_t idx=0; idx<MAX_PEOPLE; idx++) {
    if (past_points[idx] != UNDEF_POINT) past_total_masses++;
  }

  if (past_total_masses > 0) {
    for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
      if (past_points[idx] != UNDEF_POINT) {
        float conf = confidence(idx);
        float max_distance = MAX_DISTANCE + conf * DISTANCE_BONUS;
        float min_score = 100;
        uint8_t min_index = UNDEF_POINT;
        for (uint8_t j=0; j<total_masses; j++) {
          float d = euclidean_distance(past_points[idx], points[j]);

          // if switching sides with low confidence, don't pair
          if (!sameSide(points[j], past_points[idx]) &&
              (conf < AVG_CONF_THRESHOLD || norm_pixels[points[j]]/d < MIN_TRAVEL_RATIO)) {
            continue;
          }

          if (d < max_distance) {
            float ratioP = min(norm_pixels[points[j]]/conf, conf/norm_pixels[points[j]]);
            if (!crossed[idx]) ratioP *= 2.0; // ratio matters less once point is crossed
            float directionBonus = 0;
            uint8_t sp_axis = AXIS(past_points[idx]);
            uint8_t np_axis = AXIS(points[j]);
            if (SIDE1(starting_points[idx])) {
              if (crossed[idx]) {
                if (np_axis < sp_axis) directionBonus = 0.1;
              } else if (np_axis > sp_axis) directionBonus = 0.1;
            } else {
              if (crossed[idx]) {
                if (np_axis > sp_axis) directionBonus = 0.1;
              } else if (np_axis < sp_axis) directionBonus = 0.1;
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
            score *= max(totalDistance(idx), 0.2) + (crossed[idx] ? 3.0 : 0.0);
            score *= (1.0 - abs(norm_pixels[points[i]] - conf));
            score /= max(d, 0.9);
          }
          if (score - max_score > 0.05) {
            max_score = score;
            max_idx = idx;
          } else if (max_score - score < 0.05 ) {
            if (max_idx == UNDEF_POINT) {
              max_idx = idx;
            } else {
              // if 2 competing points have the same score, pick the closer one
              float d2 = euclidean_distance(past_points[max_idx], points[i]);
              if (d+0.05 < d2 || (d-d2 < 0.05 && conf > confidence(max_idx))) {
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
          if (pointOnEdge(points[i]) && sameSide(starting_points[idx], points[i])) {
            // always consider a point on the outer edge as just starting off
            past_points[idx] = points[i];
            starting_points[idx] = points[i];
            histories[idx] = 1;
            crossed[idx] = 0;
            reverted[idx] = false;
            avg_norms[idx] = (avg_norms[idx] + norm_pixels[points[i]])/
                                  ((float)count[idx] + 1.0);
            avg_heights[idx] = (int)round(((float)avg_heights[idx] + (float)heightCache[i])/
                                          ((float)count[idx] + 1.0));
            avg_widths[idx] = (int)round(((float)avg_widths[idx] + (float)widthCache[i])/
                                          ((float)count[idx] + 1.0));
            count[idx] = 1;
          } else {
            if (past_points[idx] != points[i]) {
              if (!sameSide(points[i], past_points[idx])) {
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
            avg_norms[idx] += norm_pixels[points[i]];
            avg_heights[idx] += heightCache[i];
            avg_widths[idx] += widthCache[i];
            count[idx]++;
          }
          if (zombieCount[idx]) zombieCount[idx]++;
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
      uint8_t ah = 0;
      uint8_t aw = 0;
      uint8_t c = 1;
      uint8_t z = 0;
      bool retroMatched = false;
      bool nobodyInFront = true;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        for (uint8_t j=0; j<temp_forgotten_num; j++) {
          if (temp_forgotten_points[j] != UNDEF_POINT) {
            // if switching sides with low confidence or moving too far, don't pair
            float d = euclidean_distance(temp_forgotten_points[j], points[i]);
            if (d >= 3.0 || !sameSide(points[i], temp_forgotten_points[j]) &&
                (temp_forgotten_norms[j] < AVG_CONF_THRESHOLD ||
                norm_pixels[points[i]]/d < MIN_TRAVEL_RATIO)) {
              continue;
            }

            sp = temp_forgotten_starting_points[j];
            h = points[i] == sp ? 1 : min(temp_forgotten_histories[j], MIN_HISTORY);
            cross = temp_forgotten_crossed[j];
            revert = temp_forgotten_reverted[j];
            an += temp_forgotten_norms[j];
            ah += temp_forgotten_heights[j];
            aw += temp_forgotten_widths[j];
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
          if (forgotten_past_points[j] != UNDEF_POINT) {
            // if switching sides with low confidence or moving too far, don't pair
            float d = euclidean_distance(forgotten_past_points[j], points[i]);
            if (d >= 3.0 || (!sameSide(points[i], forgotten_past_points[j]) &&
                (forgotten_norms[j] < AVG_CONF_THRESHOLD ||
                norm_pixels[points[i]]/d < MIN_TRAVEL_RATIO))) {
              continue;
            }

            sp = forgotten_starting_points[j];
            h = points[i] == sp ? 1 : min(forgotten_histories[j], MIN_HISTORY);
            cross = forgotten_crossed[j];
            revert = forgotten_reverted[j];
            an += forgotten_norms[j]/((float)cycles_since_forgotten + 2.0);
            ah += forgotten_heights[j];
            aw += forgotten_widths[j];
            c++;
            z = 1;
            forgotten_past_points[j] = UNDEF_POINT;
            retroMatched = true;
            break;
          }
        }
      }

      if (!retroMatched && pointInMiddle(sp)) {
        bool nobodyOnBoard = true;
        if (past_total_masses > 0) {
          for (uint8_t j=0; j<MAX_PEOPLE; j++) {
            if (past_points[j] != UNDEF_POINT &&
                (count[j] > 1 || crossed[j] ||
                  (pointOnEdge(past_points[j]) && confidence(j) > 0.6) ||
                  avgHeight(j) > heightCache[i]) &&
                euclidean_distance(past_points[j], sp) < MAX_DISTANCE + DISTANCE_BONUS) {
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

        // if point is at least 2 rows high and has mid confidence with nobody ahead...
        if (nobodyInFront && an > 0.6 && heightCache[i] > 1) {
          // and it is in row 5, allow it (door might've just opened)
          if (AXIS(sp) == (GRID_EXTENT/2 + 1) && pixelActive(sp + GRID_EXTENT)) {
            retroMatched = true;
          } else if (AXIS(sp) == (GRID_EXTENT/2)) {
            // or if it's in row 4 and there is somebody behind, assume this is due
            // to a splitting of the person behind
            if (!nobodyOnBoard && pixelActive(sp-GRID_EXTENT)) {
              retroMatched = true;
            } else if (nobodyOnBoard && an > HIGH_CONF_THRESHOLD &&
                        pixelActive(sp+GRID_EXTENT)) {
              // or because person was already through door by the time it opened
              retroMatched = true;
              sp += GRID_EXTENT;
            }
          }
        }
      }

      // ignore new points on side 1 immediately after door opens,
      // or all new points immediately after door closes
      if ((doorOpenedAgo(1) && SIDE1(sp)) || doorClosedAgo(3)) continue;

      // ignore new points that showed up in middle 2 rows of grid
      if (retroMatched ||
          (nobodyInFront && an > AVG_CONF_THRESHOLD && pointOnBorder(sp)) ||
          pointOnSmallBorder(sp)) {
        for (uint8_t j=0; j<MAX_PEOPLE; j++) {
          // look for first empty slot in past_points to use
          if (past_points[j] == UNDEF_POINT) {
            past_points[j] = points[i];
            histories[j] = h;
            starting_points[j] = sp;
            crossed[j] = cross;
            reverted[j] = revert;
            avg_norms[j] = an;
            avg_heights[j] = ah + heightCache[i];
            avg_widths[j] = aw + widthCache[i];
            count[j] = c;
            zombieCount[j] = z;
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
    memcpy(forgotten_heights, temp_forgotten_heights, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_widths, temp_forgotten_widths, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_crossed, temp_forgotten_crossed, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_reverted, temp_forgotten_reverted, (MAX_PEOPLE*sizeof(bool)));
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

      // print chart of what we saw in 8x8 grid
      for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
        SERIAL_PRINT(F(" "));
        if (norm_pixels[idx] < 0.001)
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
  amg.begin();

  // setup reed switches
   DDRD =  DDRD & B11100111;  // set pins 3 and 4 as inputs
  PORTD = PORTD | B00011000;  // pull pins 3 and 4 high

  LOWPOWER_DELAY(SLEEP_1S);
  publish(FIRMWARE_VERSION, 0, 10);

  // give sensor 12sec to stabilize
  LOWPOWER_DELAY(SLEEP_8S);
  LOWPOWER_DELAY(SLEEP_4S);

  memset(past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));

  amg.readPixels(avg_pixels);

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    cur_pixels_hash += sq(avg_pixels[i] + i);
    if (avg_pixels[i] < 0) avg_pixels[i] = 0;
    if (avg_pixels[i] > 65) avg_pixels[i] = 65;
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!pixelsChanged()) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (norm_pixels[i] < 0 || norm_pixels[i] > 65) continue;
      avg_pixels[i] += FAST_ALPHA * (norm_pixels[i] - avg_pixels[i]);
    }
  }
}

void loop_frd() {
  checkDoorState();
  processSensor();
}
