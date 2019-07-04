#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.4.13"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE            1.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            2.0  // max distance that a point is allowed to move
#define DISTANCE_BONUS          2.5  // max extra distance a hot point can move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              4    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define CONFIDENCE_THRESHOLD    0.3  // consider a point if we're 30% confident
#define AVG_CONF_THRESHOLD      0.4  // consider a set of points if we're 40% confident
#define HIGH_CONF_THRESHOLD     0.8  // give points over 80% confidence extra benefits
#define GRADIENT_THRESHOLD      3.0  // 3º temp change gives us 100% confidence of person
#define MIN_TRAVEL_RATIO        0.2  // ratio of norm/distance that a point must pass

#define REED_PIN_CLOSE          3
#define REED_PIN_AJAR           4

#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

uint16_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float cur_pixels_hash = 0;

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
bool crossed[MAX_PEOPLE];
float past_norms[MAX_PEOPLE];
float avg_norms[MAX_PEOPLE];
uint16_t avg_heights[MAX_PEOPLE];
uint16_t avg_widths[MAX_PEOPLE];
uint16_t count[MAX_PEOPLE];

float forgotten_norms[MAX_PEOPLE];
uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
uint8_t forgotten_heights[MAX_PEOPLE];
uint8_t forgotten_widths[MAX_PEOPLE];
bool forgotten_crossed[MAX_PEOPLE];
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
#define SIDE(p)         ( SIDE1(p) ? 1 : 2 )
#define PIXEL_ACTIVE(i) ( norm_pixels[(i)] > CONFIDENCE_THRESHOLD )
#define confidence(x)   ( avg_norms[(x)]/((float)count[(x)]) )
#define avgHeight(x)    ( (float)avg_heights[(x)]/((float)count[x]) )
#define avgWidth(x)     ( (float)avg_widths[(x)]/((float)count[x]) )
#define iavgHeight(x)   ( (int)round(avgHeight(x)) )
#define iavgWidth(x)    ( (int)round(avgWidth(x)) )
#define totalDistance(x)( euclidean_distance(starting_points[(x)], past_points[(x)]) )
#define doorOpenedAgo(x)( frames_since_door_open < (x) && door_state == DOOR_OPEN )
#define doorClosedAgo(x)( frames_since_door_open < (x) && door_state == DOOR_CLOSED )
#define bgPixel(x)      ( ((float)avg_pixels[(x)])/1000.0 )

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
#else
  #define pointOnBorder(i)      ( AXIS(i) <= 3 || AXIS(i) >= 6 )
  #define pointOnSmallBorder(i) ( AXIS(i) <= 2 || AXIS(i) >= 7 )
  #define pointInMiddle(i)      ( AXIS(i) > 2 && AXIS(i) < 7 )
  #define pointOnEdge(i)        ( AXIS(i) == 1 || AXIS(i) == 8 )
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
  for (uint8_t x = i-1; x >= 0 && norm_pixels[x] > 0.001 && AXIS(x) == startAxis; x--) {
    width++;
  }

  return width;
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

void publishEvents() {
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT && histories[i] > MIN_HISTORY &&
          SIDE(starting_points[i]) != SIDE(past_points[i]) &&
          confidence(i) > AVG_CONF_THRESHOLD) {
      int diff = AXIS(starting_points[i]) - AXIS(past_points[i]);
      // point cleanly crossed grid
      if (abs(diff) >= 3 || totalDistance(i) >= 6) {
        uint8_t width = iavgWidth(i);
        uint8_t height = iavgHeight(i);
        uint8_t mass = width*10 + height;
        if (SIDE1(past_points[i])) {
          publish("1", mass, 10);
          // artificially shift starting point ahead 1 row so that
          // if user turns around now, algorithm considers it an exit
          int s = past_points[i] - GRID_EXTENT;
          starting_points[i] = max(s, 0);
        } else {
          publish("2", mass, 10);
          int s = past_points[i] + GRID_EXTENT;
          starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
        }
        histories[i] = 1;
        crossed[i] = true;
        avg_norms[i] = confidence(i);
        avg_heights[i] = height;
        avg_widths[i] = width;
        count[i] = 1;
      }
    }
  }
}

void publishMaybeEvents(uint8_t idx) {
  if (!crossed[idx] && histories[idx] > MIN_HISTORY && totalDistance(idx) > 3) {
    int diff = AXIS(starting_points[idx]) - AXIS(past_points[idx]);
    if ((abs(diff) >= 3 || totalDistance(idx) >= (5-diff)) &&
        confidence(idx) > HIGH_CONF_THRESHOLD) {
      if (SIDE1(past_points[idx])) {
        publish("s1", -1);
      } else {
        publish("s2", -1);
      }
    } else if (count[idx] >= histories[idx] &&
        (histories[idx] >= (2*MIN_HISTORY) || confidence(idx) > AVG_CONF_THRESHOLD)) {
      // we don't know what happened, add door to suspicious list
      if (SIDE1(past_points[idx])) {
        publish("s1", -1);
      } else {
        publish("s2", -1);
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

  // calculate CSM gradient
  float bgm = GRADIENT_THRESHOLD;
  float fgm = GRADIENT_THRESHOLD;
  float bgmt;
  float fgmt;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // reading is invalid if less than -20 or greater than 100,
    // but we use a smaller range than that to determine validity
    if (norm_pixels[i] < 0 || norm_pixels[i] > 65) {
      norm_pixels[i] = bgPixel(i);
      continue;
    }

    if (NOT_AXIS(i) < GRID_EXTENT) {
      fgmt = norm_pixels[i+1] - norm_pixels[i];
      fgmt = abs(fgmt);
      bgmt = (norm_pixels[i+1] - bgPixel(i+1)) - (norm_pixels[i] - bgPixel(i));
      bgmt = abs(bgmt);

      fgm = max(fgmt, fgm);
      bgm = max(bgmt, bgm);
    }
  }

  float offGrid[GRID_EXTENT];
  float std;
  float rowAvg;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    std = norm_pixels[i] - bgPixel(i);

    if (NOT_AXIS(i) == 1) {
      rowAvg = 0.0;
      for (uint8_t ri=i; ri<i+GRID_EXTENT; ri++) {
        rowAvg += norm_pixels[ri];
      }
      rowAvg /= (float)GRID_EXTENT;

      fgmt = std/fgm;
      bgmt = (norm_pixels[i]-rowAvg)/bgm;

      uint8_t x = AXIS(i)-1;
      offGrid[x] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
      if (abs(offGrid[x]) > 1) {
        offGrid[x] = offGrid[x] > 0 ? 1.0 : -1.0;
      }
    }

    // normalize points
    if (NOT_AXIS(i) == GRID_EXTENT) {
      fgmt = -std/fgm;
      bgmt = (rowAvg-norm_pixels[i])/bgm;
      norm_pixels[i] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
      if (abs(norm_pixels[i]) > 1) {
        norm_pixels[i] = norm_pixels[i] > 0 ? 1.0 : -1.0;
      }
    } else {
      fgmt = (norm_pixels[i+1] - norm_pixels[i])/fgm;
      bgmt = (norm_pixels[i+1] - bgPixel(i+1) - std)/bgm;

      norm_pixels[i] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
    }

    // update average baseline
    // implicit alpha of 0.001
    if (std < 1 && norm_pixels[i] < AVG_CONF_THRESHOLD) {
      // increase alpha to 0.01
      std *= 10.0;
    } else if (std >= 10 && norm_pixels[i] >= HIGH_CONF_THRESHOLD) {
      // lower alpha to 0.0001
      std *= 0.1;
    }
    avg_pixels[i] += ((int)round(std));
  }

  // find horizontal midpoint of blob
  float pos;
  float neg;
  uint8_t maxi;
  uint8_t mini;
  bool positive;
  for (uint8_t r=0; r<AMG88xx_PIXEL_ARRAY_SIZE; r+=GRID_EXTENT) {
    pos = 0.0;
    neg = 0.0;
    maxi = UNDEF_POINT;
    mini = UNDEF_POINT;
    positive = true;

    for (uint8_t c=0; c<GRID_EXTENT-1; c++) {
      uint8_t i = r + c;
      if (abs(norm_pixels[i]) <= CONFIDENCE_THRESHOLD) {
        norm_pixels[i] = 0;
        continue;
      }
      bool new_positive = norm_pixels[i] > 0;
      if (new_positive == positive) {
        if (positive) {
          if (norm_pixels[i] >= pos) {
            if (maxi != UNDEF_POINT && maxi != i) norm_pixels[maxi] = 0;
            pos = norm_pixels[i];
            maxi = i;
          } else {
            norm_pixels[i] = 0;
          }
        } else {
          if (norm_pixels[i] <= neg) {
            if (mini != UNDEF_POINT && mini != i) norm_pixels[mini] = 0;
            neg = norm_pixels[i];
            mini = i;
          } else {
            norm_pixels[i] = 0;
          }
        }
      } else {
        if (maxi != UNDEF_POINT && mini != UNDEF_POINT) {
          // we found a pair, ship it
          uint8_t diff = abs(maxi - mini);
          uint8_t upper_pos = max(maxi, mini);
          uint8_t lower_pos = min(maxi, mini);
          uint8_t new_pos = upper_pos - diff/2;
          norm_pixels[new_pos] = max(pos, -neg);
          for (uint8_t x = new_pos + 1; x <= upper_pos; x++) {
            norm_pixels[x] = norm_pixels[new_pos] - 0.05;
          }
          for (uint8_t x = lower_pos; x < new_pos; x++) {
            norm_pixels[x] = norm_pixels[new_pos] - 0.05;
          }

          // reset trackers for rest of row
          pos = 0.0;
          neg = 0.0;
          maxi = UNDEF_POINT;
          mini = UNDEF_POINT;
        }

        positive = new_positive;
        if (positive) {
          pos = norm_pixels[i];
          maxi = i;
        } else {
          neg = norm_pixels[i];
          mini = i;
        }
      }
    }

    if (maxi != UNDEF_POINT && mini == UNDEF_POINT) {
      float edgePoint = offGrid[AXIS(r)-1];
      if (-edgePoint > CONFIDENCE_THRESHOLD) {
        neg = edgePoint;
        mini = r;
      }
      edgePoint = norm_pixels[r+GRID_EXTENT-1];
      if (-edgePoint > CONFIDENCE_THRESHOLD && (mini == UNDEF_POINT || edgePoint < neg)) {
        neg = edgePoint;
        mini = r+GRID_EXTENT-1;
      }
    } else if (maxi == UNDEF_POINT && mini != UNDEF_POINT) {
      float edgePoint = offGrid[AXIS(r)-1];
      if (edgePoint > CONFIDENCE_THRESHOLD) {
        pos = edgePoint;
        maxi = r;
      }
      edgePoint = norm_pixels[r+GRID_EXTENT-1];
      if (edgePoint > CONFIDENCE_THRESHOLD && (maxi == UNDEF_POINT || edgePoint > pos)) {
        pos = edgePoint;
        maxi = r+GRID_EXTENT-1;
      }
    }

    if (maxi != UNDEF_POINT && mini != UNDEF_POINT) {
      // we found a pair, ship it
      uint8_t diff = abs(maxi - mini);
      uint8_t upper_pos = max(maxi, mini);
      uint8_t lower_pos = min(maxi, mini);
      uint8_t new_pos = upper_pos - diff/2;
      norm_pixels[new_pos] = max(pos, -neg);
      for (uint8_t x = new_pos + 1; x <= upper_pos; x++) {
        norm_pixels[x] = norm_pixels[new_pos] - 0.05;
      }
      for (uint8_t x = lower_pos; x < new_pos; x++) {
        norm_pixels[x] = norm_pixels[new_pos] - 0.05;
      }
      if (upper_pos != r+GRID_EXTENT-1) norm_pixels[r+GRID_EXTENT-1] = 0;
    } else {
      if (maxi != UNDEF_POINT) norm_pixels[maxi] = 0;
      if (mini != UNDEF_POINT) norm_pixels[mini] = 0;
      norm_pixels[r+GRID_EXTENT-1] = 0;
    }
  }

  // find vertical midpoint of blob
  for (uint8_t c = 0; c < GRID_EXTENT; c++) {
    pos = 0;
    mini = UNDEF_POINT;
    for (uint8_t r = 0; r < AMG88xx_PIXEL_ARRAY_SIZE; r += GRID_EXTENT) {
      uint8_t i = c + r;
      if (norm_pixels[i] < 0.001) {
        if (mini != UNDEF_POINT) {
          // set maxNorm
          uint8_t diff = i - mini;
          diff /= 16;
          uint8_t new_pos = i - (diff + 1)*GRID_EXTENT;
          for (uint8_t x = mini; x < i; x += GRID_EXTENT) {
            norm_pixels[x] = x == new_pos ? pos : pos - 0.05;
          }

          mini = UNDEF_POINT;
          pos = 0;
        }
      } else {
        if (mini == UNDEF_POINT) mini = i;
        if (norm_pixels[i] > pos) pos = norm_pixels[i];
      }
    }
    if (mini != UNDEF_POINT) {
      uint8_t i = AMG88xx_PIXEL_ARRAY_SIZE + c;
      uint8_t diff = i - mini;
      diff /= 16;
      uint8_t new_pos = i - (diff + 1)*GRID_EXTENT;
      for (uint8_t x = mini; x < i; x += GRID_EXTENT) {
        norm_pixels[x] = x == new_pos ? pos : pos - 0.05;
      }
    }
  }

  return true;
}

// insert element i into an array at position j, shift everything else over to make room
#define INSERT_POINT_HERE ({                        \
  for (uint8_t x=active_pixel_count; x>j; x--) {    \
    ordered_indexes[x] = ordered_indexes[x-1];      \
  }                                                 \
  ordered_indexes[j] = i;                           \
  added = true;                                     \
  break;                                            \
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
        if (diff > 0.01) {
          // point i has higher confidence, place it in front of existing point j
          INSERT_POINT_HERE;
        } else if (diff > -0.001) { // both points are equal...
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
            } else if (AXIS(i) > AXIS(ordered_indexes[j]) || SIDEL(i)) {
              // insert point in hopes of finding a peak later
              INSERT_POINT_HERE;
            }
          } else {
            // points aren't nearby, prefer points that are closer to middle of grid
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
      // and that it isn't part of a blob we already registered
      if (euclidean_distance(ordered_indexes[j], idx) <= MIN_DISTANCE ||
           (cycles_since_forgotten > 0 && connectedPoints(ordered_indexes[j], idx))) {
        distinct = false;
        break;
      }
    }
    if (distinct && cycles_since_forgotten == 0) {
      bool maybe_forgotten = false;
      for (uint8_t f = 0; f < forgotten_num; f++) {
        if (forgotten_past_points[f] != UNDEF_POINT &&
            euclidean_distance(forgotten_past_points[f], idx) < 3) {
          maybe_forgotten = true;
          break;
        }
      }
      if (!maybe_forgotten) {
        for (uint8_t j=0; j<total_masses; j++) {
          if (connectedPoints(points[j], idx)) {
            distinct = false;
            break;
          }
        }
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
  bool temp_forgotten_crossed[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;
  uint8_t forgotten_num_frd = 0;

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT ({                                                                   \
    if (frames_since_door_open > 0 && (count[idx] > 1 || crossed[idx]) &&                   \
          confidence(idx) > AVG_CONF_THRESHOLD) {                                           \
      publishMaybeEvents(idx);                                                              \
      temp_forgotten_points[temp_forgotten_num] = past_points[idx];                         \
      temp_forgotten_norms[temp_forgotten_num] = confidence(idx);                           \
      temp_forgotten_starting_points[temp_forgotten_num] = starting_points[idx];            \
      temp_forgotten_histories[temp_forgotten_num] = histories[idx];                        \
      temp_forgotten_heights[temp_forgotten_num] = iavgHeight(idx);                         \
      temp_forgotten_widths[temp_forgotten_num] = iavgWidth(idx);                           \
      temp_forgotten_crossed[temp_forgotten_num] = crossed[idx];                            \
      temp_forgotten_num++;                                                                 \
    }                                                                                       \
    forgotten_num_frd++;                                                                    \
    pairs[idx] = UNDEF_POINT;                                                               \
    past_points[idx] = UNDEF_POINT;                                                         \
    avg_heights[idx] = 0;                                                                   \
    avg_widths[idx] = 0;                                                                    \
    histories[idx] = 0;                                                                     \
    count[idx] = 0;                                                                         \
    crossed[idx] = false;                                                                   \
  })

  uint8_t past_total_masses = 0;
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT) past_total_masses++;
  }

  if (past_total_masses > 0) {
    for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
      if (past_points[idx] != UNDEF_POINT) {
        // if door just opened/closed, drop any existing points
        if (frames_since_door_open < 1) {
          FORGET_POINT;
          continue;
        }

        float conf = confidence(idx);
        float max_distance = MAX_DISTANCE + conf * DISTANCE_BONUS;
        float min_distance = 0;
        float min_score = 100;
        uint8_t min_index = UNDEF_POINT;
        for (uint8_t j=0; j<total_masses; j++) {
          float d = euclidean_distance(past_points[idx], points[j]);

          // if switching sides with low confidence, don't pair
          if (SIDE(points[j]) != SIDE(past_points[idx]) &&
              (conf < AVG_CONF_THRESHOLD || conf/d < MIN_TRAVEL_RATIO ||
              norm_pixels[points[j]]/d < MIN_TRAVEL_RATIO)) {
            continue;
          }

          if (d < max_distance) {
            float aHeight = avgHeight(idx);
            float aWidth = avgWidth(idx);
            float ratioH = min(heightCache[j]/aHeight, aHeight/heightCache[j]);
            float ratioW = min(widthCache[j]/aWidth, aWidth/widthCache[j]);
            float ratioP = min(norm_pixels[points[j]]/conf, conf/norm_pixels[points[j]]);
            float score = (d/max_distance) - ratioP - ratioH - ratioW +
                              max(AVG_CONF_THRESHOLD - norm_pixels[points[j]], 0.0);
            if (min_score - score > 0.05) {
              min_score = score;
              min_index = j;
              min_distance = d;
            } else if (min_index != UNDEF_POINT && score - min_score < 0.05) {
              // score is the same, pick the point that lets this one move farthest
              float sd1 = euclidean_distance(starting_points[idx], points[j]);
              float sd2 = euclidean_distance(starting_points[idx], points[min_index]);
              if (sd1 > sd2) {
                min_index = j;
                min_distance = d;
              }
            }
          }
        }

        if (min_index != UNDEF_POINT && crossed[idx] &&
            (total_masses + forgotten_num_frd) < past_total_masses &&
            (AXIS(past_points[idx]) <= min(min_distance, 2) ||
            ((GRID_EXTENT+1) - AXIS(past_points[idx])) <= min(min_distance, 2))) {
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
    }
  }

  for (uint8_t i=0; i<total_masses; i++) {
    if (taken[i] > 1) {
      // more than one past point is trying to match with this single current point...
      float max_score = 0.0;
      uint8_t max_idx = UNDEF_POINT;
      for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
        if (past_points[idx] != UNDEF_POINT && pairs[idx] == i) {
          float score = confidence(idx);
          if (score + 0.05 < AVG_CONF_THRESHOLD) {
            score = 0.0;
          } else {
            score *= max(totalDistance(idx), 0.2) + (crossed[idx] ? 4.0 : 0.0);
            score *= (1.0 - abs(norm_pixels[points[i]] - past_norms[idx]));
            score /= max(euclidean_distance(past_points[idx], points[i]), 0.9);
          }
          if (score - max_score > 0.05) {
            max_score = score;
            max_idx = idx;
          } else if (max_score - score < 0.05 ) {
            if (max_idx == UNDEF_POINT) {
              max_idx = idx;
            } else {
              // if 2 competing points have the same score, pick the closer one
              float d1 = euclidean_distance(past_points[idx], points[i]);
              float d2 = euclidean_distance(past_points[max_idx], points[i]);
              if (d1+0.05 < d2 || (d1-d2 < 0.05 && confidence(idx) > confidence(max_idx))) {
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
          if (pointOnEdge(points[i]) && SIDE(starting_points[idx]) == SIDE(points[i])) {
            // always consider a point on the outer edge as just starting off
            past_points[idx] = points[i];
            starting_points[idx] = points[i];
            past_norms[idx] = norm_pixels[points[i]];
            histories[idx] = 1;
            avg_norms[idx] = (avg_norms[idx] + past_norms[idx])/((float)count[idx] + 1.0);
            avg_heights[idx] = (int)round(((float)avg_heights[idx] + (float)heightCache[i])/
                                          ((float)count[idx] + 1.0));
            avg_widths[idx] = (int)round(((float)avg_widths[idx] + (float)widthCache[i])/
                                          ((float)count[idx] + 1.0));
            count[idx] = 1;
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
            avg_heights[idx] += heightCache[i];
            avg_widths[idx] += widthCache[i];
            count[idx]++;
          }
          break;
        }
      }
    } else if (taken[i] == 0) {
      // new point appeared (no past point found), start tracking it
      uint16_t h = 1;
      uint8_t sp = points[i];
      bool cross = false;
      float an = norm_pixels[points[i]];
      uint8_t ah = 0;
      uint8_t aw = 0;
      uint8_t c = 1;
      bool retroMatched = false;
      bool nobodyInFront = true;

      // ignore new points that are on border below average threshold, or too small
      if ((an < AVG_CONF_THRESHOLD && pointOnEdge(sp)) ||
            (widthCache[i] <= 2 && heightCache[i] == 1)) continue;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        for (uint8_t j=0; j<temp_forgotten_num; j++) {
          if (temp_forgotten_points[j] != UNDEF_POINT) {
            // if switching sides with low confidence or moving too far, don't pair
            float d = euclidean_distance(temp_forgotten_points[j], points[i]);
            if (d >= 3.0 || SIDE(points[i]) != SIDE(temp_forgotten_points[j]) &&
                (temp_forgotten_norms[j] < AVG_CONF_THRESHOLD ||
                temp_forgotten_norms[j]/d < MIN_TRAVEL_RATIO ||
                norm_pixels[points[i]]/d < MIN_TRAVEL_RATIO)) {
              continue;
            }

            sp = temp_forgotten_starting_points[j];
            h = points[i] == sp ? 1 : min(temp_forgotten_histories[j], MIN_HISTORY);
            cross = temp_forgotten_crossed[j];
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
            if (d >= 3.0 || (SIDE(points[i]) != SIDE(forgotten_past_points[j]) &&
                (forgotten_norms[j] < AVG_CONF_THRESHOLD ||
                forgotten_norms[j]/d < MIN_TRAVEL_RATIO ||
                norm_pixels[points[i]]/d < MIN_TRAVEL_RATIO))) {
              continue;
            }

            sp = forgotten_starting_points[j];
            h = points[i] == sp ? 1 : min(forgotten_histories[j], MIN_HISTORY);
            cross = forgotten_crossed[j];
            an += forgotten_norms[j]/((float)cycles_since_forgotten + 2.0);
            ah += forgotten_heights[j];
            aw += forgotten_widths[j];
            c++;
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
                (count[j] > 1 || crossed[j] || avgHeight(j) >= heightCache[i]) &&
                euclidean_distance(past_points[j], sp) <
                  max(avgHeight(j), avgWidth(j)) + MAX_DISTANCE + DISTANCE_BONUS) {
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

        if (nobodyInFront && an > 0.6 && doorOpenedAgo(3)) {
          // if point is starting in row 5 and grid is empty, allow it
          if (AXIS(sp) == (GRID_EXTENT/2 + 1)) {
            retroMatched = true;
          } else if (nobodyOnBoard && an > HIGH_CONF_THRESHOLD &&
                      AXIS(sp) == (GRID_EXTENT/2) && PIXEL_ACTIVE(sp+GRID_EXTENT)) {
            retroMatched = true;
            sp += GRID_EXTENT;
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
            past_norms[j] = norm_pixels[points[i]];
            avg_norms[j] = an;
            avg_heights[j] = ah + heightCache[i];
            avg_widths[j] = aw + widthCache[i];
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
    memcpy(forgotten_heights, temp_forgotten_heights, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_widths, temp_forgotten_widths, (MAX_PEOPLE*sizeof(uint8_t)));
    memcpy(forgotten_crossed, temp_forgotten_crossed, (MAX_PEOPLE*sizeof(bool)));
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
        SERIAL_PRINT(norm_pixels[idx]);
        SERIAL_PRINT(F(" "));
        if (x(idx) == GRID_EXTENT) SERIAL_PRINTLN();
      }
      SERIAL_PRINTLN();
      SERIAL_FLUSH;
    }
  #endif
}

void checkDoorState() {
  uint8_t last_door_state = door_state;
  if (digitalRead(REED_PIN_CLOSE) == LOW) {
    door_state = DOOR_CLOSED;
  } else {
    door_state = digitalRead(REED_PIN_AJAR) == LOW ? DOOR_AJAR : DOOR_OPEN;
  }
  if (last_door_state != door_state) {
    frames_since_door_open = 0;
  }
  if (((door_state == DOOR_CLOSED && last_published_door_state != DOOR_CLOSED) ||
       (door_state != DOOR_CLOSED && last_published_door_state != DOOR_OPEN)) &&
      publish(door_state == DOOR_CLOSED ? "d0" : "d1", 0)) {
    last_published_door_state = door_state == DOOR_CLOSED ? DOOR_CLOSED : DOOR_OPEN;
  }
}

void initialize() {
  amg.begin();

  pinMode(REED_PIN_CLOSE, INPUT_PULLUP);
  pinMode(REED_PIN_AJAR, INPUT_PULLUP);

  blink(2);
  publish(FIRMWARE_VERSION, 10);

  // give sensor 12sec to stabilize
  blink(24);
  // let sensor calibrate with light off
  LOWPOWER_DELAY(SLEEP_1S);

  memset(histories, 0, (MAX_PEOPLE*sizeof(uint16_t)));
  memset(crossed, false, (MAX_PEOPLE*sizeof(bool)));
  memset(past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));

  amg.readPixels(norm_pixels);

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    cur_pixels_hash += sq(norm_pixels[i] + i);
    if (norm_pixels[i] > 65) norm_pixels[i] = 65;
    if (norm_pixels[i] < 0) norm_pixels[i] = 0;
    avg_pixels[i] = ((int)round(norm_pixels[i] * 1000.0));
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!pixelsChanged()) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (norm_pixels[i] > 65) norm_pixels[i] = 65;
      if (norm_pixels[i] < 0) norm_pixels[i] = 0;
      avg_pixels[i] += ((int)round(100.0 * (norm_pixels[i] - bgPixel(i)))); // alpha of 0.1
    }
  }
}

void loop_frd() {
  checkDoorState();
  processSensor();
}
