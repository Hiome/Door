#define PRINT_RAW_DATA      // uncomment to print graph of what sensor is seeing

#define FIRMWARE_VERSION        "V0.4.1"
#define YAXIS                        // axis along which we expect points to move (x or y)
#define GRID_EXTENT             8    // size of grid (8x8)
#define MIN_DISTANCE            2.5  // min distance for 2 peaks to be separate people
#define MAX_DISTANCE            3.0  // max distance that a point is allowed to move
#define DISTANCE_BONUS          2.5  // max extra distance a hot point can move
#define MIN_HISTORY             3    // min number of times a point needs to be seen
#define MAX_PEOPLE              4    // most people we support in a single frame
#define MAX_EMPTY_CYCLES        2    // max empty cycles to remember forgotten points
#define CONFIDENCE_THRESHOLD    0.1  // consider a point if we're 10% confident
#define AVG_CONF_THRESHOLD      0.3  // consider a set of points if we're 30% confident
#define HIGH_CONF_THRESHOLD     0.7  // give points over 70% confidence extra benefits
#define GRADIENT_THRESHOLD      3.0  // 3º temp change gives us 100% confidence of person
#define T_THRESHOLD             3    // min squared standard deviations of change for a pixel
#define MIN_NEIGHBORS           3    // min size of halo effect to consider a point legit
#define NUM_STD_DEV             3.0  // max num of std dev to include in trimmed average

#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

uint16_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
uint16_t std_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float cur_pixels_hash = 0;

uint8_t past_points[MAX_PEOPLE];
uint8_t starting_points[MAX_PEOPLE];
uint16_t histories[MAX_PEOPLE];
bool crossed[MAX_PEOPLE];
float past_norms[MAX_PEOPLE];
float avg_norms[MAX_PEOPLE];
uint16_t count[MAX_PEOPLE];

float forgotten_norms[MAX_PEOPLE];
uint8_t forgotten_past_points[MAX_PEOPLE];
uint8_t forgotten_starting_points[MAX_PEOPLE];
uint16_t forgotten_histories[MAX_PEOPLE];
bool forgotten_crossed[MAX_PEOPLE];
uint8_t forgotten_num = 0;
uint8_t cycles_since_forgotten = 0;

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
#else
  #define AXIS          x
  #define NOT_AXIS      y
  #define SIDE1(p)      ( (AXIS(p)) <= (GRID_EXTENT/2) )
  #error Double check all your code, this is untested
#endif
#define UNDEF_POINT     ( AMG88xx_PIXEL_ARRAY_SIZE + 10 )
#define UPPER_BOUND     ( GRID_EXTENT+1 )
#define BORDER_PADDING  ( GRID_EXTENT/4 )
#define SIDE(p)         ( SIDE1(p) ? 1 : 2 )
#define PIXEL_ACTIVE(i) ( norm_pixels[(i)] > CONFIDENCE_THRESHOLD )
#define confidence(x)   ( avg_norms[(x)]/((float)count[(x)]) )
#define totalDistance(x)( euclidean_distance(starting_points[(x)], past_points[(x)]) )
#define bgPixel(x)      ( ((float)avg_pixels[(x)])/1000.0 )
#define stdPixel(x)     ( ((float)std_pixels[(x)])/1000.0 )
#define MAHALANBOIS(x,t)( sq(norm_pixels[(x)]-bgPixel(x))/stdPixel(x) >= (t) )

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
  #define pointOnBorder(i) ( (i) < (GRID_EXTENT * 3) || (i) >= (GRID_EXTENT * 5) )
  #define pointInMiddle(i) ( (i) >= (GRID_EXTENT * 2) && (i) < (GRID_EXTENT * 6) )
  #define pointOnEdge(i)   ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
#else
  #define pointOnBorder(i) ( AXIS(i) < (GRID_EXTENT/2) || AXIS(i) > (GRID_EXTENT/2 + 1) )
  #define pointInMiddle(i) ( AXIS(i) > BORDER_PADDING &&            \
                             AXIS(i) < (UPPER_BOUND-BORDER_PADDING) )
  #define pointOnEdge(i)   ( AXIS(i) == 1 || AXIS(i) == GRID_EXTENT )
#endif

void publishEvents() {
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT && histories[i] > MIN_HISTORY &&
          SIDE(starting_points[i]) != SIDE(past_points[i]) &&
          confidence(i) > AVG_CONF_THRESHOLD) {
      int diff = AXIS(starting_points[i]) - AXIS(past_points[i]);
      // point cleanly crossed grid
      if (abs(diff) >= 3 || totalDistance(i) >= 6) {
        if (SIDE1(past_points[i])) {
          publish("1", 10);
          // artificially shift starting point ahead 1 row so that
          // if user turns around now, algorithm considers it an exit
          int s = past_points[i] - GRID_EXTENT;
          starting_points[i] = max(s, 0);
        } else {
          publish("2", 10);
          int s = past_points[i] + GRID_EXTENT;
          starting_points[i] = min(s, (AMG88xx_PIXEL_ARRAY_SIZE-1));
        }
        histories[i] = 1;
        crossed[i] = true;
        avg_norms[i] = past_norms[i];
        count[i] = 1;
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

  // ignore points that don't have enough neighbors
  bool ignorable[AMG88xx_PIXEL_ARRAY_SIZE];
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    // reading is invalid if less than -20 or greater than 100,
    // but we use a smaller range than that to determine validity
    if (norm_pixels[i] < 0 || norm_pixels[i] > 80) {
      norm_pixels[i] = bgPixel(i);
    }

    if (MAHALANBOIS(i, T_THRESHOLD)) {
      uint8_t neighbors = 0;
      if (AXIS(i) > 1) {
        // not top of row
        if (MAHALANBOIS(i - GRID_EXTENT, T_THRESHOLD)) neighbors++;
        if (NOT_AXIS(i) > 1 && MAHALANBOIS(i-(GRID_EXTENT+1), T_THRESHOLD)) neighbors++;
        if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i-(GRID_EXTENT-1), T_THRESHOLD))
          neighbors++;
      }
      if (AXIS(i) < GRID_EXTENT) {
        // not bottom of row
        if (MAHALANBOIS(i + GRID_EXTENT, T_THRESHOLD)) neighbors++;
        if (NOT_AXIS(i) > 1 && MAHALANBOIS(i+(GRID_EXTENT-1), T_THRESHOLD)) neighbors++;
        if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i+(GRID_EXTENT+1), T_THRESHOLD))
          neighbors++;
      }
      if (NOT_AXIS(i) > 1 && MAHALANBOIS(i-1, T_THRESHOLD)) neighbors++;
      if (NOT_AXIS(i) < GRID_EXTENT && MAHALANBOIS(i+1, T_THRESHOLD)) neighbors++;

      ignorable[i] = neighbors < MIN_NEIGHBORS;
    } else {
      ignorable[i] = true;
    }
  }

  // calculate CSM gradient
  float bgm = GRADIENT_THRESHOLD;
  float fgm = GRADIENT_THRESHOLD;
  float bgmt;
  float fgmt;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (NOT_AXIS(i) < GRID_EXTENT && (!ignorable[i] || !ignorable[i+1])) {
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
  float var;
  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    std = norm_pixels[i] - bgPixel(i);

    if (NOT_AXIS(i) == 1) {
      uint8_t x = AXIS(i)-1;
      fgmt = std/fgm;
      bgmt = std/bgm;
      offGrid[x] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
      if (abs(offGrid[x]) > 1) {
        offGrid[x] = offGrid[x] > 0 ? 1.0 : -1.0;
      }
    }

    // normalize points
    if (NOT_AXIS(i) == GRID_EXTENT) {
      fgmt = -std/fgm;
      bgmt = -std/bgm;
      norm_pixels[i] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
      if (abs(norm_pixels[i]) > 1) {
        norm_pixels[i] = norm_pixels[i] > 0 ? 1.0 : -1.0;
      }
    } else if (!ignorable[i] || !ignorable[i+1]) {
      fgmt = (norm_pixels[i+1] - norm_pixels[i])/fgm;
      bgmt = (norm_pixels[i+1] - bgPixel(i+1) - std)/bgm;

      norm_pixels[i] = abs(fgmt) < abs(bgmt) ? fgmt : bgmt;
    } else {
      norm_pixels[i] = 0.0;
    }

    // update average baseline
    var = (abs(std) > 0.1 ? sq(std) : 0.01) - stdPixel(i);
    // implicit alpha of 0.001
    if (abs(std) < 1) {
      // increase alpha to 0.01
      std *= 10.0;
    } else if (abs(std) >= 10) {
      // lower alpha to 0.0001
      std *= 0.1;
    }
    // implicit alpha of 0.001
    if (var < 1) {
      // increase alpha to 0.01
      var *= 10.0;
    } else if (var >= 10) {
      // lower alpha to 0.0001
      var *= 0.1;
    }
    avg_pixels[i] += ((int)round(std));
    std_pixels[i] += ((int)round(var));
    if (std_pixels[i] < 10) std_pixels[i] = 10;
  }

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

    float start = offGrid[AXIS(r)-1];
    if (abs(start) > CONFIDENCE_THRESHOLD) {
      positive = start > 0;
      if (positive) {
        pos = start;
        maxi = r;
      } else {
        neg = start;
        mini = r;
      }
    }

    for (uint8_t c=0; c<GRID_EXTENT; c++) {
      uint8_t i = r + c;
      if (abs(norm_pixels[i]) <= CONFIDENCE_THRESHOLD) {
        norm_pixels[i] = 0;
        continue;
      }
      bool new_positive = norm_pixels[i] > 0;
      if (new_positive == positive) {
        if (positive) {
          if (norm_pixels[i] > pos) {
            if (maxi != UNDEF_POINT) norm_pixels[maxi] = 0;
            pos = norm_pixels[i];
            maxi = i;
          } else {
            norm_pixels[i] = 0;
          }
        } else {
          if (norm_pixels[i] < neg) {
            if (mini != UNDEF_POINT) norm_pixels[mini] = 0;
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
          if (diff < 7) {
            uint8_t new_pos = max(maxi, mini) - diff/2;
            norm_pixels[new_pos] = max(pos, -neg);
          }
          norm_pixels[maxi] = 0;
          norm_pixels[mini] = 0;

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

    if (maxi != UNDEF_POINT && mini != UNDEF_POINT) {
      // we found a pair, ship it
      uint8_t diff = abs(maxi - mini);
      if (diff < 7) {
        uint8_t new_pos = max(maxi, mini) - diff/2;
        norm_pixels[new_pos] = max(pos, -neg);
      }
    }

    if (maxi != UNDEF_POINT) norm_pixels[maxi] = 0;
    if (mini != UNDEF_POINT) norm_pixels[mini] = 0;
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
        if (norm_pixels[i] > norm_pixels[ordered_indexes[j]] &&
            (norm_pixels[i] - norm_pixels[ordered_indexes[j]] >= 0.05 ||
            euclidean_distance(i, ordered_indexes[j]) > MIN_DISTANCE)) {
          // point i has higher confidence, place it in front of existing point j
          INSERT_POINT_HERE;
        } else if (abs(norm_pixels[ordered_indexes[j]] - norm_pixels[i]) < 0.05 &&
            euclidean_distance(i, ordered_indexes[j]) <= MIN_DISTANCE) {
          // both points have similar confidence and are next to each other,
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
              // new point is closer, insert it here and raise its confidence so we stay
              // in sorted order
              norm_pixels[i] = max(norm_pixels[i], norm_pixels[ordered_indexes[j]]);
              INSERT_POINT_HERE;
            } else {
              // otherwise, raise confidence of other point to keep sorted order consistent
              norm_pixels[ordered_indexes[j]] = max(norm_pixels[i],
                                                    norm_pixels[ordered_indexes[j]]);
            }
          } else {
            // prefer later point because it might end up close to a future peak.
            // note: this disadvantages points on side 2 because it's more likely to
            // drag them back to the bottom edge, while points on side 1 will be dragged
            // to middle.
            norm_pixels[i] = max(norm_pixels[i], norm_pixels[ordered_indexes[j]]);
            INSERT_POINT_HERE;
          }
        } // else i is much less than j, so place it later in queue
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

void loop_frd() {
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
  bool temp_forgotten_crossed[MAX_PEOPLE];
  uint8_t temp_forgotten_num = 0;
  uint8_t forgotten_num_frd = 0;

  // track forgotten point states in temporary local variables and reset global ones
  #define FORGET_POINT ({                                                         \
    if (confidence(idx) > AVG_CONF_THRESHOLD && !pointOnEdge(past_points[idx])) { \
      temp_forgotten_points[temp_forgotten_num] = past_points[idx];               \
      temp_forgotten_norms[temp_forgotten_num] = past_norms[idx];                 \
      temp_forgotten_starting_points[temp_forgotten_num] = starting_points[idx];  \
      temp_forgotten_histories[temp_forgotten_num] = histories[idx];              \
      temp_forgotten_crossed[temp_forgotten_num] = crossed[idx];                  \
      temp_forgotten_num++;                                                       \
    }                                                                             \
    forgotten_num_frd++;                                                          \
    pairs[idx] = UNDEF_POINT;                                                     \
    past_points[idx] = UNDEF_POINT;                                               \
    histories[idx] = 0;                                                           \
    crossed[idx] = false;                                                         \
  })

  uint8_t past_total_masses = 0;
  for (uint8_t i=0; i<MAX_PEOPLE; i++) {
    if (past_points[i] != UNDEF_POINT) past_total_masses++;
  }

  if (past_total_masses > 0) {
    for (uint8_t idx=0; idx < MAX_PEOPLE; idx++) {
      if (past_points[idx] != UNDEF_POINT) {
        float max_distance = MAX_DISTANCE + past_norms[idx] * DISTANCE_BONUS;
        float min_distance = 0;
        float min_score = 100;
        uint8_t min_index = UNDEF_POINT;
        for (uint8_t j=0; j<total_masses; j++) {
          // if more than a 3x difference between these points, don't pair them
          if ((norm_pixels[points[j]] < past_norms[idx] &&
              norm_pixels[points[j]] * 3.0 < past_norms[idx]) ||
              (past_norms[idx] < norm_pixels[points[j]] &&
              past_norms[idx] * 3.0 < norm_pixels[points[j]])) {
            continue;
          }

          float d = euclidean_distance(past_points[idx], points[j]);

          // if switching sides with low confidence, don't pair
          if (SIDE(points[j]) != SIDE(past_points[idx]) &&
              (confidence(idx) < AVG_CONF_THRESHOLD ||
              norm_pixels[points[j]]/d < 0.1)) {
            continue;
          }

          if (d < max_distance) {
            float score = (d/max_distance) - (norm_pixels[points[j]]/past_norms[idx]) +
                            max(AVG_CONF_THRESHOLD - norm_pixels[points[j]], 0.0);
            if (score < min_score) {
              min_score = score;
              min_index = j;
              min_distance = d;
            } else if (min_index != UNDEF_POINT && abs(score - min_score) < 0.01) {
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
            (AXIS(past_points[idx]) <= min(min_distance, BORDER_PADDING) ||
            (UPPER_BOUND - AXIS(past_points[idx])) <= min(min_distance, BORDER_PADDING))) {
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
          if (score < AVG_CONF_THRESHOLD) {
            score = 0.0;
          } else {
            score *= max(totalDistance(idx), 0.5) + (crossed[idx] ? 4.0 : 0.0);
            score *= (1.0 - abs(norm_pixels[points[i]] - past_norms[idx]));
            score /= max(euclidean_distance(past_points[idx], points[i]), 0.9);
          }
          if (score > max_score) {
            max_score = score;
            max_idx = idx;
          } else if (abs(score - max_score) < 0.01 ) {
            if (max_idx == UNDEF_POINT) {
              max_idx = idx;
            } else {
              // if 2 competing points have the same score, pick the closer one
              float d1 = euclidean_distance(past_points[idx], points[i]);
              float d2 = euclidean_distance(past_points[max_idx], points[i]);
              if (d1 < d2 || (abs(d1-d2) < 0.01 && past_norms[idx] > past_norms[max_idx])) {
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
            avg_norms[idx] = past_norms[idx];
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
      bool retroMatched = false;
      bool nobodyInMiddle = true;

      if (temp_forgotten_num > 0 && !pointOnEdge(points[i])) {
        // first let's check points on death row from this frame for a match
        for (uint8_t j=0; j<temp_forgotten_num; j++) {
          if (temp_forgotten_points[j] != UNDEF_POINT &&
              // point cannot be more than 3x warmer than forgotten point
              ((temp_forgotten_norms[j] <= an && temp_forgotten_norms[j]*3.0 > an) ||
                (an < temp_forgotten_norms[j] && an*3.0 > temp_forgotten_norms[j]))) {
            // if switching sides with low confidence or moving too far, don't pair
            float d = euclidean_distance(temp_forgotten_points[j], points[i]);
            if (d >= MAX_DISTANCE || (SIDE(points[i]) != SIDE(temp_forgotten_points[j]) &&
                (temp_forgotten_norms[j] < AVG_CONF_THRESHOLD ||
                norm_pixels[points[i]]/d < 0.1))) {
              continue;
            }

            h = temp_forgotten_histories[j];
            sp = temp_forgotten_starting_points[j];
            cross = temp_forgotten_crossed[j];
            if (points[i] == sp) {
              h = 1;
            } else if (SIDE(temp_forgotten_points[j]) != SIDE(points[i])) {
              h = min(h + 1, MIN_HISTORY);
            } else {
              h++;
            }
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
              ((forgotten_norms[j] <= an && forgotten_norms[j]*3.0 > an) ||
                (an < forgotten_norms[j] && an*3.0 > forgotten_norms[j]))) {
            // if switching sides with low confidence or moving too far, don't pair
            float d = euclidean_distance(forgotten_past_points[j], points[i]);
            if (d >= MAX_DISTANCE || (SIDE(points[i]) != SIDE(forgotten_past_points[j]) &&
                (forgotten_norms[j] < AVG_CONF_THRESHOLD ||
                norm_pixels[points[i]]/d < 0.1))) {
              continue;
            }

            h = forgotten_histories[j];
            sp = forgotten_starting_points[j];
            cross = forgotten_crossed[j];
            an = 0.0;
            if (points[i] == sp) {
              h = 1;
            } else if (SIDE(forgotten_past_points[j]) != SIDE(points[i])) {
              h = min(h, MIN_HISTORY);
            }
            forgotten_past_points[j] = UNDEF_POINT;
            retroMatched = true;
            break;
          }
        }
      }

      if (!retroMatched) {
        for (uint8_t j=0; j<MAX_PEOPLE; j++) {
          if (past_points[j] != UNDEF_POINT && (histories[j] > 1 || crossed[j]) &&
                pointInMiddle(past_points[j]) && confidence(j) > AVG_CONF_THRESHOLD &&
                euclidean_distance(past_points[j], points[i]) < 5.0) {
            // there's already a person in the middle of the grid
            // so it's unlikely a new valid person just appeared in the middle
            // (person can't be running and door wasn't closed)
            nobodyInMiddle = false;
            break;
          }
        }

        if (nobodyInMiddle && an > AVG_CONF_THRESHOLD && AXIS(sp) == (GRID_EXTENT/2 + 1)) {
          // if point is starting in row 5, pretend it started in 6
          retroMatched = true;
        }
      }

      // ignore new points that showed up in middle 2 rows of grid
      if (retroMatched ||
          (nobodyInMiddle && an > AVG_CONF_THRESHOLD && pointOnBorder(sp)) ||
          !pointInMiddle(sp)) {
        for (uint8_t j=0; j<MAX_PEOPLE; j++) {
          // look for first empty slot in past_points to use
          if (past_points[j] == UNDEF_POINT) {
            past_points[j] = points[i];
            histories[j] = h;
            starting_points[j] = sp;
            crossed[j] = cross;
            past_norms[j] = norm_pixels[points[i]];
            avg_norms[j] = an;
            count[j] = 1;
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

void initialize() {
  amg.begin();

  blink(2);
  publish(FIRMWARE_VERSION, 10);

  // give sensor 12sec to stabilize
  blink(24);
  // let sensor calibrate with light off
  LOWPOWER_DELAY(SLEEP_1S);

  memset(histories, 0, (MAX_PEOPLE*sizeof(uint16_t)));
  memset(crossed, false, (MAX_PEOPLE*sizeof(bool)));
  memset(past_points, UNDEF_POINT, (MAX_PEOPLE*sizeof(uint8_t)));
  memset(std_pixels, 10, (AMG88xx_PIXEL_ARRAY_SIZE*sizeof(uint8_t)));

  amg.readPixels(norm_pixels);

  for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    cur_pixels_hash += sq(norm_pixels[i] + i);
    avg_pixels[i] = ((int)round(norm_pixels[i] * 1000.0));
  }

  float std;
  float var;
  for (uint8_t k=0; k < 10; k++) {
    while (!pixelsChanged()) {
      // wait for pixels to change
      LOWPOWER_DELAY(SLEEP_30MS);
    }

    for (uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      std = norm_pixels[i] - bgPixel(i);
      var = (abs(std) > 0.1 ? sq(std) : 0.01) - stdPixel(i);
      // implicit alpha of 0.1
      avg_pixels[i] += ((int)round(100.0 * std));
      std_pixels[i] += ((int)round(100.0 * var));
      if (std_pixels[i] < 10) std_pixels[i] = 10;
    }
  }
}
