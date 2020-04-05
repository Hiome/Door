float calcMaxDistance(uint8_t height, uint8_t width, uint8_t neighbors, uint8_t confidence) {
  return 3.0 + (height+width+neighbors)/4.0 + (confidence/100.0);
}
#define MAX_DIST_FORMULA ( calcMaxDistance(height, width, neighbors, confidence) )

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
  coord_t   max_position;         //:6 0-63
  uint8_t   confidence;           //:7 0-100
  fint1_t   max_temp_drift;
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
    if (!real() || !crossed) return false;

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

  #define FRD_EVENT         0
  #define SUSPICIOUS_EVENT  1
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
    max_position = past_position;
    if (old_crossed) crossed = 0;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  bool publishMaybeEvent() {
    if (!real()) return false;

    if (door_state != DOOR_OPEN && frames_since_door_open == 0 && door_side == side()) {
      return false;
    }

    if ((history >= MIN_HISTORY || (history == 2 && avg_fgm > 150 && avg_bgm > 150)) &&
        (!crossed || !reverted) && starting_side() != side()) {
      publishPacket(FRD_EVENT);
      return true;
    } else if (avg_fgm > 150 && avg_bgm > 150 &&
                (!crossed || SIDE(max_position) != starting_side()) &&
                axis_distance(max_position, past_position) >= 2) {
      starting_position = max_position;
      publishPacket(SUSPICIOUS_EVENT);
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
  .starting_position=0,
  .max_position=0,
  .confidence=0,
  .max_temp_drift=0,
  .count=0,
  .history=0,
  .retreating=false,
  .neighbors=0,
  .height=0,
  .forgotten_count=0,
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

void clearPointsAfterDoorClose() {
  if (checkDoorState()) {
    cycles_since_forgotten = MAX_EMPTY_CYCLES;
    side1Point = 0;
    side2Point = 0;

    for (idx_t i = 0; i<MAX_PEOPLE; i++) {
      known_people[i].forget();
      known_people[i] = UNDEF_PERSON;
      forgotten_people[i].publishMaybeEvent();
      forgotten_people[i] = UNDEF_PERSON;
    }
  }
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

void forget_person(idx_t idx, Person (&temp_forgotten_people)[MAX_PEOPLE],
                    idx_t (&pairs)[MAX_PEOPLE], uint8_t &temp_forgotten_num) {
  Person p = known_people[(idx)];
  if (p.confidence > 30 &&
      (p.checkForRevert() || axis_distance(p.starting_position, p.past_position) > 1 ||
        axis_distance(p.max_position, p.past_position) > 1)) {
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

void remember_person(Person (&arr)[MAX_PEOPLE], coord_t point, uint8_t &h, coord_t &sp,
        coord_t &mp, uint8_t &mj, fint1_t &md, uint8_t &cross, bool &revert, uint16_t &c,
        uint8_t &fc, uint8_t height, uint8_t width, uint8_t neighbors, uint8_t conf) {
  float maxD = calcMaxDistance(height, width, neighbors, conf);
  idx_t pi = findClosestPerson(arr, point, maxD);
  if (pi != UNDEF_INDEX) {
    Person p = arr[pi];

    axis_t ppaxis = AXIS(p.past_position);
    if ((SIDE1(p.starting_position) && ppaxis-1 > AXIS(point)) ||
        (SIDE2(p.starting_position) && ppaxis+1 < AXIS(point))) {
      // this point is moved behind previous position, just start over
      return;
    }

    if (p.history <= MIN_HISTORY && p.side() != p.starting_side() &&
          pointOnSmallBorder(p.past_position) &&
          p.history <= (MIN_HISTORY+1 - normalizeAxis(ppaxis))) {
      // impossible for this person to ever do anything useful with its life, kill it
      return;
    }

    // point is ahead of starting point at least
    sp = p.starting_position;
    h = min(p.history, MIN_HISTORY);

    fint1_t tempDrift = floatToFint1(p.difference_from_point(point));
    md = max(tempDrift, p.max_temp_drift);

    uint8_t axisJump = max_axis_jump(p.past_position, point);
    mj = max(axisJump, p.max_jump);

    if ((SIDE1(sp) && AXIS(mp) < AXIS(p.max_position)) ||
        (SIDE2(sp) && AXIS(mp) > AXIS(p.max_position))) {
      mp = p.max_position;
    }
    cross = p.crossed;
    revert = p.reverted;
    c = p.count + 1;
    fc = p.forgotten_count + 1;
    arr[pi] = UNDEF_PERSON;
  }
}
