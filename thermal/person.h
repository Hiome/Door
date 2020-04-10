float calcMaxDistance(uint8_t height, uint8_t width, uint8_t neighbors, uint8_t confidence) {
  float d = 3.0 + (height+width+neighbors)/4.0 + (confidence/100.0);
  return min(d, 5.5);
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
    return calcMaxDistance(height, width, neighbors, confidence);
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
  uint8_t   max_jump          :4; // 0-7 (only need 3 bits)

  uint8_t   avg_height        :4; // only need 3 bits
  uint8_t   avg_width         :4; // only need 3 bits

  uint8_t   avg_neighbors;        // only need 4 bits
  uint8_t   avg_confidence;
  uint8_t   blobSize;
  uint8_t   noiseSize;

  uint16_t  avg_bgm;
  uint16_t  avg_fgm;

  float     raw_temp;
  float     fgm;

  bool      real() { return past_position != UNDEF_POINT; };
  uint8_t   starting_side() { return SIDE(starting_position); };
  uint8_t   side() { return SIDE(past_position); };
  float     total_distance() {
    return euclidean_distance(starting_position, past_position);
  };
  float     max_distance() {
    return calcMaxDistance(height, width, neighbors, confidence);
  };
  float     max_allowed_temp_drift() {
    return maxTempDiffForFgd(fgm);
  };
  float     difference_from_point(coord_t a) {
    return abs(raw_pixels[(a)] - raw_temp);
  };

  #define METALENGTH  48
  void generateMeta(char *meta) {
    sprintf(meta, "%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%u",
      avg_confidence,                     // 3  100
      avg_bgm,                            // 4  1020
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
      max_temp_drift,                     // 3  250
      forgotten_count                     // 1  3
    );                                    // + 14 'x' + 1 null => 48 total
  };

  uint8_t _publishFrd(const char* msg, uint8_t retries) {
    char meta[METALENGTH];
    generateMeta(meta);
    return publish(msg, meta, retries);
  };

  void publishPacket() {
    // door has been closed/ajar for more than 1 frame, no way anybody crossed
    if (door_state != DOOR_OPEN && frames_since_door_open) return;

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
    retreating = false;
    count = 1;
    history = 1;
    max_position = past_position;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  bool publishMaybeEvent() {
    // door has been closed/ajar for more than 1 frame, no way anybody crossed
    if (door_state != DOOR_OPEN && frames_since_door_open) return false;

    if (history >= MIN_HISTORY && starting_side() != side()) {
      publishPacket();
      return true;
    } else if (avg_fgm > 150 && avg_bgm > 150 &&
                axis_distance(max_position, starting_position) >= 2) {
      if (axis_distance(max_position, past_position) >= 2) {
        if (crossed && SIDE(max_position) == starting_side()) return false;
        starting_position = max_position;
      } else if (axis_distance(starting_position, past_position) < 2) return false;

      if (crossed) {
        char rBuf[3];
        sprintf(rBuf, "s%u", crossed);
        _publishFrd(rBuf, 3);
      } else _publishFrd("s", 3);

      return true;
    }

    return false;
  };
} Person;

Person known_people[MAX_PEOPLE];
Person forgotten_people[MAX_PEOPLE];
uint8_t forgotten_expirations[MAX_PEOPLE] = { 0 };

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
  .avg_height=0,
  .avg_width=0,
  .avg_neighbors=0,
  .avg_confidence=0,
  .blobSize=0,
  .noiseSize=0,
  .avg_bgm=0,
  .avg_fgm=0,
  .raw_temp=0,
  .fgm=0
};

void publishEvents() {
  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    if (known_people[i].real() && known_people[i].history > MIN_HISTORY &&
        known_people[i].starting_side() != known_people[i].side()) {
      Person p = known_people[i];
      p.publishPacket();
      known_people[i] = p; // update known_people array
    }
  }
}

void clearPointsAfterDoorClose() {
  if (checkDoorState()) {
    for (idx_t i = 0; i<MAX_PEOPLE; i++) {
      if (known_people[i].real() && known_people[i].avg_fgm < 150) {
        known_people[i].publishMaybeEvent();
        known_people[i] = UNDEF_PERSON;
      }

      if (forgotten_people[i].real()) {
        forgotten_people[i].publishMaybeEvent();
        forgotten_people[i] = UNDEF_PERSON;
      }
    }
  }
}

void store_forgotten_person(Person p, uint8_t cnt) {
  idx_t useIdx = UNDEF_INDEX;
  uint8_t min_conf = p.confidence;
  for (idx_t j = 0; j < MAX_PEOPLE; j++) {
    if (!forgotten_people[j].real()) {
      // slot is empty, use it and stop looking for another
      useIdx = j;
      break;
    } else if (forgotten_people[j].confidence < min_conf) {
      // this slot is lower confidence, consider using it
      min_conf = forgotten_people[j].confidence;
      useIdx = j;
    }
  }
  if (useIdx != UNDEF_INDEX) {
    // we found a slot! save person in there
    if (forgotten_people[useIdx].real()) {
      forgotten_people[useIdx].publishMaybeEvent();
    }
    forgotten_people[useIdx] = p;
    forgotten_expirations[useIdx] = cnt;
    SERIAL_PRINTLN(F("s"));
  }
}

void expireForgottenPeople() {
  for (idx_t i = 0; i < MAX_PEOPLE; i++) {
    if (forgotten_people[i].real()) {
      if (forgotten_expirations[i] == 0) {
        forgotten_people[i].publishMaybeEvent();
        forgotten_people[i] = UNDEF_PERSON;
        SERIAL_PRINTLN(F("f"));
      } else {
        forgotten_expirations[i]--;
      }
    }
  }
}

void forget_person(idx_t idx, idx_t (&pairs)[MAX_PEOPLE]) {
  Person p = known_people[(idx)];
  if (p.confidence > 30 && (axis_distance(p.starting_position, p.past_position) > 1 ||
        axis_distance(p.max_position, p.past_position) > 1)) {
    if (p.forgotten_count < MAX_FORGOTTEN_COUNT) {
      store_forgotten_person(p, MAX_EMPTY_CYCLES);
    } else {
      // we're giving up on this point, but at least publish what we have
      p.publishMaybeEvent();
    }
  }
  pairs[(idx)] = UNDEF_INDEX;
  known_people[(idx)] = UNDEF_PERSON;
}
#define FORGET_POINT ( forget_person(idx, pairs) )

idx_t findClosestPerson(coord_t i, float maxDistance) {
  idx_t pidx = UNDEF_INDEX;
  float minTemp = 1.0;
  float maxTemp = maxTempDiffForFgd(fgDiff(i));
  maxDistance = min(maxDistance, 4.5);
  for (idx_t x=0; x<MAX_PEOPLE; x++) {
    if (forgotten_people[x].real()) {
      Person p = forgotten_people[x];
      float dist = euclidean_distance(p.past_position, i);
      float maxD = p.max_distance();
      if (dist > min(maxDistance, maxD)) continue;

      float tempDiff = p.difference_from_point(i);
      float maxT = p.max_allowed_temp_drift();
      if (tempDiff > min(maxTemp, maxT)) continue;

      float tempRatio = tempDiff/maxTemp;
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

void remember_person(coord_t &sp, coord_t &mp, uint8_t &mj, fint1_t &md,
    uint8_t &cross, uint8_t &h, uint16_t &c, uint8_t &fc, float maxDistance) {
  coord_t pi = findClosestPerson(sp, maxDistance);
  if (pi == UNDEF_INDEX) return;
  Person p = forgotten_people[pi];

  axis_t ppaxis = AXIS(p.past_position);
  if ((SIDE1(p.starting_position) && ppaxis-1 > AXIS(sp)) ||
      (SIDE2(p.starting_position) && ppaxis+1 < AXIS(sp))) {
    // this point is moved behind previous position, just start over
    return;
  }

  if (p.history <= MIN_HISTORY && p.side() != p.starting_side() &&
        pointOnSmallBorder(p.past_position) &&
        p.history <= (MIN_HISTORY+1 - normalizeAxis(ppaxis))) {
    // impossible for this person to ever do anything useful with its life, kill it
    return;
  }

  fint1_t tempDrift = floatToFint1(p.difference_from_point(sp));
  md = max(tempDrift, p.max_temp_drift);

  uint8_t axisJump = max_axis_jump(p.past_position, sp);
  mj = max(axisJump, p.max_jump);

  if ((SIDE1(p.starting_position) && AXIS(mp) < AXIS(p.max_position)) ||
      (SIDE2(p.starting_position) && AXIS(mp) > AXIS(p.max_position))) {
    mp = p.max_position;
  }
  cross = p.crossed;
  sp = p.starting_position;
  h = min(p.history, MIN_HISTORY);
  c = p.count + 1;
  fc = p.forgotten_count + 1;

  forgotten_people[pi] = UNDEF_PERSON;
}
