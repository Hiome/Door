float calcMaxDistance(uint8_t height, uint8_t width, uint8_t neighbors, uint8_t confidence) {
  return 3.0 + (height+width+neighbors)/4.0 + (confidence/100.0);
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
    float f = fgm();
    float b = bgm();
    return max(f, b);
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

  uint8_t   height            :4; // 0-7 (only need 3 bits)
  uint8_t   width             :4; // 0-7 (only need 3 bits)

  uint8_t   crossed           :4; // 0-9
  uint8_t   max_jump          :4; // 0-7 (only need 3 bits)

  uint8_t   avg_height        :4; // only need 3 bits
  uint8_t   avg_width         :4; // only need 3 bits

  uint8_t   avg_neighbors;        // only need 4 bits
  uint8_t   avg_confidence;
  uint8_t   blobSize;
  uint8_t   noiseSize;
  uint8_t   forgotten_count;

  uint16_t  avg_bgm;
  uint16_t  avg_fgm;

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
    return calcMaxDistance(height, width, neighbors, confidence);
  };
  float     max_allowed_temp_drift() {
    return max(fgm, bgm);
  };
  float     difference_from_point(coord_t a) {
    return abs(raw_pixels[(a)] - raw_temp);
  };

  #define METALENGTH  50
  void generateMeta(char *meta) {
    // TODO add reporting of (uint8_t)raw_temp and drop history
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
      forgotten_count                     // 3  250
    );                                    // + 14 'x' + 1 null => 50 total
  };

  uint8_t _publishFrd(const char* msg, uint8_t retries = RETRY_COUNT) {
    char meta[METALENGTH];
    generateMeta(meta);
    return hiome.publish(msg, meta, retries, SERIAL_DEBUG);
  };

  void publishPacket() {
    if (SIDE1(past_position)) {
      crossed = _publishFrd("1");
      if (!crossed) return;
      // artificially shift starting point ahead 1 row so that
      // if user turns around now, algorithm considers it an exit
      int8_t s = ((int8_t)past_position) - ((int8_t)GRID_EXTENT);
      starting_position = max(s, 0);
    } else {
      crossed = _publishFrd("2");
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
    if (door_state != DOOR_OPEN && frames_since_door_open > 0) return false;
    // door literally just opened this frame, no way anybody crossed
    if (door_state == DOOR_OPEN && frames_since_door_open < 2) return false;

    if (history >= MIN_HISTORY && starting_side() != side()) {
      publishPacket();
      return true;
    } else if (avg_fgm > 100 && avg_bgm > 100 &&
                axis_distance(max_position, starting_position) > 2) {
      if (axis_distance(max_position, past_position) > 2) {
        if (crossed && SIDE(max_position) == starting_side()) return false;
        starting_position = max_position;
      } else if (axis_distance(starting_position, past_position) <= 2) return false;

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
  .width=0,
  .crossed=0,
  .max_jump=0,
  .avg_height=0,
  .avg_width=0,
  .avg_neighbors=0,
  .avg_confidence=0,
  .blobSize=0,
  .noiseSize=0,
  .forgotten_count=0,
  .avg_bgm=0,
  .avg_fgm=0,
  .raw_temp=0,
  .bgm=0,
  .fgm=0
};

void publishEvents() {
  // door has been closed/ajar for more than 1 frame, no way anybody crossed
  if (door_state != DOOR_OPEN && frames_since_door_open > 0) return;
  if (door_state == DOOR_OPEN && frames_since_door_open < 2) return;

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
      if (known_people[i].real()) {
        bool clearPoint;
        if (known_people[i].confidence < 50) {
          // clear all points with low confidence, regardless of transition
          clearPoint = true;
        } else if (door_state == DOOR_CLOSED || previous_door_state == DOOR_OPEN) {
          // door just closed, publish whatever we have and forget all points
          clearPoint = true;
        } else if (previous_door_state == DOOR_AJAR &&
                    known_people[i].starting_side() == ajar_side) {
          // door is ajar and this person started on the side that the door is ajar
          clearPoint = true;
        } else if (door_state == DOOR_AJAR && (known_people[i].side() == ajar_side ||
                      known_people[i].starting_side() == ajar_side)) {
          // door is ajar and this person is somehow on the side that the door is ajar
          clearPoint = true;
        } else if (previous_door_state == DOOR_CLOSED &&
                    doorSide(known_people[i].starting_position)) {
          // door just opened
          clearPoint = true;
        }

        if (clearPoint) {
          known_people[i].publishMaybeEvent();
          known_people[i] = UNDEF_PERSON;
        }
      }

      if (forgotten_people[i].real()) {
        forgotten_people[i].publishMaybeEvent();
        forgotten_people[i] = UNDEF_PERSON;
      }
    }
  }
}

void forget_person(idx_t idx, idx_t (&pairs)[MAX_PEOPLE*2], uint8_t expiration = MAX_EMPTY_CYCLES) {
  idx_t useIdx = UNDEF_INDEX;
  uint8_t min_conf = known_people[idx].confidence;
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
      // clear old slot first
      forgotten_people[useIdx].publishMaybeEvent();
      pairs[useIdx+MAX_PEOPLE] = UNDEF_INDEX;
    }
    // if (known_people[idx].forgotten_count < 250) {
    //   ++known_people[idx].forgotten_count;
    // }
    forgotten_people[useIdx] = known_people[idx];
    forgotten_expirations[useIdx] = expiration;
  }
  known_people[(idx)] = UNDEF_PERSON;
}

void expireForgottenPeople() {
  for (idx_t i = 0; i < MAX_PEOPLE; i++) {
    if (forgotten_people[i].real()) {
      if (forgotten_expirations[i] == 0) {
        forgotten_people[i].publishMaybeEvent();
        forgotten_people[i] = UNDEF_PERSON;
      } else {
        --forgotten_expirations[i];
      }
    }
  }
}

Person getPersonFromIdx(idx_t idx) {
  if (idx < MAX_PEOPLE) {
    if (!known_people[idx].real()) return UNDEF_PERSON;
    return known_people[idx];
  } else {
    if (!forgotten_people[idx-MAX_PEOPLE].real() ||
          forgotten_expirations[idx-MAX_PEOPLE] == MAX_EMPTY_CYCLES) {
      // point is not real or it was *just* forgotten
      // this will skip a point that was meant to be remembered for longer if they
      // happen to have exactly MAX_EMPTY_CYCLES of life left. Oh well.
      return UNDEF_PERSON;
    }
    return forgotten_people[idx-MAX_PEOPLE];
  }
}

bool updateKnownPerson(Person p, idx_t (&pairs)[MAX_PEOPLE*2]) {
  idx_t useIdx = UNDEF_INDEX;
  uint8_t min_conf = p.confidence;
  for (idx_t j = 0; j < MAX_PEOPLE; j++) {
    if (!known_people[j].real()) {
      // slot is empty, use it and stop looking for another
      useIdx = j;
      break;
    } else if (known_people[j].confidence < min_conf) {
      // this slot is lower confidence, consider using it
      min_conf = known_people[j].confidence;
      useIdx = j;
    }
  }
  if (useIdx != UNDEF_INDEX) {
    // we found a slot! save person in there
    if (known_people[useIdx].real()) {
      forget_person(useIdx, pairs);
    }
    pairs[(useIdx)] = UNDEF_INDEX;
    known_people[useIdx] = p;
    return true;
  }
  return false;
}
