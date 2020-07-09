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

const uint8_t FACING_SIDE1 = 0;
const uint8_t FACING_SIDE2 = 1;

typedef struct {
  coord_t   past_position;        //:7 0-63 + UNDEF_POINT
  coord_t   min_position;         //:6 0-63
  coord_t   max_position;         //:6 0-63
  uint8_t   confidence;           //:7 0-100
  uint16_t  count;

  uint8_t   d1_count;
  uint8_t   d2_count;

  uint8_t   height            :3; // 0-7
  uint8_t   direction         :1; // 0-1
  uint8_t   width             :4; // 0-7 (only need 3 bits)

  uint8_t   neighbors         :4; // 0-8
  uint8_t   avg_neighbors     :4; // 0-8

  uint8_t   avg_height        :3;
  uint8_t   published         :2;
  uint8_t   avg_width         :3;

  uint8_t   avg_confidence;
  uint8_t   blobSize;
  uint8_t   noiseSize;

  uint16_t  avg_bgm;
  uint16_t  avg_fgm;

  float     raw_temp;
  float     bgm;
  float     fgm;

  bool      real() { return past_position != UNDEF_POINT; };
  coord_t   starting_position() { return direction == FACING_SIDE1 ? max_position : min_position; };
  uint8_t   starting_side() { return SIDE(starting_position()); };
  uint8_t   side() { return SIDE(past_position); };
  float     total_distance() {
    return euclidean_distance(starting_position(), past_position);
  };
  uint8_t   history() {
    if (direction == FACING_SIDE1) return d1_count > d2_count ? d1_count - d2_count : 0;
    return d2_count > d1_count ? d2_count - d1_count : 0;
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

  void _publishFrd(uint8_t retries = HIOME_RETRY_COUNT) {
    char msg[HIOME_MAX_MESSAGE_LENGTH];
    sprintf(msg, "%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%ux%u",
      avg_confidence,                     // 3  100
      avg_bgm,                            // 4  1020
      avg_fgm,                            // 4  1020
      starting_position(),                // 2  64
      past_position,                      // 2  64
      d1_count,                           // 3  255
      d2_count,                           // 3  255
      count,                              // 5  60000
      blobSize,                           // 2  60
      noiseSize,                          // 2  60
      avg_neighbors,                      // 1  8
      avg_height,                         // 1  7
      avg_width,                          // 1  7
      uint8_t(raw_temp)                   // 2  35
    );                                    // + 13 'x' + 1 null => 49 total
    hiome.publish(msg, retries, SERIAL_DEBUG);
  };

  void publishPacket() {
    _publishFrd();
    count = 1;
    d1_count = 0;
    d2_count = 0;
    published = direction == FACING_SIDE1 ? 1 : 2;
    min_position = past_position;
    max_position = past_position;
  };

  bool publishable() {
    if (published == side() && side() == starting_side()) return false;
    if (history() <= 1) return false;
    if (published && (published != side() || axis_distance(starting_position(), past_position) > 2)) return true;
    return d1_count + d2_count > 2 && published == 0 && axis_distance(starting_position(), past_position) > 2;
  };

  // called when a point is about to be forgotten to diagnose if min history is an issue
  void publishMaybeEvent() {
    // door has been closed/ajar for more than 1 frame, no way anybody crossed
    if (door_state != DOOR_OPEN && frames_since_door_open > 0) return;
    // door literally just opened this frame, no way anybody crossed
    if (door_state == DOOR_OPEN && frames_since_door_open < 2) return;

    if (publishable()) publishPacket();
  };
} Person;

Person known_people[MAX_PEOPLE];
Person forgotten_people[MAX_PEOPLE];
uint8_t forgotten_expirations[MAX_PEOPLE] = { 0 };
uint8_t forgotten_starting_expiration[MAX_PEOPLE] = { 0 };

const Person UNDEF_PERSON = {
  .past_position=UNDEF_POINT,
  // (╯°□°）╯︵ ┻━┻  avr-gcc doesn't implement non-trivial designated initializers...
  .min_position=0,
  .max_position=0,
  .confidence=0,
  .count=0,
  .d1_count=0,
  .d2_count=0,
  .height=0,
  .direction=0,
  .width=0,
  .neighbors=0,
  .avg_neighbors=0,
  .avg_height=0,
  .published=0,
  .avg_width=0,
  .avg_confidence=0,
  .blobSize=0,
  .noiseSize=0,
  .avg_bgm=0,
  .avg_fgm=0,
  .raw_temp=0,
  .bgm=0,
  .fgm=0
};

idx_t maybe_idx = UNDEF_INDEX;
Person maybe_person;

void publish_maybe_person(idx_t i) {
  return;
  if (maybe_idx == i) {
    // door has been closed/ajar for more than 1 frame, no way anybody crossed
    if (door_state != DOOR_OPEN && frames_since_door_open > 0) return;
    // door literally just opened this frame, no way anybody crossed
    if (door_state == DOOR_OPEN && frames_since_door_open < 2) return;

    maybe_person._publishFrd();
    maybe_idx = UNDEF_INDEX;
    if (known_people[i].count > maybe_person.count)
      known_people[i].count -= (maybe_person.count - 1);
    else
      known_people[i].count = 1;
    if (known_people[i].d1_count > maybe_person.d1_count)
      known_people[i].d1_count -= maybe_person.d1_count;
    else
      known_people[i].d1_count = 0;
    if (known_people[i].d2_count > maybe_person.d2_count)
      known_people[i].d2_count -= maybe_person.d2_count;
    else
      known_people[i].d2_count = 0;
    if (maybe_person.direction == FACING_SIDE1) {
      known_people[i].published = 1;
      known_people[i].max_position = known_people[i].past_position;
    } else {
      known_people[i].published = 2;
      known_people[i].min_position = known_people[i].past_position;
    }
  }
}

void publishEvents() {
  // door has been closed/ajar for more than 1 frame, no way anybody crossed
  if (door_state != DOOR_OPEN && frames_since_door_open > 0) return;
  if (door_state == DOOR_OPEN && frames_since_door_open < 2) return;

  for (idx_t i=0; i<MAX_PEOPLE; i++) {
    if (!known_people[i].real()) continue;
    // if (maybe_idx == i) {
    //   if (known_people[i].direction == maybe_person.direction) {
    //     // clear fork if person corrected their direction before straying too far
    //     if (maybe_person.direction == FACING_SIDE1) {
    //       if (AXIS(known_people[i].past_position) <= AXIS(maybe_person.past_position)) {
    //         maybe_idx = UNDEF_INDEX;
    //       }
    //     } else if (AXIS(known_people[i].past_position) >= AXIS(maybe_person.past_position)) {
    //       maybe_idx = UNDEF_INDEX;
    //     }
    //   } else if (axis_distance(known_people[i].past_position, maybe_person.past_position) >= 2) {
    //     // person strayed far enough, bombs away
    //     publish_maybe_person(i);
    //   }
    // }
    if (pointOnBorder(known_people[i].past_position) && known_people[i].publishable()) {
      Person p = known_people[i];
      p.publishPacket();
      known_people[i] = p; // update known_people array
    } else if (pointOnTBEdge(known_people[i].past_position)) {
      // reset person when they reach top or bottom edge
      known_people[i].count = 1;
      known_people[i].d1_count = 0;
      known_people[i].d2_count = 0;
      known_people[i].published = 0;
      known_people[i].direction = known_people[i].side() == 1 ? FACING_SIDE2 : FACING_SIDE1;
      known_people[i].min_position = known_people[i].past_position;
      known_people[i].max_position = known_people[i].past_position;
      if (maybe_idx == i) maybe_idx = UNDEF_INDEX;
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
          // door was ajar and this person started on the side that the door is ajar
          clearPoint = true;
        } else if (door_state == DOOR_AJAR && (known_people[i].side() == ajar_side ||
                      known_people[i].starting_side() == ajar_side)) {
          // door is ajar and this person is somehow on the side that the door is ajar
          clearPoint = true;
        } else if (previous_door_state == DOOR_CLOSED &&
                    doorSide(known_people[i].starting_position())) {
          // door just opened
          clearPoint = true;
        }

        if (clearPoint) {
          publish_maybe_person(i);
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
  publish_maybe_person(idx);
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
    forgotten_people[useIdx] = known_people[idx];
    forgotten_expirations[useIdx] = expiration;
    forgotten_starting_expiration[useIdx] = expiration;
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
      forget_person(useIdx, pairs, 1);
    }
    pairs[(useIdx)] = UNDEF_INDEX;
    known_people[useIdx] = p;
    return true;
  }
  return false;
}

// limit unholy human/noise pairing by preventing neighbor count from doubling in one jump
bool safeToMerge(uint8_t personPosition, uint8_t personSize, uint8_t personNeighbors,
                 uint8_t pointPosition, uint8_t pointSize, uint8_t pointNeighbors) {
  if (personSize > pointSize) {
    // person would be shrinking
    if (pointOnEdge(personPosition)) {
      if (pointSize*2 < personSize && !pointOnEdge(pointPosition)) {
        // person is on edge of grid and is double the size of new point, which is in middle of grid.
        // this is likely the end of a person
        return false;
      }
    } else if (pointSize*4 < personNeighbors && !pointOnEdge(pointPosition)) {
      // both person and new point are in middle of grid, but person is 4x larger than new point
      // this is the same as points[j].blobSize == 1 && p.neighbors > 4
      return false;
    }
  } else {
    // person would be growing
    if (pointOnEdge(pointPosition)) {
      if (personSize*2 < pointSize && !pointOnEdge(personPosition)) {
        // new point is on edge of grid and is double the size of person, who is in middle of grid.
        // this is likely the start of a new person
        return false;
      }
    } else if (personSize*4 < pointNeighbors && !pointOnEdge(personPosition)) {
      // both person and new point are in middle of grid, but new point is 4x larger than person
      // this is the same as p.blobSize == 1 && points[j].neighbors > 4
      return false;
    }
  }
  return true;
}
