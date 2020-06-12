// a person and point have been paired together! Update the person's stats accordingly
bool added = false;
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] == i) {
    Person p = idx < MAX_PEOPLE ? known_people[idx] : forgotten_people[idx-MAX_PEOPLE];
    if (!p.real()) break;

    PossiblePerson pp = points[i];

    if (p.past_position != pp.current_position) {
      axis_t new_axis = AXIS(pp.current_position);
      axis_t old_axis = AXIS(p.past_position);
      if (new_axis != old_axis) {
        // check if direction changed
        if (p.direction == FACING_SIDE1) {
          if (new_axis > old_axis) {
            // changed direction from 1 to 2
            if (idx < MAX_PEOPLE && (maybe_idx == UNDEF_INDEX || maybe_person.confidence < p.confidence) && p.publishable()) {
              maybe_person = p;
              maybe_idx = idx;
            }
            p.direction = FACING_SIDE2;
            if (p.d2_count < 250) ++p.d2_count;
          } else {
            // still moving in direction 1
            if (p.d1_count < 250) ++p.d1_count;
            if (p.d2_count > 0) --p.d2_count;
          }
        } else {
          if (new_axis < old_axis) {
            // changed direction from 2 to 1
            if (idx < MAX_PEOPLE && (maybe_idx == UNDEF_INDEX || maybe_person.confidence < p.confidence) && p.publishable()) {
              maybe_person = p;
              maybe_idx = idx;
            }
            p.direction = FACING_SIDE1;
            if (p.d1_count < 250) ++p.d1_count;
          } else {
            // still moving in direction 2
            if (p.d2_count < 250) ++p.d2_count;
            if (p.d1_count > 0) --p.d1_count;
          }
        }
      }

      // check if max or min edge was pushed
      if (new_axis <= AXIS(p.min_position)) {
        p.min_position = pp.current_position;
      }
      if (new_axis >= AXIS(p.max_position)) {
        p.max_position = pp.current_position;
      }

      // update current state where position changed
      p.past_position = pp.current_position;
    }

    // update current state
    p.confidence = pp.confidence;
    p.raw_temp = pp.raw_temp();
    p.bgm = pp.bgm();
    p.fgm = pp.fgm();
    p.neighbors = pp.neighbors;
    p.height = pp.height;
    p.width = pp.width;
    p.blobSize = (uint8_t)pp.blobSize;
    p.noiseSize = pp.noiseSize;
    #define UPDATE_RUNNING_AVG(o,n) ( o = ((n) + ((o)*2))/3 )
    UPDATE_RUNNING_AVG(p.avg_bgm, floatToFint2(p.bgm));
    UPDATE_RUNNING_AVG(p.avg_fgm, floatToFint2(p.fgm));
    UPDATE_RUNNING_AVG(p.avg_height, p.height);
    UPDATE_RUNNING_AVG(p.avg_width, p.width);
    UPDATE_RUNNING_AVG(p.avg_neighbors, p.neighbors);
    p.avg_confidence = (((uint16_t)p.confidence + (((uint16_t)p.avg_confidence)*2))/3);
    if (p.count < 60000) ++p.count;

    if (idx < MAX_PEOPLE) {
      // already a known person, update in place
      known_people[idx] = p;
      added = true;
    } else {
      // this is a forgotten person, free up its forgotten slot
      forgotten_people[idx-MAX_PEOPLE] = UNDEF_PERSON;
      pairs[idx] = UNDEF_INDEX;
      // and then insert it into known_people instead
      if (updateKnownPerson(p, pairs)) added = true;
    }
    break;
  }
}

if (!added) {
  taken[i] = 0;
}
