// a person and point have been paired together! Update the person's stats accordingly
bool added = false;
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] == i) {
    Person p = getPersonFromIdx(idx);
    if (!p.real()) break;

    PossiblePerson pp = points[i];

    fint1_t td = floatToFint1(p.difference_from_point(pp.current_position));
    p.max_temp_drift = max(p.max_temp_drift, td);

    if (p.past_position != pp.current_position) {
      if (AXIS(pp.current_position) == AXIS(p.starting_position) ||
          (SIDE1(p.starting_position) &&
            AXIS(pp.current_position) < AXIS(p.past_position)) ||
          (SIDE2(p.starting_position) &&
            AXIS(pp.current_position) > AXIS(p.past_position))) {
        // point moved backwards
        if ((SIDE1(p.starting_position) &&
              AXIS(pp.current_position) <= AXIS(p.starting_position)) ||
            (SIDE2(p.starting_position) &&
              AXIS(pp.current_position) >= AXIS(p.starting_position))) {
          // reset history if point is further back than where it started
          p.starting_position = pp.current_position;
          p.history = 1;
          p.retreating = false;
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
      } else if (AXIS(p.past_position) != AXIS(pp.current_position)) {
        // "always forward, forward always" - Luke Cage
        if (!p.retreating || (SIDE1(p.starting_position) &&
              AXIS(pp.current_position) >= AXIS(p.max_position)) ||
            (SIDE2(p.starting_position) &&
              AXIS(pp.current_position) <= AXIS(p.max_position))) {
          // we beat our previous max position record
          p.max_position = pp.current_position;
        }
        if (!p.retreating || axis_distance(p.past_position, pp.current_position) > 1) {
          ++p.history;
        }
        p.retreating = false;
        if (pp.side() != p.side() || p.history > 5 || idx >= MAX_PEOPLE) {
          // point just crossed threshold, let's reduce its history to force
          // it to spend another cycle on this side before we count the event
          p.history = min(p.history, MIN_HISTORY);
        }
      }

      // update current state where position changed
      uint8_t axisJump = max_axis_jump(p.past_position, pp.current_position);
      p.max_jump = max(axisJump, p.max_jump);
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
    #define UPDATE_RUNNING_AVG(o,n) ( o = ((n) + ((o)*2))/3 )
    UPDATE_RUNNING_AVG(p.avg_bgm, floatToFint2(p.bgm));
    UPDATE_RUNNING_AVG(p.avg_fgm, floatToFint2(p.fgm));
    UPDATE_RUNNING_AVG(p.avg_height, p.height);
    UPDATE_RUNNING_AVG(p.avg_width, p.width);
    UPDATE_RUNNING_AVG(p.avg_neighbors, p.neighbors);
    UPDATE_RUNNING_AVG(p.blobSize, (uint8_t)pp.blobSize);
    UPDATE_RUNNING_AVG(p.noiseSize, pp.noiseSize);
    p.avg_confidence = (((uint16_t)p.confidence + (((uint16_t)p.avg_confidence)*2))/3);
    if (p.count < 60000) ++p.count;

    if (idx < MAX_PEOPLE) {
      // already a known person, update in place
      known_people[idx] = p;
      added = true;
    } else {
      // this is a forgotten person, free up its forgotten slot
      forgotten_people[idx-MAX_PEOPLE] = UNDEF_PERSON;
      pairs[idx-MAX_PEOPLE] = UNDEF_INDEX;
      // and then insert it into known_people instead
      if (updateKnownPerson(p, pairs)) added = true;
    }
    break;
  }
}

if (!added) {
  taken[i] = 0;
}
