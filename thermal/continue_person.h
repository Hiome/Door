// a person and point have been paired together! Update the person's stats accordingly
bool added = false;
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] == i) {
    Person p = idx < MAX_PEOPLE ? known_people[idx] : forgotten_people[idx-MAX_PEOPLE];
    if (!p.real()) break;

    PossiblePerson pp = points[i];

    // Filter whether we *really* want to pair these points...
    // these filters could go in process_person to prevent the pairing in the first place, but that
    // would artificially make this point pair with another even if this is indeed the best match

    if ((p.starting_side() != pp.side() || axis_distance(p.max_position, pp.current_position) > 2) &&
        // point has crossed and it is on the top or bottom edge since last frame
        ((pointOnTBEdge(p.past_position) && pointOnTBEdge(pp.current_position)) ||
          // or it has been in the same position on the left or right edge since last frame
         (p.past_position == pp.current_position && pointOnLREdge(pp.current_position)))) {
      // assume we're done and break, point will be forgotten and published if applicable
      break;
    }

    if (p.blobSize > pp.blobSize) {
      // person would be shrinking
      if (pp.blobSize*2 < p.blobSize && pointOnEdge(p.past_position)) {
        // person is on edge of grid and is double the size of new point, this is likely the end of a person
        break;
      }
    } else if (p.blobSize*2 < pp.blobSize && pointOnEdge(pp.current_position)) {
      // person would be growing, but new point is on edge of grid.
      // If new point is double the size of person, this is likely the start of a new person
      break;
    }

    fint1_t td = floatToFint1(p.difference_from_point(pp.current_position));
    if (td > 250) td = 250;
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
        if ((SIDE1(p.starting_position) &&
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
