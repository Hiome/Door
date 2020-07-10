// find which point this person is most interested in pairing with
Person p;
if (idx < MAX_PEOPLE) {
  if (!known_people[idx].real()) continue;
  p = known_people[idx];
} else {
  if (!forgotten_people[idx-MAX_PEOPLE].real()) continue;
  // don't process point that was forgotten earlier in this loop again
  if (forgotten_expirations[idx-MAX_PEOPLE] == MAX_EMPTY_CYCLES &&
      forgotten_starting_expiration[idx-MAX_PEOPLE] == MAX_EMPTY_CYCLES) {
    continue;
  }
  p = forgotten_people[idx-MAX_PEOPLE];
}

idx_t max_index = UNDEF_INDEX;
float max_score = 0;
float maxTperson = p.max_allowed_temp_drift();
float maxDperson = p.max_distance();

// pair this person with a point in current frame
for (idx_t j=0; j<total_masses; j++) {
  if (!safeToMerge(p.past_position, p.blobSize, p.neighbors,
                   points[j].current_position, points[j].blobSize, points[j].neighbors)) {
    continue;
  }

  // can't jump too far
  float d = euclidean_distance(p.past_position, points[j].current_position);
  float maxDpoint = points[j].max_distance();
  maxDpoint = min(maxDperson, maxDpoint);
  if (d > min(maxDpoint, 6)) continue;

  // can't shift temperature too much
  float tempDiff = p.difference_from_point(points[j].current_position);
  float maxTpoint = points[j].max_allowed_temp_drift();
  if (tempDiff > min(maxTpoint, maxTperson) + 2) continue;

  float dScore = 1 - ((d+0.1)/(maxDperson+0.2));
  float tScore = 1 - ((tempDiff+0.1)/(maxTperson+2.2));
  if (dScore < 0.5 && tScore < 0.5) continue;
  float score = dScore * tScore;
  score += (0.001*points[j].confidence);
  score += (0.01*points[j].neighbors);

  // add inertia - a person that is moving forward is more likely to keep moving forward than change direction
  if (p.direction == FACING_SIDE1) {
    if (p.d1_count > p.d2_count && AXIS(points[j].current_position) > AXIS(p.past_position)) {
      score *= 0.95;
    }
  } else if (p.d2_count > p.d1_count && AXIS(points[j].current_position) < AXIS(p.past_position)) {
    score *= 0.95;
  }

  if (score > max_score) {
    max_score = score;
    max_index = j;
  }
}

if (max_index == UNDEF_INDEX) {
  // still not found...
  if (idx < MAX_PEOPLE) {
    forget_person(idx, pairs);
  }
} else {
  ++taken[max_index];
  pairs[idx] = max_index;
}
