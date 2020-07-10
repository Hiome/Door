// more than one person is trying to match with this single point, pick the best one...
idx_t max_idx = UNDEF_INDEX;
float max_score = 0;
float maxT = points[i].max_allowed_temp_drift();
float maxD = points[i].max_distance();
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] != i) continue;

  Person p;
  if (idx < MAX_PEOPLE) {
    if (!known_people[idx].real()) continue;
    p = known_people[idx];
  } else {
    if (!forgotten_people[idx-MAX_PEOPLE].real()) continue;
    p = forgotten_people[idx-MAX_PEOPLE];
  }

  // prefer people who didn't take crazy leaps to get here
  float d = euclidean_distance(p.past_position, points[i].current_position);
  // prefer people with more similar temps
  float tempDiff = p.difference_from_point(points[i].current_position);

  float dScore = 1 - ((d+0.1)/(maxD+0.2));
  float tScore = 1 - ((tempDiff+0.1)/(maxT+2.2));
  float score = dScore * tScore;
  score += (0.001*p.confidence);
  score += (0.01*p.neighbors);
  score += (0.05*p.history());

  // a forgotten point needs to overcome a larger hurdle to be revived
  if (idx >= MAX_PEOPLE) {
    score -= (0.05*(forgotten_starting_expiration[idx-MAX_PEOPLE] - forgotten_expirations[idx-MAX_PEOPLE]));
  }

  if (score > max_score) {
    max_score = score;
    max_idx = idx;
  }
}

// once we've chosen our winning point, forget the rest...
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] != i || idx == max_idx) continue;
  // does this look like two blobs combining into one?
  if (idx < MAX_PEOPLE && known_people[idx].real()) {
    // if new point is a merger, it must be bigger than old person
    if (points[i].neighbors >= 4 && points[i].blobSize > known_people[idx].blobSize) {
      forget_person(idx, pairs, 5);
    } else {
      forget_person(idx, pairs);
    }
  }
  pairs[idx] = UNDEF_INDEX;
}

// we found a match
taken[i] = max_idx == UNDEF_INDEX ? 0 : 1;
