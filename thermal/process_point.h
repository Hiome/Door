// more than one person is trying to match with this single point, pick the best one...
idx_t max_idx = UNDEF_INDEX;
float max_score = -100.0;
float score;
float maxT = points[i].max_allowed_temp_drift();
float maxD = points[i].max_distance();
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] != i) continue;

  Person p = getPersonFromIdx(idx);
  if (!p.real()) continue;

  // prefer people with more neighbors
  score = (0.03*((float)p.neighbors));

  // prefer people with more similar temps
  float tempDiff = p.difference_from_point(points[i].current_position);
  score -= sq(tempDiff/maxT);

  // prefer people who didn't take crazy leaps to get here
  float d = euclidean_distance(p.past_position, points[i].current_position);
  score -= sq(d/maxD);

  if (score >= max_score + 0.05) {
    max_score = score;
    max_idx = idx;
  } else if (score > max_score - 0.05) {
    uint8_t matchedHistory;
    if (max_idx < MAX_PEOPLE) {
      matchedHistory = known_people[max_idx].history;
    } else {
      matchedHistory = forgotten_people[max_idx-MAX_PEOPLE].history;
    }
    if (p.history > matchedHistory) {
      // if 2 competing points have the same score, pick the one with more history
      max_score = score;
      max_idx = idx;
    }
  }
}

// once we've chosen our winning point, forget the rest...
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] != i || idx == max_idx) continue;
  // does this look like two blobs combining into one?
  if (idx < MAX_PEOPLE) {
    if (points[i].confidence > 50 && points[i].neighbors >= 4 &&
          known_people[idx].count > 1 && known_people[idx].confidence > 50 &&
          // new point is a merger, so must be bigger than old person
          points[i].blobSize > known_people[idx].blobSize &&
          // old person should be at least 1/4th this point's size to bother remembering
          known_people[idx].blobSize*4 > points[i].blobSize &&
          known_people[idx].forgotten_count < MAX_FORGOTTEN_COUNT) {
      store_forgotten_person(idx, (5*MAX_EMPTY_CYCLES));
      known_people[idx] = UNDEF_PERSON;
    } else {
      forget_person(idx, pairs);
    }
  }
  pairs[idx] = UNDEF_INDEX;
  taken[i]--;
  if (taken[i] == 1) break;
}
