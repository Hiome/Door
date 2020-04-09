// more than one person is trying to match with this single point, pick the best one...
idx_t max_idx = UNDEF_INDEX;
float max_score = -100.0;
float score;
float maxT = points[i].max_allowed_temp_drift();
float maxD = points[i].max_distance();
for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
  if (known_people[idx].real() && pairs[idx] == i) {
    Person p = known_people[idx];
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
    } else if (score > max_score - 0.05 && p.history > known_people[max_idx].history) {
      // if 2 competing points have the same score, pick the one with more history
      max_score = score;
      max_idx = idx;
    }
  }
}

// once we've chosen our winning point, forget the rest...
for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
  if (known_people[idx].real() && pairs[idx] == i && idx != max_idx) {
    // does this look like two blobs combining into one?
    if (points[i].confidence > 50 && points[i].neighbors >= 4 &&
          known_people[idx].count > 1 && known_people[idx].confidence > 50 &&
          // new point is a merger, so must be bigger than old person
          points[i].blobSize > known_people[idx].blobSize &&
          // old person should be at least 1/4th this point's size to bother remembering
          known_people[idx].blobSize*4 > points[i].blobSize &&
          known_people[idx].forgotten_count < MAX_FORGOTTEN_COUNT) {
      store_forgotten_person(known_people[idx], (5*MAX_EMPTY_CYCLES));
      pairs[idx] = UNDEF_INDEX;
      known_people[idx] = UNDEF_PERSON;
    } else {
      FORGET_POINT;
    }
    taken[i]--;
    if (taken[i] == 1) break;
  }
}
