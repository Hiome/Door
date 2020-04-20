// more than one person is trying to match with this single point, pick the best one...
idx_t min_idx = UNDEF_INDEX;
float min_score = 100;
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

  float score = (tempDiff/maxT);
  score = min(score, 1.1);
  score += (d/maxD);
  score -= (0.0001*p.confidence);
  score -= (0.01*p.neighbors);
  score += (float(p.noiseSize)/float(p.blobSize));

  if (score > 1.6) continue;

  if (score <= min_score - 0.05) {
    min_score = score;
    min_idx = idx;
  } else if (score < min_score + 0.05) {
    uint8_t matchedHistory;
    if (min_idx < MAX_PEOPLE) {
      matchedHistory = known_people[min_idx].history;
    } else {
      matchedHistory = forgotten_people[min_idx-MAX_PEOPLE].history;
    }
    if (p.history > matchedHistory) {
      // if 2 competing points have the same score, pick the one with more history
      min_score = score;
      min_idx = idx;
    }
  }
}

// once we've chosen our winning point, forget the rest...
for (idx_t idx=0; idx < MAX_PEOPLE*2; idx++) {
  if (pairs[idx] != i || idx == min_idx) continue;
  // does this look like two blobs combining into one?
  if (idx < MAX_PEOPLE && known_people[idx].real()) {
    if (points[i].confidence > 50 && points[i].neighbors >= 4 &&
          known_people[idx].count > 1 && known_people[idx].confidence > 50 &&
          // new point is a merger, so must be bigger than old person
          points[i].blobSize > known_people[idx].blobSize &&
          // old person should be at least 1/4th this point's size to bother remembering
          known_people[idx].blobSize*4 > points[i].blobSize) {
      forget_person(idx, pairs, (5*MAX_EMPTY_CYCLES));
    } else {
      forget_person(idx, pairs);
    }
  }
  pairs[idx] = UNDEF_INDEX;
}

// we found a match
taken[i] = min_idx == UNDEF_INDEX ? 0 : 1;
