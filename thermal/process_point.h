// more than one person is trying to match with this single point, pick the best one...
idx_t max_idx = UNDEF_INDEX;
float max_score = -100.0;
float score;
float maxT = points[i].max_allowed_temp_drift();
float maxD = points[i].max_distance();
for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
  Person p = known_people[idx];
  if (p.real() && pairs[idx] == i) {
    // prefer people with more neighbors
    score = (0.03*((float)p.neighbors));

    // prefer people with more similar temps
    float tempDiff = p.difference_from_point(points[i].current_position);
    tempDiff = max(tempDiff, 1);
    score -= sq(tempDiff/maxT);

    // prefer people who didn't take crazy leaps to get here
    float d = euclidean_distance(p.past_position, points[i].current_position);
    score -= sq(d/maxD);

    if (score >= max_score + 0.1) {
      max_score = score;
      max_idx = idx;
    } else if (score >= max_score - 0.1 &&
        (p.total_distance() + p.history) >
          (known_people[max_idx].total_distance() + known_people[max_idx].history)) {
      // if 2 competing points have the same score, pick the one with more history
      max_score = score;
      max_idx = idx;
    }
  }
}

// once we've chosen our winning point, forget the rest...
bool holyMatrimony = points[i].confidence > 50 && points[i].neighbors >= 4 &&
                      !pointOnSmallBorder(points[i].current_position);
for (idx_t idx=0; idx < MAX_PEOPLE; idx++) {
  if (known_people[idx].real() && pairs[idx] == i && idx != max_idx) {
    // does this look like two blobs combining into one?
    if (holyMatrimony && known_people[idx].neighbors &&
          known_people[idx].count > 1 && known_people[idx].confidence > 50) {
      if (SIDE1(known_people[idx].starting_position)) {
        side1Point = max(known_people[idx].confidence, side1Point);
        SERIAL_PRINTLN(F("s1Pt"));
      } else {
        side2Point = max(known_people[idx].confidence, side2Point);
        SERIAL_PRINTLN(F("s2Pt"));
      }
    }
    FORGET_POINT;
    taken[i]--;
    if (taken[i] == 1) break;
  }
}
