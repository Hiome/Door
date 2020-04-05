// find which point this person is most interested in pairing with
Person p = known_people[idx];
if (!p.real()) continue;

idx_t min_index = UNDEF_INDEX;
float min_score = 100;
float maxTperson = p.max_allowed_temp_drift();
float maxDperson = p.max_distance();

// pair this person with a point in current frame
for (idx_t j=0; j<total_masses; j++) {
  PossiblePerson pp = points[j];

  float maxDpoint = pp.max_distance();
  // choose larger range of 2 points as max distance
  maxDpoint = max(maxDpoint, maxDperson);
  maxDpoint = min(maxDpoint, 5.5); // don't let the D grow too big

  float d = euclidean_distance(p.past_position, pp.current_position);
  if (d > maxDpoint) continue;

  // can't shift more than 2-5º at once
  float maxTpoint = pp.max_allowed_temp_drift();
  float tempDiff = p.difference_from_point(pp.current_position);
  if (tempDiff > max(maxTperson, maxTpoint)) continue;

  float score = sq(d/maxDperson) + sq(max(tempDiff, 1)/maxTperson);
  if (!p.crossed || pointOnSmallBorder(p.starting_position)) {
    score -= (0.02*((float)pp.neighbors));
  }

  // distance is high AND temp diff is high
  if (d > 2 && score > 1.8) continue;

  if (score <= (min_score - 0.1) || (score <= (min_score + 0.1) &&
        tempDiff < p.difference_from_point(points[min_index].current_position))) {
    // either score is less than min score, or if it's similar,
    // choose the point with more similar raw temp
    min_score = score;
    min_index = j;
  }
}

if (min_index == UNDEF_INDEX) {
  // still not found...
  FORGET_POINT;
} else {
  taken[min_index]++;
  pairs[idx] = min_index;
}
