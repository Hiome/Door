// find which point this person is most interested in pairing with
Person p = getPersonFromIdx(idx);
if (!p.real()) continue;

idx_t min_index = UNDEF_INDEX;
float min_score = 100;
float maxTperson = p.max_allowed_temp_drift();
float maxDperson = p.max_distance();

// pair this person with a point in current frame
for (idx_t j=0; j<total_masses; j++) {
  // can't jump too far
  float d = euclidean_distance(p.past_position, points[j].current_position);
  float maxDpoint = points[j].max_distance();
  if (d > min(maxDperson, maxDpoint)) continue;

  // can't shift temperature too much
  float tempDiff = p.difference_from_point(points[j].current_position);
  float maxTpoint = points[j].max_allowed_temp_drift();
  if (tempDiff > min(maxTperson, maxTpoint)) continue;

  float score = sq(d/maxDperson) + sq(tempDiff/maxTperson);
  score -= (0.01*((float)(points[j].neighbors)));

  if (score <= (min_score - 0.05) || (score < (min_score + 0.05) &&
        tempDiff < p.difference_from_point(points[min_index].current_position))) {
    // either score is less than min score, or if it's similar,
    // choose the point with more similar raw temp
    min_score = score;
    min_index = j;
  }
}

if (min_index == UNDEF_INDEX) {
  // still not found...
  if (idx < MAX_PEOPLE) {
    forget_person(idx);
  }
} else {
  ++taken[min_index];
  pairs[idx] = min_index;
}
