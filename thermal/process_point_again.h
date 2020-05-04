// give forgotten people a second chance at adopting this point
idx_t min_index = UNDEF_INDEX;
float min_score = 100;
float maxTpoint = points[i].max_allowed_temp_drift();
float maxDpoint = points[i].max_distance();
for (idx_t fidx=0; fidx < MAX_PEOPLE; fidx++) {
  if (!forgotten_people[fidx].real() || pairs[fidx+MAX_PEOPLE] != UNDEF_INDEX) continue;

  if (forgotten_people[fidx].neighbors > points[i].neighbors) {
    if (points[i].blobSize*2 < forgotten_people[fidx].neighbors) continue;
  } else {
    if (forgotten_people[fidx].blobSize*2 < points[i].neighbors) continue;
  }

  // can't jump too far
  float d = euclidean_distance(forgotten_people[fidx].past_position, points[i].current_position);
  float maxDperson = forgotten_people[fidx].max_distance();
  maxDperson = min(maxDperson, maxDpoint);
  if (d > min(maxDperson, 6)) continue;

  // can't shift temperature too much
  float tempDiff = forgotten_people[fidx].difference_from_point(points[i].current_position);
  float maxTperson = forgotten_people[fidx].max_allowed_temp_drift();
  if (tempDiff > min(maxTpoint, maxTperson) + 2) continue;

  float score = (d/maxDpoint) + (tempDiff/maxTpoint);
  score -= (0.0001*forgotten_people[fidx].confidence);
  score -= (0.01*forgotten_people[fidx].neighbors);

  if (score <= (min_score - 0.05) || (score < (min_score + 0.05) && tempDiff <
        forgotten_people[min_index].difference_from_point(points[i].current_position))) {
    // either score is less than min score, or if it's similar,
    // choose the point with more similar raw temp
    min_score = score;
    min_index = fidx;
  }
}

if (min_index != UNDEF_INDEX) {
  pairs[min_index+MAX_PEOPLE] = i;
  taken[i] = 1;
}
