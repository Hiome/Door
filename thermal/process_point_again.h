// give forgotten people a second chance at adopting this point
idx_t max_index = UNDEF_INDEX;
float max_score = -100;
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

  float dScore = 1 - ((d+0.1)/(maxDpoint+0.2));
  float tScore = 1 - ((tempDiff+0.1)/(maxTpoint+0.2));
  if (dScore < 0.2 && tScore < 0.2) continue;

  float score = dScore * tScore;
  score *= (0.9 + (0.001*forgotten_people[fidx].confidence));
  score *= (0.92 + (0.01*forgotten_people[fidx].neighbors));
  score *= (1 - (0.05*(forgotten_starting_expiration[fidx] - forgotten_expirations[fidx])));

  if (forgotten_people[fidx].direction == FACING_SIDE1) {
    if (forgotten_people[fidx].d1_count > forgotten_people[fidx].d2_count &&
        AXIS(points[i].current_position) > AXIS(forgotten_people[fidx].past_position)) {
      score *= 0.85;
    }
  } else if (forgotten_people[fidx].d2_count > forgotten_people[fidx].d1_count &&
        AXIS(points[i].current_position) < AXIS(forgotten_people[fidx].past_position)) {
    score *= 0.85;
  }

  if (score >= (max_score + 0.005) || (score > (max_score - 0.005) &&
        tempDiff < forgotten_people[max_index].difference_from_point(points[i].current_position))) {
    // either score is greater than max score, or if it's similar,
    // choose the point with more similar raw temp
    max_score = score;
    max_index = fidx;
  }
}

if (max_index != UNDEF_INDEX) {
  pairs[max_index+MAX_PEOPLE] = i;
  taken[i] = 1;
}
