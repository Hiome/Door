float   raw_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
fint3_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float global_bgm = 0;
float global_fgm = 0;
float cavg1 = 0;
float cavg2 = 0;

float bgPixel(coord_t x) {
  return fint3ToFloat(avg_pixels[(x)]);
}

float constrainedPixel(coord_t x) {
  return constrain(raw_pixels[(x)], MIN_TEMP, MAX_TEMP);
}

float diffFromPoint(coord_t a, coord_t b) {
  return abs(raw_pixels[(a)] - raw_pixels[(b)]);
}

// calculate difference from foreground
float fgDiff(coord_t i) {
  float t = constrainedPixel(i);
  float fgmt1 = abs(t - cavg1);
  float fgmt2 = abs(t - cavg2);
  return min(fgmt1, fgmt2);
}

// calculate difference from background
float bgDiff(coord_t i) {
  float std = constrainedPixel(i) - bgPixel(i);
  return abs(std);
}

uint8_t calcGradient(float diff, float scale) {
  if (diff < 0.5) return 0;
  if ((uint8_t)diff > 10) return 100;
  diff /= scale;
  if (diff > 0.995) return 100;
  return (uint8_t)(diff*100.0);
}

// calculate foreground gradient percent
uint8_t calcFgm(coord_t i) {
  return calcGradient(fgDiff(i), global_fgm);
}

// calculate background gradient percent
uint8_t calcBgm(coord_t i) {
  // skip calculating bgm if door is closed
  if (door_state == DOOR_CLOSED && doorSide(i)) {
    return 0;
  }

  return calcGradient(bgDiff(i), global_bgm);
}

// calculate foreground gradient scale
void calculateFgm() {
  global_fgm = FOREGROUND_GRADIENT;
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float fgmt = fgDiff(i);
    if (fgmt < global_fgm) continue;
    if (fgmt > 10) fgmt = 10;
    if (global_bgm > 1.0) fgmt *= (calcBgm(i)/100.0);
    global_fgm = max(fgmt, global_fgm);
  }
}

// calculate background gradient scale
void calculateBgm() {
  global_bgm = BACKGROUND_GRADIENT;
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    float bgmt = bgDiff(i);
    if (bgmt < global_bgm) continue;
    if (bgmt > 10) bgmt = 10;
    bgmt *= (calcFgm(i)/100.0);
    global_bgm = max(bgmt, global_bgm);
  }
}

float trimMean(uint8_t side) {
  coord_t sortedPixels[(AMG88xx_PIXEL_ARRAY_SIZE/2)];
  uint8_t baseLine = side == 1 ? 0 : (AMG88xx_PIXEL_ARRAY_SIZE/2);
  for (uint8_t i = 0; i < (AMG88xx_PIXEL_ARRAY_SIZE/2); i++) {
    // sort clusters by raw temp
    coord_t n = i + baseLine;
    bool added = false;
    for (uint8_t j=0; j<i; j++) {
      if (constrainedPixel(n) > constrainedPixel(sortedPixels[j])) {
        for (int8_t x=i; x>j; x--) {
          sortedPixels[x] = sortedPixels[(x-1)];
        }
        sortedPixels[j] = n;
        added = true;
        break;
      }
    }
    if (!added) {
      // append n to end of array
      sortedPixels[i] = n;
    }
  }

  float avg = 0;
  uint8_t total = 0;
  for (uint8_t i=6; i < (AMG88xx_PIXEL_ARRAY_SIZE/2); i++) {
    // drop the 6^ warmest points when calculating mean to skew it lower
    // this might cause issues if the person is actually cooler than the background,
    // such as when keith is sweaty. We shall see!
    avg += constrainedPixel(sortedPixels[i]);
    total++;
    if ((frames_since_door_open >= MAX_DOOR_CHANGE_FRAMES
        #ifndef RECESSED
          || (side == ajar_side && (door_state == DOOR_AJAR || previous_door_state == DOOR_AJAR))
        #endif
        ) && (uint8_t)bgDiff(sortedPixels[i]) == 0) {
      // double weight of points with low background diff
      avg += constrainedPixel(sortedPixels[i]);
      total++;
    }
  }

  SERIAL_PRINTLN(total); // should be between 26 - 52

  if (!total) {
    SERIAL_PRINTLN(F("xxx"));
    return 0; // no chosen points, skip this frame (should be impossible)
  }

  return avg/total;
}

bool normalizePixels() {
  if (!amg.readPixels(raw_pixels)) return false;

  // calculate trimmed average
  cavg1 = trimMean(1);
  cavg2 = trimMean(2);

  if (((int8_t)cavg1) == 0 || ((int8_t)cavg2) == 0) return false;

  // calculate CSM gradients
  global_bgm = 0; // needed to skip bgm check in first calculateFgm
  calculateFgm();
  // run 2 passes to amplify real points over noise
  calculateBgm();
  calculateFgm();
  calculateBgm();
  calculateFgm();

  return true;
}

float calculateNewBackground(coord_t i) {
  // implicit alpha of 0.001 because avg_pixels is raw_pixels*1000.0
  float currBg = bgPixel(i);
  float std = constrainedPixel(i) - currBg;
  float bgd = abs(std);
  if (bgd < 0.5) return 10*std; // alpha = 0.01

  float cavg = SIDE1(i) ? cavg1 : cavg2;
  float cgd = cavg - currBg;

  if ((std < 0 && cgd < 0) || (std > 0 && cgd > 0)) {
    // foreground average and this point's temp are on same side of background
    float acgd = abs(cgd);
    if (acgd > 1 && acgd < bgd) {
      // foreground average is more than 1º away, but closer than point's temp
      // so move towards it quickly
      return cgd;
    }
  }

  // otherwise move very slowly towards this point's temp
  // increment/decrement average by 0.001. Every 1º change will take 100 sec to learn.
  return std < 0 ? -1 : 1;
}

void updateBgAverage() {
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    #ifdef RECESSED
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) {
    #else
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES && SIDE(i) != ajar_side &&
        (door_state == DOOR_AJAR || previous_door_state == DOOR_AJAR)) {
    #endif
      // door just changed, reset avg_pixels to current foreground average
      // since we know nothing about this brave new world's background
      avg_pixels[i] = floatToFint3(SIDE1(i) ? cavg1 : cavg2);
      continue;
    }

    // yes we can use += here and rely on type promotion, but I want to be absolutely
    // explicit that we need to use int32_t and not int16_t to avoid overflow
    int32_t temp = ((int32_t)avg_pixels[i]) + ((int32_t)calculateNewBackground(i));
    avg_pixels[i] = constrain(temp, ((uint16_t)MIN_TEMP)*1000, ((uint16_t)MAX_TEMP)*1000);
  }
}

void startBgAverage() {
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    raw_pixels[i] = 0.0;
  }

  amg.readPixels(raw_pixels);

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] = floatToFint3(constrainedPixel(i));
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!amg.readPixels(raw_pixels)) {
      // wait for pixels to change
      hiome.wait(SLEEP_30MS);
    }

    for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      float std = constrainedPixel(i) - bgPixel(i);
      // alpha of 0.3
      int32_t temp = ((int32_t)avg_pixels[i]) + ((int32_t)(300.0 * std));
      avg_pixels[i] = constrain(temp, ((uint16_t)MIN_TEMP)*1000, ((uint16_t)MAX_TEMP)*1000);
    }
  }
}
