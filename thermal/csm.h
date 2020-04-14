float   raw_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
fint3_t avg_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float global_bgm = 0;
float global_fgm = 0;
float cavg1 = 0;
float cavg2 = 0;

float bgPixel(coord_t x) {
  return fint3ToFloat(avg_pixels[(x)]);
}

float diffFromPoint(coord_t a, coord_t b) {
  return abs(raw_pixels[(a)] - raw_pixels[(b)]);
}

// calculate difference from foreground
float fgDiff(coord_t i) {
  float t = constrain(raw_pixels[(i)], MIN_TEMP, MAX_TEMP);
  float fgmt1 = abs(t - cavg1);
  float fgmt2 = abs(t - cavg2);
  return min(fgmt1, fgmt2);
}

// calculate difference from background
float bgDiff(coord_t i) {
  float std = constrain(raw_pixels[(i)], MIN_TEMP, MAX_TEMP) - bgPixel(i);
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
  // skip calculating bgm if door just opened
  // since we have no idea what the background is anyway
  #ifdef RECESSED
    if (door_state == DOOR_CLOSED) return 0;
    if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) {
      return 100;
    }
  #else
    if (SIDE2(i)) {
      if (door_state == DOOR_CLOSED) return 0;
      if (frames_since_door_open < MAX_DOOR_CHANGE_FRAMES) {
        return 100;
      }
    }
  #endif

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

float maxTempDiffForFgd(float f) {
  f *= 0.9;
  return min(f, 20.0);
}

float maxTempDiffForPoint(coord_t x) {
  return maxTempDiffForFgd(fgDiff(x));
}

float trimMean(uint8_t side) {
  coord_t sortedPixels[32];
  uint8_t total = 0;
  coord_t maxI = 32*side;
  for (coord_t i = (maxI - 32); i < maxI; i++) {
    // sort clusters by raw temp
    if (((uint8_t)raw_pixels[i]) > MIN_TEMP && ((uint8_t)raw_pixels[i]) < MAX_TEMP) {
      bool added = false;
      for (uint8_t j=0; j<total; j++) {
        if (raw_pixels[i] > raw_pixels[(sortedPixels[j])]) {
          for (int8_t x=total; x>j; x--) {
            sortedPixels[x] = sortedPixels[(x-1)];
          }
          sortedPixels[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        // append i to end of array
        sortedPixels[total] = i;
      }
      total++;
    }
  }

  SERIAL_PRINTLN(total); // should always == 32

  float avg = 0;
  uint8_t newTotal = 0;
  for (idx_t i = 0; i < total-6; i++) {
    // drop the top 6 warmest points when calculating mean to skew it lower
    // this might cause issues if the person is actually cooler than the background,
    // such as when keith is sweaty. We shall see!
    avg += raw_pixels[(sortedPixels[i])];
    newTotal++;
    if (frames_since_door_open >= MAX_DOOR_CHANGE_FRAMES &&
        (uint8_t)bgDiff(sortedPixels[i]) == 0) {
      // double weight of points with low background diff
      avg += raw_pixels[(sortedPixels[i])];
      newTotal++;
    }
  }

  SERIAL_PRINTLN(newTotal); // should be between 26 - 52

  if (!newTotal) {
    SERIAL_PRINTLN(F("xxx"));
    return 0; // no chosen points, skip this frame (should be impossible)
  }

  return avg/newTotal;
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
  float std = raw_pixels[(i)] - currBg;
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
  #ifdef RECESSED
    if (door_state == DOOR_CLOSED) {
      return;
    }
  #endif

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (door_state == DOOR_AJAR && SIDE(i) == ajar_side) continue;
    #ifndef RECESSED
      if (door_state == DOOR_CLOSED && SIDE2(i)) {
        return;
      }
    #endif

    // ignore extreme raw pixels
    if (((uint8_t)raw_pixels[(i)]) <= MIN_TEMP || ((uint8_t)raw_pixels[(i)]) >= MAX_TEMP) {
      continue;
    }

    // yes we can use += here and rely on type promotion, but I want to be absolutely
    // explicit that we need to use int32_t and not int16_t to avoid overflow
    int32_t temp = ((int32_t)avg_pixels[i]) + ((int32_t)round(calculateNewBackground(i)));
    avg_pixels[i] = temp;
  }
}

void startBgAverage() {
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    raw_pixels[i] = 0.0;
  }

  amg.readPixels(raw_pixels);

  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    avg_pixels[i] = floatToFint3(constrain(raw_pixels[i], MIN_TEMP, MAX_TEMP));
  }

  for (uint8_t k=0; k < 10; k++) {
    while (!amg.readPixels(raw_pixels)) {
      // wait for pixels to change
      hiome.sleep(SLEEP_30MS);
    }

    for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      if (((uint8_t)raw_pixels[i]) <= MIN_TEMP || ((uint8_t)raw_pixels[i]) >= MAX_TEMP) {
        continue;
      }
      float std = raw_pixels[i] - bgPixel(i);
      // alpha of 0.3
      int32_t temp = ((int32_t)avg_pixels[i]) + ((int32_t)(300.0 * std));
      avg_pixels[i] = temp;
    }
  }
}
