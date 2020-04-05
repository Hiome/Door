bool isNeighborly(coord_t a, coord_t b) {
  return int_distance(a, b) == 1;
}

uint8_t loadNeighbors(coord_t i, coord_t (&nArray)[8]) {
  uint8_t nc = 0;
  axis_t nai = NOT_AXIS(i);

  if (i >= GRID_EXTENT) { // not top row
    // top
    nArray[nc] = i-GRID_EXTENT;
    nc++;
    // top left
    if (nai > 1) {
      nArray[nc] = i-(GRID_EXTENT+1);
      nc++;
    }
    // top right
    if (nai < GRID_EXTENT) {
      nArray[nc] = i-(GRID_EXTENT-1);
      nc++;
    }
  }
  if (i < GRID_EXTENT*7) { // not bottom row
    // bottom
    nArray[nc] = i + GRID_EXTENT;
    nc++;
    // bottom left
    if (nai > 1) {
      nArray[nc] = i+(GRID_EXTENT-1);
      nc++;
    }
    // bottom right
    if (nai < GRID_EXTENT) {
      nArray[nc] = i+(GRID_EXTENT+1);
      nc++;
    }
  }
  // left
  if (nai > 1) {
    nArray[nc] = i-1;
    nc++;
  }
  // right
  if (nai < GRID_EXTENT) {
    nArray[nc] = i+1;
    nc++;
  }

  return nc;
}

uint8_t neighborsCount(coord_t i,float mt,uint8_t (&norm_pixels)[AMG88xx_PIXEL_ARRAY_SIZE]) {
  uint8_t nc = 0;
  coord_t neighbors[8];
  uint8_t totalNc = loadNeighbors(i, neighbors);
  for (uint8_t x = 0; x < totalNc; x++) {
    coord_t n = neighbors[x];
    if (norm_pixels[n] > CONFIDENCE_THRESHOLD && diffFromPoint(n, i) < mt) nc++;
  }
  return nc;
}

bool compareNeighboringPixels(coord_t x, coord_t y, coord_t i, float f) {
  float d = diffFromPoint(x, i);
  return (d + 0.5) < diffFromPoint(y, i) && d < maxTempDiffForFgd(f);
}
