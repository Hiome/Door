uint8_t findCurrentPoints() {
  // sort pixels by confidence
  coord_t ordered_indexes_temp[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t norm_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t active_pixel_count = 0;
  for (coord_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    uint8_t bgm = calcBgm(i);
    uint8_t fgm = calcFgm(i);
    norm_pixels[i] = min(bgm, fgm);

    if (norm_pixels[i] > CONFIDENCE_THRESHOLD) {
      bool added = false;
      for (uint8_t j=0; j<active_pixel_count; j++) {
        if (norm_pixels[i] > norm_pixels[(ordered_indexes_temp[j])] ||
            (norm_pixels[i] == 100 &&
              // since we cap fg/bgDiffs to 10º, if 2 points are
              // both at 100%, choose the one with higher fgDiff
              fgDiff(i) > fgDiff(ordered_indexes_temp[j]))) {
          for (int8_t x=active_pixel_count; x>j; x--) {
            ordered_indexes_temp[x] = ordered_indexes_temp[(x-1)];
          }
          ordered_indexes_temp[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        // append i to end of array
        ordered_indexes_temp[active_pixel_count] = i;
      }
      active_pixel_count++;
    }
  }

  // adjust sorted pixels based on neighbor count and position
  coord_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  coord_t sibling_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  for (uint8_t z=0; z<active_pixel_count; z++) {
    coord_t i = ordered_indexes_temp[z];
    bool added = false;
    float mt = fgDiff(i);
    uint8_t nci = neighborsCount(i, mt, norm_pixels);
    for (uint8_t j=0; j<z; j++) {
      coord_t oj = sibling_indexes[j];
      if ((norm_pixels[i]*2) > norm_pixels[oj] && diffFromPoint(oj, i) < min(mt/4, 0.75)) {
        float mt2 = fgDiff(ordered_indexes[j]);
        uint8_t ncj = neighborsCount(ordered_indexes[j], mt2, norm_pixels);
        if (nci > (ncj + 1)) {
          // prefer the point that's more in middle of blob
          added = true;
        } else if (nci >= ncj) {
          // prefer point closer to middle of grid
          // it is tempting to limit this to only when temp doesn't change, but beware!
          // that forces the blob to sometimes make large leaps that get blocked
          axis_t edge1 = AXIS(i);
          axis_t edge2 = AXIS(ordered_indexes[j]);

          // use columns instead of rows if same row
          if (edge1 == edge2) {
            edge1 = NOT_AXIS(i);
            edge2 = NOT_AXIS(ordered_indexes[j]);
          }

          // calculate row # from opposite edge
          edge1 = normalizeAxis(edge1);
          edge2 = normalizeAxis(edge2);

          added = edge1 > edge2;
        }

        if (added) {
          // insert point i in front of j
          for (int8_t x=z; x>j; x--) {
            sibling_indexes[x] = sibling_indexes[(x-1)];
            ordered_indexes[x] = ordered_indexes[(x-1)];
          }
          sibling_indexes[j] = oj;
          ordered_indexes[j] = i;
          break;
        }
      }
    }
    if (!added) {
      // append i to end of array
      ordered_indexes[z] = i;
      sibling_indexes[z] = i;
    }
  }

  // DBSCAN algorithm will identify core points and label every point to a cluster number
  uint8_t sorted_size = 0;
  uint8_t total_masses = 0;
  // 0 means no cluster, first cluster starts at 1
  uint8_t clusterNum[AMG88xx_PIXEL_ARRAY_SIZE] = { 0 };
  uint8_t clusterIdx = 0;
  for (uint8_t y=0; y<active_pixel_count; y++) {
    coord_t current_point = ordered_indexes[y];
    if (current_point == UNDEF_POINT) continue;

    clusterIdx++;
    clusterNum[current_point] = clusterIdx;
    ordered_indexes_temp[sorted_size] = current_point;
    sorted_size++;
    ordered_indexes[y] = UNDEF_POINT;
    float mt = fgDiff(current_point);
    uint8_t neighbors = 0;
    uint8_t blobSize = 1;
    axis_t minAxis = AXIS(current_point);
    axis_t maxAxis = minAxis;
    axis_t minNAxis = NOT_AXIS(current_point);
    axis_t maxNAxis = minNAxis;

    // scan all points added after current_point, since they must be part of same blob
    for (uint8_t x=sorted_size-1; x<sorted_size; x++) {
      coord_t blobPoint = ordered_indexes_temp[x];
      if (blobPoint != current_point) {
        // find how many neighbors this point has
        uint8_t fnc = 0;
        coord_t blobNeighbors[8];
        uint8_t nc = loadNeighbors(blobPoint, blobNeighbors);
        for (uint8_t bn = 0; bn < nc; bn++) {
          if (clusterNum[blobNeighbors[bn]] == clusterIdx) {
            if (blobNeighbors[bn] == current_point) {
              // we are still neighboring current point, give us more time
              fnc = 3;
              break;
            }
            fnc++;
            if (fnc == 2) break;
          }
        }
        // if point only has 1 connection to this blob, don't expand on this point
        if (fnc == 1) continue;
      }

      for (uint8_t k=y+1; k<active_pixel_count; k++) {
        // scan all known points after current_point to find neighbors to point x
        if (ordered_indexes[k] != UNDEF_POINT &&
            isNeighborly(ordered_indexes[k], blobPoint) &&
            diffFromPoint(ordered_indexes[k], current_point) < mt) {
          clusterNum[ordered_indexes[k]] = clusterIdx;
          ordered_indexes_temp[sorted_size] = ordered_indexes[k];
          sorted_size++;
          blobSize++;
          axis_t axisn = AXIS(ordered_indexes[k]);
          minAxis = min(minAxis, axisn);
          maxAxis = max(maxAxis, axisn);
          axis_t naxisn = NOT_AXIS(ordered_indexes[k]);
          minNAxis = min(minNAxis, naxisn);
          maxNAxis = max(maxNAxis, naxisn);
          if (blobPoint == current_point) neighbors++;
          ordered_indexes[k] = UNDEF_POINT;
        }
      }
    }

    uint8_t height = maxAxis - minAxis;
    uint8_t width = maxNAxis - minNAxis;
    uint8_t dimension = max(height, width) + 1;
    axis_t curr_axis = AXIS(current_point);
    axis_t curr_naxis = NOT_AXIS(current_point);
    uint8_t boundingBox = min(dimension, 5);
    uint8_t bgd = (uint8_t)bgDiff(current_point);
    // ignore a blob that fills less than 1/3 of its bounding box
    // a blob with 9 points will always pass density test
    if ((blobSize + min(bgd, (uint8_t)mt))*3 >= sq(boundingBox)) {
      uint8_t noiseSize = 0;
      float mt_constrained = mt*0.7;
      mt_constrained = constrain(mt_constrained, 0.6, 1.6);
      for (coord_t n = 0; n < AMG88xx_PIXEL_ARRAY_SIZE; n++) {
        if (clusterNum[n] == clusterIdx) continue;

        float dp = diffFromPoint(n, current_point);
        if (dp > mt_constrained) continue;

        if (dp < 0.3 || (norm_pixels[n] > CONFIDENCE_THRESHOLD &&
              (dimension > 5 || int_distance(n, current_point) < dimension+2)) ||
              AXIS(n) == curr_axis || NOT_AXIS(n) == curr_naxis) {
          // when diff is more than 0.3º, point either needs to be 0 conf or
          // within a limited radius to person
          noiseSize++;
        }
      }

      if (noiseSize <= blobSize) {
        SERIAL_PRINT(F("+ "));
        SERIAL_PRINTLN(current_point);

        PossiblePerson pp = {
          .current_position=current_point,
          .confidence=norm_pixels[current_point],
          .blobSize=blobSize,
          .noiseSize=noiseSize,
          .neighbors=neighbors,
          .height=height,
          .width=width
        };
        points[total_masses] = pp;
        total_masses++;
        if (total_masses == MAX_PEOPLE) break;
      } else {
        SERIAL_PRINT(F("x "));
        SERIAL_PRINT(current_point);
        SERIAL_PRINT(F(" noise="));
        SERIAL_PRINTLN(noiseSize);
      }
    } else {
      SERIAL_PRINT(F("x "));
      SERIAL_PRINT(current_point);
      SERIAL_PRINTLN(F(" nodensity"));
    }
  }

  return total_masses;
}
