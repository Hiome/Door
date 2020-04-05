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
        if (norm_pixels[i] > norm_pixels[(ordered_indexes_temp[j])]) {
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
    float fgd = fgDiff(i);
    float mt = maxTempDiffForFgd(fgd);
    uint8_t nci = neighborsCount(i, mt, norm_pixels);
    for (uint8_t j=0; j<z; j++) {
      coord_t oj = sibling_indexes[j];
      if ((norm_pixels[i]*2) > norm_pixels[oj] && diffFromPoint(oj, i) < min(fgd-0.1, 0.6)) {
        float mt2 = maxTempDiffForPoint(ordered_indexes[j]);
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
    float fgd = fgDiff(current_point);
    float mt = maxTempDiffForFgd(fgd);

    // scan all points added after current_point, since they must be part of same blob
    for (uint8_t x=sorted_size-1; x<sorted_size; x++) {
      coord_t blobPoint = ordered_indexes_temp[x];
      float mtb;
      if (blobPoint == current_point) {
        mtb = mt;
      } else {
        mtb = maxTempDiffForPoint(blobPoint);
        // find how many neighbors this point has
        uint8_t fnc = 0;
        coord_t foundNeighbor = UNDEF_POINT;
        coord_t blobNeighbors[8];
        uint8_t nc = loadNeighbors(blobPoint, blobNeighbors);
        for (uint8_t bn = 0; bn < nc; bn++) {
          if (clusterNum[blobNeighbors[bn]] == clusterIdx) {
            fnc++;
            if (fnc == 2) break;
            foundNeighbor = blobNeighbors[bn];
          }
        }
        if (fnc == 1) {
          // if point only has 1 connection to this blob, maybe it's time to stop expanding
          bool skippable = true;
          for (uint8_t bn = 0; bn < nc; bn++) {
            if (clusterNum[blobNeighbors[bn]] == 0 &&
                  diffFromPoint(blobNeighbors[bn], blobPoint) < mtb) {
              // keep expanding if there are actually more connections to the blob
              if (isNeighborly(blobNeighbors[bn], foundNeighbor)) {
                skippable = false;
                break;
              }
//              else {
//                // scan all neighbors of new point to see if it's touching this blob
//                coord_t blobNeighbors2[8];
//                uint8_t nc2 = loadNeighbors(blobNeighbors[bn], blobNeighbors2);
//                for (uint8_t bn2 = 0; bn2 < nc2; bn2++) {
//                  // don't double count the current point
//                  if (blobNeighbors2[bn2] == blobPoint) continue;
//                  // check the other neighbors for a connection
//                  if (clusterNum[blobNeighbors2[bn2]] == clusterIdx) {
//                    // it is! We must keep expanding this blob then
//                    skippable = false;
//                    break;
//                  }
//                }
//                if (!skippable) break;
//              }
            }
          }
          if (skippable) continue;
        }
      }

      for (uint8_t k=y+1; k<active_pixel_count; k++) {
        // scan all known points after current_point to find neighbors to point x
        if (ordered_indexes[k] != UNDEF_POINT &&
            isNeighborly(ordered_indexes[k], blobPoint) &&
            (diffFromPoint(ordered_indexes[k], blobPoint) < mtb ||
              diffFromPoint(ordered_indexes[k], current_point) < mt)) {
          clusterNum[ordered_indexes[k]] = clusterIdx;
          ordered_indexes_temp[sorted_size] = ordered_indexes[k];
          sorted_size++;
          ordered_indexes[k] = UNDEF_POINT;
        }
      }
    }

    // check if point is too small/large to be a valid person
    uint8_t blobSize = 1;
    uint8_t totalBlobSize = 1;
    axis_t minAxis = AXIS(current_point);
    axis_t maxAxis = minAxis;
    axis_t minNAxis = NOT_AXIS(current_point);
    axis_t maxNAxis = minNAxis;
    uint8_t neighbors = 0;
    for (coord_t n = 0; n < AMG88xx_PIXEL_ARRAY_SIZE; n++) {
      if (clusterNum[n] == clusterIdx && n != current_point) {
        totalBlobSize++;

        float dp = diffFromPoint(n, current_point);
        if (dp > mt) continue;

        blobSize++;
        axis_t axisn = AXIS(n);
        minAxis = min(minAxis, axisn);
        maxAxis = max(maxAxis, axisn);
        axis_t naxisn = NOT_AXIS(n);
        minNAxis = min(minNAxis, naxisn);
        maxNAxis = max(maxNAxis, naxisn);
        if (isNeighborly(n, current_point)) neighbors++;
      }
    }

    if (totalBlobSize > 60) {
      SERIAL_PRINT(F("x "));
      SERIAL_PRINT(current_point);
      SERIAL_PRINTLN(F(" toobig"));
      continue;
    }

    uint8_t height = maxAxis - minAxis;
    uint8_t width = maxNAxis - minNAxis;
    uint8_t dimension = max(height, width) + 1;
    uint8_t boundingBox = min(dimension, 5);
    uint8_t bgd = (uint8_t)bgDiff(current_point);
    // ignore a blob that fills less than 1/3 of its bounding box
    // a blob with 9 points will always pass density test
    if ((totalBlobSize + min((uint8_t)fgd, bgd))*3 >= sq(boundingBox)) {
      uint8_t noiseSize = 0;
      float mt_constrained = mt*0.7;
      mt_constrained = constrain(mt_constrained, 0.51, 1.51);
      for (coord_t n = 0; n < AMG88xx_PIXEL_ARRAY_SIZE; n++) {
        if (clusterNum[n] == clusterIdx) continue;

        float dp = diffFromPoint(n, current_point);
        if (dp > mt_constrained) continue;

        if (norm_pixels[n] < CONFIDENCE_THRESHOLD) {
          // only count 0 confidence points within 0.3º (regardless of distance)
          if (dp > 0.3) continue;
        } else if (dimension < 5 && int_distance(n, current_point) >= dimension+2) {
          // only count points with confidence within dimension+2 distance
          continue;
        }

        noiseSize++;
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
