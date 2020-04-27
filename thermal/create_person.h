// new point appeared (no past point found), start tracking it
coord_t sp = points[i].current_position;
float bgm = points[i].bgm();
float fgm = points[i].fgm();

// point is right in middle, drag it to the correct side
if (normalizeAxis(AXIS(sp)) == 4) {
  if (frames_since_door_open < 3) {
    // door is currently ajar and point is on side of door
    if ((SIDE(sp) == ajar_side && (door_state == DOOR_AJAR ||
          // or door was ajar, so no way person is already at row 4
          previous_door_state == DOOR_AJAR)) ||
          // or door was closed and point was not behind the door
          (previous_door_state == DOOR_CLOSED && !doorSide(sp))) {
      // so drag drag drag
      if (SIDE1(sp)) {
        sp += GRID_EXTENT;
      } else {
        sp -= GRID_EXTENT;
      }
    }
  } else if (frames_since_door_open >= MAX_DOOR_CHANGE_FRAMES && points[i].height) {
    // door didn't just open, but it's possible user doesn't have magnet installed
    // so drag point to side where it has more heat
    coord_t spa = sp - GRID_EXTENT;
    coord_t spb = sp + GRID_EXTENT;
    float maxTemp = min(fgm, bgm);
    if (SIDE1(sp)) {
      if (compareNeighboringPixels(spb,spa,sp,maxTemp)) {
        sp += GRID_EXTENT;
      }
    } else if (compareNeighboringPixels(spa,spb,sp,maxTemp)) {
      sp -= GRID_EXTENT;
    }
  }
}

// door is currently ajar and point is on side of door
if ((SIDE(sp) == ajar_side && (door_state == DOOR_AJAR ||
    // or door was *just* ajar, so no way a point is already starting there
    (previous_door_state == DOOR_AJAR && !frames_since_door_open))) ||
    // or door was *just* closed and point is not behind that door
    (previous_door_state == DOOR_CLOSED && !frames_since_door_open && !doorSide(sp))) {
  // so ignore it
  continue;
}

Person p = {
  .past_position=points[i].current_position,
  .starting_position=sp,
  .max_position=points[i].current_position,
  .confidence=points[i].confidence,
  .max_temp_drift=0,
  .count=1,
  .history=1,
  .retreating=false,
  .neighbors=points[i].neighbors,
  .height=points[i].height,
  .width=points[i].width,
  .max_jump=0,
  .crossed=false,
  .avg_neighbors=points[i].neighbors,
  .avg_height=points[i].height,
  .avg_width=points[i].width,
  .avg_confidence=points[i].confidence,
  .blobSize=(uint8_t)points[i].blobSize,
  .noiseSize=(uint8_t)points[i].noiseSize,
  .avg_bgm=floatToFint2(bgm),
  .avg_fgm=floatToFint2(fgm),
  .raw_temp=points[i].raw_temp(),
  .bgm=bgm,
  .fgm=fgm
};
updateKnownPerson(p, pairs);
