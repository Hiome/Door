// ignore what looks to be a door
if (points[i].width == 7 && points[i].height < 2) continue;

// new point appeared (no past point found), start tracking it
coord_t sp = points[i].current_position;
float bgm = points[i].bgm();
float fgm = points[i].fgm();

if (points[i].height && normalizeAxis(AXIS(sp)) == 4) {
  // if point is right in middle, drag it to the side it appears to be coming from
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

if (door_state == DOOR_AJAR && SIDE(sp) == ajar_side) continue;

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
  .crossed=0,
  .max_jump=0,
  .avg_height=points[i].height,
  .avg_width=points[i].width,
  .avg_neighbors=points[i].neighbors,
  .avg_confidence=points[i].confidence,
  .blobSize=(uint8_t)points[i].blobSize,
  .noiseSize=(uint8_t)points[i].noiseSize,
  .forgotten_count=0,
  .avg_bgm=floatToFint2(bgm),
  .avg_fgm=floatToFint2(fgm),
  .raw_temp=points[i].raw_temp(),
  .bgm=bgm,
  .fgm=fgm
};
updateKnownPerson(p, pairs);
