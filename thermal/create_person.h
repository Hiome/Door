// new point appeared (no past point found), start tracking it
coord_t sp = points[i].current_position;
coord_t mp = sp;
uint8_t mj = 0;
fint1_t md = 0;
uint8_t cross = 0;
uint8_t h = 1;
uint16_t c = 1;
uint8_t fc = 0;
float bgm = points[i].bgm();
float fgm = points[i].fgm();

// first let's check points on death row from this frame for a match
remember_person(sp, mp, mj, md, cross, h, c, fc, points[i].max_distance());

if (c == 1 && points[i].height && bgm > 1.5 && fgm > 1.5 && normalizeAxis(AXIS(sp)) == 4) {
  // if point is right in middle, drag it to the side it appears to be coming from
  coord_t spa = sp - GRID_EXTENT;
  coord_t spb = sp + GRID_EXTENT;
  if (SIDE1(sp)) {
    if (compareNeighboringPixels(spb,spa,sp,fgm)) {
      sp += GRID_EXTENT;
      h++;
    }
  } else if (compareNeighboringPixels(spa,spb,sp,fgm)) {
    sp -= GRID_EXTENT;
    h++;
  }
}

for (idx_t j=0; j<MAX_PEOPLE; j++) {
  // look for first empty slot in known_people to use
  if (!known_people[j].real()) {
    Person p = {
      .past_position=points[i].current_position,
      .starting_position=sp,
      .max_position=mp,
      .confidence=points[i].confidence,
      .max_temp_drift=md,
      .count=c,
      .history=h,
      .retreating=false,
      .neighbors=points[i].neighbors,
      .height=points[i].height,
      .forgotten_count=fc,
      .width=points[i].width,
      .crossed=cross,
      .max_jump=mj,
      .avg_height=points[i].height,
      .avg_width=points[i].width,
      .avg_neighbors=points[i].neighbors,
      .avg_confidence=points[i].confidence,
      .blobSize=(uint8_t)points[i].blobSize,
      .noiseSize=(uint8_t)points[i].noiseSize,
      .avg_bgm=floatToFint2(bgm),
      .avg_fgm=floatToFint2(fgm),
      .raw_temp=points[i].raw_temp(),
      .fgm=fgm
    };
    known_people[j] = p;
    break;
  }
}
