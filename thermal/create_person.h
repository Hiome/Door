// new point appeared (no past point found), start tracking it
coord_t sp = points[i].current_position;
coord_t mp = sp;
uint8_t mj = 0;
fint1_t md = 0;
uint8_t cross = 0;
bool revert = false;
uint8_t conf = points[i].confidence;
float rt = points[i].raw_temp();
float b = points[i].bgm();
float f = points[i].fgm();
uint8_t n = points[i].neighbors;
uint8_t height = points[i].height;
uint8_t width = points[i].width;
uint8_t h = 1;
uint16_t c = 1;
uint8_t fc = 0;

if (temp_forgotten_num > 0 && !pointOnEdge(points[i].current_position)) {
  // first let's check points on death row from this frame for a match
  remember_person(temp_forgotten_people, points[i].current_position, h, sp, mp, mj,
        md, cross, revert, c, fc, height, width, n, conf);
}

if (c == 1 && cycles_since_forgotten < MAX_EMPTY_CYCLES) {
  // second let's check past forgotten points for a match
  remember_person(forgotten_people, points[i].current_position, h, sp, mp, mj,
        md, cross, revert, c, fc, height, width, n, conf);
}

axis_t spAxis = normalizeAxis(AXIS(sp));
if (c == 1 && spAxis >= 3 && b > 1.5 && f > 1.5) {
  // if point is right in middle, drag it to the side it appears to be coming from
  coord_t a = sp - GRID_EXTENT;
  coord_t b = sp + GRID_EXTENT;
  bool djo = doorJustOpened();
  if (djo && spAxis == 4 && SIDE(sp) == door_side) {
    if (door_side == 1) {
      sp += GRID_EXTENT;
    } else {
      sp -= GRID_EXTENT;
    }
    h++;
  } else if (!djo && (side1Point || side2Point) && otherPersonExists(sp)) {
    // there's another person in the frame, assume this is a split of that person
    // TODO look into this logic tomorrow
    if (side1Point > (side2Point+10) && (spAxis == 4 ||
          compareNeighboringPixels(a,b,sp,f))) {
      if (SIDE2(sp)) {
        h++;
        while (SIDE2(sp)) {
          sp -= GRID_EXTENT;
        }
      }
      side1Point = 0;
      side2Point = 0;
    } else if (side2Point > (side1Point+10) && (spAxis == 4 ||
          compareNeighboringPixels(b,a,sp,f))) {
      if (SIDE1(sp)) {
        h++;
        while (SIDE1(sp)) {
          sp += GRID_EXTENT;
        }
      }
      side1Point = 0;
      side2Point = 0;
    }
  } else if (!djo && spAxis == 4 && height > 0) {
    // catch entries on door open for people who did not setup the door contact magnet
    if (SIDE1(sp)) {
      if (compareNeighboringPixels(b,a,sp,f)) {
        sp += GRID_EXTENT;
        h++;
      }
    } else if (compareNeighboringPixels(a,b,sp,f)) {
      sp -= GRID_EXTENT;
      h++;
    }
  }
}

// ignore new points if door is not open or immediately after door opens
if (door_state == DOOR_CLOSED || ((frames_since_door_open < 2 ||
      door_state == DOOR_AJAR) && SIDE(sp)==door_side)) {
  continue;
}

for (idx_t j=0; j<MAX_PEOPLE; j++) {
  // look for first empty slot in known_people to use
  if (!known_people[j].real()) {
    Person p = {
      .past_position=points[i].current_position,
      .starting_position=sp,
      .max_position=mp,
      .confidence=conf,
      .max_temp_drift=md,
      .count=c,
      .history=h,
      .retreating=false,
      .neighbors=n,
      .height=height,
      .forgotten_count=fc,
      .width=width,
      .crossed=cross,
      .max_jump=mj,
      .reverted=revert,
      .avg_bgm=floatToFint2(b),
      .avg_fgm=floatToFint2(f),
      .avg_height=height,
      .avg_width=width,
      .avg_neighbors=n,
      .avg_confidence=conf,
      .blobSize=(uint8_t)points[i].blobSize,
      .noiseSize=(uint8_t)points[i].noiseSize,
      .raw_temp=rt,
      .fgm=f
    };
    known_people[j] = p;
    break;
  }
}
