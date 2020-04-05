#ifdef PRINT_RAW_DATA

//if (total_masses > 0) { // ignore frames where nothing happened
if (true) {
  for (idx_t i = 0; i<MAX_PEOPLE; i++) {
    Person p = known_people[i];
    if (p.real()) {
      SERIAL_PRINT(p.past_position);
      SERIAL_PRINT(F(" ("));
      SERIAL_PRINT(p.starting_position);
      SERIAL_PRINT(F("-"));
      SERIAL_PRINT(p.history);
      SERIAL_PRINT(F("-"));
      SERIAL_PRINT(p.neighbors);
      SERIAL_PRINT(F("),"));
    }
  }
  SERIAL_PRINTLN();

  SERIAL_PRINT(cavg1);
  SERIAL_PRINT(F(", "));
  SERIAL_PRINTLN(cavg2);
  // SERIAL_PRINTLN(amg.readThermistor());
  SERIAL_PRINTLN(global_bgm);
  SERIAL_PRINTLN(global_fgm);
  // float avg_avg = 0;
  // for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
  //  avg_avg += bgPixel(idx);
  // }
  // avg_avg /= 64.0;
  // SERIAL_PRINTLN(avg_avg);

  // print chart of what we saw in 8x8 grid
  for (coord_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
    SERIAL_PRINT(raw_pixels[idx]);
    uint8_t bgm = calcBgm(idx);
    uint8_t fgm = calcFgm(idx);
    uint8_t norm = min(bgm, fgm);
    if (norm < CONFIDENCE_THRESHOLD) {
      SERIAL_PRINT(F("---"));
    } else {
      if (norm < 100) SERIAL_PRINT(F(" "));
      SERIAL_PRINT(norm);
    }
    if (xCoord(idx) == GRID_EXTENT)
      SERIAL_PRINTLN();
    else
      SERIAL_PRINT(F("  "));
  }
  // SERIAL_PRINTLN(F("avg"));
  // for (uint8_t idx=0; idx<AMG88xx_PIXEL_ARRAY_SIZE; idx++) {
  //  SERIAL_PRINT(F(" "));
  //  SERIAL_PRINT(avg_pixels[idx]);
  //  SERIAL_PRINT(F(" "));
  //  if (xCoord(idx) == GRID_EXTENT) SERIAL_PRINTLN();
  // }
  SERIAL_PRINTLN();
  // SERIAL_FLUSH;
}

#endif
