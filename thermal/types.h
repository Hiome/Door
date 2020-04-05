// Custom type definitions to make sure we don't mix up types when debugging algorithm

// row or column # (1-GRID_EXTENT)
typedef uint8_t  axis_t;
// pixel coordinate (0-AMG88xx_PIXEL_ARRAY_SIZE + UNDEF_POINT)
typedef uint8_t  coord_t;
// person array index (0-MAX_PEOPLE + UNDEF_INDEX)
typedef uint8_t  idx_t;
// float -> uint with 1 precision (multiplied by 10 and truncated)
// min float value of 0, max float value of 25.5
typedef uint8_t  fint1_t;
// float -> uint with 2 precision (multiplied by 100 and truncated)
// min float value of 0, max float value of 655.35
typedef uint16_t fint2_t;
// float -> uint with 3 precision (multiplied by 1000 and truncated)
// min float value of 0, max float value of 65.535
typedef uint16_t fint3_t;

float fint1ToFloat(fint1_t a) {
  return ((float)a)/10.0;
}

fint1_t floatToFint1(float a) {
  return ((fint1_t)(a*10.0));
}

float fint2ToFloat(fint2_t a) {
  return ((float)a)/100.0;
}

fint2_t floatToFint2(float a) {
  return ((fint2_t)(a*100.0));
}

float fint3ToFloat(fint3_t a) {
  return ((float)a)/1000.0;
}

fint3_t floatToFint3(float a) {
  return ((fint3_t)(a*1000.0));
}
