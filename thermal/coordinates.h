#ifdef OPTIMIZE_FOR_SERIAL

// Replace above progmem with following functions to save 146 bytes of storage space
axis_t xCoord(coord_t p) {
  return (p % GRID_EXTENT) + 1;
}
axis_t yCoord(coord_t p) {
  return (p/GRID_EXTENT) + 1;
}

#else

// store in-memory so we don't have to do math every time
const axis_t xcoordinates[AMG88xx_PIXEL_ARRAY_SIZE] PROGMEM = {
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8
};
const axis_t ycoordinates[AMG88xx_PIXEL_ARRAY_SIZE] PROGMEM = {
  1,  1,  1,  1,  1,  1,  1,  1,
  2,  2,  2,  2,  2,  2,  2,  2,
  3,  3,  3,  3,  3,  3,  3,  3,
  4,  4,  4,  4,  4,  4,  4,  4,
  5,  5,  5,  5,  5,  5,  5,  5,
  6,  6,  6,  6,  6,  6,  6,  6,
  7,  7,  7,  7,  7,  7,  7,  7,
  8,  8,  8,  8,  8,  8,  8,  8
};

#define xCoord(p) ( (axis_t)pgm_read_byte_near(xcoordinates + (p)) )
#define yCoord(p) ( (axis_t)pgm_read_byte_near(ycoordinates + (p)) )

#endif

// check if point is on the top or bottom edges
// xxxxxxxx
// xxxxxxxx
// xxxxxxxx
// oooooooo
// oooooooo
// xxxxxxxx
// xxxxxxxx
// xxxxxxxx
#ifdef YAXIS
  #define AXIS                  yCoord
  #define NOT_AXIS              xCoord
  #define SIDE1(p)              ( (p) < (AMG88xx_PIXEL_ARRAY_SIZE/2) )
  #define SIDE2(p)              ( (p) >= (AMG88xx_PIXEL_ARRAY_SIZE/2) )
  #define pointOnBorder(i)      ( (i) < (GRID_EXTENT * 3) || (i) >= (GRID_EXTENT * 5) )
  #define pointOnSmallBorder(i) ( (i) < (GRID_EXTENT * 2) || (i) >= (GRID_EXTENT * 6) )
  #define pointOnEdge(i)        ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
  #define pointOnLREdge(i)      ( NOT_AXIS(i) == 1 || NOT_AXIS(i) == GRID_EXTENT )
#else
  #define AXIS                  xCoord
  #define NOT_AXIS              yCoord
  #define SIDE1(p)              ( (AXIS(p) <= (GRID_EXTENT/2) )
  #define SIDE2(p)              ( (AXIS(p) > (GRID_EXTENT/2) )
  #define pointOnBorder(i)      ( AXIS(i) <= 3 || AXIS(i) >= 6 )
  #define pointOnSmallBorder(i) ( AXIS(i) <= 2 || AXIS(i) >= 7 )
  #define pointOnEdge(i)        ( AXIS(i) == 1 || AXIS(i) == GRID_EXTENT )
  #define pointOnLREdge(i)      ( (i) < GRID_EXTENT || (i) >= (GRID_EXTENT * 7) )
  #error Double check all your code, this is untested
#endif

uint8_t SIDE(coord_t p) {
  return SIDE1(p) ? 1 : 2;
}

axis_t normalizeAxis(axis_t p) {
  return (p > 4 ? (GRID_EXTENT+1 - p) : p);
}

float euclidean_distance(coord_t p1, coord_t p2) {
  // calculate euclidean distance instead
  int8_t yd = ((int8_t)yCoord(p2)) - ((int8_t)yCoord(p1));
  int8_t xd = ((int8_t)xCoord(p2)) - ((int8_t)xCoord(p1));
  return (float)sqrt(sq(yd) + sq(xd)) + 0.05;
}

uint8_t int_distance(coord_t a, coord_t b) {
  return ((uint8_t)(euclidean_distance((a), (b)) + 0.1));
}

uint8_t axis_distance(coord_t p1, coord_t p2) {
  int8_t axisJump = ((int8_t)AXIS(p1)) - ((int8_t)AXIS(p2));
  return abs(axisJump);
}

uint8_t not_axis_distance(coord_t p1, coord_t p2) {
  int8_t axisJump = ((int8_t)NOT_AXIS(p1)) - ((int8_t)NOT_AXIS(p2));
  return abs(axisJump);
}

uint8_t max_axis_jump(coord_t p1, coord_t p2) {
  uint8_t axisJump = axis_distance(p1, p2);
  uint8_t notAxisJump = not_axis_distance(p1, p2);
  return max(axisJump, notAxisJump);
}