#define FIRMWARE_VERSION     "V0.1"
#define GRID_EXTENT          8
#define TEMP_BUFFER          2
#define MAX_DISTANCE         3

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float calibrated_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

volatile boolean motion = false;
void motionISR() {
  motion = true;
}

void initialize() {
  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), motionISR, RISING);

  amg.begin();

  delay(100); // let sensor boot up
  amg.readPixels(calibrated_pixels);
}

// store in-memory so we don't have to do math every time
const uint8_t xcoordinates[64] = {
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8,
  1,  2,  3,  4,  5,  6,  7,  8
};

const uint8_t ycoordinates[64] = {
  1,  1,  1,  1,  1,  1,  1,  1,
  2,  2,  2,  2,  2,  2,  2,  2,
  3,  3,  3,  3,  3,  3,  3,  3,
  4,  4,  4,  4,  4,  4,  4,  4,
  5,  5,  5,  5,  5,  5,  5,  5,
  6,  6,  6,  6,  6,  6,  6,  6,
  7,  7,  7,  7,  7,  7,  7,  7,
  8,  8,  8,  8,  8,  8,  8,  8
};

// calculate manhattan distance between 2 indices
uint8_t distance(uint8_t p1, uint8_t p2) {
  int8_t yd = ycoordinates[p2] - ycoordinates[p1];
  int8_t xd = xcoordinates[p2] - xcoordinates[p1];
  return abs(yd) + abs(xd);
}

void loop() {
  amg.readPixels(pixels);

  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  float ordered_temps[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t count = 0;
  for(uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    float t = pixels[i];
    if (t - calibrated_pixels[i] > TEMP_BUFFER) {
      bool added = false;
      for (uint8_t j=0; j<count; j++) {
        if (t > ordered_temps[j]) {
          for (uint8_t x=count; x>j; x--) {
            ordered_temps[x] = ordered_temps[x-1];
            ordered_indexes[x] = ordered_indexes[x-1];
          }
          ordered_temps[j] = t;
          ordered_indexes[j] = i;
          added = true;
          break;
        }
      }
      if (!added) {
        ordered_temps[count] = t;
        ordered_indexes[count] = i;
      }
      count++;
    }
  }

  #define NOT_LEFT_EDGE ( idx % GRID_EXTENT > 0 )
  #define NOT_RIGHT_EDGE ( idx % GRID_EXTENT < GRID_EXTENT-1 )
  #define MIDDLE_LEFT ( pixels[idx-1] > calibrated_pixels[idx-1] + TEMP_BUFFER )
  #define TOP_LEFT ( idx > GRID_EXTENT && pixels[idx-(GRID_EXTENT+1)] > calibrated_pixels[idx-(GRID_EXTENT+1)] + TEMP_BUFFER )
  #define BOTTOM_LEFT ( idx < (AMG88xx_PIXEL_ARRAY_SIZE - GRID_EXTENT) && pixels[idx+(GRID_EXTENT-1)] > calibrated_pixels[idx+(GRID_EXTENT-1)] + TEMP_BUFFER )
  #define MIDDLE_RIGHT ( pixels[idx+1] > calibrated_pixels[idx+1] + TEMP_BUFFER )
  #define TOP_RIGHT ( idx > (GRID_EXTENT-1) && pixels[idx-(GRID_EXTENT-1)] > calibrated_pixels[idx-(GRID_EXTENT-1)] + TEMP_BUFFER )
  #define BOTTOM_RIGHT ( idx < (AMG88xx_PIXEL_ARRAY_SIZE - GRID_EXTENT) && pixels[idx+(GRID_EXTENT+1)] > calibrated_pixels[idx+(GRID_EXTENT+1)] + TEMP_BUFFER )
  #define TOP_MIDDLE ( idx > (GRID_EXTENT-1) && pixels[idx-GRID_EXTENT] > calibrated_pixels[idx-GRID_EXTENT] + TEMP_BUFFER )
  #define BOTTOM_MIDDLE ( idx < (AMG88xx_PIXEL_ARRAY_SIZE - GRID_EXTENT) && pixels[idx+GRID_EXTENT] > calibrated_pixels[idx+GRID_EXTENT] + TEMP_BUFFER )

  uint8_t total_masses = 0;
  uint8_t points[5];
  for(uint8_t i=0; i<count; i++){
    uint8_t idx = ordered_indexes[i];
    bool distinct = true;
    for (uint8_t j=0; j<i; j++) {
      if (distance(ordered_indexes[j], idx) <= MAX_DISTANCE) {
        distinct = false;
        break;
      }
    }
    if (distinct && (
      (NOT_LEFT_EDGE && (MIDDLE_LEFT || TOP_LEFT || BOTTOM_LEFT)) ||
      (NOT_RIGHT_EDGE && (MIDDLE_RIGHT || TOP_RIGHT || BOTTOM_RIGHT)) ||
      TOP_MIDDLE || BOTTOM_MIDDLE)) {
        points[total_masses] = idx;
        total_masses++;
      }
  }

  if (total_masses > 0) {
    for(int i = 0; i<total_masses; i++) {
      Serial.print(points[i]);
      Serial.print(", ");
    }
    Serial.println();

    Serial.print("[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      float curr = pixels[i-1];
      float last = calibrated_pixels[i-1];
      if (curr - last > TEMP_BUFFER) {
        Serial.print(curr);
      } else
        Serial.print("-----");
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    Serial.println();
  }
}

