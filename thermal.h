#define FIRMWARE_VERSION     "V0.1"
#define MASS_SIZE            3

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

//int sort_desc(const void* cmp1, const void* cmp2) {
//  // Need to cast the void * to int *
//  float a = ((float*)cmp1)[2];
//  float b = ((float*)cmp2)[2];
//  return b - a;
//}

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

uint8_t distance(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  int8_t yd = y2 - y1;
  int8_t xd = x2 - x1;
  return abs(yd) + abs(xd);
}

uint8_t distanceBetween(uint8_t p1, uint8_t p2) {
//  uint8_t x1 = p1 % 8 + 1;
//  uint8_t y1 = p1/8 + 1;
//  uint8_t x2 = p2 % 8 + 1;
//  uint8_t y2 = p2/8 + 1;
  return distance(xcoordinates[p1], ycoordinates[p1], xcoordinates[p2], ycoordinates[p2]);
}

void loop() {
  amg.readPixels(pixels);

  uint8_t ordered_indexes[AMG88xx_PIXEL_ARRAY_SIZE];
  float ordered_temps[AMG88xx_PIXEL_ARRAY_SIZE];
  uint8_t count = 0;
  for(uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
    float t = pixels[i];
    if (t - calibrated_pixels[i] > 2) {
      bool added = false;
      for (uint8_t j=0; i<count; j++) {
        if (t > ordered_temps[j]) {
          for (uint8_t x=count-1; x>j; x--) {
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

  uint8_t total_masses = 0;
  uint8_t points[5];
  for(uint8_t i=0; i<count; i++){
    uint8_t idx = ordered_indexes[i];
    bool distinct = true;
    for (uint8_t j=0; j<i; j++) {
      if (distanceBetween(ordered_indexes[j], idx) < 2) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      uint8_t empty = 0;
      uint8_t avail = 0;
      if (i%8 > 0) {
        avail++;
        if (pixels[idx-1] <= calibrated_pixels[idx-1] + 2) empty++;
      }
      if (i > 7 && i%8 < 7) {
        avail++;
        if (pixels[idx-7] <= calibrated_pixels[idx-7] + 2) empty++;
      }
      if (i > 7) {
        avail++;
        if (pixels[idx-8] <= calibrated_pixels[idx-8] + 2) empty++;
      }
      if (i > 8 && i%8 > 0) {
        avail++;
        if (pixels[idx-9] <= calibrated_pixels[idx-9] + 2) empty++;
      }
      if (i%8 < 7) {
        avail++;
        if (pixels[idx+1] <= calibrated_pixels[idx+1] + 2) empty++;
      }
      if (i < 56 && i%8 < 7) {
        avail++;
        if (pixels[idx+9] <= calibrated_pixels[idx+9] + 2) empty++;
      }
      if (i < 56) {
        avail++;
        if (pixels[idx+8] <= calibrated_pixels[idx+8] + 2) empty++;
      }
      if (i < 56 && i%8 > 0) {
        avail++;
        if (pixels[idx+7] <= calibrated_pixels[idx+7] + 2) empty++;
      }
      if (empty*4 <= avail) {
        points[total_masses] = idx;
        total_masses++;
      }
    }
  }

  if (total_masses > 0) {
//    float weighted_points[AMG88xx_PIXEL_ARRAY_SIZE];
//    uint8_t total_points = 0;
//
//    for(uint8_t i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++){
//      if (masses[i] > 0) {
//        float m = 1.0;
//        for(uint8_t j=0; j<AMG88xx_PIXEL_ARRAY_SIZE; j++){
//          if (masses[j] > 0 && j != i) {
//            uint8_t x1 = i % 8 + 1;
//            uint8_t y1 = i/8 + 1;
//            uint8_t x2 = j % 8 + 1;
//            uint8_t y2 = j/8 + 1;
//            uint8_t d = distance(x1, y1, x2, y2);
//            m += 1.0/(d * d);
//          }
//        }
//        weighted_points[i] = m;
//        total_points++;
//      } else {
//        weighted_points[i] = 0;
//      }
//    }
//

//    uint8_t centers[total_masses];
//    uint8_t chosen_centers = 1;
//    centers[0] = ordered_indexes[0];
//    if (total_masses > 1) {
//      for(uint8_t i=1; i<total_points; i++) {
//        bool isDistinct = true;
//        for(uint8_t j=0; j<chosen_centers; j++){
//          uint8_t d = distanceBetween(centers[j], ordered_indexes[i]);
//          if (d < 2) {
//            isDistinct = false;
//            break;
//          }
//        }
//        if (isDistinct) {
//          centers[chosen_centers] = ordered_indexes[i];
//          chosen_centers++;
//        }
//        if (chosen_centers == total_masses) break;
//      }
//    }
    

    Serial.print("[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      float curr = pixels[i-1];
      float last = calibrated_pixels[i-1];
      if (curr - last > 2) {
        Serial.print(curr);
      } else
        Serial.print("-----");
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    Serial.println();
    Serial.println(total_masses);
  }
}

