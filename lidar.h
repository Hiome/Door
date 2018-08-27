#define FIRMWARE_VERSION     "V0.2"
#define TIMEOUT_THRESHOLD    3000
#define CONFIDENCE_THRESHOLD 3
#define WAKE_CYCLES          500  // ~10 seconds
#define ERROR_MARGIN         10

// Learning rates
#define POWER       256
#define ALPHA       18UL    //  7% of 256
#define PADDING     218UL   // 85% of 256

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

volatile boolean motion = false;
uint16_t cyclesRemaining = 1;
uint16_t sensor1_average = 0;
uint16_t sensor2_average = 0;
uint16_t sensor1_range = 0;
uint16_t sensor2_range = 0;
uint16_t sensor1_prev = 0;
uint16_t sensor2_prev = 0;
uint8_t start = 0;
uint8_t _prev = 0;
uint8_t directions = 0;
uint8_t confidence = 0;

void reset_sensor() {
  start = 0;
  _prev = 0;
  directions = 0;
  confidence = 0;
}

void motionISR() {
  motion = true;
}

void calibrate() {
  blink(4); // give the user 2 seconds to get out of the way
  digitalWrite(LED, HIGH);

  Sprint("Calibrating... ");
  uint16_t range = 0;
  uint32_t sum1 = 0;
  uint32_t sum2 = 0;
  uint16_t count1 = 0;
  uint16_t count2 = 0;
  sensor1.startContinuous();
  sensor2.startContinuous();

  for(uint16_t i = 0; i < 300; i++) {
    range = sensor1.readRangeContinuousMillimeters();
    if (range < TIMEOUT_THRESHOLD) {
      sum1 += range;
      count1++;
    }

    range = sensor2.readRangeContinuousMillimeters();
    if (range < TIMEOUT_THRESHOLD) {
      sum2 += range;
      count2++;
    }

    delay(1); // needed to reset watchdog timer
  }

  sensor1_average = count1 < 100 ? TIMEOUT_THRESHOLD : sum1/count1;
  sensor2_average = count2 < 100 ? TIMEOUT_THRESHOLD : sum2/count2;

  Sprintln("done.");
  Sprint("sensor 1 average: ");
  Sprintln(sensor1_average);
  Sprint("sensor 2 average: ");
  Sprintln(sensor2_average);
}

void initialize() {
  pinMode(PIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR), motionISR, RISING);

  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);

  Wire.begin();

  Sprintln("Initializing Sensor 1...");
  pinMode(xshut1, INPUT);
  delay(150);
  sensor1.init();
  delay(100);
  sensor1.setAddress((uint8_t)22);
  
  Sprintln("Initializing Sensor 2...");
  pinMode(xshut2, INPUT);
  delay(150);
  sensor2.init();
  delay(100);
  sensor2.setAddress((uint8_t)25);

  sensor1.setMeasurementTimingBudget(30000);
  sensor2.setMeasurementTimingBudget(30000);

  calibrate();
}

#define SENSOR_LOW   0
#define SENSOR_HIGH  1
#define SENSOR_ERROR 2

uint8_t read_sensor1() {
  sensor1_prev  = sensor1_range;
  sensor1_range = sensor1.readRangeContinuousMillimeters();

  if (sensor1_range > TIMEOUT_THRESHOLD) {
    if (sensor1_average < TIMEOUT_THRESHOLD &&
        sensor1_prev < (PADDING * sensor1_average / POWER)) {
      Sprintln("sensor 1 error");
      return SENSOR_ERROR;
    }

    // otherwise treat it as sensor low
    return SENSOR_LOW;
  }

  if (sensor1_range < (PADDING * sensor1_average / POWER)) {
    Sprint("sensor1: ");
    Sprint(sensor1_range);
    Sprint("/");
    Sprintln(sensor1_average);
    return SENSOR_HIGH;
  }

  int16_t diff = sensor1_range - sensor1_average;
  sensor1_average += (ALPHA * diff / POWER);
  return SENSOR_LOW;
}

uint8_t read_sensor2() {
  sensor2_prev  = sensor2_range;
  sensor2_range = sensor2.readRangeContinuousMillimeters();

  if (sensor2_range > TIMEOUT_THRESHOLD) {
    if (sensor2_average < TIMEOUT_THRESHOLD &&
        sensor2_prev < (PADDING * sensor2_average / POWER)) {
      Sprintln("sensor 2 error");
      return SENSOR_ERROR;
    }

    // otherwise treat it as sensor low
    return SENSOR_LOW;
  }

  if (sensor2_range < (PADDING * sensor2_average / POWER)) {
    Sprint("sensor2: ");
    Sprint(sensor2_range);
    Sprint("/");
    Sprintln(sensor2_average);
    return SENSOR_HIGH;
  }

  int16_t diff = sensor2_range - sensor2_average;
  sensor2_average += (ALPHA * diff / POWER);
  return SENSOR_LOW;
}

void process_data() {
  if (directions) {
    if (directions % 2 == 0) {
      // activity just ended with enough data points.
      publish((char*)(start == 1 ? "2" : "1"));
      if (directions > 3) {
        // it's possible (albeit unlikely) multiple people went through,
        // so send a motion alert too so Hiome knows something happened
        publish((char*)(start == 1 ? "m2" : "m1"));
      }
    } else {
      // we never made it to a different side,
      // so send motion alert for starting side
      publish((char*)(start == 1 ? "m1" : "m2"));
    }
  }
  reset_sensor();
}

void run_sensor() {
  uint8_t s1 = read_sensor1();
  uint8_t s2 = read_sensor2();

  if (s1 == SENSOR_ERROR || s2 == SENSOR_ERROR) {
    if (s1 == s2) return; // both are errors, give up
    if (s1 == SENSOR_ERROR && (_prev == 1 || s2 == SENSOR_LOW)) {
      // give up if the current direction sensor failed or if other one is known low
      return;
    }
    if (s2 == SENSOR_ERROR && (_prev == 2 || s1 == SENSOR_LOW)) {
      // give up if the current direction sensor failed or if other one is known low
      return;
    }
  }

  uint8_t closer_sensor;
  uint8_t extra_confident = 1;
  if (s1 == s2) { // either no activity or in between both sensors
    if (s1 == SENSOR_LOW) { // there's no LIDAR activity
      process_data();
      return;
    }

    // we are in the middle of both lasers
    int16_t range_diff = sensor1_range - sensor2_range;
    range_diff = abs(range_diff);
    if (range_diff < ERROR_MARGIN) {
      cyclesRemaining++;
      return;
    }

    // try to guess what direction we're moving
    closer_sensor = (sensor1_range < sensor2_range ? 1 : 2);
    if (range_diff > 75) extra_confident++;
    Sprint("guessing direction ");
    Sprintln(closer_sensor);
  } else { // user is on one side or the other
    int16_t change = 0;
    if (s1 == SENSOR_HIGH) {
      closer_sensor = 1;
      if (sensor1_prev < TIMEOUT_THRESHOLD) {
        change = sensor1_range - sensor1_prev;
      }
      if (s2 == SENSOR_ERROR) extra_confident--;
    } else if (s2 == SENSOR_HIGH) {
      closer_sensor = 2;
      if (sensor2_prev < TIMEOUT_THRESHOLD) {
        change = sensor2_range - sensor2_prev;
      }
      if (s1 == SENSOR_ERROR) extra_confident--;
    }
    // if subject is not moving, burn down cyclesRemaining so
    // we eventually go to sleep if subject continues to not move
    abs(change) < ERROR_MARGIN ? cyclesRemaining-- : extra_confident++;
  }

  cyclesRemaining++;
  if (closer_sensor == _prev) {
    confidence += extra_confident;
  } else {
    _prev = closer_sensor;
    confidence = extra_confident;
  }

  if (confidence >= CONFIDENCE_THRESHOLD) {
    if (start) {
      if ((closer_sensor == start && directions % 2 == 0) ||
          (closer_sensor != start && directions % 2 == 1)) {
        directions++;
        Sprint("## setting end ");
        Sprintln(closer_sensor);
      }
    } else {
      start = closer_sensor;
      directions = 1;
      Sprint("## setting start ");
      Sprintln(closer_sensor);
    }
  }
}

void loop() {
  if (motion) {
    cyclesRemaining = WAKE_CYCLES;
    motion = false;
  }

  if (--cyclesRemaining == 0) {
    sensor1.stopContinuous();
    Sprintln("disabled sensor 1");
    sensor2.stopContinuous();
    Sprintln("disabled sensor 2");

    // if we have any data here, it's because of a door.
    // drop last direction and check data one last time
    if (directions) directions--;
    process_data();

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);

    sensor1.startContinuous();
    Sprintln("enabled sensor 1");
    sensor2.startContinuous();
    Sprintln("enabled sensor 2");
  }

  run_sensor();
}

