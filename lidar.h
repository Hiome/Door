#define TIMEOUT_THRESHOLD    3000
#define CONFIDENCE_THRESHOLD 4
#define WAKE_CYCLES          500  // ~10 seconds
#define ERROR_MARGIN         10

// Learning rates
#define POWER       256
#define ALPHA       18UL    //  7% of 256
#define PADDING     230UL   // 90% of 256

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
uint8_t sensor1_timeouts = 0;
uint8_t sensor2_timeouts = 0;

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

  sensor1_average = count1 < 200 ? TIMEOUT_THRESHOLD : sum1/count1;
  sensor2_average = count2 < 200 ? TIMEOUT_THRESHOLD : sum2/count2;

  Sprintln("done.");
  Sprintln(sensor1_average);
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
    if (sensor1_average < TIMEOUT_THRESHOLD) {
      if (++sensor1_timeouts > CONFIDENCE_THRESHOLD) {
        sensor1_timeouts = 0;
        sensor1_average = TIMEOUT_THRESHOLD;
        return SENSOR_LOW;
      }
      Sprintln("sensor 1 error");
      return SENSOR_ERROR;
    }

    // otherwise treat it as sensor low
    return SENSOR_LOW;
  }

  sensor1_timeouts = 0;

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
    if (sensor2_average < TIMEOUT_THRESHOLD) {
      if (++sensor2_timeouts > CONFIDENCE_THRESHOLD) {
        sensor2_timeouts = 0;
        sensor2_average = TIMEOUT_THRESHOLD;
        return SENSOR_LOW;
      }
      Sprintln("sensor 2 error");
      return SENSOR_ERROR;
    }

    // otherwise treat it as sensor low
    return SENSOR_LOW;
  }

  sensor2_timeouts = 0;

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

uint8_t _start = 0;
uint8_t _end = 0;
uint8_t _prev = 0;
int8_t confidence = 0;

void reset_sensor() {
  if (_prev) Sprintln("-----");

  _start = 0;
  _end = 0;
  _prev = 0;
  confidence = 0;
}

void run_sensor() {
  uint8_t s1 = read_sensor1();
  uint8_t s2 = read_sensor2();

  if (s1 == SENSOR_ERROR || s2 == SENSOR_ERROR) {
    // let's just skip this cycle and try again.
    return;
  }

  uint8_t closer_sensor;
  uint8_t extra_confident = 1;
  if (s1 == s2) { // either no activity or in between both sensors
    if (s1 == SENSOR_LOW) { // there's no LIDAR activity
      if (_start && _end) {
        // activity just ended with enough data points.
        // _start and _end can be either 1 or 2 (see below)
        if (_start > _end) {
          // moved from sensor 2 to sensor 1
          publish("2-1");
        } else if (_start < _end) {
          // moved from sensor 1 to sensor 2
          publish("1-2");
        }
      }

      reset_sensor();
      return;
    }

    // we are in the middle of both lasers
    int16_t range_diff = sensor1_range - sensor2_range;
    if (abs(range_diff) < ERROR_MARGIN) {
      cyclesRemaining++;
      confidence = 0;
      return;
    }

    // try to guess what direction we're moving
    closer_sensor = (sensor1_range < sensor2_range ? 1 : 2);
    if (abs(range_diff) > 75) extra_confident++;
    Sprint("guessing direction ");
    Sprintln(closer_sensor);
  } else { // user is on one side or the other
    int16_t change = 0;
    if (s1) {
      closer_sensor = 1;
      if (sensor1_prev < TIMEOUT_THRESHOLD) {
        change = sensor1_range - sensor1_prev;
      }
    } else {
      closer_sensor = 2;
      if (sensor2_prev < TIMEOUT_THRESHOLD) {
        change = sensor2_range - sensor2_prev;
      }
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
    if (_start) {
      _end = closer_sensor;
      Sprint("## setting end ");
      Sprintln(_end);
    } else {
      _start = closer_sensor;
      Sprint("## setting start ");
      Sprintln(_start);
    }
  }
}

void loop() {
  if (motion) {
    cyclesRemaining = WAKE_CYCLES;
    motion = false;
  }

  if (--cyclesRemaining == 0) {
    reset_sensor();
    sensor1.stopContinuous();
    Sprintln("disabled sensor 1");
    sensor2.stopContinuous();
    Sprintln("disabled sensor 2");

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);

    sensor1.startContinuous();
    Sprintln("enabled sensor 1");
    sensor2.startContinuous();
    Sprintln("enabled sensor 2");
  }

  run_sensor();

  // prevent an overflow
  if (confidence > 100) {
    confidence = 10;
  } else if (confidence < -20) {
    confidence = -5;
  }
}

