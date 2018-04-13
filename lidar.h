#define TIMEOUT_THRESHOLD    7000
#define CONFIDENCE_THRESHOLD 3

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

volatile uint8_t cyclesRemaining;
volatile boolean enable_sensor1;
volatile boolean enable_sensor2;
void motion1() {
  enable_sensor1 = true;
  cyclesRemaining = 125;
}
void motion2() {
  enable_sensor2 = true;
  cyclesRemaining = 125;
}

static uint16_t avg1 = 0;
static uint16_t avg2 = 0;
static uint16_t sensor1_range = 0;
static uint16_t sensor2_range = 0;
void calibrate() {
  blink(4); // give the user 2 seconds to get out of the way
  digitalWrite(LED, HIGH);

  Sprint("Calibrating for ~30 seconds... ");
  uint16_t range = 0;
  uint32_t sum1 = 0;
  uint32_t sum2 = 0;
  uint16_t count1 = 0;
  uint16_t count2 = 0;
  for (uint16_t i = 0; i < 750; i++) {
    range = sensor1.readRangeSingleMillimeters();
    if (range < TIMEOUT_THRESHOLD) {
      sum1 += range;
      count1++;
    }

    range = sensor2.readRangeSingleMillimeters();
    if (range < TIMEOUT_THRESHOLD) {
      sum2 += range;
      count2++;
    }

    delay(1); // needed to reset watchdog timer
  }

  avg1 = sum1/count1;
  avg2 = sum2/count2;

  Sprintln("done.");

  char avg_arr1[5];
  char avg_arr2[5];
  sprintf(avg_arr1, "%d", avg1);
  sprintf(avg_arr2, "%d", avg2);
  publish(avg_arr1);
  publish(avg_arr2);
}

void initialize() {
  pinMode(PIR1, INPUT);
  enableInterrupt(PIR1, motion1, RISING);
  pinMode(PIR2, INPUT);
  enableInterrupt(PIR2, motion2, RISING);

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

  sensor1.setMeasurementTimingBudget(20000);
  sensor2.setMeasurementTimingBudget(20000);

  calibrate();
  cyclesRemaining = 1;
  enable_sensor1 = true;
  enable_sensor2 = true;
}

#define SENSOR_LOW   0
#define SENSOR_HIGH  1
#define SENSOR_ERROR 2

uint8_t read_sensor1() {
  if (!enable_sensor1) {
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_ON);
    return SENSOR_ERROR;
  }

  uint16_t range = sensor1.readRangeSingleMillimeters();
  if (range > TIMEOUT_THRESHOLD) {
    Sprintln("sensor 1 error");
    return SENSOR_ERROR;
  }

  sensor1_range = range;
  static const uint16_t padded_avg = avg1 * 0.85;

  if (range < padded_avg) {
    Sprint("sensor1: ");
    Sprint(range);
    Sprint("/");
    Sprintln(avg1);
    return SENSOR_HIGH;
  } else {
    return SENSOR_LOW;
  }
}

uint8_t read_sensor2() {
  if (!enable_sensor2) {
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_ON);
    return SENSOR_ERROR;
  }

  uint16_t range = sensor2.readRangeSingleMillimeters();
  if (range > TIMEOUT_THRESHOLD) {
    Sprintln("sensor 2 error");
    return SENSOR_ERROR;
  }

  sensor2_range = range;
  static const uint16_t padded_avg = avg2 * 0.85;
  
  if (range < padded_avg) {
    Sprint("sensor2: ");
    Sprint(range);
    Sprint("/");
    Sprintln(avg2);
    return SENSOR_HIGH;
  } else {
    return SENSOR_LOW;
  }
}

static uint8_t _start = 0;
static uint8_t _end = 0;
static int8_t confidence = 0;

void reset_sensor() {
  _start = 0;
  _end = 0;
  confidence = 0;
}

void run_sensor() {
  uint8_t s1 = read_sensor1();
  uint8_t s2 = read_sensor2();

  if (s1 == SENSOR_HIGH && !enable_sensor2) {
    // motion detected, turn on the other sensor
    s2 = SENSOR_LOW;
    enable_sensor2 = true;
  } else if (s2 == SENSOR_HIGH && !enable_sensor1) {
    // motion detected, turn on the other sensor
    s1 = SENSOR_LOW;
    enable_sensor1 = true;
  } else if (s1 == SENSOR_ERROR || s2 == SENSOR_ERROR) {
    // at least one of the readings is definitely missing,
    // let's just skip this cycle and try again.
    confidence--;
    confidence = max(confidence, 0);
    return;
  }

  if (s1 == s2) {
    // either there is no activity or user is in the middle of both
    // sensors, so we know nothing about directional intent anyway.
    if (s1 == SENSOR_LOW && s2 == SENSOR_LOW) {
      // there's no LIDAR activity
      if (confidence >= CONFIDENCE_THRESHOLD) {
        // activity just ended with enough data points.
        // _start and _end can be either 1 or 2 (see below)
        if (_start > _end) {
          // moved from sensor 2 to sensor 1
          publish("2-1");
          Sprintln("published 2-1\n\n");
        } else if (_start < _end) {
          // moved from sensor 1 to sensor 2
          publish("1-2");
          Sprintln("published 1-2\n\n");
        }
      }

      reset_sensor();
      return;
    }

    // we are in the middle of both lasers
    confidence++;
    cyclesRemaining++;
    if (sensor1_range != sensor2_range) {
      // try to guess what direction we're moving
      uint8_t closer_sensor = (sensor1_range < sensor2_range ? 1 : 2);
      if (_start == 0) {
        // somehow we didn't pick up a starting side, let's guess it
        _start = closer_sensor;
        Sprint("guessing start ");
        Sprintln(_start);
      } else {
        // guess ending direction, in case we don't pick up the ending side
        _end = closer_sensor;
        Sprint("guessing direction ");
        Sprintln(_end);
      }
    }
    return;
  }

  // user is on one side or the other
  confidence++;
  cyclesRemaining++;
  // there is activity on one side or the other
  if (_start == 0) {
    // activity is just starting.
    _start = (s1 == 1 ? 1 : 2);
  } else {
    // activity is ongoing, track where it might end.
    _end = (s1 == 1 ? 1 : 2);
  }
}

void loop() {
  cyclesRemaining--;
  if (confidence > 100) {
    // prevent an overflow
    confidence = 10;
  }

  if (cyclesRemaining == 0) {
    reset_sensor();
    enable_sensor1 = false;
    enable_sensor2 = false;
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  }

  run_sensor();
}
