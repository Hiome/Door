#define TIMEOUT_THRESHOLD    7000
#define CONFIDENCE_THRESHOLD 5

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

uint8_t cyclesRemaining = 1;
boolean enabled_sensor1 = false;
boolean enabled_sensor2 = false;
uint16_t avg1 = 0;
uint16_t avg2 = 0;
uint16_t sensor1_range = 0;
uint16_t sensor2_range = 0;

volatile boolean motion_triggered = false;

void motion() {
  motion_triggered = true;
}

void calibrate() {
  blink(4); // give the user 2 seconds to get out of the way
  digitalWrite(LED, HIGH);

  Sprint("Calibrating for ~30 seconds... ");
  uint16_t range = 0;
  uint32_t sum1 = 0;
  uint32_t sum2 = 0;
  uint16_t count1 = 0;
  uint16_t count2 = 0;
  while (count1 < 500 || count2 < 500) {
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
  }

  avg1 = sum1/count1;
  avg2 = sum2/count2;

  Sprintln("done.");
}

void initialize() {
  pinMode(PIR0, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR0), motion, RISING);
//  pinMode(PIR1, INPUT);
//  enableInterruptFast(PIR1, RISING);
//  pinMode(PIR2, INPUT);
//  enableInterruptFast(PIR2, RISING);

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
}

#define SENSOR_LOW   0
#define SENSOR_HIGH  1
#define SENSOR_ERROR 2

uint8_t read_sensor1() {
  if (!enabled_sensor1) {
    return SENSOR_ERROR;
  }

  static const uint16_t padded_avg = avg1 * 0.85;

  sensor1_range = sensor1.readRangeContinuousMillimeters();

  if (sensor1_range > TIMEOUT_THRESHOLD) {
    Sprintln("sensor 1 error");
    return SENSOR_ERROR;
  }

  if (sensor1_range < padded_avg) {
    Sprint("sensor1: ");
    Sprint(sensor1_range);
    Sprint("/");
    Sprintln(avg1);
    return SENSOR_HIGH;
  } else {
    return SENSOR_LOW;
  }
}

uint8_t read_sensor2() {
  if (!enabled_sensor2) {
    return SENSOR_ERROR;
  }

  static const uint16_t padded_avg = avg2 * 0.85;

  sensor2_range = sensor2.readRangeContinuousMillimeters();

  if (sensor2_range > TIMEOUT_THRESHOLD) {
    Sprintln("sensor 2 error");
    return SENSOR_ERROR;
  }

  if (sensor2_range < padded_avg) {
    Sprint("sensor2: ");
    Sprint(sensor2_range);
    Sprint("/");
    Sprintln(avg2);
    return SENSOR_HIGH;
  } else {
    return SENSOR_LOW;
  }
}

uint8_t _start = 0;
uint8_t _end = 0;
int8_t confidence = 0;

void reset_sensor() {
  _start = 0;
  _end = 0;
  confidence = 0;
}

void enable_sensor1() {
  sensor1.startContinuous();
  enabled_sensor1 = true;
  Sprintln("enabled sensor 1");
}
void enable_sensor2() {
  sensor2.startContinuous();
  enabled_sensor2 = true;
  Sprintln("enabled sensor 2");
}
void disable_sensor1() {
  sensor1.stopContinuous();
  enabled_sensor1 = false;
  Sprintln("disabled sensor 1");
}
void disable_sensor2() {
  sensor2.stopContinuous();
  enabled_sensor2 = false;
  Sprintln("disabled sensor 2");
}

void run_sensor() {
  uint8_t s1 = read_sensor1();
  uint8_t s2 = read_sensor2();

  if (s1 == SENSOR_HIGH && !enabled_sensor2) {
    s2 = SENSOR_LOW;
    enable_sensor2();
  } else if (s2 == SENSOR_HIGH && !enabled_sensor1) {
    s1 = SENSOR_LOW;
    enable_sensor1();
  } else if (s1 == SENSOR_ERROR || s2 == SENSOR_ERROR) {
    // at least one of the readings is definitely missing,
    // let's just skip this cycle and try again.
    confidence = max(confidence - 1, 0);
    return;
  }

  if (s1 == s2) {
    // either there is no activity or user is in the middle of both
    // sensors, so we know nothing about directional intent anyway.
    if (s1 == SENSOR_LOW) {
      // there's no LIDAR activity
      if (confidence >= CONFIDENCE_THRESHOLD) {
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

  if (cyclesRemaining == 0 /*&& motion1_triggered == 0 && motion2_triggered == 0*/) {
    reset_sensor();
    disable_sensor1();
    disable_sensor2();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  }
  
//  if (motion1_triggered > 0) {
  if (motion_triggered) {
    if (!enabled_sensor1) {
      enable_sensor1();
    }
    if (!enabled_sensor2) {
      enable_sensor2();
    }
    cyclesRemaining = 250;
//    motion1_triggered = 0;
    motion_triggered = false;
  }
//  if (motion2_triggered > 0) {
//    if (!enabled_sensor2) {
//      enable_sensor2();
//    }
//    cyclesRemaining = 250;
//    motion2_triggered = 0;
//  }

  run_sensor();
}
