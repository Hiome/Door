#define TIMEOUT_THRESHOLD    7000
#define CONFIDENCE_THRESHOLD 3
#define WAKE_CYCLES          500  // ~10 seconds

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

volatile boolean motion_triggered = false;
uint16_t cyclesRemaining = 1;
uint16_t avg1 = 0;
uint16_t avg2 = 0;
uint16_t sensor1_range = 0;
uint16_t sensor2_range = 0;

void motion() {
  motion_triggered = true;
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
  while (count1 < 500 || count2 < 500) {
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

  avg1 = sum1/count1;
  avg2 = sum2/count2;

  Sprintln("done.");
}

void initialize() {
  pinMode(PIR0, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR0), motion, RISING);

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
uint8_t _prev = 0;
int8_t confidence = 0;

void reset_sensor() {
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
    confidence--;
    return;
  }

  uint8_t closer_sensor;
  if (s1 == s2) {
    // either there is no activity or user is in the middle of both
    // sensors, so we know nothing about directional intent anyway.
    if (s1 == SENSOR_LOW) {
      // there's no LIDAR activity
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
    if (sensor1_range == sensor2_range) {
      cyclesRemaining++;
      return;
    }

    // try to guess what direction we're moving
    closer_sensor = (sensor1_range < sensor2_range ? 1 : 2);
    Sprint("guessing direction ");
    Sprintln(closer_sensor);
  } else {
    // user is on one side or the other
    closer_sensor = (s1 == SENSOR_HIGH ? 1 : 2);
  }

  cyclesRemaining++;
  if (closer_sensor == _prev) {
    confidence++;
  } else {
    _prev = closer_sensor;
    confidence = 1;
  }

  if (confidence == CONFIDENCE_THRESHOLD) {
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
  if (motion_triggered) {
    cyclesRemaining = WAKE_CYCLES;
    motion_triggered = false;
  }

//  if (--cyclesRemaining == 0) {
//    reset_sensor();
//    sensor1.stopContinuous();
//    Sprintln("disabled sensor 1");
//    sensor2.stopContinuous();
//    Sprintln("disabled sensor 2");
//
//    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
//
//    sensor1.startContinuous();
//    Sprintln("enabled sensor 1");
//    sensor2.startContinuous();
//    Sprintln("enabled sensor 2");
//  }

  run_sensor();

  // prevent an overflow
  if (confidence > 100) {
    confidence = 10;
  } else if (confidence < -20) {
    confidence = -5;
  }
}
