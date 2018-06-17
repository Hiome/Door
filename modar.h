#define MOTION_THRESHOLD   5000

volatile boolean motion_woke;
volatile uint8_t next_motion;
volatile uint8_t motion0_order;
volatile uint8_t motion1_order;
volatile uint8_t motion2_order;
void motion0() {
  motion0_order = next_motion;
  next_motion++;
  motion_woke = true;
  Sprintln("m0");
}
void motion1() {
  motion1_order = next_motion;
  next_motion++;
  motion_woke = true;
  Sprintln("m1");
}
void motion2() {
  motion2_order = next_motion;
  next_motion++;
  motion_woke = true;
  Sprintln("m2");
}

static unsigned long motion_woke_at;
void reset_sensor() {
  next_motion = 1;
  motion_woke = false;
  motion_woke_at = null;
  motion0_order = null;
  motion1_order = null;
  motion2_order = null;
  Sprintln("reset");
}

void initialize() {
  pinMode(PIR0, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR0), motion0, RISING);
  pinMode(PIR1, INPUT);
  enableInterrupt(PIR1, motion1, RISING);
  pinMode(PIR2, INPUT);
  enableInterrupt(PIR2, motion2, RISING);
  reset_sensor();
}

void read_sensors() {
  // if all 3 motion sensors picked up motion,
  // guess what happened based on the order
  // of the motion sensor triggers
  if (motion0_order && motion1_order && motion2_order) {
    Sprintln("reading motion sensors...");
    Sprintln(motion1_order);
    Sprintln(motion0_order);
    Sprintln(motion2_order);

    uint8_t _start = 0;
    uint8_t _end = 0;

    if (motion1_order < motion2_order) {
      _start = 1;
    } else if (motion2_order < motion1_order) {
      _start = 2;
    }
    if (motion1_order > motion0_order) {
      _end = 1;
    } else if (motion2_order > motion0_order) {
      _end = 2;
    }

    if (_start == 0 || _end == 0) {
      reset_sensor();
      return;
    }

    if (_start < _end) {
      publish("1-2");
    } else if (_start > _end) {
      publish("2-1");
    }

    reset_sensor();
  }
}

void loop() {
  read_sensors();
  if (motion_woke) {
    motion_woke_at = millis();
    motion_woke = false;
  } else if (motion_woke_at && millis() - motion_woke_at > MOTION_THRESHOLD) {
    Sprintln(millis());
    Sprintln(motion_woke_at);
    reset_sensor();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  }
}

