#define DOOR_CLOSED 0
#define DOOR_OPEN   1
#define DOOR_AJAR   2

uint8_t previous_door_state = DOOR_CLOSED;
uint8_t door_state = DOOR_OPEN;
uint8_t last_published_door_state = 9; // initialize to something invalid to force first loop
uint8_t frames_since_door_open = 0;
uint8_t ajar_side = 1;

uint8_t readDoorState() {
  #ifdef RECESSED
    bool reed3High = PIND & 0b00001000; // true if reed 3 is high (normal state)
    bool reed4High = PIND & 0b00010000; // true if reed 4 is high (normal state)
    if (reed3High && reed4High) return DOOR_OPEN;
    if (!reed3High && !reed4High) return DOOR_CLOSED;
    ajar_side = reed4High ? 2 : 1;
    return DOOR_AJAR;
  #else
    if (PIND & 0b00001000) {  // true if reed 3 is high (normal state)
      // door open if reed switch 4 is also high
      return PIND & 0b00010000 ? DOOR_OPEN : DOOR_AJAR;
    } else {
      return DOOR_CLOSED;
    }
  #endif
}

bool checkDoorState() {
  uint8_t last_door_state = door_state;
  door_state = readDoorState();

  if (door_state != last_published_door_state && hiome.publish(
    (door_state == DOOR_CLOSED ? "d0" : (door_state == DOOR_OPEN ? "d1" : "d2")), "0", 0)) {
    last_published_door_state = door_state;
  }

  if (last_door_state != door_state) {
    previous_door_state = last_door_state;
    frames_since_door_open = 0;
    return true;
  }

  return false;
}
