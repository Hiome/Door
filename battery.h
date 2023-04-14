#define FIRMWARE_VERSION        "V20.3.3"

void initialize() {
  LOWPOWER_DELAY(SLEEP_1S);
  publish(FIRMWARE_VERSION, HIOME_RETRY_COUNT*2);
}

void loop_frd() {
  LOWPOWER_DELAY(SLEEP_8S);
  beatHeart(450);
  checkForUpdates();
}
