# SmarterDoor

Hiome Sensor code.

## Setup

1. Install Arduino IDE (http://arduino.cc) and launch it

2. Grab all necessary code

```bash
cd ~/Documents/Arduino/libraries

git clone git@github.com:Hiome/LowPower.git
git clone git@github.com:Hiome/SPIFlash.git
git clone -b skipcansend git@github.com:hiome/RFM69.git
git clone git@gitlab.com:hiome/Adafruit_AMG88xx.git

cd ..

git clone git@gitlab.com:hiome/RFM69Gateway.git
git clone git@gitlab.com:hiome/Door.git
```

2. Install moteino board by adding https://lowpowerlab.github.io/MoteinoCore/package_LowPowerLab_index.json to `Additional Board Manager URLs` in Arduino IDE preferences. See https://lowpowerlab.com/guide/moteino/programming-libraries/ for more info.

3. Install moteino board via Arduino board manager.

4. Plug in sensor and flash it!


## AVRil

To compile code, install arduino-cli

```
brew install arduino-cli

arduino-cli core update-index --additional-urls https://lowpowerlab.github.io/MoteinoCore/package_LowPowerLab_index.json

arduino-cli core install arduino:avr
```

Then run `bin/avril verify` from this directory to make sure it all worked.

When you're ready to generate hex files, run `bin/avril compile`. Make sure `OUTPUT_DIR` is set correctly in script.
