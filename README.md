# SmarterDoor

Hiome Sensor code.

## Setup

1. Install Arduino IDE (http://arduino.cc) and launch it

2. Grab all necessary code

```bash
cd ~/Documents/Arduino/libraries

git clone -b lowpower git@github.com:neilgupta/Adafruit_AMG88xx.git
git clone git@github.com:LowPowerLab/LowPower.git
git clone git@github.com:LowPowerLab/SPIFlash.git
git clone -b skipcansend git@github.com:hiome/RFM69-1.git # rename dir to RFM69

cd ..

git clone git@gitlab.com:hiome/RFM69Gateway.git
git clone git@gitlab.com:hiome/Door.git
```

2. Install moteino board by adding https://lowpowerlab.github.io/MoteinoCore/package_LowPowerLab_index.json to `Additional Board Manager URLs` in Arduino IDE preferences. See https://lowpowerlab.com/guide/moteino/programming-libraries/ for more info.

3. Install moteino board via Arduino board manager.

4. Plug in sensor and flash it!
