# SmarterDoor

Hiome Sensor code.

## Setup

1. Install Arduino IDE (http://arduino.cc) and launch it

2. Grab all necessary code

```bash
cd ~/Documents/Arduino/libraries

git clone git@github.com:Hiome/LowPower.git
git clone git@github.com:Hiome/SPIFlash.git
git clone -b skipcansend git@github.com:Hiome/RFM69.git
git clone git@github.com:Hiome/Hiome_AMG88xx.git

cd ..

git clone git@github.com:Hiome/RFM69Gateway.git
git clone git@github.com:Hiome/Door.git
```

2. Install moteino board by adding https://lowpowerlab.github.io/MoteinoCore/package_LowPowerLab_index.json to `Additional Board Manager URLs` in Arduino IDE preferences. See https://lowpowerlab.com/guide/moteino/programming-libraries/ for more info.

3. Install moteino board via Arduino board manager.

4. Plug in sensor and flash it!

## Disclaimer

This code will **not** work as-is without the rest of the Hiome ecosystem. This is currently shared as a source of inspiration.

## License

All rights reserved. Copyright Hiome Inc, 2020.
