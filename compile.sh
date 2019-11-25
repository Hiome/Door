#!/bin/bash

OUTPUT_DIR="/Users/neil/Code/core-firmware"

compileHex() {
  for i in {2..25}
  do
    echo "Compiling $2/$i.hex..."
    echo -e "#define NODEID ${i}\n${1}" > config.h
    arduino-cli compile --fqbn Moteino:avr:Moteino -o "$i.hex" presence.ino
    echo "Deleting ${i}.elf"
    rm "${i}.elf"
    echo "Moving ${i}.hex to $OUTPUT_DIR/$2/$i.hex"
    mv "${i}.hex" "$OUTPUT_DIR/$2/$i.hex"
    echo ""
  done
}

compileHex "#define THERMAL" "door/r2_alpha"
compileHex "#define THERMAL\n#define R3" "door/r3_alpha"
compileHex "#define THERMAL\n#define R3\n#define RECESSED" "door/r3_recessed"

echo "All done!"
