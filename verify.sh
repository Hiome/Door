#!/bin/bash

arduino-cli compile --fqbn Moteino:avr:Moteino --warnings "all" presence.ino

echo ""
echo "✅ Verified on `date`"
echo ""
