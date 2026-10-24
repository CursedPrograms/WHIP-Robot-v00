#!/bin/bash
# flash.sh - compile and flash WHIP's ESP32 on Linux / macOS (the work is in scripts/flash.sh).
# Buddies: flash.bat, flash.ps1.
#   ./flash.sh [/dev/ttyUSB0] [--fast] [--no-compile] [--compile-only]
exec "$(dirname "$0")/scripts/flash.sh" "$@"
