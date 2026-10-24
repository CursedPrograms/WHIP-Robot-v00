#!/bin/bash
# flash_gaits.sh - compile and flash the gaits sketch (scripts/esp32_gaits). Buddies: flash_gaits.bat, flash_gaits.ps1.
#   ./flash_gaits.sh [/dev/ttyUSB0] [--fast] [--no-compile] [--compile-only]
exec "$(dirname "$0")/scripts/flash.sh" --folder esp32_gaits "$@"
