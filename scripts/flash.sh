#!/bin/bash
# flash.sh - compile and flash WHIP's ESP32 on Linux / macOS, talking you through it
# (out loud too, when spd-say or say is there). The Linux / macOS buddy of flash.ps1;
# run it with ../flash.sh or ../flash_gaits.sh.
#
#   flash.sh                      compile (only what changed) + flash, port found automatically
#   flash.sh /dev/ttyUSB0         same, on that port
#   flash.sh --fast               flash at 460800 baud (default 115200, which survives long cables)
#   flash.sh --no-compile         flash the last build (also skipped by itself when nothing changed)
#   flash.sh --compile-only       just build
#   flash.sh --folder esp32_gaits another sketch folder in scripts/ (flash_gaits.sh does this)
#
# The build is kept in build/<folder> (gitignored), so only changed files are rebuilt.

# ---- what to flash (the only lines that differ between robots) ----
ROBOT="WHIP"
FQBN="esp32:esp32:esp32:PartitionScheme=huge_app"   # his sketch needs the 3 MB app partition
BEFORE="Unplug the servo controller's wires from RX and TX on the ESP32 first."   # they share the USB flashing pins
LIBS=("IRremote" "Adafruit NeoPixel")
# --------------------------------------------------------------------

CLI_VERSION="1.1.1"
CORE_VERSION="3.3.12"   # the core every DREAM Robotics ESP32 sketch is built and tested with
ESP_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
PORT="" FAST=0 NO_COMPILE=0 COMPILE_ONLY=0 FOLDER="esp32"
while [ $# -gt 0 ]; do
    case "$1" in
        --fast) FAST=1 ;;
        --no-compile) NO_COMPILE=1 ;;
        --compile-only) COMPILE_ONLY=1 ;;
        --folder) FOLDER="$2"; shift ;;
        -*) echo "unknown option $1"; exit 1 ;;
        *) PORT="$1" ;;
    esac
    shift
done
SKETCH="$HERE/$FOLDER"
BUILD="$REPO/build/$FOLDER"

say() {
    echo ">> $*"
    if command -v spd-say >/dev/null 2>&1; then spd-say -w "$*" 2>/dev/null
    elif command -v say >/dev/null 2>&1; then say "$*" 2>/dev/null
    fi
}

# ---- tools: arduino-cli (PATH, or downloaded once), the ESP32 core and the libraries ----
CLI="$(command -v arduino-cli || true)"
[ -z "$CLI" ] && [ -x "$HOME/.local/bin/arduino-cli" ] && CLI="$HOME/.local/bin/arduino-cli"
if [ -z "$CLI" ]; then
    say "I'll download the Arduino command line tool first. Just this once."
    mkdir -p "$HOME/.local/bin"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh |
        BINDIR="$HOME/.local/bin" sh -s "$CLI_VERSION" || { say "Downloading it failed."; exit 1; }
    CLI="$HOME/.local/bin/arduino-cli"
fi
if ! "$CLI" core list | grep -q "^esp32:esp32"; then
    say "Installing the ESP32 board package. It's big, so this takes a few minutes, once."
    "$CLI" core update-index --additional-urls "$ESP_URL"
    "$CLI" core install "esp32:esp32@$CORE_VERSION" --additional-urls "$ESP_URL" ||
        { say "Installing the ESP32 board package failed. Check the internet connection."; exit 1; }
fi
for lib in "${LIBS[@]}"; do
    if ! "$CLI" lib list | grep -q "^$lib "; then
        say "Installing the $lib library."
        "$CLI" lib install "$lib" || { say "Installing $lib failed."; exit 1; }
    fi
done

# ---- port: USB serial ports only ----
if [ -z "$PORT" ] && [ $COMPILE_ONLY -eq 0 ]; then
    PORTS=()
    for p in /dev/ttyUSB* /dev/ttyACM* /dev/cu.usbserial* /dev/cu.SLAB_USBtoUART* /dev/cu.wchusbserial*; do
        [ -e "$p" ] && PORTS+=("$p")
    done
    if [ ${#PORTS[@]} -eq 1 ]; then PORT="${PORTS[0]}"
    elif [ ${#PORTS[@]} -eq 0 ]; then say "No board is plugged in. Connect $ROBOT's ESP32 by USB."; exit 1
    else say "More than one board is plugged in. Run it with the port."; echo "Ports: ${PORTS[*]}"; exit 1
    fi
fi
[ -n "$PORT" ] && echo "Port: $PORT"

# ---- servo_config.json -> the sketch's servo_config.h (servo_config.py writes it only if it changed) ----
if [ -f "$HERE/servo_config.py" ] && [ -f "$SKETCH/servo_config.h" ]; then
    PY="$REPO/venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3)"
    [ -n "$PY" ] && "$PY" "$HERE/servo_config.py"
fi

# ---- compile (incremental), or skip it when nothing changed since the last build ----
BIN="$BUILD/$FOLDER.ino.merged.bin"
if [ $NO_COMPILE -eq 0 ] && [ $COMPILE_ONLY -eq 0 ] && [ -f "$BIN" ] &&
   [ -z "$(find "$SKETCH" -type f \( -name '*.ino' -o -name '*.h' -o -name '*.cpp' -o -name '*.c' \) -newer "$BIN")" ]; then
    say "Nothing changed since the last build, so I'll flash that."
    NO_COMPILE=1
fi
if [ $NO_COMPILE -eq 0 ]; then
    say "Compiling $ROBOT."
    START=$SECONDS
    "$CLI" compile --fqbn "$FQBN" --build-path "$BUILD" "$SKETCH" || { say "Compile failed. Check the errors on screen."; exit 1; }
    say "Compiled in $((SECONDS - START)) seconds."
fi
[ $COMPILE_ONLY -eq 1 ] && exit 0
[ -f "$BIN" ] || { say "There's no build to flash yet. Run it without no compile."; exit 1; }

# ---- flash; retry until it works or you press Ctrl+C ----
ESPTOOL="$(ls -d "$HOME"/.arduino15/packages/esp32/tools/esptool_py/*/esptool \
                 "$HOME"/Library/Arduino15/packages/esp32/tools/esptool_py/*/esptool 2>/dev/null | sort -V | tail -1)"
[ -n "$ESPTOOL" ] || { say "I can't find esptool in the ESP32 board package."; exit 1; }
BAUD=115200
[ $FAST -eq 1 ] && BAUD=460800
[ -n "$BEFORE" ] && say "$BEFORE"
echo "If it doesn't connect by itself: hold BOOT, tap EN, then let go of BOOT."
until "$ESPTOOL" --chip esp32 --port "$PORT" --baud "$BAUD" --before default-reset --after hard-reset \
        write-flash -z 0x0 "$BIN"; do
    if [ ! -e "$PORT" ]; then say "I can't open $PORT. Is the board plugged in?"; exit 1; fi
    say "Flash failed. I'll try again."
    sleep 2
done
say "Flash done. $ROBOT is restarting."
