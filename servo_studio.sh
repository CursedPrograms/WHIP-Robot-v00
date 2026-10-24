#!/bin/bash
# servo_studio.sh - WHIP Servo Studio (scripts/servo_studio.py), from the venv (made first if needed).
# Buddies: servo_studio.bat, servo_studio.ps1.   ./servo_studio.sh [--file gait_real.xml]
cd "$(dirname "$0")"
[ -x venv/bin/python ] || ./setup.sh || exit 1
exec venv/bin/python scripts/servo_studio.py "$@"
