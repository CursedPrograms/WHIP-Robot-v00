#!/bin/bash
# run.sh - starts WHIP's controller (setup first if there's no venv yet). Buddies: run.bat, run.ps1.
cd "$(dirname "$0")"
[ -x venv/bin/python ] || ./setup.sh || exit 1
exec venv/bin/python scripts/controller.py "$@"
