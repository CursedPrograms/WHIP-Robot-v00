#!/bin/bash
# options.sh - the options menu for this bot (options.py edits config.json). Buddies: options.bat, options.ps1.
# Uses the bot's own venv when there is one, and installs pygame into it if missing.
cd "$(dirname "$0")"
PY=python3
for v in venv venv311 .venv scripts/.venv; do
    if [ -x "$v/bin/python" ]; then PY="$v/bin/python"; break; fi
done
"$PY" -c "import pygame" 2>/dev/null || "$PY" -m pip install pygame
exec "$PY" options.py "$@"
