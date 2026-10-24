#!/bin/bash
# freeze_requirements.sh - writes requirements.txt from what's installed in venv/.
# Buddies: freeze_requirements.bat, freeze_requirements.ps1.
cd "$(dirname "$0")"
PY=python3
[ -x venv/bin/python ] && PY=venv/bin/python
echo "Generating requirements.txt from $PY..."
"$PY" -m pip freeze > requirements.txt
echo "requirements.txt written."
