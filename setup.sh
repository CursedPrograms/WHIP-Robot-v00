#!/bin/bash
# setup.sh - makes venv/ and installs requirements.txt into it. Buddies: setup.bat, setup.ps1.
set -e
cd "$(dirname "$0")"
if [ ! -x venv/bin/python ]; then
    echo "Creating venv..."
    python3 -m venv venv
fi
venv/bin/python -m pip install --quiet --upgrade pip
venv/bin/python -m pip install -r requirements.txt
echo "Ready: ./run.sh starts the controller."
