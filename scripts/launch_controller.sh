#!/bin/bash
# launch_controller.sh - WHIP's controller, from the repo's venv (made on the first run).
# Buddies: launch_controller.bat, launch_controller.ps1.
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../run.sh" "$@"
