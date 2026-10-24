# launch_controller.ps1 - WHIP's controller, from the repo's venv (made on the first run).
# Buddies: launch_controller.bat, launch_controller.sh.
& "$PSScriptRoot\..\run.ps1" @args
exit $LASTEXITCODE
