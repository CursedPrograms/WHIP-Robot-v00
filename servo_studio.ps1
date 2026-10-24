# servo_studio.ps1 - WHIP Servo Studio (scripts\servo_studio.py), from the venv (made first if needed).
# Buddies: servo_studio.bat, servo_studio.sh.   .\servo_studio.ps1 [--file gait_real.xml]
Set-Location $PSScriptRoot
if (-not (Test-Path "venv\Scripts\python.exe")) { & "$PSScriptRoot\setup.ps1"; if ($LASTEXITCODE -ne 0) { exit 1 } }
& "venv\Scripts\python.exe" "scripts\servo_studio.py" @args
exit $LASTEXITCODE
