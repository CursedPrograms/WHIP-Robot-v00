# run.ps1 - starts WHIP's controller (setup first if there's no venv yet). Buddies: run.bat, run.sh.
Set-Location $PSScriptRoot
if (-not (Test-Path "venv\Scripts\python.exe")) { & "$PSScriptRoot\setup.ps1"; if ($LASTEXITCODE -ne 0) { exit 1 } }
& "venv\Scripts\python.exe" "scripts\controller.py" @args
exit $LASTEXITCODE
