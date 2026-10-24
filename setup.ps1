# setup.ps1 - makes venv\ and installs requirements.txt into it. Buddies: setup.bat, setup.sh.
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
if (-not (Test-Path "venv\Scripts\python.exe")) {
    Write-Host "Creating venv..."
    if (Get-Command py -ErrorAction SilentlyContinue) { py -3 -m venv venv } else { python -m venv venv }
}
& "venv\Scripts\python.exe" -m pip install --quiet --upgrade pip
& "venv\Scripts\python.exe" -m pip install -r requirements.txt
if ($LASTEXITCODE -ne 0) { exit 1 }
Write-Host "Ready: run.bat starts the controller."
