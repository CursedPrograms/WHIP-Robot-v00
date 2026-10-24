# freeze_requirements.ps1 - writes requirements.txt from what's installed in venv\.
# Buddies: freeze_requirements.bat, freeze_requirements.sh.
Set-Location $PSScriptRoot
$py = if (Test-Path "venv\Scripts\python.exe") { "venv\Scripts\python.exe" } else { "python" }
Write-Host "Generating requirements.txt from $py..."
& $py -m pip freeze | Out-File -Encoding ascii requirements.txt
Write-Host "requirements.txt written."
