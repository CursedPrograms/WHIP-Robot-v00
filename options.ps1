# options.ps1 - the options menu for this bot (options.py edits config.json). Buddies: options.bat, options.sh.
# Uses the bot's own venv when there is one, and installs pygame into it if missing.
Set-Location $PSScriptRoot
$py = @("venv", "venv311", ".venv", "scripts\.venv") | ForEach-Object { "$_\Scripts\python.exe" } |
    Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $py) { $py = "python" }
& $py -c "import pygame" 2>$null
if ($LASTEXITCODE -ne 0) { & $py -m pip install pygame }
& $py options.py @args
exit $LASTEXITCODE
