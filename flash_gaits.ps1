# flash_gaits.ps1 - compile and flash the gaits sketch (scripts\esp32_gaits). Buddies: flash_gaits.bat, flash_gaits.sh.
#   .\flash_gaits.ps1 [COM6] [-Fast] [-NoCompile] [-CompileOnly]
& "$PSScriptRoot\scripts\flash.ps1" -Folder esp32_gaits @args
exit $LASTEXITCODE
