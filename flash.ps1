# flash.ps1 - compile and flash WHIP's ESP32 (the work is in scripts\flash.ps1). Buddies: flash.bat, flash.sh.
#   .\flash.ps1 [COM6] [-Fast] [-NoCompile] [-CompileOnly]
& "$PSScriptRoot\scripts\flash.ps1" @args
exit $LASTEXITCODE
