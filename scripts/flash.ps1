# flash.ps1 - compile and flash WHIP's ESP32, talking you through it out loud
# so you can keep your hands and eyes on the board. Run it with flash.bat.
#
#   flash.bat            compile (only what changed) + flash, port found automatically
#   flash.bat COM6       same, on COM6
#   flash.bat -Fast      flash at 460800 baud (default 115200, which survives long cables)
#   flash.bat -NoCompile flash the last build without compiling (it also skips the
#                        compile by itself when no sketch file changed since the last build)
#   flash.bat -CompileOnly  just build (fast after the first time)
#
# The build is kept in build\esp32 (gitignored), so after the first compile
# only the files you changed are rebuilt - unlike the Arduino IDE, which
# starts from scratch in a temp folder before every upload.
param(
    [string]$Port = "",
    [switch]$Fast,
    [switch]$NoCompile,
    [switch]$CompileOnly
)

# ---- what to flash (the only lines that differ between robots) ----
$Robot  = "WHIP"
$Sketch = Join-Path $PSScriptRoot "esp32"
$Fqbn   = "esp32:esp32:esp32"   # default partitions
$Before = ""   # anything to say before flashing ("" = nothing)
# --------------------------------------------------------------------

$ErrorActionPreference = "Stop"
$Repo  = Split-Path $PSScriptRoot -Parent
$Build = Join-Path $Repo "build\esp32"

Add-Type -AssemblyName System.Speech
$voice = New-Object System.Speech.Synthesis.SpeechSynthesizer
function Say([string]$text) { Write-Host ">> $text" -ForegroundColor Cyan; $voice.Speak($text) }

# ---- tools: arduino-cli from the Arduino IDE (or PATH), esptool from the ESP32 core ----
$cli = @(
    "$env:ProgramFiles\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
    "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
) | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $cli) { $cli = (Get-Command arduino-cli -ErrorAction SilentlyContinue).Source }
if (-not $cli) { Say "I can't find the Arduino IDE. Install it first."; exit 1 }

$esptool = Get-ChildItem "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\*\esptool.exe" -ErrorAction SilentlyContinue |
    Sort-Object { [version]($_.Directory.Name -replace '[^\d.].*$', '') } -Descending | Select-Object -First 1
if (-not $esptool) { Say "The E S P 32 board package isn't installed in the Arduino IDE."; exit 1 }

# ---- port ----
if (-not $Port -and -not $CompileOnly) {
    $boards = (& $cli board list --format json | ConvertFrom-Json)
    $ports = @(($boards.detected_ports + $boards) | Where-Object { $_.port.protocol -eq "serial" } | ForEach-Object { $_.port.address } | Select-Object -Unique)
    if ($ports.Count -eq 1) { $Port = $ports[0] }
    elseif ($ports.Count -eq 0) { Say "No board is plugged in. Connect $Robot's E S P 32 by U S B."; exit 1 }
    else { Say "More than one board is plugged in. Run flash dot bat with the port, like C O M 6."; Write-Host "Ports: $($ports -join ', ')"; exit 1 }
}
Write-Host "Port: $Port"

# ---- compile (incremental), or skip it when nothing changed since the last build ----
$bin = Join-Path $Build ((Split-Path $Sketch -Leaf) + ".ino.merged.bin")
if (-not $NoCompile -and -not $CompileOnly -and (Test-Path $bin)) {
    $newest = Get-ChildItem $Sketch -File -Recurse -Include *.ino, *.h, *.cpp, *.c |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($newest.LastWriteTime -lt (Get-Item $bin).LastWriteTime) {
        Say "Nothing changed since the last build, so I'll flash that."
        $NoCompile = $true
    }
}
if (-not $NoCompile) {
    Say "Compiling $Robot."
    $t = [Diagnostics.Stopwatch]::StartNew()
    & $cli compile --fqbn $Fqbn --build-path $Build $Sketch
    if ($LASTEXITCODE -ne 0) { Say "Compile failed. Check the errors on screen."; exit 1 }
    Say ("Compiled in {0} seconds." -f [int]$t.Elapsed.TotalSeconds)
}
if ($CompileOnly) { exit 0 }
if (-not (Test-Path $bin)) { Say "There's no build to flash yet. Run it without no compile."; exit 1 }

# ---- flash, talking through it; retry until it works or you close the window ----
$baud = if ($Fast) { 460800 } else { 115200 }
if ($Before) { Say $Before }
while ($true) {
    Say "Ready. Hold boot, tap E N, then let go of boot."
    $psi = New-Object Diagnostics.ProcessStartInfo
    $psi.FileName = $esptool.FullName
    $psi.Arguments = "--chip esp32 --port $Port --baud $baud --connect-attempts 0 --before default-reset --after hard-reset write-flash -z 0x0 `"$bin`""
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $p = New-Object Diagnostics.Process
    $p.StartInfo = $psi
    $lines = [Collections.Concurrent.ConcurrentQueue[string]]::new()
    $handler = { if ($EventArgs.Data) { $Event.MessageData.Enqueue($EventArgs.Data) } }
    $e1 = Register-ObjectEvent $p OutputDataReceived -Action $handler -MessageData $lines
    $e2 = Register-ObjectEvent $p ErrorDataReceived -Action $handler -MessageData $lines
    [void]$p.Start(); $p.BeginOutputReadLine(); $p.BeginErrorReadLine()

    $connected = $false; $lastPct = -1; $failed = $false; $noPort = $false
    $waitStart = Get-Date; $lastNag = Get-Date
    while (-not $p.HasExited -or $lines.Count -gt 0) {
        $line = $null
        while ($lines.TryDequeue([ref]$line)) {
            if ($line -notmatch 'Writing at') { Write-Host $line }
            if (-not $connected -and $line -match 'Connected to|Chip type|Chip is') {
                $connected = $true
                Say "Connected. Let go of the button."
            }
            if ($line -match '\((\d+) ?%\)') {
                $pct = [int]$Matches[1]
                foreach ($mark in 25, 50, 75) { if ($lastPct -lt $mark -and $pct -ge $mark) { Say "$mark percent." } }
                $lastPct = $pct
            }
            if ($line -match 'could not open port|does not exist|PermissionError|Access is denied') { $noPort = $true }
            if ($line -match 'fatal error|stopped responding|Failed') { $failed = $true }
        }
        if (-not $connected -and ((Get-Date) - $lastNag).TotalSeconds -ge 20) {
            $lastNag = Get-Date
            Say "Still waiting. Hold boot and tap E N."
        }
        Start-Sleep -Milliseconds 200
    }
    Unregister-Event -SourceIdentifier $e1.Name; Unregister-Event -SourceIdentifier $e2.Name
    if ($p.ExitCode -eq 0 -and -not $failed) {
        Say "Flash done. $Robot is restarting."
        exit 0
    }
    if ($noPort) {
        Say "I can't open $Port. Is the board plugged in, and is the serial monitor closed?"
        exit 1
    }
    Say "Flash failed. I'll try again."
    Start-Sleep -Seconds 2
}
