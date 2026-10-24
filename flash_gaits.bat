@echo off
REM Compile and flash the gaits sketch (scripts\esp32_gaits: tripod, ripple, wave), talking you through it.
REM   flash_gaits.bat [COM6] [-Fast] [-NoCompile] [-CompileOnly]
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\flash.ps1" -Folder esp32_gaits %*
if errorlevel 1 pause
