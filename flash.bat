@echo off
REM Compile (only what changed) and flash WHIP's ESP32, talking you through it.
REM   flash.bat [COM6] [-Fast] [-NoCompile] [-CompileOnly]
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\flash.ps1" %*
if errorlevel 1 pause
