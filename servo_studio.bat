@echo off
REM servo_studio.bat - WHIP Servo Studio: his servos as sliders, linkages and WHIP himself (servo_studio.ps1 does the work).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0servo_studio.ps1" %*
if errorlevel 1 pause
