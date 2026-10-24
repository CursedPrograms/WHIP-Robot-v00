@echo off
REM launch_controller.bat - WHIP's controller, from the repo's venv (launch_controller.ps1).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0launch_controller.ps1" %*
if errorlevel 1 pause
