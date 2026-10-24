@echo off
REM run.bat - starts WHIP's controller (runs setup first if there's no venv yet).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run.ps1" %*
if errorlevel 1 pause
