@echo off
REM freeze_requirements.bat - writes requirements.txt from venv\ (freeze_requirements.ps1).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0freeze_requirements.ps1" %*
if errorlevel 1 pause
