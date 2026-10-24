@echo off
REM setup.bat - makes venv\ and installs requirements.txt into it (setup.ps1 does the work).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup.ps1" %*
if errorlevel 1 pause
