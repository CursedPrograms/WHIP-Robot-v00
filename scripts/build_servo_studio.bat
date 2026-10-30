@echo off
rem Builds builds\WHIP-Servo-Studio.exe. Needs: pip install pyinstaller pygame requests pyserial
cd /d "%~dp0"
python -m PyInstaller --noconfirm --onefile --windowed --name WHIP-Servo-Studio --icon ..\images\whip.ico ^
  --distpath ..\builds --workpath %TEMP%\whip_studio --specpath %TEMP%\whip_studio ^
  --collect-all pygame servo_studio_exe.py
