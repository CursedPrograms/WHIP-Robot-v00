[![Twitter: @NorowaretaGemu](https://img.shields.io/badge/X-@NorowaretaGemu-blue.svg?style=flat)](https://x.com/NorowaretaGemu)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
  
<br>
<div align="center">
  <a href="https://ko-fi.com/cursedentertainment">
    <img src="https://ko-fi.com/img/githubbutton_sm.svg" alt="ko-fi" style="width: 20%;"/>
  </a>
</div>
<br>

<div align="center">
  <img alt="Python" src="https://img.shields.io/badge/python%20-%23323330.svg?&style=for-the-badge&logo=python&logoColor=white"/>
</div>
<div align="center">
  <img alt="Git" src="https://img.shields.io/badge/git%20-%23323330.svg?&style=for-the-badge&logo=git&logoColor=white"/>
  <img alt="PowerShell" src="https://img.shields.io/badge/PowerShell-%23323330.svg?&style=for-the-badge&logo=powershell&logoColor=white"/>
  <img alt="Shell" src="https://img.shields.io/badge/Shell-%23323330.svg?&style=for-the-badge&logo=gnu-bash&logoColor=white"/>
  <img alt="Batch" src="https://img.shields.io/badge/Batch-%23323330.svg?&style=for-the-badge&logo=windows&logoColor=white"/>
</div>
<br>

# WHIP: Walking Hexapod Intelligence Platform

- [KIDA-Robot-v00](https://github.com/CursedPrograms/KIDA-Robot-v00)
- [KIDA-Robot-v01](https://github.com/CursedPrograms/KIDA-Robot-v01)
- [NORA-Robot-v00](https://github.com/CursedPrograms/NORA-Robot-v00)
- [DREAM](https://github.com/CursedPrograms/DREAM)
- [RIFT](https://github.com/CursedPrograms/RIFT)

---

## Prerequisites


### Software
- [Arduino IDE](https://docs.arduino.cc/software/ide/)

### Hardware

### Microcontrollers
| **Component** | **Details** |
|-----------|---------|
| Microcontroller 0 | ESP32 Servo Controller Board | Dev0 |
| Microcontroller 1 | Arduino UNO | Dev1 |

### Chassis & Motion
| **Component** | **Details** |
|-----------|---------|
| Chassis | 18DOF hexapod chassis |
| Motors | 18 × MG995 180° Servo Motors |

### User Controllers
| **Component** | **Details** |
|-----------|---------|
| Interface | PC, Android, iPhone |
| Controller | PS2 Controller + Receiver |

### Power System
| **Component** | **Details** |
|-----------|---------|
| Battery | 3s 21700 (12.6V in series) |
| Voltage Regulator | XL4016 DC-DC Buck Converter (12.6V → 6V) |

### Sensors
| **Component** | **Details** |
|-----------|---------|
| Ultrasonic Sensors | HC-SR04 |
| IMU SENSOR | MPU6050 |

---

# Schematics
### Power Schematic
```
3S 21700 BATTERIES ──────► XL4016 12.6V ──────► XL4016 Output 6V
XL4016 Output 6V:
├── + ──────► ESP32 Servo Controller Board +
├── – ──────► ESP32 Servo Controller Board - 
├── + ──────► Arduino UNO +
└── – ──────► Arduino UNO - 
```

**ARDUINO (DEV0):**
```
USB-C (DEV0) ──────► USB-C (DEV1) - Serial Communication
```
#### Libraries:
```
- Wire.h
- Adafruit_PWMServoDriver.h
- PS2X_lib.h
```
```
POWER:
├── XL4016 6V ──────► 
└── GND ─────► Common GND (modules)

Leg 1 = Front  Left  → channels  0,  1,  2
Leg 2 = Middle Left  → channels  3,  4,  5
Leg 3 = Back   Left  → channels  6,  7,  8
Leg 4 = Front  Right → channels  9, 10, 11
Leg 5 = Middle Right → channels 12, 13, 14
Leg 6 = Back   Right → channels 15, 16, 17

PS2 Reciever Connection
```

**ARDUINO (DEV0):**
```
USB-C (DEV1) ──────► USB-C (DEV0) - Serial Communication + Power
```
#### Libraries:
```
- Wire.h
- Adafruit_PWMServoDriver.h
- PS2X_lib.h
- MPU6050.h / I2Cdev.h
```
```
MPU6050 (Gyro + Accelerometer)
SDA  ─────► A4 (UNO)
SCL  ─────► A5 (UNO)
Ultrasonic Sensor (HC-SR04)
TRIG ─────► D7
ECHO ─────► D6
```

#### Sensors
- MPU6050 (Gyro + Accelerometer)
```
VCC  ─────► 5V
GND  ─────► GND
SDA  ─────► A4 (UNO)
SCL  ─────► A5 (UNO)
```
- Ultrasonic Sensor (HC-SR04)
```
VCC  ─────► 5V
GND  ─────► GND
TRIG ─────► D7
ECHO ─────► D6
```
---
### Network Setup:

#### Connect to [NORA-Robot-v00](https://github.com/CursedPrograms/NORA-Robot-v00):
- ap_ssid     = "NORA";
- ap_password = "12345678";

#### Connect to [RIFT](https://github.com/CursedPrograms/RIFT):
- autoconnect on rift: localhost:5000, dream: localhost:5001 or whip: localhost:5006

---

### Hardware Configuration

## How to Run:

### Install Requirements

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

<br>
<div align="center">
© Cursed Entertainment 2026
</div>
<br>
<div align="center">
<a href="https://cursed-entertainment.itch.io/" target="_blank">
    <img src="https://github.com/CursedPrograms/cursedentertainment/raw/main/images/logos/logo-wide-grey.png"
        alt="CursedEntertainment Logo" style="width:250px;">
</a>
</div>
