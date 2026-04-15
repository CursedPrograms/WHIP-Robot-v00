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
| Voltage Regulator | XL4016 DC-DC Buck Converter (12V → 6V) |

### Sensors
| **Component** | **Details** |
|-----------|---------|
| Ultrasonic Sensors | HC-SR04 |

---

#### Libraries:

- Wire.h
- Adafruit_PWMServoDriver.h
- PS2X_lib.h

---
### Network Setup:

#### Connect to NORA:
- ap_ssid     = "NORA";
- ap_password = "12345678";

### Hardware Configuration
---
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
