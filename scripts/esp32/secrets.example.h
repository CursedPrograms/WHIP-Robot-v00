// secrets.example.h - copy to secrets.h (same folder) and fill in: the fleet's WiFi passwords.
// The sketch uses secrets.h when there is one, else these placeholders.
//
//   copy secrets.example.h secrets.h        (Windows)
//   cp secrets.example.h secrets.h          (Linux)
#pragma once

// The fleet network: the "RIFT" router (the hub, RIFT, is at 192.168.4.1).
#define FLEET_SSID      "RIFT"
#define FLEET_PASSWORD  "change-me"

// Every other network on the ladder (DREAM, NINA, TINA, the controllers, NORA, WHIP himself...)
// uses the fleet's fallback password, the same on every robot.
#define NORA_PASSWORD   "change-me"     // at least 8 characters, or the ESP32 won't start WHIP's AP
