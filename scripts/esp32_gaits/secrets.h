// secrets.h - the fleet's WiFi passwords (committed on purpose: the fleet's, the same in every robot).
// The layout is secrets.example.h's.
#pragma once

// The fleet network: the "RIFT" router (the hub, RIFT, is at 192.168.4.1).
#define FLEET_SSID      "RIFT"
#define FLEET_PASSWORD  "mmvbkbak"

// Every other network on the ladder (DREAM, NINA, TINA, the controllers, NORA, WHIP himself...)
// uses the fleet's fallback password, the same on every robot.
#define NORA_PASSWORD   "mmvbkbak"
