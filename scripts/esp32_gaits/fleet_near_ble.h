// fleet_near_ble.h - the Bluetooth LE half of fleet_near.h.
//
// The Pi robots (KIDA-00, KIDA-01) can't speak ESP-NOW, but they have
// Bluetooth LE. So NORA and WHIP also advertise the fleet beacon over BLE and
// scan for everyone else's: that's how they hear the KIDAs, and how the KIDAs
// hear them. What BLE hears goes into the same list as ESP-NOW (fleetNearJson),
// marked "ble"; a robot heard on both radios shows "espnow+ble".
//
// The beacon: manufacturer data, company id 0xFFFF (the "no company" test id),
// then "DRFL", version 1, the robot's state (one byte: parked / driving /
// yielding / user - an advert is too small for the Brainfuck ESP-NOW sends) and
// its name. The advert is refreshed whenever the state changes.
//
// The scan is gentle (100 ms listening every second) so WiFi - NORA hosts
// the whole fleet's network - keeps most of the shared 2.4 GHz radio.
//
//   fleetNearBleBegin("NORA");   after fleetNearBegin(), once
//   fleetNearBleLoop();          every loop (the BLE stack advertises and scans
//                                on its own; this only keeps its memory bounded)

#pragma once
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <BLEScan.h>
#include "fleet_near.h"

class FleetBleListener : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    if (!d.haveManufacturerData()) return;
    String md = d.getManufacturerData();
    // 0xFF 0xFF | 'D' 'R' 'F' 'L' | 1 | state | name...
    if (md.length() < 9 || (uint8_t)md[0] != 0xFF || (uint8_t)md[1] != 0xFF || md.substring(2, 6) != "DRFL" || (uint8_t)md[6] != 1) return;
    char name[12] = {0};
    md.substring(8, 8 + 11).toCharArray(name, sizeof(name));
    fleetNearRecord(name, d.getRSSI(), FLEET_VIA_BLE, (uint8_t)md[7]);
  }
};

static uint8_t fleetBleAdState = 0xFF;

static void fleetBleSetAd(uint8_t state) {
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  BLEAdvertisementData data;
  data.setFlags(0x06);                                  // general discoverable, no classic BR/EDR in the ad
  String md;
  md += (char)0xFF;
  md += (char)0xFF;
  md += "DRFL";
  md += (char)1;
  md += (char)state;
  md += String(fleetSelf).substring(0, 11);
  data.setManufacturerData(md);
  adv->setAdvertisementData(data);
  fleetBleAdState = state;
}

static bool fleetNearBleBegin(const char* selfName) {
  BLEDevice::init(selfName);
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  fleetBleSetAd(fleetNearMyState());
  adv->setMinInterval(800);                             // 800 x 0.625 ms = every 500 ms, like ESP-NOW
  adv->setMaxInterval(880);
  adv->start();

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new FleetBleListener(), true);   // true: report repeats, so RSSI keeps updating
  scan->setActiveScan(false);                           // just listen; no scan requests
  scan->setInterval(1000);
  scan->setWindow(100);
  scan->start(0, nullptr, false);                       // 0 = scan forever, in the background
  return true;
}

// The BLE library keeps every device it has heard; phones nearby keep changing
// their addresses, so on a robot that runs for days that list would grow until
// the heap ran out. Our own table (fleet_near.h) is all we need.
static void fleetNearBleLoop() {
  uint8_t st = fleetNearMyState();
  if (st != fleetBleAdState) fleetBleSetAd(st);         // tell the KIDAs straight away when this robot's state changes
  static unsigned long nextClear = 0;
  if (millis() >= nextClear) {
    nextClear = millis() + 30000;
    BLEDevice::getScan()->clearResults();
  }
}
