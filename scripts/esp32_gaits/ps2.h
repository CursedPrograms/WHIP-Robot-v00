/*
  ps2.h - a PlayStation 2 pad on WHIP himself: the 9-pin PS2 plug's receiver (wired or the wireless
  kind sold for Arduino), read straight off four pins, no library (the protocol PS2X_lib speaks).

  Wiring, on his ACEBOTT ESP32 Max board (from its pin diagram):
      DAT  -> GPIO 27 - Yellow  (H12's signal pin; the pull-up is the ESP32's own)
      CMD  -> GPIO 25  - Gray (H10's signal pin)
      ATT  -> GPIO 26  - Yellow (H11's signal pin)
      CLK  -> GPIO 32  - Purple (H16's signal pin)
      VCC  -> 3.3 V   - Red  (H16's middle pin: rows H16-H21 are 3.3 V)
      GND  -> GND - Black
  Not from an H1-H12 middle pin: those are 5 V, and a receiver on 5 V answers in 5 V, which the
  ESP32 can't take.

  ps2::begin() once, ps2::poll() as often as you like (it reads every POLL_MS): then ps2::pad has
  the sticks (-1..1, up / left = +) and the buttons, and ps2::pressed(BTN_...) is true once per press.
  Nothing plugged in or no pad paired: ps2::pad.connected is false and nothing else changes.
*/
#pragma once
#include <Arduino.h>

namespace ps2 {

const int PIN_DAT = 27, PIN_CMD = 25, PIN_ATT = 26, PIN_CLK = 32;
const unsigned long POLL_MS = 30;
const float DEADZONE = 0.15;

// Buttons: bit n of the 16-bit word the pad sends (bytes 3 and 4, 0 = pressed)
enum Button : uint16_t {
  BTN_SELECT = 0x0001, BTN_L3 = 0x0002, BTN_R3 = 0x0004, BTN_START = 0x0008,
  BTN_UP = 0x0010, BTN_RIGHT = 0x0020, BTN_DOWN = 0x0040, BTN_LEFT = 0x0080,
  BTN_L2 = 0x0100, BTN_R2 = 0x0200, BTN_L1 = 0x0400, BTN_R1 = 0x0800,
  BTN_TRIANGLE = 0x1000, BTN_CIRCLE = 0x2000, BTN_CROSS = 0x4000, BTN_SQUARE = 0x8000,
};

struct Pad {
  bool connected = false;
  uint16_t buttons = 0;          // held (1 = pressed)
  float lx = 0, ly = 0, rx = 0, ry = 0;   // -1..1: x + = left, y + = up
};
Pad pad;
uint16_t pressedNow = 0;         // went down at the last read
unsigned long lastPoll = 0, lastConfig = 0;
uint8_t lastRaw[9] = { 0 };      // the receiver's last answer, for /ps2 (all ff: nothing drives DAT)

inline uint8_t transfer(uint8_t out) {
  uint8_t in = 0;
  for (uint8_t i = 0; i < 8; i++) {   // LSB first; the pad answers while the clock is low
    digitalWrite(PIN_CMD, (out >> i) & 1);
    digitalWrite(PIN_CLK, LOW);
    delayMicroseconds(5);
    if (digitalRead(PIN_DAT)) in |= 1 << i;
    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(5);
  }
  delayMicroseconds(8);
  return in;
}

inline void packet(const uint8_t* out, uint8_t* in, uint8_t n) {
  digitalWrite(PIN_CMD, HIGH);
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_ATT, LOW);
  delayMicroseconds(15);
  for (uint8_t i = 0; i < n; i++) in[i] = transfer(out[i]);
  digitalWrite(PIN_ATT, HIGH);
}

// Analog mode, locked (so the sticks are always sent): enter config, set analog, leave config
inline void configure() {
  static const uint8_t enter[] = { 0x01, 0x43, 0x00, 0x01, 0x00 };
  static const uint8_t analog[] = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
  static const uint8_t leave[] = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
  uint8_t in[9];
  packet(enter, in, sizeof enter); delay(2);
  packet(analog, in, sizeof analog); delay(2);
  packet(leave, in, sizeof leave);
}

inline void begin() {
  pinMode(PIN_DAT, INPUT_PULLUP);
  pinMode(PIN_CMD, OUTPUT);
  pinMode(PIN_ATT, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  digitalWrite(PIN_ATT, HIGH);
  digitalWrite(PIN_CLK, HIGH);
  configure();
}

inline float stick(uint8_t v) {   // 0..255, 128 the middle -> -1..1 with a dead zone
  float f = (128.0f - v) / 127.0f;
  if (f > 1) f = 1;
  if (f < -1) f = -1;
  return fabsf(f) < DEADZONE ? 0 : f;
}

// Read the pad (every POLL_MS). True if it answered.
inline bool poll() {
  if (millis() - lastPoll < POLL_MS) return pad.connected;
  lastPoll = millis();
  static const uint8_t ask[] = { 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t in[9];
  packet(ask, in, sizeof ask);
  memcpy(lastRaw, in, sizeof lastRaw);
  const bool ok = in[2] == 0x5A && (in[1] == 0x41 || in[1] == 0x73 || in[1] == 0x79);
  if (!ok) {
    pad.connected = false;
    pad.buttons = pressedNow = 0;
    pad.lx = pad.ly = pad.rx = pad.ry = 0;
    if (millis() - lastConfig > 1000) { lastConfig = millis(); configure(); }   // plugged in later / paired late
    return false;
  }
  if (in[1] == 0x41 && millis() - lastConfig > 1000) { lastConfig = millis(); configure(); }   // digital only: ask again
  const uint16_t held = ~(in[3] | (in[4] << 8));
  pressedNow = held & ~pad.buttons;
  pad.buttons = held;
  pad.connected = true;
  if (in[1] != 0x41) {
    pad.rx = stick(in[5]); pad.ry = stick(in[6]); pad.lx = stick(in[7]); pad.ly = stick(in[8]);
  } else {
    pad.lx = pad.ly = pad.rx = pad.ry = 0;
  }
  return true;
}

// What the last answer says, in words (for /ps2 and the debug console)
inline const char* diagnosis() {
  bool allFF = true, allZero = true;
  for (uint8_t i = 1; i < 9; i++) { allFF &= lastRaw[i] == 0xFF; allZero &= lastRaw[i] == 0x00; }
  if (pad.connected) return "pad answering";
  if (allFF) return "no answer: nothing drives DAT (wiring: DAT / ATT swapped? receiver powered from 3.3 V? try a 4.7-10k pull-up DAT -> 3.3 V)";
  if (allZero) return "DAT stuck low (DAT shorted, or on the wrong pin)";
  if (lastRaw[2] == 0x5A) return "receiver answers, but no pad paired (press START / the pad's power, check its light)";
  return "garbled answer (loose wire, or CLK / CMD swapped)";
}

inline bool held(uint16_t b) { return pad.buttons & b; }
inline bool pressed(uint16_t b) {   // once per press
  if (!(pressedNow & b)) return false;
  pressedNow &= ~b;
  return true;
}

}  // namespace ps2
