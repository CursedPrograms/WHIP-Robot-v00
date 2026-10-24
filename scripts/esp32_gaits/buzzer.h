/*
  buzzer.h - WHIP's buzzer: short chirps for what he's doing, played without
  stopping anything (update() moves to the next note when it's time).

  Wiring: a passive buzzer (piezo) + -> GPIO19 through ~100 ohm, - -> GND. An
  active buzzer (one that beeps by itself on 3-5 V) works too, but it only has
  its own pitch: every tune comes out as beeps. A big magnetic buzzer wants a
  transistor (2N2222 / S8050) rather than the pin straight.

  Sound on at every boot; the web page's SOUND button, /buzzer?on=0, or B mutes it.
*/
#pragma once
#include <Arduino.h>

namespace buzzer {

const int PIN = 19;

struct Note { uint16_t hz; uint16_t ms; };   // hz 0: a rest; ms 0: the end
const Note BOOT[]      = { {1300, 50}, {0, 20}, {1700, 50}, {0, 20}, {2300, 90}, {0, 0} };
const Note WALL[]      = { {500, 90}, {0, 60}, {500, 90}, {0, 0} };
const Note TILT[]      = { {1800, 80}, {1200, 80}, {1800, 80}, {1200, 80}, {0, 0} };
const Note MODE[]      = { {2000, 35}, {0, 0} };
const Note GAIT1[]     = { {1500, 60}, {0, 0} };
const Note GAIT2[]     = { {1500, 60}, {0, 50}, {1500, 60}, {0, 0} };
const Note GAIT3[]     = { {1500, 60}, {0, 50}, {1500, 60}, {0, 50}, {1500, 60}, {0, 0} };
const Note ON[]        = { {900, 50}, {1400, 50}, {2000, 70}, {0, 0} };
const Note OFF[]       = { {2000, 50}, {1400, 50}, {900, 70}, {0, 0} };
const Note LOCK[]      = { {400, 120}, {0, 40}, {300, 160}, {0, 0} };
const Note SHUTDOWN[]  = { {1600, 120}, {1200, 120}, {800, 120}, {500, 260}, {0, 0} };

bool muted = false;
const Note* playing = nullptr;
uint8_t at = 0;
unsigned long noteEnd = 0;

inline void begin() {
  ledcAttach(PIN, 2000, 10);
  ledcWriteTone(PIN, 0);
}

inline void startNote() {
  ledcWriteTone(PIN, playing[at].hz);
  noteEnd = millis() + playing[at].ms;
}

// Start a tune (cuts off the one playing)
inline void play(const Note* tune) {
  if (muted) return;
  playing = tune;
  at = 0;
  startNote();
}

inline bool busy() { return playing != nullptr; }

// Call often: moves on to the next note when it's time.
inline void update() {
  if (!playing || (long)(millis() - noteEnd) < 0) return;
  at++;
  if (playing[at].ms == 0) { ledcWriteTone(PIN, 0); playing = nullptr; return; }
  startNote();
}

// Play a whole tune and wait for it (shutdown: nothing else runs after).
inline void playNow(const Note* tune) {
  play(tune);
  while (busy()) { update(); delay(2); }
}

inline void setMuted(bool m) {
  muted = m;
  if (m) { ledcWriteTone(PIN, 0); playing = nullptr; }
}

}  // namespace buzzer
