/*
  leds.h - WHIP's 4x4 RGB LED grid (16 WS2812 / NeoPixel, Adafruit_NeoPixel).

  Wiring: DIN -> GPIO18 (through a ~330 ohm resistor if you have one), 5V -> the
  ESP32's 5 V, GND -> GND (shared with the ESP32). Not the 6 V servo rail: it's
  over the LEDs' 5.3 V limit. A 3.3 V data line drives them fine nearly always;
  if they flicker, power the grid at 4.5-5 V or add a level shifter.

  The grid is WHIP seen from above, head at the top:

      LF   eye  eye   RF       legs: dim on the ground, bright while that foot is in the air
      LM   mode mode  RM             (his gait, live, as he walks)
      LM   mode mode  RM       mode: obstacle blue, IR violet, web green, MIND pink, follow teal
      LB   tail tail  RB       eyes: red when there's a wall; tail: orange while tilt safety is off

  Tilt stop: the whole grid flashes orange. Legs locked: everything dim red. Lying down:
  his mode colour, very dim and slow. Picked up: violet flicker. Dancing: colours.
  If the picture comes out turned or mirrored, set GRID_ROTATE / GRID_MIRROR;
  if every other row is backwards, the grid is wired zig-zag: GRID_SERPENTINE.
  Brightness stays low (GRID_BRIGHTNESS): 16 LEDs at full white pull ~1 A.
*/
#pragma once
#include <Adafruit_NeoPixel.h>
#include "gaits.h"

namespace leds {

const int     PIN             = 18;
const uint8_t SIDE            = 4;
const uint8_t COUNT           = SIDE * SIDE;
const uint8_t GRID_BRIGHTNESS = 40;     // of 255
const uint8_t GRID_ROTATE     = 0;      // quarter turns clockwise, 0-3: where LED 0 sits vs his head
const bool    GRID_MIRROR     = false;  // left and right swapped
const bool    GRID_SERPENTINE = false;  // rows wired zig-zag (every other row runs backwards)

enum Look : uint8_t { OBSTACLE, IR, WEB, MIND, FOLLOW, LOCKED, TILT_STOP, HELD, SHUTDOWN };

struct State {
  Look look;
  bool walking;          // a gait is running: show the feet
  const ik::Gait* gait;  // ...this one
  bool wall;             // something too close ahead
  bool tiltOff;          // tilt safety switched off
  bool resting;          // lying down
  bool dancing;
};

// Which leg each cell shows (row 0 = his head), or -1 for the middle
const int8_t CELL_LEG[SIDE][SIDE] = {
  { ik::LF, -1, -1, ik::RF },
  { ik::LM, -1, -1, ik::RM },
  { ik::LM, -1, -1, ik::RM },
  { ik::LB, -1, -1, ik::RB },
};

Adafruit_NeoPixel px(COUNT, PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastShow = 0;

inline void begin() {
  px.begin();
  px.setBrightness(GRID_BRIGHTNESS);
  px.clear();
  px.show();
}

// The LED number for the cell at (row, col) of the picture above
inline uint8_t index(uint8_t row, uint8_t col) {
  if (GRID_MIRROR) col = SIDE - 1 - col;
  for (uint8_t q = 0; q < GRID_ROTATE % 4; q++) {   // a quarter turn clockwise
    const uint8_t r = row;
    row = col;
    col = SIDE - 1 - r;
  }
  if (GRID_SERPENTINE && (row & 1)) col = SIDE - 1 - col;
  return row * SIDE + col;
}

inline uint32_t scaled(uint8_t r, uint8_t g, uint8_t b, float k) {
  return Adafruit_NeoPixel::Color((uint8_t)(r * k), (uint8_t)(g * k), (uint8_t)(b * k));
}

inline void lookColour(Look l, uint8_t& r, uint8_t& g, uint8_t& b) {
  switch (l) {
    case OBSTACLE: r = 100; g = 150; b = 230; break;   // the fleet's accent blue
    case IR:       r = 150; g = 80;  b = 230; break;
    case WEB:      r = 70;  g = 215; b = 100; break;
    case MIND:     r = 230; g = 80;  b = 180; break;
    case FOLLOW:   r = 0;   g = 200; b = 190; break;
    default:       r = 200; g = 64;  b = 64;  break;
  }
}

// Call often (every loop and every wait): redraws at ~30 fps, cheap otherwise.
// force: draw now (before he stops calling it, as at shutdown).
inline void update(const State& s, bool force = false) {
  const unsigned long now = millis();
  if (!force && now - lastShow < 33) return;
  lastShow = now;

  if (s.look == SHUTDOWN) { px.clear(); px.show(); return; }
  if (s.look == TILT_STOP) {
    const uint32_t c = (now / 250) % 2 ? scaled(255, 150, 0, 1) : 0;
    for (uint8_t i = 0; i < COUNT; i++) px.setPixelColor(i, c);
    px.show();
    return;
  }

  if (s.look == HELD) {   // picked up: a violet flicker
    for (uint8_t i = 0; i < COUNT; i++) px.setPixelColor(i, scaled(170, 60, 255, (now / 90 + i) % 3 ? 0.25f : 1.0f));
    px.show();
    return;
  }
  if (s.dancing) {        // a rainbow running round the grid
    for (uint8_t i = 0; i < COUNT; i++) px.setPixelColor(i, px.ColorHSV((uint16_t)(now * 40 + i * 4096)));
    px.show();
    return;
  }

  uint8_t r, g, b;
  lookColour(s.look, r, g, b);
  float breathe = s.look == LOCKED ? 0.15f : 0.35f + 0.25f * sinf(now / 600.0f);
  if (s.resting) breathe = 0.04f + 0.04f * sinf(now / 1500.0f);

  for (uint8_t row = 0; row < SIDE; row++) {
    for (uint8_t col = 0; col < SIDE; col++) {
      const int8_t leg = CELL_LEG[row][col];
      uint32_t c = scaled(r, g, b, breathe);

      if (leg >= 0) {
        c = scaled(r, g, b, s.walking ? 0.12f : breathe * 0.6f);
        if (s.walking && s.gait && s.gait->air[leg]) {   // that foot is up: brighter, whiter the higher
          const float lift = constrain((s.gait->foot[leg].z - ik::homeZ()) / ik::LIFT, 0.0f, 1.0f);
          const float k = 0.3f + 0.6f * lift;
          c = Adafruit_NeoPixel::Color(r + (uint8_t)((255 - r) * k), g + (uint8_t)((255 - g) * k),
                                       b + (uint8_t)((255 - b) * k));
        }
      } else if (row == 0 && s.wall) {
        c = scaled(255, 0, 0, (now / 150) % 2 ? 1.0f : 0.4f);
      } else if (row == SIDE - 1 && s.tiltOff) {
        c = scaled(255, 150, 0, 0.8f);
      }
      px.setPixelColor(index(row, col), c);
    }
  }
  px.show();
}

}  // namespace leds
