// STAY CLOSE on a PC: g++ -std=c++17 -I../esp32_gaits stay_test.cpp -o stay_test
#include <cmath>
#include <cstdio>

#include "fleet_stay.h"

static int failed = 0;
#define CHECK(x) do { if (!(x)) { printf("FAILED line %d: %s\n", __LINE__, #x); failed++; } } while (0)

static void stopsWhenSheGoesQuiet() {
  Stay s;
  stayStart(s);
  CHECK(stayStep(s, true, -70, false, 0) == STAY_WALK);
  CHECK(stayStep(s, false, -70, false, 100) == STAY_STOP);
}

static void stopsWhenClose() {
  Stay s;
  stayStart(s);
  CHECK(stayStep(s, true, -48, false, 0) == STAY_STOP);
  CHECK(s.arrived);
  CHECK(stayStep(s, true, -54, false, 100) == STAY_STOP);   // between the two: doesn't twitch
  CHECK(stayStep(s, true, -60, false, 200) == STAY_WALK);   // you walked off
}

static void keepsGoingWhileItHelps() {
  Stay s;
  stayStart(s);
  CHECK(stayStep(s, true, -75, false, 0) == STAY_WALK);
  CHECK(stayStep(s, true, -72, false, 4000) == STAY_WALK);
  CHECK(s.misses == 0);
}

static void turnsWhenItDoesnt() {
  Stay s;
  stayStart(s);
  stayStep(s, true, -70, false, 0);
  CHECK(stayStep(s, true, -71, false, 4000) == STAY_LEFT);
  CHECK(stayStep(s, true, -71, false, 4500) == STAY_LEFT);
  CHECK(stayStep(s, true, -71, false, 4900) == STAY_WALK);   // 800 ms done
  CHECK(stayStep(s, true, -71, false, 8900) == STAY_RIGHT);  // the other way, for longer
  CHECK(stayStep(s, true, -71, false, 10000) == STAY_RIGHT);
}

static void aWallTurnsIt() {
  Stay s;
  stayStart(s);
  stayStep(s, true, -70, false, 0);
  CHECK(stayStep(s, true, -70, true, 500) == STAY_LEFT);
}

// A pretend room: she's fixed, the signal falls with distance, and it's noisy. Does it get there?
static void findsHerInARoom() {
  int reached = 0;
  for (int seed = 0; seed < 50; seed++) {
    unsigned r = seed * 2654435761u + 7;
    auto noise = [&]() { r = r * 1103515245u + 12345u; return ((r >> 16 & 1023) / 1023.0f - 0.5f) * 3.0f; };
    Stay s;
    stayStart(s);
    float x = -6 + seed % 5, y = -5, a = seed * 0.7f, smooth = -80;
    bool got = false;
    for (unsigned long t = 0; t < 180000 && !got; t += 100) {
      const float d = sqrtf(x * x + y * y) + 0.1f;
      const float rssi = -50 - 20 * log10f(d) + noise();
      smooth += (rssi - smooth) * 0.3f;
      const StayMove m = stayStep(s, true, smooth, false, t);
      if (m == STAY_WALK) { x += 0.04f * cosf(a); y += 0.04f * sinf(a); }
      if (m == STAY_LEFT) a += 0.2f;
      if (m == STAY_RIGHT) a -= 0.2f;
      if (s.arrived) got = true;
    }
    if (got && sqrtf(x * x + y * y) < 3.0f) reached++;
  }
  printf("found her in %d of 50 pretend rooms\n", reached);
  CHECK(reached >= 35);
}

int main() {
  stopsWhenSheGoesQuiet();
  stopsWhenClose();
  keepsGoingWhileItHelps();
  turnsWhenItDoesnt();
  aWallTurnsIt();
  findsHerInARoom();
  printf(failed ? "%d FAILED\n" : "all passed\n", failed);
  return failed != 0;
}
