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

// ---- going round things: the smart part (stayStepSmart) ----
static StaySight sight(float f, float l = -1, float r = -1, bool slide = false) { StaySight v = { f, l, r, slide }; return v; }

static void creepsWhenSomethingsAhead() {
  Stay s;
  stayStart(s);
  CHECK(stayStepSmart(s, true, -70, sight(200), 0) == STAY_WALK);
  CHECK(stayStepSmart(s, true, -70, sight(45), 100) == STAY_CREEP);
}

static void slidesTowardTheRoom() {
  Stay s;
  stayStart(s);
  stayStepSmart(s, true, -70, sight(200, 80, 100, true), 0);
  CHECK(stayStepSmart(s, true, -70, sight(20, 40, 150, true), 100) == STAY_SLIDE_RIGHT);   // more room right
  CHECK(stayStepSmart(s, true, -70, sight(20, 40, 150, true), 600) == STAY_SLIDE_RIGHT);
  CHECK(stayStepSmart(s, true, -70, sight(200, 40, 150, true), 900) == STAY_SLIDE_RIGHT);  // clear: a little more
  CHECK(stayStepSmart(s, true, -70, sight(200, 40, 150, true), 1600) == STAY_WALK);       // past it: on
  CHECK(s.misses == 0);                                                                   // not a bad guess
}

static void triesTheOtherSideThenBacksUp() {
  Stay s;
  stayStart(s);
  stayStepSmart(s, true, -70, sight(200, 100, 100, true), 0);
  const StayMove first = stayStepSmart(s, true, -70, sight(20, 100, 100, true), 100);
  CHECK(first == STAY_SLIDE_LEFT || first == STAY_SLIDE_RIGHT);
  const StayMove second = stayStepSmart(s, true, -70, sight(20, 100, 100, true), 2700);
  CHECK(second != first && (second == STAY_SLIDE_LEFT || second == STAY_SLIDE_RIGHT));
  CHECK(stayStepSmart(s, true, -70, sight(20, 100, 100, true), 5300) == STAY_BACK);
}

static void boxedInBacksUp() {
  Stay s;
  stayStart(s);
  stayStepSmart(s, true, -70, sight(200, 100, 100, true), 0);
  CHECK(stayStepSmart(s, true, -70, sight(20, 15, 15, true), 100) == STAY_BACK);
  CHECK(stayStepSmart(s, true, -70, sight(60, 15, 40, true), 600) == STAY_BACK);
  const StayMove after = stayStepSmart(s, true, -70, sight(60, 15, 40, true), 1100);
  CHECK(after == STAY_RIGHT);                                                             // turns toward the room
}

static void noSlideTurnsTowardTheRoom() {
  Stay s;
  stayStart(s);
  stayStepSmart(s, true, -70, sight(200, 30, 120), 0);
  CHECK(stayStepSmart(s, true, -70, sight(20, 30, 120), 100) == STAY_RIGHT);
}

// A pretend room with a wall across the middle, between the robot and RITA. The robot's sensors
// see the wall; it must find her without ever walking into it.
static bool inWall(float x, float y) { return y > -0.6f && y < -0.4f && x > -3.0f && x < 2.0f; }
static float ray(float x, float y, float a) {
  for (float d = 0.05f; d < 3.0f; d += 0.05f)
    if (inWall(x + d * cosf(a), y + d * sinf(a))) return d * 100;
  return 300;
}
static void findsHerRoundAWall(bool canSlide, int need) {
  int reached = 0, crashes = 0;
  for (int seed = 0; seed < 50; seed++) {
    unsigned r = seed * 2654435761u + 11;
    auto noise = [&]() { r = r * 1103515245u + 12345u; return ((r >> 16 & 1023) / 1023.0f - 0.5f) * 3.0f; };
    Stay s;
    stayStart(s);
    float x = -1.5f + (seed % 5) * 0.6f, y = -4.0f, a = 1.57f + (seed % 7 - 3) * 0.3f, smooth = -80;
    bool got = false;
    for (unsigned long t = 0; t < 240000 && !got; t += 100) {
      const float d = sqrtf(x * x + y * y) + 0.1f;
      smooth += (-50 - 20 * log10f(d) + noise() - smooth) * 0.3f;
      const StaySight v = { ray(x, y, a), ray(x, y, a + 1.57f), ray(x, y, a - 1.57f), canSlide };
      const StayMove m = stayStepSmart(s, true, smooth, v, t);
      float nx = x, ny = y;
      const float step = m == STAY_CREEP ? 0.02f : 0.04f;
      if (m == STAY_WALK || m == STAY_CREEP) { nx += step * cosf(a); ny += step * sinf(a); }
      if (m == STAY_BACK) { nx -= 0.03f * cosf(a); ny -= 0.03f * sinf(a); }
      if (m == STAY_SLIDE_LEFT) { nx += 0.03f * cosf(a + 1.57f); ny += 0.03f * sinf(a + 1.57f); }
      if (m == STAY_SLIDE_RIGHT) { nx += 0.03f * cosf(a - 1.57f); ny += 0.03f * sinf(a - 1.57f); }
      if (m == STAY_LEFT) a += 0.2f;
      if (m == STAY_RIGHT) a -= 0.2f;
      if (inWall(nx, ny)) crashes++;
      else { x = nx; y = ny; }
      if (s.arrived) got = true;
    }
    if (got && sqrtf(x * x + y * y) < 3.0f) reached++;
  }
  printf("%s: found her round a wall in %d of 50 rooms, %d bumps into it\n",
         canSlide ? "sidestepping (NORA / WHIP)" : "turning only", reached, crashes);
  CHECK(reached >= need);
  CHECK(crashes == 0);
}

int main() {
  stopsWhenSheGoesQuiet();
  stopsWhenClose();
  keepsGoingWhileItHelps();
  turnsWhenItDoesnt();
  aWallTurnsIt();
  findsHerInARoom();
  creepsWhenSomethingsAhead();
  slidesTowardTheRoom();
  triesTheOtherSideThenBacksUp();
  boxedInBacksUp();
  noSlideTurnsTowardTheRoom();
  findsHerRoundAWall(true, 30);
  findsHerRoundAWall(false, 25);
  printf(failed ? "%d FAILED\n" : "all passed\n", failed);
  return failed != 0;
}
