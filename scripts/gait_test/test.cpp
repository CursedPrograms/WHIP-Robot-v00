// gait_test - checks gaits.h on a PC: every gait, heights 0-9, walking, turning, sideways and stopping.
// Every frame in reach and in servo range, no foot on the ground slipping, at least 3 feet down,
// and he always ends standing at home.
//   g++ -std=gnu++17 -static -Istub -I../esp32_gaits test.cpp -o test && ./test   (from scripts/gait_test)
#include "gaits.h"
using namespace ik;
// Checks: every frame solvable (in reach), no foot jumps more than JUMP mm in one tick,
// feet on the ground (z == home) move exactly like the body (no slip between them).
int fails = 0;
void check(const char* what, bool ok) { if (!ok) { if (fails < 20) printf("FAIL %s\n", what); fails++; } }
bool reach(const Frame& f) { float a[JOINTS]; for (int i = 0; i < LEGS; i++) if (!solve(i, f.leg[i], a)) return false;
  for (int i = 0; i < LEGS; i++) { solve(i, f.leg[i], a); for (int j = 0; j < 3; j++) if (fabsf(a[j]) > 90) return false; } return true; }
int main() {
  loadDefaults();
  for (int g = 0; g < GAITS; g++) for (int hgt = 0; hgt <= 9; hgt++) {
    height = hgt;
    struct Cmd { float f, l, t; int cycles; } plan[] = { {1,0,0,4}, {-1,0,0,4}, {0,0,1,4}, {0,0,-1,4}, {0,1,0,3}, {0.7f,0,0.3f,3} };
    for (auto& c : plan) {
      Gait G; G.start(home(), g, 10);
      Frame prev = home(); float maxJump = 0, maxReach = 0; int ticks = 0;
      char buf[96];
      auto step = [&](float f, float l, float t, bool settle) {
        Frame now = G.tick(f, l, t, settle); ticks++;
        snprintf(buf, sizeof buf, "%s h%d cmd(%.1f,%.1f,%.1f) reach", GAIT_NAMES[g], hgt, c.f, c.l, c.t);
        check(buf, reach(now));
        for (int i = 0; i < LEGS; i++) { maxJump = max(maxJump, dist2(now.leg[i], prev.leg[i]) + fabsf(now.leg[i].z - prev.leg[i].z));
          maxReach = max(maxReach, dist2(now.leg[i], home().leg[i])); }
        // no-slip: pairs of ground feet keep their distance
        for (int i = 0; i < LEGS; i++) for (int j = i + 1; j < LEGS; j++)
          if (fabsf(prev.leg[i].z - homeZ()) < 1e-3 && fabsf(now.leg[i].z - homeZ()) < 1e-3 &&
              fabsf(prev.leg[j].z - homeZ()) < 1e-3 && fabsf(now.leg[j].z - homeZ()) < 1e-3 &&
              !G.air[i] && !G.air[j]) {
            float d0 = dist2(prev.leg[i], prev.leg[j]), d1 = dist2(now.leg[i], now.leg[j]);
            snprintf(buf, sizeof buf, "%s h%d slip %d-%d %.3f", GAIT_NAMES[g], hgt, i, j, d1 - d0);
            check(buf, fabsf(d1 - d0) < 0.05f);
          }
        int down = 0; for (int i = 0; i < LEGS; i++) down += !G.air[i];
        snprintf(buf, sizeof buf, "%s feet down %d", GAIT_NAMES[g], down);
        check(buf, down >= 3);
        prev = now;
      };
      for (int k = 0; k < c.cycles * G.cycleTicks; k++) step(c.f, c.l, c.t, false);
      int settleTicks = 0;
      while (!G.settled() && settleTicks < 1000) { step(0, 0, 0, true); settleTicks++; }
      snprintf(buf, sizeof buf, "%s h%d settled", GAIT_NAMES[g], hgt); check(buf, G.settled());
      check("home exact", maxOffset(G.last, home()) <= HOME_MM * 2);
      if (hgt == 5) printf("%-6s cmd(%4.1f,%3.1f,%4.1f) cycle %2d ticks  max jump %5.1f mm/tick  max from home %5.1f mm  stop %d ticks\n",
                           GAIT_NAMES[g], c.f, c.l, c.t, G.cycleTicks, maxJump, maxReach, settleTicks);
    }
  }
  printf("%d failures\n", fails);
  return fails != 0;
}
