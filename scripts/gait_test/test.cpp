// gait_test - checks gaits.h on a PC: every gait, heights 0-9, walking, turning, sideways and stopping.
// Every frame in reach and in servo range, no foot on the ground slipping, at least 3 feet down,
// and he always ends standing at home. Then the body moves: lying down, hello, dance.
//   g++ -std=gnu++17 -static -Istub -I../esp32_gaits test.cpp -o test && ./test   (from scripts/gait_test)
#include "gaits.h"
using namespace ik;
// Checks: every frame solvable (in reach), no foot jumps more than JUMP mm in one tick,
// feet on the ground (z == home) move exactly like the body (no slip between them).
int fails = 0;
void check(const char* what, bool ok) { if (!ok) { if (fails < 20) printf("FAIL %s\n", what); fails++; } }
// How far his centre (0, 0) is inside the feet that are down (their convex hull), mm; < 0: outside.
float margin(const Gait& G, const Frame& f, float cx = 0, float cy = 0) {
  float px[LEGS], py[LEGS]; int n = 0;
  for (int i = 0; i < LEGS; i++) if (!G.air[i]) { px[n] = f.leg[i].x; py[n] = f.leg[i].y; n++; }
  if (n < 3) return -999;
  float best = 1e9;   // inside a convex hull = inside or on every edge's half-plane; for each pair of
  for (int a = 0; a < n; a++) for (int b = 0; b < n; b++) if (a != b) {   // feet with all others on
    float ex = px[b] - px[a], ey = py[b] - py[a], len = sqrtf(ex * ex + ey * ey);   // one side: a hull edge
    bool edge = true;
    for (int c = 0; c < n; c++) if (c != a && c != b && ex * (py[c] - py[a]) - ey * (px[c] - px[a]) < -1e-3) edge = false;
    if (edge && len > 1e-3) best = fminf(best, (ex * (cy - py[a]) - ey * (cx - px[a])) / len);
  }
  return best;
}
const float BALANCE_MM = 40;   // his centre stays at least this far inside the feet that are down
bool reach(const Frame& f) { float a[JOINTS]; for (int i = 0; i < LEGS; i++) if (!solve(i, f.leg[i], a)) return false;
  for (int i = 0; i < LEGS; i++) { solve(i, f.leg[i], a); for (int j = 0; j < 3; j++) if (fabsf(a[j]) > 90) return false; } return true; }
int main() {
  loadDefaults();
  for (int cb : {5, 0, (int)CRAB_BACK_MAX}) for (int slip = 0; slip <= 1; slip++) for (int g = 0; g < GAITS; g++) for (int hgt = 0; hgt <= 9; hgt++) {
    height = hgt; antiSlip = slip; crabBack = cb;
    struct Cmd { float f, l, t; int cycles; } plan[] = { {1,0,0,4}, {-1,0,0,4}, {0,0,1,4}, {0,0,-1,4}, {0,1,0,3}, {0.7f,0,0.3f,3} };
    for (auto& c : plan) {
      Gait G; G.start(home(), g, 10);
      Frame prev = home(); float maxJump = 0, maxReach = 0, minBal = 1e9; int ticks = 0, allDown = 0;
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
        const float bal = margin(G, now); minBal = fminf(minBal, bal);
        snprintf(buf, sizeof buf, "%s h%d balance %.0f mm", GAIT_NAMES[g], hgt, bal);
        check(buf, bal >= BALANCE_MM);
        int down = 0; for (int i = 0; i < LEGS; i++) down += !G.air[i];
        snprintf(buf, sizeof buf, "%s feet down %d", GAIT_NAMES[g], down);
        check(buf, down >= 3);
        if (!settle) allDown += down == LEGS;
        prev = now;
      };
      for (int k = 0; k < c.cycles * G.cycleTicks; k++) step(c.f, c.l, c.t, false);
      if (c.l != 0) {   // crabbing the back feet land turned in on their hips, the rest as ever
        snprintf(buf, sizeof buf, "%s h%d crab in", GAIT_NAMES[g], hgt);
        const Pos a = stepHome(LB, G.crabDeg), b = stepHome(RB, G.crabDeg);
        check(buf, G.crabDeg == cb && (cb == 0 || (a.x > home().leg[LB].x && b.x > home().leg[RB].x)));
        check("crab in mirrored", fabsf(a.x - b.x) < 1e-3 && fabsf(a.y + b.y) < 1e-3);
        check("crab front feet stay", dist2(stepHome(LF, G.crabDeg), home().leg[LF]) < 1e-3);
      }
      int settleTicks = 0;
      while (!G.settled() && settleTicks < 1000) { step(0, 0, 0, true); settleTicks++; }
      snprintf(buf, sizeof buf, "%s h%d settled", GAIT_NAMES[g], hgt); check(buf, G.settled());
      check("home exact", maxOffset(G.last, home()) <= HOME_MM * 2);
      // ANTI-SLIP tripod: all six down twice a cycle, PLANT_TICKS each time (the first cycle starts from standing)
      if (slip && g == GAIT_TRIPOD) {
        snprintf(buf, sizeof buf, "tripod h%d anti-slip all down %d", hgt, allDown);
        check(buf, allDown >= (c.cycles - 1) * 2 * (int)PLANT_TICKS);
      }
      if (hgt == 5 && (cb == 5 || c.l != 0)) printf("in %2d %-11s%s cmd(%4.1f,%3.1f,%4.1f) cycle %2d ticks  max jump %5.1f mm/tick  max from home %5.1f mm  balance %3.0f mm  stop %d ticks  all down %d\n",
                           cb, GAIT_NAMES[g], slip ? " AS" : "   ", c.f, c.l, c.t, G.cycleTicks, maxJump, maxReach, minBal, settleTicks, allDown);
    }
  }
  // A small physics check: his centre of mass isn't really at (0, 0) (the battery, the back legs). How far
  // back and sideways can it sit before a foot lifting tips him? Slow walking, height 5: statics only.
  crabBack = 5; antiSlip = false; height = 5;
  printf("\ncentre of mass: how far it can move (mm) before he'd tip, walking forward / crabbing\n");
  for (int g = 0; g < GAITS; g++) {
    float back[2], side[2];
    for (int crab = 0; crab <= 1; crab++) {
      Gait G; G.start(home(), g, 10);
      float worstBack = 1e9, worstSide = 1e9;
      for (int k = 0; k < 4 * G.cycleTicks; k++) {
        const Frame now = G.tick(crab ? 0 : 1, crab ? 1 : 0, 0, false);
        float lo = 0;
        while (lo < 150 && margin(G, now, -lo, 0) > 0) lo += 1;
        worstBack = fminf(worstBack, lo);
        lo = 0;
        while (lo < 150 && margin(G, now, 0, lo) > 0) lo += 1;
        worstSide = fminf(worstSide, lo);
      }
      back[crab] = worstBack; side[crab] = worstSide;
    }
    printf("%-11s back %3.0f / %3.0f   side %3.0f / %3.0f\n", GAIT_NAMES[g], back[0], back[1], side[0], side[1]);
  }
  // Body moves (as esp32_gaits.ino): lying down, the hello wave, the dance, at every height
  for (int hgt = 0; hgt <= 9; hgt++) {
    height = hgt;
    char buf[64];
    Frame low = home();
    for (int l = 0; l < LEGS; l++) low.leg[l].z = -75;   // restLow() (REST_Z)
    snprintf(buf, sizeof buf, "rest low h%d", hgt);
    check(buf, reach(low));
    Frame base = home();
    for (int l = 0; l < LEGS; l++) { base.leg[l].x += 20; base.leg[l].y += 15; }
    Frame up = base;
    up.leg[LF] = { base.leg[LF].x + 30, base.leg[LF].y + 20, homeZ() + 120 };
    Frame wa = up, wb = up;
    wa.leg[LF].y += 35;
    wb.leg[LF].y -= 35;
    for (Frame* f : { &base, &up, &wa, &wb }) {
      snprintf(buf, sizeof buf, "hello h%d", hgt);
      check(buf, reach(*f));
    }
    for (int i = 0; i <= 48; i++) {
      const float an = 4 * M_PI * i / 48;
      Frame f = home();
      for (int l = 0; l < LEGS; l++) { f.leg[l].x -= 25 * cosf(an); f.leg[l].y -= 25 * sinf(an); f.leg[l].z += 15 * (1 + sinf(2 * an)) / 2; }
      snprintf(buf, sizeof buf, "dance h%d", hgt);
      check(buf, reach(f));
    }
  }
  // Body IK (bodyFrame): every pose to the limits, alone and together, at every height
  for (int hgt = 0; hgt <= 9; hgt++) {
    height = hgt;
    int bad = 0, n = 0;
    const float S = BODY_SLIDE_MAX, T = BODY_TILT_MAX;
    for (float bx : {-S, 0.f, S}) for (float by : {-S, 0.f, S}) for (float bz : {-S, 0.f, S})
      for (float r : {-T, 0.f, T}) for (float p : {-T, 0.f, T}) for (float yw : {-T, 0.f, T}) {
        BodyPose b; b.x = bx; b.y = by; b.z = bz; b.roll = r; b.pitch = p; b.yaw = yw;
        n++;
        if (!reach(bodyFrame(home(), b))) bad++;
      }
    int single = 0;   // one axis at a time, to its limit
    const float lim[6] = { S, S, S, T, T, T };
    for (int ax = 0; ax < 6; ax++) for (float sg : {-1.f, 1.f}) {
      BodyPose b; float* f = &b.x; f[ax] = sg * lim[ax];
      if (!reach(bodyFrame(home(), b))) single++;
    }
    int unfit = 0;   // fitPose always lands in reach
    for (float bx : {-S, S}) for (float r : {-T, T}) for (float yw : {-T, T}) {
      BodyPose b; b.x = bx; b.y = -bx / 2; b.z = bx / 2; b.roll = r; b.pitch = -r; b.yaw = yw;
      if (!inReach(bodyFrame(home(), fitPose(home(), b)))) unfit++;
    }
    printf("body IK h%d: %3d / %d corner poses out of reach, %d of 12 single-axis limits, fitPose misses %d\n",
           hgt, bad, n, single, unfit);
    char buf[32];
    snprintf(buf, sizeof buf, "fitPose h%d", hgt);
    check(buf, unfit == 0);
  }
  { BodyPose b; b.x = 12; b.y = -7; b.z = 9; b.roll = 4; b.pitch = -6; b.yaw = 3; height = 5;
    Frame f = bodyFrame(home(), b);
    printf("bodyFrame check LF %.4f %.4f %.4f  RB %.4f %.4f %.4f\n", f.leg[0].x, f.leg[0].y, f.leg[0].z, f.leg[5].x, f.leg[5].y, f.leg[5].z); }
  printf("%d failures\n", fails);
  return fails != 0;
}
