/*
  gaits.h - WHIP's gait engine: tripod, ripple, wave, quad, amble and metachronal from one set of rules.

  Every leg runs through the same cycle, each starting at its own point in it
  (its phase offset). For part of the cycle the foot is in the air, swinging
  forward to its landing spot; the rest of the time it is on the ground and
  slides back at the speed the body moves, so all the feet on the ground move
  together and none of them slips. The gaits only differ in how long a foot is
  in the air and which leg goes when:

    tripod  3 feet down, 2 groups of 3 step by turns         fastest
    ripple  4 feet down, a step runs front to back each side  looks the most like an insect
    wave    5 feet down, one leg at a time, back to front     slowest, easiest on the servos
    quad    4 feet down, 3 pairs step by turns, back to front a tetrapod: between tripod and ripple
    amble   4 feet down, 3 diagonal pairs (none straight across)  spreads the load differently
    metachronal  5 feet down, one leg at a time round the body   a "Mexican wave": tight corridors
  Every one keeps his centre at least 79 mm inside the feet that are down (gait_test checks it
  while he walks); lifting two legs straight across, or two neighbours in a row, puts it on the edge.

  Smooth on purpose (the legs worked loose on their horns when the old gait
  jerked them):
    - a swinging foot leaves and lands at the speed the ground feet move, so no
      foot changes speed at once (cubic Hermite, matched end speeds);
    - the body speeds up and slows down over a stance instead of at once;
    - stopping is done by stepping: the feet walk home, they're never dragged.

  Pure maths, no I/O: the sketch calls tick() once per move and sends the frame.
  Units: millimetres and radians, "per tick" = per move sent to the controller.
*/
#pragma once
#include "ik_gait.h"

namespace ik {

enum GaitId : uint8_t { GAIT_TRIPOD, GAIT_RIPPLE, GAIT_WAVE, GAIT_QUAD, GAIT_AMBLE, GAIT_METACHRONAL, GAITS };   // new ones go last: the number is saved
const char* const GAIT_NAMES[GAITS] = { "tripod", "ripple", "wave", "quad", "amble", "metachronal" };

// How many sixths of the cycle a foot is in the air, and where in the cycle (in
// sixths) each leg lifts: LF, LM, LB, RF, RM, RB.
struct GaitDef { uint8_t air; uint8_t lift[LEGS]; };
const GaitDef GAIT_DEFS[GAITS] = {
  { 3, { 0, 3, 0, 3, 0, 3 } },   // tripod: LF LB RM, then RF RB LM (as W0-W3)
  { 2, { 4, 2, 0, 1, 5, 3 } },   // ripple: LB LM LF on the left, RB RM RF half a cycle on
  { 1, { 2, 1, 0, 5, 4, 3 } },   // wave: LB LM LF RB RM RF
  { 2, { 4, 2, 0, 2, 0, 4 } },   // quad (tetrapod): LB+RM, then LM+RF, then LF+RB; always 2 up, 4 down
  { 2, { 4, 0, 2, 2, 4, 0 } },   // amble: LM+RB, then LB+RF, then LF+RM (diagonals, none straight across)
  { 1, { 2, 1, 0, 3, 4, 5 } },   // metachronal: LB LM LF RF RM RB, round the body
};

// A step reaches STRIDE_X ahead of / behind home (WALK_X less room for the swing
// to dip back before it goes forward), STRIDE_Y to the side, or turns TURN_RAD either way (the reference
// turns 20 degrees a step: +-10).
const float STRIDE_X = WALK_X * 0.9f;
const float STRIDE_Y = WALK_X * 0.5f;   // sideways: the middle legs reach in and out, and run out first
const float TURN_RAD = 10.0f / RAD2DEG * 0.9f;
const float HOME_MM  = 1.0f;   // a foot this close to home doesn't need a step to get there
// ANTI-SLIP (/gait?antislip=1, saved): a foot that lands carries his weight for PLANT_TICKS moves
// before the next one lifts (in tripod: all six down), so he never shifts his weight onto feet
// still settling. Same stride, a longer cycle: he walks slower.
const uint16_t PLANT_TICKS = 2;

inline float dist2(Pos a, Pos b) { return sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); }

inline float maxOffset(const Frame& a, const Frame& b) {
  float m = 0;
  for (uint8_t i = 0; i < LEGS; i++) m = max(m, dist2(a.leg[i], b.leg[i]) + fabsf(a.leg[i].z - b.leg[i].z));
  return m;
}

struct Gait {
  bool     running = false;
  uint8_t  id = GAIT_TRIPOD;
  bool     plant = false;      // started with antiSlip on
  uint16_t airTicks = 10;      // ticks a foot is in the air
  uint16_t cycleTicks = 20;    // ticks for every leg to step once
  uint16_t t = 0;              // tick in the cycle
  Frame    last;               // the frame tick() returned last
  Pos      foot[LEGS];
  Pos      liftOff[LEGS];      // where a swinging foot left the ground
  Frame    lead;               // FOOT LEAD (ik::footLead): each swinging foot further along its step...
  uint8_t  leadMask = 0;       // ... for these legs (a bit each): its foot servo goes by that, ahead of the thigh
  bool     air[LEGS];
  float    vx = 0, vy = 0, w = 0;   // body velocity now: mm, mm, rad per tick
  float    tx = 0, ty = 0, tw = 0;   // ... and where it's heading
  bool     homing = false;     // settle: the body has stopped, feet walking home
  uint16_t homeT = 0;

  uint16_t groundTicks() const { return cycleTicks - airTicks; }
  uint16_t liftTick(uint8_t leg) const { return GAIT_DEFS[id].lift[leg] * cycleTicks / 6; }

  // Start from the feet where they are (standing), with `air` ticks per swing.
  void start(const Frame& now, uint8_t gait, uint16_t airT) {
    id = gait < GAITS ? gait : GAIT_TRIPOD;
    airTicks = airT;
    plant = antiSlip;
    // the lifts are a sixth of the cycle apart: 6 / air of them per swing, each PLANT_TICKS longer
    cycleTicks = (airT + (plant ? PLANT_TICKS : 0)) * 6 / GAIT_DEFS[id].air;
    t = 0; vx = vy = w = tx = ty = tw = 0; homing = false;
    const float z = homeZ();
    for (uint8_t i = 0; i < LEGS; i++) { foot[i] = { now.leg[i].x, now.leg[i].y, z }; air[i] = false; }
    for (uint8_t i = 0; i < LEGS; i++) last.leg[i] = foot[i];
    running = true;
  }

  // Ground speed of a foot at p (relative to the body): the body's motion, backwards.
  Pos groundVel(Pos p) const { return { w * p.y - vx, -w * p.x - vy, 0 }; }

  float vMax()  const { return 2 * STRIDE_X / groundTicks(); }
  float vyMax() const { return 2 * STRIDE_Y / groundTicks(); }
  float wMax()  const { return 2 * TURN_RAD / groundTicks(); }

  // One tick closer to `target`. Speeding up takes a whole cycle: from standing, a
  // leg can wait nearly a cycle for its first step. Slowing down only ever shortens
  // a stride, so it's quicker: one swing.
  float toward(float v, float target, float full) const {
    const float r = (fabsf(target) < fabsf(v) || target * v < 0) ? 1.0f / airTicks : 1.0f / cycleTicks;
    v += constrain(target - v, -full * r, full * r);
    return fabsf(v) < full * 1e-4f ? 0 : v;
  }

  // Where foot `leg` lands, `inTicks` from now, so its time on the ground is centred
  // on home: half of how far the body will go meanwhile, ahead of home. Worked out
  // from the speed he'll have then, not now, or a foot landing while he speeds up
  // ends up pushed back past its stride.
  Pos target(uint8_t leg, uint16_t inTicks) const {
    float x = vx, y = vy, a = w, X = 0, Y = 0, A = 0;
    const uint16_t g = groundTicks();
    for (uint16_t i = 0; i < inTicks + g; i++) {
      x = toward(x, tx, vMax()); y = toward(y, ty, vyMax()); a = toward(a, tw, wMax());
      if (i >= inTicks) { X += x; Y += y; A += a; }
    }
    const Pos h = home().leg[leg];
    const float c = cosf(A / 2), s = sinf(A / 2);
    return { c * h.x - s * h.y + X / 2, s * h.x + c * h.y + Y / 2, h.z };
  }

  bool settled() const {
    if (vx != 0 || vy != 0 || w != 0) return false;
    const Frame h = home();
    for (uint8_t i = 0; i < LEGS; i++) if (air[i] || dist2(foot[i], h.leg[i]) > HOME_MM) return false;
    return true;
  }

  // One move. fwd / left / turn (anticlockwise) are -1..1; settle brings him to a stop
  // and walks every foot home (then settled() is true).
  const Frame& tick(float fwd, float left, float turn, bool settle) {
    if (settle) fwd = left = turn = 0;
    const float n = sqrtf(fwd * fwd + left * left) + fabsf(turn);   // walking and turning share the reach
    if (n > 1) { fwd /= n; left /= n; turn /= n; }
    tx = fwd * vMax(); ty = left * vyMax(); tw = turn * wMax();
    vx = toward(vx, tx, vMax());
    vy = toward(vy, ty, vyMax());
    w  = toward(w, tw, wMax());

    bool anyAir = false;
    for (uint8_t i = 0; i < LEGS; i++) anyAir |= air[i];
    // Stopped with every foot down: walk them home in two tripods (fast whatever the gait)
    if (settle && !homing && !anyAir && vx == 0 && vy == 0 && w == 0) { homing = true; homeT = 0; }

    t = (t + 1) % cycleTicks;
    if (homing) homeT++;
    leadMask = 0;
    const float z = homeZ(), c = cosf(w), s = sinf(w);
    const Frame h = home();

    for (uint8_t i = 0; i < LEGS; i++) {
      uint16_t k;   // ticks since this leg lifted: 1..airTicks in the air
      if (homing) {
        const bool groupB = GAIT_DEFS[GAIT_TRIPOD].lift[i] != 0;
        k = groupB ? (homeT > airTicks ? homeT - airTicks : 0) : homeT;
        if (k > airTicks) k = 0;
      } else {
        k = (t + cycleTicks - liftTick(i)) % cycleTicks;
      }
      const bool swingNow = k >= 1 && k <= airTicks;

      // lift only at the start of the swing (a late lift would jump along the path);
      // a foot already home doesn't step while he's stopping
      if (k == 1 && !air[i] && !(settle && dist2(foot[i], h.leg[i]) <= HOME_MM)) {
        air[i] = true;
        liftOff[i] = foot[i];
      }

      if (air[i]) {
        const float u = (float)k / airTicks;
        const Pos L = liftOff[i], T = target(i, airTicks - k);
        const Pos vL = groundVel(L), vT = groundVel(T);
        const float u2 = u * u, u3 = u2 * u;
        const float h00 = 2 * u3 - 3 * u2 + 1, h01 = 3 * u2 - 2 * u3;
        const float h10 = (u3 - 2 * u2 + u) * airTicks, h11 = (u3 - u2) * airTicks;
        const float up = 10 * u2 * (1 - u) * (1 - u) * 1.6f * LIFT;   // peaks at LIFT, as swing()
        foot[i] = { h00 * L.x + h01 * T.x + h10 * vL.x + h11 * vT.x,
                    h00 * L.y + h01 * T.y + h10 * vL.y + h11 * vT.y, z + up };
        if (footLead && k < airTicks && swingNow) {
          // the same path, further along: footLead 9 = the foot's servo there at ~half the swing
          const float v = min(1.0f, u * (1 + footLead * 0.1f)), v2 = v * v, v3 = v2 * v;
          const float g00 = 2 * v3 - 3 * v2 + 1, g01 = 3 * v2 - 2 * v3;
          const float g10 = (v3 - 2 * v2 + v) * airTicks, g11 = (v3 - v2) * airTicks;
          lead.leg[i] = { g00 * L.x + g01 * T.x + g10 * vL.x + g11 * vT.x,
                          g00 * L.y + g01 * T.y + g10 * vL.y + g11 * vT.y,
                          z + 10 * v2 * (1 - v) * (1 - v) * 1.6f * LIFT };
          leadMask |= 1 << i;
        }
        if (k >= airTicks || !swingNow) { foot[i] = T; air[i] = false; }   // landed
      } else {
        const Pos p = foot[i];   // on the ground: the body turns and moves over it
        foot[i] = { c * p.x + s * p.y - vx, -s * p.x + c * p.y - vy, z };
      }
    }
    if (homing && homeT >= 2 * airTicks) homing = false;

    for (uint8_t i = 0; i < LEGS; i++) last.leg[i] = foot[i];
    return last;
  }
};

}  // namespace ik
