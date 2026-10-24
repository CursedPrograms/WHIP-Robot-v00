/*
  ik_gait.h - WHIP walks by inverse kinematics instead of recorded poses.

  Ported from Jakob Leander's hexapod (https://github.com/JakobLeander/hexapod,
  MIT licence, video series https://www.youtube.com/@jakobleander): WHIP is built
  on the same chassis, so his leg lengths, hip positions, IK and tripod key frames
  are used as they are. What's WHIP's own: the servo controller (RTrobot 32ch over
  serial, "#<ch>P<us>T<ms>", instead of a Pololu Maestro), the channel map, and a
  per-servo trim / direction set on the /calib page and kept in flash (NVS), so
  calibrating needs no reflash.

  Coordinates, in millimetres from the body centre: x forward, y left, z up
  (feet are below the body, so z is negative).

  Servo angles (degrees, -90..+90, 0 = the servo's middle, 1500 us + trim):
    hip   0 = the leg points straight out from the body, along the line from the
            body centre through that hip
    knee  0 = the thigh is level
    foot  0 = the foot hangs down from the thigh at 104 degrees (90 + 14: the
            curved foot's tip isn't straight below its servo)
  So "every servo in the middle" (the ZERO pose on /calib) is WHIP standing like
  a table: legs out, thighs level, feet down. Trim each servo until it looks
  like that, and the IK lines up.

  The leg maths, measurements and key frames are from Jakob Leander's hexapod,
  used under its licence:

  MIT License

  Copyright (c) 2025 JakobLeander

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

namespace ik {

// ---- the chassis (Leg.h, hexapod.ino, Poses.h of the reference) -----------
const float THIGH_LENGTH       = 84;    // knee servo horn to foot servo horn
const float FOOT_LENGTH        = 127;   // foot servo horn to the tip of the foot
const float HIP_TO_KNEE_LENGTH = 28;    // hip servo horn to knee servo horn
const float FOOT_TIP_DEG       = 14;    // the foot's tip isn't straight below its servo
const float RAD2DEG            = 180.0 / M_PI;

enum LegId   { LF, LM, LB, RF, RM, RB, LEGS };
enum JointId { HIP, KNEE, FOOT, JOINTS };
const char* const LEG_NAMES[LEGS]     = { "LF", "LM", "LB", "RF", "RM", "RB" };
const char* const JOINT_NAMES[JOINTS] = { "hip", "knee", "foot" };

// Hip joints, mm from the body centre
const float ANCHOR[LEGS][2] = { {74, 39}, {0, 64}, {-74, 39}, {74, -39}, {0, -64}, {-74, -39} };

// Poses (mm)
// Height: how far the feet are below the hip joints (height 0-9 on /calib maps between these).
// Jakob's original walks low, knees up and the body hanging between them. WHIP carries a 3S
// LiPo (a big RC-car pack) under the body, so he stands higher. Both kept, to fine-tune later
// on the real robot (TODO.md). Worked out for this chassis: every walking / turning frame is
// in reach down to -170 mm; past about -130 the thighs slope down instead of up.
const float JAKOB_HOME_Z_MIN = -60;    // his original: height 0
const float JAKOB_HOME_Z_MAX = -140;   // his original: height 9 (height 5 = -104, knees ~16 deg up)
const float WHIP_HOME_Z_MIN  = -90;    // WHIP's, for the battery: height 0 (check its clearance)
const float WHIP_HOME_Z_MAX  = -170;   // WHIP's: height 9 (height 5 = -134, thighs about level)
const float HOME_Z_MIN = WHIP_HOME_Z_MIN;   // in use: swap to JAKOB_* to try his stance
const float HOME_Z_MAX = WHIP_HOME_Z_MAX;
const float SLEEP_Z    = HOME_Z_MIN;   // resting: lowest stance, not on the floor (the battery's there)
const float LIFT       = 50;      // how high a foot lifts when stepping (60 still in reach, joints ~75 deg)
const float FB_X       = 150;     // front / back feet ahead / behind the centre
const float WIDTH_FB   = 123;     // front / back feet out to the side
const float WIDTH_M    = 177;     // middle feet out to the side
const float WALK_X     = 50;      // how far a foot moves back and forth per step (55 is out of reach at height 9)

// Servo: 1500 us + trim at 0 degrees. 10.49 us a degree is the reference's
// Maestro range (576-2464 us for 180 degrees); MG995/MG996R are close to it.
const float US_PER_DEG = 10.49;
const int   US_CENTER  = 1500;

struct Pos   { float x, y, z; };
struct Frame { Pos leg[LEGS]; };
struct ServoCal { uint8_t ch; bool rev; int16_t trim; };

// Defaults from WHIP's own gait data: side A (channels 1-9) is the left side,
// legs 1 / 4 / 7 front to back, hip / knee / foot = coxa / femur / tibia; side B
// (24-32) is the right, mirrored (femur and tibia turn the other way). The XML
// lifts legs 1, 7 and 27 together, which is the reference's first tripod (LF, LB,
// RM): a good sign. Check each with WIGGLE on /calib before trusting it.
const ServoCal DEFAULT_CAL[LEGS][JOINTS] = {
  { {1, false, 0},  {2, false, 0},  {3, false, 0}  },   // LF
  { {4, false, 0},  {5, false, 0},  {6, false, 0}  },   // LM
  { {7, false, 0},  {8, false, 0},  {9, false, 0}  },   // LB
  { {24, false, 0}, {25, true, 0},  {26, true, 0}  },   // RF
  { {27, false, 0}, {28, true, 0},  {29, true, 0}  },   // RM
  { {30, false, 0}, {31, true, 0},  {32, true, 0}  },   // RB
};

ServoCal cal[LEGS][JOINTS];
bool     calibrated = false;   // saved from /calib at least once (until then: DEFAULT_CAL, all trims 0)
uint8_t  height     = 5;       // 0-9 (body low to high)
uint8_t  speed      = 5;       // 0-9 (slow to fast)

Preferences prefs;

inline void loadDefaults() { memcpy(cal, DEFAULT_CAL, sizeof(cal)); }

inline void load() {
  loadDefaults();
  prefs.begin("whip_ik", true);
  if (prefs.getBytesLength("cal") == sizeof(cal)) prefs.getBytes("cal", cal, sizeof(cal));
  calibrated = prefs.getBool("calibrated", false);
  height     = prefs.getUChar("height", 5);
  speed      = prefs.getUChar("speed", 5);
  prefs.end();
}

inline void save() {
  prefs.begin("whip_ik", false);
  prefs.putBytes("cal", cal, sizeof(cal));
  prefs.putBool("calibrated", calibrated);
  prefs.putUChar("height", height);
  prefs.putUChar("speed", speed);
  prefs.end();
}

// ---- inverse kinematics (Leg::setFootPosition of the reference) -----------
// Foot position (body frame) -> hip / knee / foot angles. False if out of reach.
inline bool solve(uint8_t leg, Pos p, float out[JOINTS]) {
  const float ax = ANCHOR[leg][0], ay = ANCHOR[leg][1];
  const float anchorDeg = atan2f(ay, ax) * RAD2DEG;
  float x = p.x - ax, y = p.y - ay, z = p.z;     // anchor z = 0 (knee height)

  float hipDeg = atan2f(y, x) * RAD2DEG - anchorDeg;
  while (hipDeg > 180) hipDeg -= 360;            // the back legs' anchors sit near +-180
  while (hipDeg < -180) hipDeg += 360;

  // The knee joint is HIP_TO_KNEE_LENGTH out from the hip, towards the foot
  const float len = sqrtf(x * x + y * y);
  if (len < 1) return false;
  x = p.x - (ax + x * HIP_TO_KNEE_LENGTH / len);
  y = p.y - (ay + y * HIP_TO_KNEE_LENGTH / len);

  const float h = sqrtf(x * x + y * y);
  const float l = sqrtf(h * h + z * z);
  const float footDeg = acosf((THIGH_LENGTH * THIGH_LENGTH + FOOT_LENGTH * FOOT_LENGTH - l * l)
                              / (2 * THIGH_LENGTH * FOOT_LENGTH)) * RAD2DEG - 90 - FOOT_TIP_DEG;
  const float vB = acosf((l * l + THIGH_LENGTH * THIGH_LENGTH - FOOT_LENGTH * FOOT_LENGTH) / (2 * l * THIGH_LENGTH));
  const float kneeDeg = (vB + atan2f(z, h)) * RAD2DEG;

  if (isnan(hipDeg) || isnan(kneeDeg) || isnan(footDeg)) return false;
  out[HIP] = hipDeg; out[KNEE] = kneeDeg; out[FOOT] = footDeg;
  return true;
}

inline int toUs(uint8_t leg, uint8_t joint, float deg) {
  deg = constrain(deg, -90.0f, 90.0f);
  const ServoCal& c = cal[leg][joint];
  return US_CENTER + c.trim + (int)lroundf((c.rev ? -deg : deg) * US_PER_DEG);
}

// "#<ch>P<us>" for all 18 servos (no T: the caller adds it). False if any foot is out of reach.
inline bool line(const Frame& f, char* out, size_t n) {
  size_t used = 0;
  out[0] = '\0';
  for (uint8_t leg = 0; leg < LEGS; leg++) {
    float a[JOINTS];
    if (!solve(leg, f.leg[leg], a)) return false;
    for (uint8_t j = 0; j < JOINTS; j++) {
      int w = snprintf(out + used, n - used, "#%uP%d", cal[leg][j].ch, toUs(leg, j, a[j]));
      if (w < 0 || used + w >= n) return false;
      used += w;
    }
  }
  return true;
}

// Every servo at 0 degrees + its trim: the calibration pose
inline void zeroLine(char* out, size_t n) {
  size_t used = 0;
  out[0] = '\0';
  for (uint8_t leg = 0; leg < LEGS; leg++)
    for (uint8_t j = 0; j < JOINTS; j++)
      used += snprintf(out + used, n - used, "#%uP%d", cal[leg][j].ch, toUs(leg, j, 0));
}

// Ease in and out (smootherstep, 10t^3 - 15t^4 + 6t^5): 0 to 1 starting and ending at
// zero speed and zero acceleration, so a joint never jumps to full speed or stops dead
// between steps -- the jerk that worked the legs off their servo horns.
inline float ease(float t) { return t * t * t * (10 + t * (6 * t - 15)); }

inline Pos lerp(Pos a, Pos b, float t) { return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t }; }

inline Frame lerp(const Frame& a, const Frame& b, float t) {
  Frame f;
  for (uint8_t i = 0; i < LEGS; i++) f.leg[i] = lerp(a.leg[i], b.leg[i], t);
  return f;
}

// A stepping foot's path from a to b, peaking `lift` above them: a quintic Bezier
// through a, a, a+up, b+up, b, b. The doubled ends make the foot leave and land
// at zero speed and the path has no corners, so the servos speed up and slow down
// gradually instead of reversing at once. The peak is 10/16 of `up`, hence 1.6.
inline Pos swing(Pos a, Pos b, float lift, float t) {
  const float u = 1 - t;
  const float toB = ease(t);   // Bernstein 3 + 4 + 5 = smootherstep
  const float up  = 10 * t * t * u * u * 1.6f * lift;              // (Bernstein 2 + 3) * up
  return { a.x + (b.x - a.x) * toB, a.y + (b.y - a.y) * toB, a.z + (b.z - a.z) * toB + up };
}

// One step, `from` to `land`, t 0..1. `lift` is the key frame in between that says
// which feet step (the ones it raises) and how high; they arc over on swing(), the
// feet on the ground move in straight lines, eased the same way so all six feet
// start and stop together.
inline Frame step(const Frame& from, const Frame& lift, const Frame& land, float t) {
  Frame f;
  for (uint8_t i = 0; i < LEGS; i++) {
    const float up = lift.leg[i].z - land.leg[i].z;
    f.leg[i] = up > 1 ? swing(from.leg[i], land.leg[i], up, t) : lerp(from.leg[i], land.leg[i], ease(t));
  }
  return f;
}

// ---- key frames (Poses.h of the reference) --------------------------------
inline float homeZ() { return HOME_Z_MIN + (HOME_Z_MAX - HOME_Z_MIN) * height / 9.0f; }

// The home stance at height z, then each leg's offset
inline Frame stance(float zA, float zB, float dxA = 0, float dxB = 0) {
  // tripod A = LF, LB, RM; tripod B = RF, RB, LM. Middle legs step the other way to front / back.
  return { { { FB_X + dxA,  WIDTH_FB, zA }, { 0 - dxB,  WIDTH_M, zB }, { -FB_X + dxA,  WIDTH_FB, zA },
             { FB_X - dxB, -WIDTH_FB, zB }, { 0 + dxA, -WIDTH_M, zA }, { -FB_X - dxB, -WIDTH_FB, zB } } };
}

inline Frame home()  { float z = homeZ(); return stance(z, z); }
inline Frame sleepPose() { return stance(SLEEP_Z, SLEEP_Z); }

// Walking: W0 lift tripod A, W1 set it down ahead while B pushes back,
// W2 lift tripod B, W3 set it down ahead while A pushes back.
inline Frame W0() { float z = homeZ(); return stance(z + LIFT, z); }
inline Frame W1() { float z = homeZ(); return stance(z, z, WALK_X, WALK_X); }
inline Frame W2() { float z = homeZ(); return stance(z, z + LIFT); }
inline Frame W3() { float z = homeZ(); return stance(z, z, -WALK_X, -WALK_X); }

// Turning: tripod A lands turned anticlockwise, then tripod B turns back (R1, R3 from the reference)
inline Frame R0() { return W0(); }
inline Frame R1() {
  Frame f = home();
  f.leg[LF] = { 99, 167, f.leg[LF].z }; f.leg[LB] = { -183, 64, f.leg[LB].z }; f.leg[RM] = { 61, -166, f.leg[RM].z };
  return f;
}
inline Frame R2() { return W2(); }
inline Frame R3() {
  Frame f = home();
  f.leg[LM] = { -61, 166, f.leg[LM].z }; f.leg[RF] = { 183, -64, f.leg[RF].z }; f.leg[RB] = { -99, -167, f.leg[RB].z };
  return f;
}

// ms per interpolation step: speed 9 = 30 ms, speed 0 = 120 ms
inline uint16_t stepMs() { return 120 - speed * 10; }
// Interpolation steps between key frames, as in the reference: a foot only follows its
// path (a straight line, or the swing arc) if every joint is worked out again along the way.
const uint8_t STEPS = 5;
// Walking and turning: at 9600 baud a line takes ~150 ms to send whatever stepMs says,
// so speed sets how many moves a key frame takes instead: speed 0 = 6, speed 9 = 3.
inline uint8_t walkSteps() { return 6 - speed / 3; }

}  // namespace ik
