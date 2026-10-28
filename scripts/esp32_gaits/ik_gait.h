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
#include "servo_config.h"   // each joint's servo type (servo_config.json, written by scripts/servo_config.py)

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
// Which way each leg points with its hip servo at 0 degrees (1500 us), anticlockwise from straight
// ahead: Jakob's, along the line from the body centre through the hip (atan2 of ANCHOR: 27.8
// degrees for the corners), as the old sketch (scripts/esp32) has it and walks forward with.
// 2026-10-27 to 28 the corners were set diagonal (45) and he "swam": every corner foot pushed
// 17 degrees off straight back, front and back ones against each other. Kept for reference:
// const float MOUNT_DEG[LEGS] = { 45, 90, 135, -45, -90, -135 };   // diagonal corners: not his
const float MOUNT_DEG[LEGS] = { 27.8f, 90, 152.2f, -27.8f, -90, -152.2f };

// Poses (mm)
// Height: how far the feet are below the hip joints (height 0-9 on /calib maps between these).
// Jakob's original walks low, knees up and the body hanging between them. WHIP carries a 3S
// LiPo (a big RC-car pack) under the body, so he stands higher. Both kept, to fine-tune later
// on the real robot (TODO.md). Worked out for this chassis: every walking / turning frame is
// in reach down to -170 mm; past about -130 the thighs slope down instead of up.
const float JAKOB_HOME_Z_MIN = -60;    // his original: height 0
const float JAKOB_HOME_Z_MAX = -140;   // his original: height 9 (height 5 = -104, knees ~16 deg up)
const float WHIP_HOME_Z_MIN  = -80;    // WHIP's, for the battery: height 0 (check its clearance; -90 until 2026-10-28)
const float WHIP_HOME_Z_MAX  = -160;   // WHIP's: height 9 (height 5 = -124; -170 / -134 until 2026-10-28: the user asked lower)
const float HOME_Z_MIN = WHIP_HOME_Z_MIN;   // in use: swap to JAKOB_* to try his stance
const float HOME_Z_MAX = WHIP_HOME_Z_MAX;
const float SLEEP_Z    = HOME_Z_MIN;   // resting: lowest stance, not on the floor (the battery's there)
const float LIFT       = 50;      // how high a foot lifts when stepping (60 still in reach, joints ~75 deg)
const float FB_X       = 160;     // front / back feet ahead / behind the centre (150 until 2026-10-26: legs bumped)
const float WIDTH_FB   = 123;     // front / back feet out to the side
const float WIDTH_M    = 177;     // middle feet out to the side
const float WALK_X     = 40;      // how far a foot moves back and forth per step (50 until 2026-10-26: with FB_X 150 the
                                  // neighbouring legs came ~68 mm apart and touched; now ~82, same furthest reach)

// Servo: its middle (us_center, 1500 on an MG995) + trim at 0 degrees, us_per_deg a degree (MG995 / MG996R:
// 10.49, the reference's Maestro range, 576-2464 us for 180 degrees). Per joint, from servo_config.h:
// a different servo is a change in servo_config.json, not here. These two are the MG995's, for the rest.
const float US_PER_DEG = 10.49;
const int   US_CENTER  = 1500;

struct Pos   { float x, y, z; };
struct Frame { Pos leg[LEGS]; };
struct ServoCal { uint8_t ch; bool rev; int16_t trim; };

// How WHIP is wired: each leg on the servo board's pins nearest it. Channels 1-3 sit by his
// right back leg, so side A (1-9) is the right side, back to front; side B (24-32) is the
// left, front to back (30-32 by his left back leg). hip / knee / foot = coxa / femur / tibia;
// side B's femur and tibia turn the other way (WHIP's own gait data), and every hip turns
// clockwise (seen from above) for a bigger pulse, against the IK's anticlockwise: rev.
// A direction and a trim belong to the servo, so they go with its channel.
const ServoCal DEFAULT_CAL[LEGS][JOINTS] = {
  { {24, true, 0}, {25, true, 0},  {26, true, 0}  },   // LF
  { {27, true, 0}, {28, true, 0},  {29, true, 0}  },   // LM
  { {30, true, 0}, {31, true, 0},  {32, true, 0}  },   // LB
  { {7, true, 0},  {8, false, 0},  {9, false, 0}  },   // RF
  { {4, true, 0},  {5, false, 0},  {6, false, 0}  },   // RM
  { {1, true, 0},  {2, false, 0},  {3, false, 0}  },   // RB
};
// 1: channels 1-9 were taken for the left legs front to back, 24-32 the right, hips not
// reversed. The two mistakes cancelled for walking forward, not for turning (he turned the
// wrong way, waved a back leg). 2: as wired, hips reversed: remap() moves a saved one over.
const uint8_t MAP_VERSION = 2;

ServoCal cal[LEGS][JOINTS];
bool     calibrated = false;   // saved from /calib at least once (until then: DEFAULT_CAL, all trims 0)
uint8_t  height     = 5;       // 0-9 (body low to high)
uint8_t  speed      = 5;       // 0-9 (slow to fast)
uint8_t  gait       = 0;       // 0 tripod, 1 ripple, 2 wave (gaits.h)
bool     weave      = false;   // obstacle mode: weave left / right as he walks, so the sonar sweeps wider
bool     antiSlip   = false;   // feet planted a moment before the next ones lift (gaits.h PLANT_TICKS)
uint8_t  footLead   = 0;       // 0-9: a stepping leg's foot servo runs ahead of its thigh (0: pure IK, gaits.h)
bool     wallGait   = false;   // obstacle mode: the next gait after every wall (not saved per wall: the saved gait stays)
bool     autoStart  = false;   // at boot: stand up and walk off in obstacle mode; off: every servo at 1500 us until you give input

Preferences prefs;

inline void loadDefaults() { memcpy(cal, DEFAULT_CAL, sizeof(cal)); }

inline void save();

// A calibration saved under an older leg map: each channel's direction and trim go to the leg
// that channel really drives now (a channel moved by hand on /calib keeps its place)
inline bool remap(uint8_t version) {
  if (version >= MAP_VERSION) return false;
  ServoCal old[LEGS][JOINTS];
  memcpy(old, cal, sizeof(old));
  loadDefaults();
  for (uint8_t l = 0; l < LEGS; l++)
    for (uint8_t j = 0; j < JOINTS; j++)
      for (uint8_t ol = 0; ol < LEGS; ol++)
        for (uint8_t oj = 0; oj < JOINTS; oj++)
          if (old[ol][oj].ch == DEFAULT_CAL[l][j].ch && oj == j) cal[l][j] = old[ol][oj];
  if (version < 2) for (uint8_t l = 0; l < LEGS; l++) cal[l][HIP].rev = !cal[l][HIP].rev;   // the hips, the other way
  return true;
}

inline void load() {
  loadDefaults();
  prefs.begin("whip_ik", true);
  const bool saved = prefs.getBytesLength("cal") == sizeof(cal);
  if (saved) prefs.getBytes("cal", cal, sizeof(cal));
  const uint8_t mapVersion = prefs.getUChar("map", 1);
  calibrated = prefs.getBool("calibrated", false);
  height     = prefs.getUChar("height", 5);
  speed      = prefs.getUChar("speed", 5);
  gait       = prefs.getUChar("gait", 0);
  weave      = false;   // WEAVE is off at every boot (the user didn't like it on by itself, 2026-10-28): not read back
  antiSlip   = prefs.getBool("antislip", false);
  autoStart  = prefs.getBool("autostart", false);
  wallGait   = prefs.getBool("wallgait", false);
  footLead   = min<uint8_t>(prefs.getUChar("footlead", 0), 9);
  prefs.end();
  if (saved && remap(mapVersion)) save();   // once: the trims follow their servos to the right legs
}

inline void save() {
  prefs.begin("whip_ik", false);
  prefs.putBytes("cal", cal, sizeof(cal));
  prefs.putUChar("map", MAP_VERSION);
  prefs.putBool("calibrated", calibrated);
  prefs.putUChar("height", height);
  prefs.putUChar("speed", speed);
  prefs.putUChar("gait", gait);
  prefs.putBool("antislip", antiSlip);
  prefs.putBool("autostart", autoStart);
  prefs.putBool("wallgait", wallGait);
  prefs.putUChar("footlead", footLead);
  prefs.end();
}

// ---- inverse kinematics (Leg::setFootPosition of the reference) -----------
// Foot position (body frame) -> hip / knee / foot angles. False if out of reach.
inline bool solve(uint8_t leg, Pos p, float out[JOINTS]) {
  const float ax = ANCHOR[leg][0], ay = ANCHOR[leg][1];
  const float anchorDeg = MOUNT_DEG[leg];          // the leg's direction at 1500 us
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
  const servocfg::Servo& s = servocfg::JOINT[leg][joint];   // this joint's servo type
  deg = constrain(deg, -s.degMax, s.degMax);
  const ServoCal& c = cal[leg][joint];
  const int us = s.usCenter + c.trim + (int)lroundf((c.rev ? -deg : deg) * s.usPerDeg);
  return constrain(us, (int)s.usMin, (int)s.usMax);
}

// "#<ch>P<us>" for all 18 servos (no T: the caller adds it). False if any foot is out of reach.
// FOOT LEAD: for the legs in leadMask, the foot servo's angle comes from lead (where that foot
// will be further along its step) instead of f, so the foot gets there before the thigh does.
inline bool line(const Frame& f, char* out, size_t n, const Frame* lead = nullptr, uint8_t leadMask = 0) {
  size_t used = 0;
  out[0] = '\0';
  for (uint8_t leg = 0; leg < LEGS; leg++) {
    float a[JOINTS];
    if (!solve(leg, f.leg[leg], a)) return false;
    if (lead && (leadMask >> leg & 1)) {
      float b[JOINTS];
      if (solve(leg, lead->leg[leg], b)) a[FOOT] = b[FOOT];
    }
    for (uint8_t j = 0; j < JOINTS; j++) {
      int w = snprintf(out + used, n - used, "#%uP%d", cal[leg][j].ch, toUs(leg, j, a[j]));
      if (w < 0 || used + w >= n) return false;
      used += w;
    }
  }
  return true;
}

// The joint angles of a frame (every leg solved). False if a foot is out of reach.
inline bool anglesOf(const Frame& f, float a[LEGS][JOINTS]) {
  for (uint8_t leg = 0; leg < LEGS; leg++) if (!solve(leg, f.leg[leg], a[leg])) return false;
  return true;
}

// A line straight from joint angles (folding the legs up to rest: no foot position for that)
inline void anglesLine(const float a[LEGS][JOINTS], char* out, size_t n) {
  size_t used = 0;
  out[0] = '\0';
  for (uint8_t leg = 0; leg < LEGS; leg++)
    for (uint8_t j = 0; j < JOINTS; j++)
      used += snprintf(out + used, n - used, "#%uP%d", cal[leg][j].ch, toUs(leg, j, a[leg][j]));
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

// ---- body IK: the feet stay planted, the body slides and tilts over them ----
// (mithi's hexapod-robot-simulator calls it the inverse kinematics page.) A pose: slide x (forward),
// y (left), z (up) mm, then roll (about x), pitch (about y), yaw (about z) degrees. Each foot, from
// where it stands (`feet`, body frame at rest), is seen from the moved body: f' = R^-1 (f - t).
// whip_body.py's body_frame() is the same maths, so Servo Studio, the controller and he agree.
struct BodyPose { float x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0; };
const float BODY_SLIDE_MAX = 40;   // mm either way: each alone is in reach at heights 0-7 (gait_test);
const float BODY_TILT_MAX  = 15;   // degrees either way      together they may not be, so fitPose() scales back

inline Frame bodyFrame(const Frame& feet, const BodyPose& b) {
  const float cr = cosf(b.roll / RAD2DEG), sr = sinf(b.roll / RAD2DEG);
  const float cp = cosf(b.pitch / RAD2DEG), sp = sinf(b.pitch / RAD2DEG);
  const float cy = cosf(b.yaw / RAD2DEG), sy = sinf(b.yaw / RAD2DEG);
  // R = Rz(yaw) Ry(pitch) Rx(roll); R^-1 = R^T
  const float r00 = cy * cp, r01 = cy * sp * sr - sy * cr, r02 = cy * sp * cr + sy * sr;
  const float r10 = sy * cp, r11 = sy * sp * sr + cy * cr, r12 = sy * sp * cr - cy * sr;
  const float r20 = -sp,     r21 = cp * sr,                r22 = cp * cr;
  Frame out;
  for (uint8_t i = 0; i < LEGS; i++) {
    const float dx = feet.leg[i].x - b.x, dy = feet.leg[i].y - b.y, dz = feet.leg[i].z - b.z;
    out.leg[i] = { r00 * dx + r10 * dy + r20 * dz, r01 * dx + r11 * dy + r21 * dz, r02 * dx + r12 * dy + r22 * dz };
  }
  return out;
}

inline BodyPose scaled(const BodyPose& b, float k) {
  BodyPose o; o.x = b.x * k; o.y = b.y * k; o.z = b.z * k; o.roll = b.roll * k; o.pitch = b.pitch * k; o.yaw = b.yaw * k;
  return o;
}

// Every foot solvable, every joint inside its servo's reach (servo_config.h)
inline bool inReach(const Frame& f) {
  for (uint8_t leg = 0; leg < LEGS; leg++) {
    float a[JOINTS];
    if (!solve(leg, f.leg[leg], a)) return false;
    for (uint8_t j = 0; j < JOINTS; j++) if (fabsf(a[j]) > servocfg::JOINT[leg][j].degMax) return false;
  }
  return true;
}

// The pose, scaled back towards none at all until every foot can reach (k from 1 down, by halving);
// none at all is always in reach. k: how much of the asked-for pose he can do.
inline BodyPose fitPose(const Frame& feet, const BodyPose& b, float* k = nullptr) {
  if (inReach(bodyFrame(feet, b))) { if (k) *k = 1; return b; }
  float lo = 0, hi = 1;
  for (uint8_t i = 0; i < 10; i++) {
    const float mid = (lo + hi) / 2;
    if (inReach(bodyFrame(feet, scaled(b, mid)))) lo = mid; else hi = mid;
  }
  if (k) *k = lo;
  return scaled(b, lo);
}

inline BodyPose clampPose(BodyPose b) {
  b.x = constrain(b.x, -BODY_SLIDE_MAX, BODY_SLIDE_MAX);
  b.y = constrain(b.y, -BODY_SLIDE_MAX, BODY_SLIDE_MAX);
  b.z = constrain(b.z, -BODY_SLIDE_MAX, BODY_SLIDE_MAX);
  b.roll = constrain(b.roll, -BODY_TILT_MAX, BODY_TILT_MAX);
  b.pitch = constrain(b.pitch, -BODY_TILT_MAX, BODY_TILT_MAX);
  b.yaw = constrain(b.yaw, -BODY_TILT_MAX, BODY_TILT_MAX);
  return b;
}

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
