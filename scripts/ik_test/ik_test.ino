/*
  ================================================================================
  HEXAPOD MASTER CONTROL — REST → STAND → WALK  |  IK + MPU6050 + AVOIDANCE
  ================================================================================

  STARTUP SEQUENCE:
    1. REST   — IK-interpolated fold to rest pose
    2. WAIT   — hold 2 s, MPU leveling active
    3. STAND  — IK-interpolated rise to home pose
    4. WAIT   — hold 2 s, MPU leveling active
    5. WALK   — tripod gait, IK + MPU every frame, obstacle avoidance

  OBSTACLE AVOIDANCE:
    • Ultrasonic < 25 cm      → back + turn right
    • MPU pitch  > PITCH_LIMIT → cliff / steep ramp → back + turn right
    • MPU roll   > ROLL_LIMIT  → side tilt → strafe away from high side

  ALL MOTION goes through IK — no raw PWM writes anywhere.

  HARDWARE CALIBRATION (your notes):
  RIGHT:
    Back  Coxa  center=2000  fwd=1200  back=2500
    Front Coxa  center=1000  fwd=1200  back=2500
    Mid   Coxa  center=1500  fwd=1000  back=2500
    Femurs  up=2500  90=1500  down=500    pwm = 1500 + angle*scale
    Tibias  str=500  90=1500  JK=2500     pwm = 1500 + angle*scale
  LEFT:
    Back  Coxa  center=1000  fwd=1700  back=500
    Front Coxa  center=2000  fwd=2500  back=1200
    Mid   Coxa  center=1500  fwd=2500  back=1000
    Femurs  up=500   90=1500  down=2500   pwm = 1500 - angle*scale
    Tibias  str=2500 90=1500  JK=500      pwm = 1500 - angle*scale
*/

#include <SoftwareSerial.h>
#include <Wire.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
SoftwareSerial ssc(11, 10);   // SSC-32U  RX=11 TX=10
const int PIN_TRIG = 7;
const int PIN_ECHO = 6;

// ── MPU6050 ───────────────────────────────────────────────────────────────────
const int   MPU_ADDR    = 0x68;
const float G_SCALE     = 1.0f / 16384.0f; // ±2 g range → g per LSB
const float TILT_GAIN   = 0.35f;   // mm of Z correction per mm of reach per g
const float PITCH_LIMIT = 0.22f;   // g  (~13°) — triggers cliff avoidance
const float ROLL_LIMIT  = 0.18f;   // g  (~10°) — triggers strafe avoidance

// ── Leg geometry ──────────────────────────────────────────────────────────────
const float SEG_TIBIA  = 168.0f;
const float SEG_FEMUR  =  84.0f;
const float SEG_COXA_C = 168.0f;   // corner legs
const float SEG_COXA_M =  42.0f;   // middle legs
const float PWM_PER_RAD = 636.62f; // µs/rad  (1000 µs / (π/2))

// ── Gait parameters ───────────────────────────────────────────────────────────
const float STRIDE_FWD  = 38.0f;   // mm half-stride forward/back
const float STRIDE_LAT  = 28.0f;   // mm half-stride lateral (strafe)
const float STRIDE_TURN = 28.0f;   // mm Y-offset for turning
const float FOOT_LIFT   = 30.0f;   // mm swing clearance
const int   MS_GAIT     = 260;     // ms per tripod half-cycle
const int   MS_TRANS    = 80;      // ms per interpolation frame
const int   FRAMES_TRANS = 18;     // frames in rest<->stand transition
const int   OBSTACLE_CM  = 25;

// ── Mount angles (body frame) ─────────────────────────────────────────────────
const float MA_45  = 45.0f  * PI / 180.0f;
const float MA_90  = 90.0f  * PI / 180.0f;
const float MA_135 = 135.0f * PI / 180.0f;

// ================================================================================
// JOINT ANGLES STRUCT  (must be declared before any function that returns it)
// ================================================================================
struct JointAngles {
  float coxa;   // rad, horizontal pan
  float femur;  // rad, elevation (0 = horizontal, + = up)
  float tibia;  // rad, knee bend (0 = straight, + = bent)
  bool  valid;
};

// ================================================================================
// LEG CONFIGURATION
// ================================================================================
struct LegCfg {
  int   chCoxa, chFemur, chTibia;   // SSC-32 channel numbers
  int   legType;    // 0=back  1=mid  2=front
  bool  isLeft;
  float lCoxa, lFemur, lTibia;      // segment lengths mm
  float hX, hY, hZ;                 // HOME foot position mm (standing)
  float rX, rY, rZ;                 // REST foot position mm (folded)
  float mountAngle;                 // body-frame mount angle rad
};

LegCfg LEG[6] = {
//chCx chFe chTi  type  left    lCoxa        lFemur    lTibia   hX    hY    hZ    rX   rY    rZ    mountAngle
  { 1,  2,  3,    0,  true,  SEG_COXA_C, SEG_FEMUR, SEG_TIBIA, 240, -50, -120,  80, -20, -35,  MA_135 }, // 0 L-Back
  { 4,  5,  6,    1,  true,  SEG_COXA_M, SEG_FEMUR, SEG_TIBIA,  90,   0, -120,  50,   0, -35,  MA_90  }, // 1 L-Mid
  { 7,  8,  9,    2,  true,  SEG_COXA_C, SEG_FEMUR, SEG_TIBIA, 180,  60, -120,  80,  25, -35,  MA_45  }, // 2 L-Front
  {24, 25, 26,    2,  false, SEG_COXA_C, SEG_FEMUR, SEG_TIBIA, 180,  60, -120,  80,  25, -35,  MA_45  }, // 3 R-Front
  {27, 28, 29,    1,  false, SEG_COXA_M, SEG_FEMUR, SEG_TIBIA,  90,   0, -120,  50,   0, -35,  MA_90  }, // 4 R-Mid
  {30, 31, 32,    0,  false, SEG_COXA_C, SEG_FEMUR, SEG_TIBIA, 240, -50, -120,  80, -20, -35,  MA_135 }, // 5 R-Back
};

// ── Global tilt (g) ───────────────────────────────────────────────────────────
float gPitch = 0.0f;   // nose-up positive
float gRoll  = 0.0f;   // right-side-up positive

// ── SSC-32 command buffer ─────────────────────────────────────────────────────
char sBuf[720];

// ── State ─────────────────────────────────────────────────────────────────────
enum RobotState { ST_REST, ST_STAND, ST_WALK };
RobotState robotState = ST_REST;

// ================================================================================
// MPU6050
// ================================================================================
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);  // wake
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);  // ±2 g
  Wire.endTransmission(true);
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t ax = ((int16_t)Wire.read() << 8) | Wire.read();
  int16_t ay = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // az — not needed
  gPitch = ax * G_SCALE;
  gRoll  = ay * G_SCALE;
}

// Z correction for one leg: legs further from centre need more Z on the low side
float tiltZ(int i) {
  return -(LEG[i].hX * gPitch * TILT_GAIN)
         -(LEG[i].hY * gRoll  * TILT_GAIN);
}

// ================================================================================
// INVERSE KINEMATICS
// ================================================================================
JointAngles solveIK(int i, float wx, float wy, float wz) {
  JointAngles a;
  a.valid = false;

  float ma = LEG[i].mountAngle;
  float lc = LEG[i].lCoxa;
  float lf = LEG[i].lFemur;
  float lt = LEG[i].lTibia;

  // Rotate world-frame foot target into leg-local frame
  float lx =  wx * cosf(ma) + wy * sinf(ma);
  float ly = -wx * sinf(ma) + wy * cosf(ma);

  a.coxa = atan2f(ly, lx);

  float reach = sqrtf(lx*lx + ly*ly) - lc;  // horizontal reach past coxa pivot
  float depth = -wz;                          // positive = downward
  if (depth < 0.0f) depth = 0.0f;

  float s = sqrtf(reach*reach + depth*depth); // femur-pivot to foot
  float maxS = lf + lt - 1.0f;
  if (s > maxS) s = maxS;

  // Law of cosines: tibia angle at knee
  float cosC = (lf*lf + lt*lt - s*s) / (2.0f * lf * lt);
  cosC = constrain(cosC, -1.0f, 1.0f);
  a.tibia = acosf(cosC) - HALF_PI;   // 0 = straight

  // Femur angle
  float cosA = (lf*lf + s*s - lt*lt) / (2.0f * lf * s);
  cosA = constrain(cosA, -1.0f, 1.0f);
  a.femur = (atan2f(depth, reach) + acosf(cosA)) - HALF_PI;  // 0 = horizontal

  if (!isnan(a.coxa) && !isnan(a.femur) && !isnan(a.tibia)) a.valid = true;
  return a;
}

// ================================================================================
// PWM ENCODING
// ================================================================================
void encodeLeg(int i, float wx, float wy, float wz) {
  JointAngles a = solveIK(i, wx, wy, wz);
  if (!a.valid) return;

  bool isL = LEG[i].isLeft;
  int  tp  = LEG[i].legType;

  // ── Coxa ─────────────────────────────────────────────────────────────────────
  int pc;
  if (!isL) {  // RIGHT
    if      (tp == 0) { pc = 2000 - round(a.coxa * PWM_PER_RAD); pc = constrain(pc, 1200, 2500); }
    else if (tp == 2) { pc = 1000 + round(a.coxa * PWM_PER_RAD); pc = constrain(pc,  500, 1800); }
    else              { pc = 1500 - round(a.coxa * PWM_PER_RAD); pc = constrain(pc, 1000, 2500); }
  } else {     // LEFT
    if      (tp == 0) { pc = 1000 + round(a.coxa * PWM_PER_RAD); pc = constrain(pc,  500, 1700); }
    else if (tp == 2) { pc = 2000 + round(a.coxa * PWM_PER_RAD); pc = constrain(pc, 1200, 2500); }
    else              { pc = 1500 + round(a.coxa * PWM_PER_RAD); pc = constrain(pc, 1000, 2500); }
  }

  // ── Femur ─────────────────────────────────────────────────────────────────────
  // Right: up=2500 → +angle raises PWM.   Left: up=500 → +angle lowers PWM.
  int pf;
  if (!isL) pf = 1500 + round(a.femur * PWM_PER_RAD);
  else      pf = 1500 - round(a.femur * PWM_PER_RAD);
  pf = constrain(pf, 700, 2300);

  // ── Tibia ─────────────────────────────────────────────────────────────────────
  // Right: straight=500 → +bend raises PWM.  Left: straight=2500 → +bend lowers.
  int pt;
  if (!isL) pt = 1500 + round(a.tibia * PWM_PER_RAD);
  else      pt = 1500 - round(a.tibia * PWM_PER_RAD);
  pt = constrain(pt, 500, 2500);

  char tmp[30];
  sprintf(tmp, "#%dP%d#%dP%d#%dP%d",
    LEG[i].chCoxa, pc, LEG[i].chFemur, pf, LEG[i].chTibia, pt);
  strcat(sBuf, tmp);
}

// ── Transmit buffered command ─────────────────────────────────────────────────
void transmit(int ms) {
  char tail[16];
  sprintf(tail, "T%d\r\n", ms);
  strcat(sBuf, tail);
  ssc.print(sBuf);
  delay(ms + 12);
}

// ================================================================================
// POSE HELPERS
// ================================================================================

// Encode all 6 legs from explicit arrays, add tilt compensation, transmit
void sendPose(float px[6], float py[6], float pz[6], int ms) {
  sBuf[0] = '\0';
  mpuRead();
  for (int i = 0; i < 6; i++) {
    encodeLeg(i, px[i], py[i], pz[i] + tiltZ(i));
  }
  transmit(ms);
}

void fillHome(float px[6], float py[6], float pz[6]) {
  for (int i = 0; i < 6; i++) { px[i] = LEG[i].hX; py[i] = LEG[i].hY; pz[i] = LEG[i].hZ; }
}

void fillRest(float px[6], float py[6], float pz[6]) {
  for (int i = 0; i < 6; i++) { px[i] = LEG[i].rX; py[i] = LEG[i].rY; pz[i] = LEG[i].rZ; }
}

// Smoothly interpolate between two poses over `frames` steps
void interpPose(float ax[6], float ay[6], float az[6],
                float bx[6], float by[6], float bz[6],
                int frames, int msPerFrame) {
  for (int f = 1; f <= frames; f++) {
    float t = (float)f / (float)frames;
    float px[6], py[6], pz[6];
    for (int i = 0; i < 6; i++) {
      px[i] = ax[i] + (bx[i] - ax[i]) * t;
      py[i] = ay[i] + (by[i] - ay[i]) * t;
      pz[i] = az[i] + (bz[i] - az[i]) * t;
    }
    sendPose(px, py, pz, msPerFrame);
  }
}

// Hold a pose for durationMs, refreshing MPU correction at ~5 Hz
void holdPose(float px[6], float py[6], float pz[6], unsigned long durationMs) {
  unsigned long endAt = millis() + durationMs;
  while (millis() < endAt) {
    sendPose(px, py, pz, 180);
  }
}

// ================================================================================
// STARTUP SEQUENCE
// ================================================================================
void startupSequence() {
  float rpx[6], rpy[6], rpz[6];
  float hpx[6], hpy[6], hpz[6];
  fillRest(rpx, rpy, rpz);
  fillHome(hpx, hpy, hpz);

  Serial.println("[1/5] REST — folding down...");
  interpPose(hpx, hpy, hpz, rpx, rpy, rpz, FRAMES_TRANS, MS_TRANS);

  Serial.println("[2/5] WAIT — resting 2 s...");
  holdPose(rpx, rpy, rpz, 2000);

  Serial.println("[3/5] STAND — rising...");
  interpPose(rpx, rpy, rpz, hpx, hpy, hpz, FRAMES_TRANS, MS_TRANS);

  Serial.println("[4/5] WAIT — standing 2 s...");
  holdPose(hpx, hpy, hpz, 2000);

  Serial.println("[5/5] WALK — starting gait.");
}

// ================================================================================
// ULTRASONIC
// ================================================================================
long readCM() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long d = pulseIn(PIN_ECHO, HIGH, 22000);
  return (d == 0) ? 999L : (long)(d * 0.034f / 2.0f);
}

// ================================================================================
// GAIT ENGINE  — IK every frame, MPU every frame
// velFwd  : +1 fwd / -1 bwd
// velLat  : +1 strafe-right / -1 strafe-left
// velTurn : +1 turn-right / -1 turn-left
// phase   : 0 = group A swings (legs 0,2,4), 1 = group B swings (legs 1,3,5)
// ================================================================================
void gaitFrame(float velFwd, float velLat, float velTurn, int phase, int ms) {
  sBuf[0] = '\0';
  mpuRead();

  for (int i = 0; i < 6; i++) {
    bool inGroupA = (i == 0 || i == 2 || i == 4);
    bool swing    = (phase == 0) ? inGroupA : !inGroupA;

    // Stride vector for this leg
    float sX = 0.0f, sY = 0.0f;

    if (velFwd != 0.0f) {
      if (LEG[i].legType == 1) {
        // Middle legs: pure body-X stride
        sX = velFwd * STRIDE_FWD;
      } else {
        // Corner legs: stride along their 45° / 135° mount axis
        float ux = cosf(LEG[i].mountAngle);
        float uy = sinf(LEG[i].mountAngle);
        sX = velFwd * STRIDE_FWD * ux;
        sY = velFwd * STRIDE_FWD * uy;
      }
    }

    if (velLat  != 0.0f) { sY += velLat  * STRIDE_LAT; }
    if (velTurn != 0.0f) {
      float side = LEG[i].isLeft ? 1.0f : -1.0f;
      sY += velTurn * STRIDE_TURN * side;
    }

    // Foot target
    float tx = LEG[i].hX;
    float ty = LEG[i].hY;
    float tz = LEG[i].hZ;

    if (swing) {
      tx += sX * 0.5f;
      ty += sY * 0.5f;
      tz += FOOT_LIFT;           // IK will compute exact femur+tibia for this height
    } else {
      tx -= sX * 0.5f;           // stance: push rearward → body advances forward
      ty -= sY * 0.5f;
    }

    tz += tiltZ(i);              // MPU body-level correction applied before IK

    encodeLeg(i, tx, ty, tz);
  }
  transmit(ms);
}

// N full tripod cycles
void walk(float vF, float vL, float vT, int cycles) {
  for (int c = 0; c < cycles; c++) {
    gaitFrame(vF, vL, vT, 0, MS_GAIT);
    gaitFrame(vF, vL, vT, 1, MS_GAIT);
  }
}

// Stop and hold home pose
void stopWalk() {
  float px[6], py[6], pz[6];
  fillHome(px, py, pz);
  sendPose(px, py, pz, 300);
  delay(80);
}

// ================================================================================
// OBSTACLE AVOIDANCE
// ================================================================================
void doAvoidance(bool cliffDetected, bool rollDetected, float rollDir) {
  stopWalk();
  delay(120);

  if (cliffDetected) {
    Serial.println("[!] PITCH/CLIFF — back + turn right");
    walk(-1.0f, 0.0f, 0.0f, 3);
    stopWalk(); delay(100);
    walk( 0.0f, 0.0f, 1.0f, 4);
    stopWalk(); delay(100);

  } else if (rollDetected) {
    float lat = (rollDir > 0.0f) ? -1.0f : 1.0f;  // strafe away from high side
    Serial.print("[!] ROLL — strafe "); Serial.println(lat > 0 ? "right" : "left");
    walk(0.0f, lat, 0.0f, 3);
    stopWalk(); delay(100);

  } else {
    Serial.println("[!] OBSTACLE — back + turn right");
    walk(-1.0f, 0.0f, 0.0f, 3);
    stopWalk(); delay(100);
    walk( 0.0f, 0.0f, 1.0f, 4);
    stopWalk(); delay(100);
  }
}

// ================================================================================
// SETUP
// ================================================================================
void setup() {
  Serial.begin(9600);
  ssc.begin(9600);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  Wire.begin();
  mpuInit();
  delay(2000);  // power rail settle

  startupSequence();
  robotState = ST_WALK;
}

// ================================================================================
// MAIN LOOP
// ================================================================================
void loop() {
  long  dist       = readCM();
  mpuRead();

  bool ultraObs  = (dist > 0 && dist < OBSTACLE_CM);
  bool pitchObs  = (fabsf(gPitch) > PITCH_LIMIT);
  bool rollObs   = (fabsf(gRoll)  > ROLL_LIMIT);

  if (ultraObs || pitchObs || rollObs) {
    doAvoidance(pitchObs, rollObs, gRoll);
  } else {
    gaitFrame(1.0f, 0.0f, 0.0f, 0, MS_GAIT);
    gaitFrame(1.0f, 0.0f, 0.0f, 1, MS_GAIT);
  }

  while (ssc.available()) ssc.read();
}

/*
  ── MOTION API ──────────────────────────────────────────────────────────────
  walk( 1.0f,  0.0f,  0.0f, 1);   forward
  walk(-1.0f,  0.0f,  0.0f, 1);   backward
  walk( 0.0f,  1.0f,  0.0f, 1);   strafe right
  walk( 0.0f, -1.0f,  0.0f, 1);   strafe left
  walk( 0.0f,  0.0f,  1.0f, 1);   turn right
  walk( 0.0f,  0.0f, -1.0f, 1);   turn left
  walk( 0.7f,  0.0f,  0.4f, 1);   forward + curve right
  stopWalk();                       freeze
  ────────────────────────────────────────────────────────────────────────────
*/
