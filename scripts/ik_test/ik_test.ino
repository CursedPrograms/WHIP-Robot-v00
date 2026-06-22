/*
  ================================================================================
  HEXAPOD MASTER CONTROL — REST → STAND → WALK  |  IK + MPU6050 + AVOIDANCE
  ================================================================================

  STARTUP SEQUENCE:
    1. REST     — IK-interpolated crouch to folded/resting pose
    2. WAIT     — hold rest, MPU leveling active, 2 seconds
    3. STAND    — IK-interpolated rise to standing home pose
    4. WAIT     — hold stand, MPU leveling active, 2 seconds
    5. WALK     — tripod gait, full IK every frame, MPU leveling every frame

  OBSTACLE AVOIDANCE (runs continuously during WALK):
    • Ultrasonic < 25 cm   → stop, back up, turn right, resume
    • MPU pitch > threshold → cliff/steep drop detected, same avoidance
    • MPU roll  > threshold → side tilt, strafe away from high side

  ALL MOTION PATH:
    Every pose (rest, stand, walk swing/stance) is a 3D foot target in mm.
    solveIK() converts each foot target to (coxa, femur, tibia) joint angles.
    encodeLeg() converts angles to PWM using your hardware calibration.
    Nothing moves without going through IK.

  ─────────────────────────────────────────────────────────────────────────────
  HARDWARE CALIBRATION (your notes — do not change these):
  ─────────────────────────────────────────────────────────────────────────────
  RIGHT:
    Back  Coxa  center=2000  fwd=1200  back=2500
    Front Coxa  center=1000  fwd=1200  back=2500  (45° forward mount)
    Mid   Coxa  center=1500  fwd=1000  back=2500
    Femurs  up=2500  90°=1500  down=500   → pwm = 1500 + angle*scale
    Tibias  straight=500  90°=1500  JK=2500 → pwm = 1500 + angle*scale
  LEFT:
    Back  Coxa  center=1000  fwd=1700  back=500
    Front Coxa  center=2000  fwd=2500  back=1200
    Mid   Coxa  center=1500  fwd=2500  back=1000
    Femurs  up=500  90°=1500  down=2500  → pwm = 1500 - angle*scale
    Tibias  straight=2500  90°=1500  JK=500 → pwm = 1500 - angle*scale
  ─────────────────────────────────────────────────────────────────────────────
*/

#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial ssc(11, 10);   // SSC-32 RX=11 TX=10

// ── Pins ─────────────────────────────────────────────────────────────────────
const int TRIG     = 7;
const int ECHO     = 6;

// ── MPU6050 ──────────────────────────────────────────────────────────────────
const int   MPU_ADDR      = 0x68;
const float G_SCALE       = 1.0f / 16384.0f;  // ±2g range
// Tilt compensation: each mm of leg distance from body centre adds this many
// mm of Z correction per g of tilt.  Tune 0.2–0.6.
const float TILT_GAIN     = 0.35f;
// Thresholds for tilt-triggered avoidance (g units, ~0.17g ≈ 10°)
const float PITCH_LIMIT   = 0.22f;   // front/back tilt — cliff or ramp
const float ROLL_LIMIT    = 0.18f;   // side tilt — falling sideways

// ── Geometry ─────────────────────────────────────────────────────────────────
const float LT   = 168.0f;  // tibia  mm
const float LF   =  84.0f;  // femur  mm
const float LC_C = 168.0f;  // coxa, corner legs
const float LC_M =  42.0f;  // coxa, middle legs

const float PWM_RAD = 636.62f;   // µs per radian  (1000µs / (π/2))

// ── Gait tuning ──────────────────────────────────────────────────────────────
const float STRIDE   = 38.0f;   // mm half-stride forward/back
const float STRIDE_S = 28.0f;   // mm half-stride lateral (strafe)
const float STRIDE_T = 28.0f;   // mm lateral offset for turning
const float LIFT     = 30.0f;   // mm foot clearance during swing
const int   T_GAIT   = 260;     // ms per tripod half-cycle
const int   T_TRANS  = 80;      // ms per interpolation frame (rest↔stand)
const int   INTER_N  = 18;      // frames in rest↔stand transition
const int   OBSTACLE_CM = 25;

// ── Mount angles ─────────────────────────────────────────────────────────────
const float MA_45  = 45.0f  * PI / 180.0f;
const float MA_90  = 90.0f  * PI / 180.0f;
const float MA_135 = 135.0f * PI / 180.0f;

// ================================================================================
// LEG TABLE
// ================================================================================
struct Leg {
  int   cx, fx, tx;        // servo channels
  int   type;              // 0=back 1=mid 2=front
  bool  left;
  float lc, lf, lt;        // segment lengths mm
  float hX, hY, hZ;        // HOME foot position (standing)
  float rX, rY, rZ;        // REST foot position (folded/crouched)
  float ma;                // mount angle rad
};

/*
  HOME (hZ = -120 mm  → comfortably standing)
  REST  — coxa centred, femur nearly up, tibia tucked:
          rX same as home but shorter reach, rZ = -30 mm (high up / crouched)
*/
Leg L[6] = {
//  cx  fx  tx  typ  left    lc    lf   lt     hX    hY    hZ      rX    rY    rZ     ma
  { 1,  2,  3,   0, true,  LC_C,  LF,  LT,  240, -50, -120,    80, -20,  -35,  MA_135 }, // 0 L-Back
  { 4,  5,  6,   1, true,  LC_M,  LF,  LT,   90,   0, -120,    50,   0,  -35,  MA_90  }, // 1 L-Mid
  { 7,  8,  9,   2, true,  LC_C,  LF,  LT,  180,  60, -120,    80,  25,  -35,  MA_45  }, // 2 L-Front
  {24, 25, 26,   2, false, LC_C,  LF,  LT,  180,  60, -120,    80,  25,  -35,  MA_45  }, // 3 R-Front
  {27, 28, 29,   1, false, LC_M,  LF,  LT,   90,   0, -120,    50,   0,  -35,  MA_90  }, // 4 R-Mid
  {30, 31, 32,   0, false, LC_C,  LF,  LT,  240, -50, -120,    80, -20,  -35,  MA_135 }, // 5 R-Back
};

// ── Live tilt (updated before every frame) ───────────────────────────────────
float gTiltX = 0.0f, gTiltY = 0.0f;   // raw accel g values

// ── Serial buffer ────────────────────────────────────────────────────────────
char buf[720];

// ── State machine ────────────────────────────────────────────────────────────
enum State { ST_REST, ST_RISING, ST_STAND, ST_WALK };
State state = ST_REST;

// ================================================================================
// MPU6050
// ================================================================================
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);   // wake
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);   // ±2g
  Wire.endTransmission(true);
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  /*int16_t az =*/ Wire.read(); Wire.read();   // unused
  gTiltX = ax * G_SCALE;   // positive = nose-up pitch
  gTiltY = ay * G_SCALE;   // positive = right-side roll
}

// ── Tilt Z-offset for one leg ─────────────────────────────────────────────────
// The further a leg is from centre in the tilt direction, the more Z it needs.
float tiltComp(int i) {
  return -(L[i].hX * gTiltX * TILT_GAIN)
         -(L[i].hY * gTiltY * TILT_GAIN);
}

// ================================================================================
// INVERSE KINEMATICS
// Input : body-frame foot target (wx, wy, wz) in mm
// Output: joint angles in radians  (0 = mechanical neutral)
//   coxa  — horizontal pan    (0 = straight ahead of mount)
//   femur — elevation          (0 = horizontal, + = up)
//   tibia — knee bend          (0 = straight, + = bent)
// ================================================================================
struct JA { float c, f, t; bool ok; };

JA ik(int i, float wx, float wy, float wz) {
  JA a; a.ok = false;
  float ma = L[i].ma;
  float lc = L[i].lc, lf = L[i].lf, lt = L[i].lt;

  // Rotate body-frame target into leg-local frame
  float lx =  wx * cosf(ma) + wy * sinf(ma);
  float ly = -wx * sinf(ma) + wy * cosf(ma);

  a.c = atan2f(ly, lx);                    // coxa pan

  float r     = sqrtf(lx*lx + ly*ly) - lc; // reach past coxa pivot
  float depth = -wz;                        // positive downward
  if (depth < 0.0f) depth = 0.0f;

  float s = sqrtf(r*r + depth*depth);       // femur-pivot → foot distance
  float maxS = lf + lt - 1.0f;
  if (s > maxS) s = maxS;

  float cosC = (lf*lf + lt*lt - s*s) / (2.0f*lf*lt);
  cosC = constrain(cosC, -1.0f, 1.0f);
  a.t = acosf(cosC) - HALF_PI;             // tibia (0 = straight)

  float cosA = (lf*lf + s*s - lt*lt) / (2.0f*lf*s);
  cosA = constrain(cosA, -1.0f, 1.0f);
  a.f = (atan2f(depth, r) + acosf(cosA)) - HALF_PI;  // femur (0 = horizontal)

  if (!isnan(a.c) && !isnan(a.f) && !isnan(a.t)) a.ok = true;
  return a;
}

// ================================================================================
// PWM ENCODING  (hardware calibration)
// ================================================================================
void encLeg(int i, float wx, float wy, float wz) {
  JA a = ik(i, wx, wy, wz);
  if (!a.ok) return;

  bool  sl = L[i].left;
  int   tp = L[i].type;

  // ── Coxa ────────────────────────────────────────────────────────────────────
  int pc;
  if (!sl) {  // RIGHT
    if      (tp==0){ pc = 2000 - round(a.c * PWM_RAD); pc = constrain(pc,1200,2500); }
    else if (tp==2){ pc = 1000 + round(a.c * PWM_RAD); pc = constrain(pc, 500,1800); }
    else           { pc = 1500 - round(a.c * PWM_RAD); pc = constrain(pc,1000,2500); }
  } else {    // LEFT
    if      (tp==0){ pc = 1000 + round(a.c * PWM_RAD); pc = constrain(pc, 500,1700); }
    else if (tp==2){ pc = 2000 + round(a.c * PWM_RAD); pc = constrain(pc,1200,2500); }
    else           { pc = 1500 + round(a.c * PWM_RAD); pc = constrain(pc,1000,2500); }
  }

  // ── Femur ───────────────────────────────────────────────────────────────────
  // Right: up=2500 → +angle raises PWM.  Left: up=500 → +angle lowers PWM.
  int pf;
  if (!sl) pf = 1500 + round(a.f * PWM_RAD);
  else     pf = 1500 - round(a.f * PWM_RAD);
  pf = constrain(pf, 700, 2300);

  // ── Tibia ───────────────────────────────────────────────────────────────────
  // Right: straight=500 → +bend raises PWM.  Left: straight=2500 → +bend lowers.
  int pt;
  if (!sl) pt = 1500 + round(a.t * PWM_RAD);
  else     pt = 1500 - round(a.t * PWM_RAD);
  pt = constrain(pt, 500, 2500);

  char tmp[28];
  sprintf(tmp, "#%dP%d#%dP%d#%dP%d", L[i].cx,pc, L[i].fx,pf, L[i].tx,pt);
  strcat(buf, tmp);
}

void send(int ms) {
  char tail[16]; sprintf(tail,"T%d\r\n",ms);
  strcat(buf,tail);
  ssc.print(buf);
  delay(ms + 12);
}

// ── Encode all 6 legs from explicit positions + tilt, then transmit ───────────
void poseAndSend(float px[6], float py[6], float pz[6], int ms) {
  buf[0] = '\0';
  mpuRead();
  for (int i=0;i<6;i++) encLeg(i, px[i], py[i], pz[i] + tiltComp(i));
  send(ms);
}

// ================================================================================
// POSE HELPERS
// ================================================================================

// Fill position arrays from the home (standing) pose
void fillHome(float px[6], float py[6], float pz[6]) {
  for (int i=0;i<6;i++){ px[i]=L[i].hX; py[i]=L[i].hY; pz[i]=L[i].hZ; }
}

// Fill position arrays from the rest (folded) pose
void fillRest(float px[6], float py[6], float pz[6]) {
  for (int i=0;i<6;i++){ px[i]=L[i].rX; py[i]=L[i].rY; pz[i]=L[i].rZ; }
}

// Linearly interpolate between two poses over INTER_N frames
// t=0 → from,  t=1 → to
void interpolatePose(
  float fx[6],float fy[6],float fz[6],  // from
  float tx[6],float ty[6],float tz[6],  // to
  int frames, int msPerFrame)
{
  for (int f=1; f<=frames; f++) {
    float t = (float)f / (float)frames;
    float px[6], py[6], pz[6];
    for (int i=0;i<6;i++){
      px[i] = fx[i] + (tx[i]-fx[i])*t;
      py[i] = fy[i] + (ty[i]-fy[i])*t;
      pz[i] = fz[i] + (tz[i]-fz[i])*t;
    }
    poseAndSend(px,py,pz, msPerFrame);
  }
}

// Hold a pose for durationMs, refreshing MPU correction every 200ms
void holdPose(float px[6],float py[6],float pz[6], unsigned long durationMs) {
  unsigned long end = millis() + durationMs;
  while (millis() < end) {
    poseAndSend(px,py,pz, 180);
  }
}

// ================================================================================
// STARTUP SEQUENCE: REST → WAIT → STAND → WAIT
// ================================================================================
void doStartupSequence() {
  float rpx[6],rpy[6],rpz[6];
  float hpx[6],hpy[6],hpz[6];
  fillRest(rpx,rpy,rpz);
  fillHome(hpx,hpy,hpz);

  Serial.println("[1] REST — folding to rest pose...");
  // Start from wherever servos are (treat as home initially after power-on)
  interpolatePose(hpx,hpy,hpz, rpx,rpy,rpz, INTER_N, T_TRANS);

  Serial.println("[2] WAIT — holding rest for 2s...");
  holdPose(rpx,rpy,rpz, 2000);

  Serial.println("[3] STAND — rising to standing pose...");
  interpolatePose(rpx,rpy,rpz, hpx,hpy,hpz, INTER_N, T_TRANS);

  Serial.println("[4] WAIT — holding stand for 2s...");
  holdPose(hpx,hpy,hpz, 2000);

  Serial.println("[5] WALK — starting gait...");
}

// ================================================================================
// ULTRASONIC
// ================================================================================
long readCM() {
  digitalWrite(TRIG,LOW);  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  long d = pulseIn(ECHO,HIGH,22000);
  return (d==0) ? 999 : (d * 0.034f / 2.0f);
}

// ================================================================================
// FULL IK GAIT ENGINE
// ────────────────────────────────────────────────────────────────────────────────
//  velFwd  : +1 forward / -1 backward / 0 none
//  velLat  : +1 strafe-right / -1 strafe-left / 0 none
//  velTurn : +1 turn-right / -1 turn-left / 0 none
//
//  Each leg swings (lifts + advances) or stances (pushes body forward).
//  Foot targets are real 3D positions resolved through IK every single frame.
//  MPU tilt compensation added to Z before IK so the body stays level.
// ================================================================================
void gaitFrame(float velFwd, float velLat, float velTurn, int phase, int ms) {
  buf[0] = '\0';
  mpuRead();

  for (int i=0;i<6;i++) {
    // ── Which group swings this phase? (Tripod A={0,2,4}  B={1,3,5}) ─────────
    bool inA   = (i==0||i==2||i==4);
    bool swing = (phase==0) ? inA : !inA;

    // ── Stride vector for this leg ─────────────────────────────────────────────
    // Forward stride: corner legs step along their 45°/135° mount axis so the
    // foot traces a line parallel to the direction of travel.
    // Middle legs step pure X (body axis).
    float sX=0, sY=0;

    if (velFwd != 0.0f) {
      if (L[i].type == 1) {                    // middle — pure X
        sX = velFwd * STRIDE;
      } else {                                  // corners — along mount axis
        float ma = L[i].ma;
        float ux = cosf(ma), uy = sinf(ma);    // unit vector of mount direction
        sX = velFwd * STRIDE * ux;
        sY = velFwd * STRIDE * uy;
      }
    }

    // Lateral (strafe): all legs shift pure Y
    if (velLat != 0.0f) {
      sY += velLat * STRIDE_S;
    }

    // Turn: opposite Y sign on each side creates differential rotation
    if (velTurn != 0.0f) {
      float side = L[i].left ? 1.0f : -1.0f;
      sY += velTurn * STRIDE_T * side;
    }

    // ── Foot target ────────────────────────────────────────────────────────────
    float tx = L[i].hX;
    float ty = L[i].hY;
    float tz = L[i].hZ;

    if (swing) {
      // Advance half-stride forward and lift
      tx += sX * 0.5f;
      ty += sY * 0.5f;
      tz += LIFT;               // ← IK resolves the exact femur/tibia needed
    } else {
      // Push body forward: foot moves to rear half of stride
      tx -= sX * 0.5f;
      ty -= sY * 0.5f;
      // tz at ground — IK keeps it there
    }

    // ── MPU level compensation ─────────────────────────────────────────────────
    tz += tiltComp(i);

    encLeg(i, tx, ty, tz);
  }
  send(ms);
}

// N full tripod cycles
void walk(float vF, float vL, float vT, int cycles) {
  for (int c=0;c<cycles;c++) {
    gaitFrame(vF,vL,vT, 0, T_GAIT);
    gaitFrame(vF,vL,vT, 1, T_GAIT);
  }
}

// Immediate stop — go to home with tilt correction
void stopWalk() {
  float px[6],py[6],pz[6]; fillHome(px,py,pz);
  poseAndSend(px,py,pz, 300);
  delay(80);
}

// ================================================================================
// OBSTACLE AVOIDANCE
// ── Called whenever ultrasonic or MPU detects a problem while walking.
// ── All moves go through the same gait/IK engine.
// ================================================================================
void avoidObstacle(bool cliffDetected, bool rollDetected, float rollDir) {
  stopWalk();
  delay(120);

  if (cliffDetected) {
    // Nose-down or steep pitch — back away
    Serial.println("[!] CLIFF/STEEP — backing up");
    walk(-1.0f, 0.0f, 0.0f, 3);
    stopWalk(); delay(100);
    walk( 0.0f, 0.0f, 1.0f, 4);   // turn right
    stopWalk(); delay(100);

  } else if (rollDetected) {
    // Side tilt — strafe away from the high side
    // rollDir > 0 means right side is high → strafe left (-1)
    float lat = (rollDir > 0.0f) ? -1.0f : 1.0f;
    Serial.print("[!] ROLL — strafing "); Serial.println(lat>0?"right":"left");
    walk( 0.0f, lat, 0.0f, 3);
    stopWalk(); delay(100);

  } else {
    // Ultrasonic obstacle ahead
    Serial.println("[!] OBSTACLE — back + turn");
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
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Wire.begin();
  mpuInit();
  delay(2000);   // let power rails stabilise

  // ── Startup: rest → wait → stand → wait → start walking ──────────────────
  doStartupSequence();
  state = ST_WALK;
}

// ================================================================================
// MAIN LOOP — walking with continuous avoidance
// ================================================================================
void loop() {

  // ── 1. Read sensors ──────────────────────────────────────────────────────────
  long   dist  = readCM();
  mpuRead();   // updates gTiltX, gTiltY

  bool ultraObs  = (dist > 0 && dist < OBSTACLE_CM);
  bool pitchObs  = (fabsf(gTiltX) > PITCH_LIMIT);
  bool rollObs   = (fabsf(gTiltY) > ROLL_LIMIT);

  // ── 2. Decide ────────────────────────────────────────────────────────────────
  if (ultraObs || pitchObs || rollObs) {
    avoidObstacle(pitchObs, rollObs, gTiltY);
  } else {
    // ── 3. Normal forward walk — one full tripod cycle ────────────────────────
    //     MPU tilt compensation is applied inside gaitFrame() automatically.
    gaitFrame(1.0f, 0.0f, 0.0f, 0, T_GAIT);
    gaitFrame(1.0f, 0.0f, 0.0f, 1, T_GAIT);
  }

  // Drain stale servo controller replies
  while (ssc.available()) ssc.read();
}

/*
  ================================================================================
  MOTION API — call these from loop() for RC/Bluetooth control:
  ─────────────────────────────────────────────────────────────────────────────
  walk( 1.0f,  0.0f,  0.0f, 1);  // Forward
  walk(-1.0f,  0.0f,  0.0f, 1);  // Backward
  walk( 0.0f,  1.0f,  0.0f, 1);  // Strafe right
  walk( 0.0f, -1.0f,  0.0f, 1);  // Strafe left
  walk( 0.0f,  0.0f,  1.0f, 1);  // Turn right
  walk( 0.0f,  0.0f, -1.0f, 1);  // Turn left
  walk( 0.7f,  0.0f,  0.4f, 1);  // Forward while curving right
  stopWalk();                     // Freeze in standing pose

  IK + MPU compensation apply automatically on every call.
  ================================================================================
*/
