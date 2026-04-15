// ============================================================
//  Hexapod Controller
//  Board   : ESP32 + 32ch PCA9685 Servo Board (I2C 0x40)
//  Input   : PS2 Wireless Controller
//  Servos  : 18x MG995 @ 6V
// ============================================================
//  Required Libraries (install via Library Manager):
//    - Adafruit PWM Servo Driver Library (PCA9685)
//    - PS2X_lib by Bill Porter
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// ── PCA9685 ─────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ      50       // 50Hz for analog servos
#define SERVO_MIN       102      // ~500us  (0°)
#define SERVO_MAX       512      // ~2500us (180°)
#define SERVO_MID       307      // ~1500us (90°)

// ── PS2 Pins (ESP32) ─────────────────────────────────────────
#define PS2_DAT         19
#define PS2_CMD         23
#define PS2_ATT          5
#define PS2_CLK         18

PS2X ps2x;

// ── Leg / Servo Layout ───────────────────────────────────────
//  Each leg has 3 servos: COXA (hip), FEMUR (shoulder), TIBIA (knee)
//  Channels are 0-indexed on the PCA9685 board
//
//  Leg 1 = Front  Left  → channels  0,  1,  2
//  Leg 2 = Middle Left  → channels  3,  4,  5
//  Leg 3 = Back   Left  → channels  6,  7,  8
//  Leg 4 = Front  Right → channels  9, 10, 11
//  Leg 5 = Middle Right → channels 12, 13, 14
//  Leg 6 = Back   Right → channels 15, 16, 17

#define NUM_LEGS  6
#define COXA      0
#define FEMUR     1
#define TIBIA     2

// servoChannel[leg][joint]
const uint8_t servoChannel[NUM_LEGS][3] = {
  { 0,  1,  2},   // Leg 1 - Front  Left
  { 3,  4,  5},   // Leg 2 - Middle Left
  { 6,  7,  8},   // Leg 3 - Back   Left
  { 9, 10, 11},   // Leg 4 - Front  Right
  {12, 13, 14},   // Leg 5 - Middle Right
  {15, 16, 17},   // Leg 6 - Back   Right
};

// ── Servo Trim (degrees) ─────────────────────────────────────
//  Adjust these if a servo sits crooked at neutral
//  Positive = shift clockwise, Negative = counterclockwise
int trim[NUM_LEGS][3] = {
  {0, 0, 0},  // Leg 1
  {0, 0, 0},  // Leg 2
  {0, 0, 0},  // Leg 3
  {0, 0, 0},  // Leg 4
  {0, 0, 0},  // Leg 5
  {0, 0, 0},  // Leg 6
};

// ── Stance Angles (degrees) ──────────────────────────────────
//  RIGHT side legs are mirrored — coxa & tibia inverted
#define COXA_STAND      90    // Coxa neutral (pointing out)
#define FEMUR_STAND     60    // Femur: body height
#define TIBIA_STAND    110    // Tibia: knee angle

// Lifted leg pose (during swing phase)
#define FEMUR_LIFT      35
#define TIBIA_LIFT      70

// Step stride offset applied to coxa (degrees)
#define STRIDE_ANGLE    20

// ── State ────────────────────────────────────────────────────
bool isStanding    = false;
bool ps2Connected  = false;

// ── Helpers ─────────────────────────────────────────────────

// Convert degrees (0–180) to PCA9685 tick value
int degreesToTick(int deg) {
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, SERVO_MIN, SERVO_MAX);
}

// Write angle to a servo channel with trim applied
void writeServo(uint8_t leg, uint8_t joint, int angle) {
  int a = constrain(angle + trim[leg][joint], 0, 180);
  pwm.setPWM(servoChannel[leg][joint], 0, degreesToTick(a));
}

// Mirror angle for right-side legs (legs 3–5, index 3–5)
int mirror(uint8_t leg, uint8_t joint, int angle) {
  if (leg >= 3 && (joint == COXA || joint == TIBIA)) {
    return 180 - angle;
  }
  return angle;
}

// Write to a leg with automatic mirroring for right side
void writeLeg(uint8_t leg, int coxa, int femur, int tibia) {
  writeServo(leg, COXA,  mirror(leg, COXA,  coxa));
  writeServo(leg, FEMUR, femur);
  writeServo(leg, TIBIA, mirror(leg, TIBIA, tibia));
}

// ── Poses ────────────────────────────────────────────────────

void standUp() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    writeLeg(leg, COXA_STAND, FEMUR_STAND, TIBIA_STAND);
  }
  isStanding = true;
  delay(400);
}

void sitDown() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    writeLeg(leg, COXA_STAND, 90, 90);   // drop body to ground
  }
  isStanding = false;
  delay(400);
}

// ── Tripod Gait ──────────────────────────────────────────────
//  Group A (lift together): Legs 0, 2, 4  (FL, BL, MR)
//  Group B (lift together): Legs 1, 3, 5  (ML, FR, BR)
//
//  direction:  1 = forward, -1 = backward
//  turnDir  :  1 = right turn, -1 = left turn, 0 = straight

const uint8_t groupA[3] = {0, 2, 4};  // Front-Left, Back-Left, Mid-Right
const uint8_t groupB[3] = {1, 3, 5};  // Mid-Left, Front-Right, Back-Right

void liftGroup(const uint8_t* group, int coxaOffset) {
  for (int i = 0; i < 3; i++) {
    uint8_t leg = group[i];
    writeLeg(leg,
      COXA_STAND + coxaOffset,
      FEMUR_LIFT,
      TIBIA_LIFT);
  }
}

void plantGroup(const uint8_t* group, int coxaOffset) {
  for (int i = 0; i < 3; i++) {
    uint8_t leg = group[i];
    writeLeg(leg,
      COXA_STAND - coxaOffset,
      FEMUR_STAND,
      TIBIA_STAND);
  }
}

void tripodStep(int direction, int turnDir, int stepDelay) {
  int stride  = STRIDE_ANGLE * direction;
  int turning = STRIDE_ANGLE * turnDir / 2;

  // Phase 1 — A lifts and swings forward, B pushes back
  liftGroup( groupA,  stride + turning);
  plantGroup(groupB, -stride + turning);
  delay(stepDelay);

  // Phase 2 — A plants, B lifts and swings forward
  plantGroup(groupA, stride + turning);
  liftGroup( groupB, stride - turning);
  delay(stepDelay);
}

// ── Turn in Place ─────────────────────────────────────────────
void turnStep(int dir, int stepDelay) {
  // dir: 1 = right, -1 = left
  tripodStep(0, dir, stepDelay);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);            // SDA, SCL

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Hexapod boot...");

  // Initialise PS2 controller
  // Params: (dat, cmd, att, clk, pressures=false, rumble=false)
  int error = 1;
  int attempts = 0;
  while (error && attempts < 10) {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
    attempts++;
    delay(300);
  }

  if (error == 0) {
    ps2Connected = true;
    Serial.println("PS2 controller found.");
  } else {
    Serial.println("PS2 not found — running stand-only mode.");
  }

  sitDown();
}

// ── Main Loop ────────────────────────────────────────────────
void loop() {
  if (!ps2Connected) {
    delay(100);
    return;
  }

  ps2x.read_gamepad(false, 0);

  // ── Button: Triangle = Stand / Sit toggle
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    if (isStanding) sitDown();
    else            standUp();
  }

  if (!isStanding) {
    delay(50);
    return;
  }

  // ── Speed from R2 (hold for fast) ──────────────────────────
  int stepDelay = ps2x.Button(PSB_R2) ? 80 : 140;

  // ── Left Stick — Walk ──────────────────────────────────────
  int ly = ps2x.Analog(PSS_LY);  // 0 = up, 255 = down, 128 = center
  int lx = ps2x.Analog(PSS_LX);  // 0 = left, 255 = right

  int deadzone = 40;
  int fwdBack  = 128 - ly;   // positive = forward
  int strafe   = lx - 128;   // positive = right

  // ── Right Stick — Turn ────────────────────────────────────
  int rx      = ps2x.Analog(PSS_RX);
  int turning = rx - 128;    // positive = turn right

  // ── Walk forward / backward
  if (abs(fwdBack) > deadzone) {
    int dir = (fwdBack > 0) ? 1 : -1;
    tripodStep(dir, 0, stepDelay);
  }
  // ── Turn (right stick)
  else if (abs(turning) > deadzone) {
    int dir = (turning > 0) ? 1 : -1;
    turnStep(dir, stepDelay);
  }
  // ── Idle: hold standing pose
  else {
    standUp();
  }

  // ── Cross = sit down immediately ──────────────────────────
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    sitDown();
  }

  // ── Square = reset to stand neutral ───────────────────────
  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    standUp();
  }

  delay(20);
}