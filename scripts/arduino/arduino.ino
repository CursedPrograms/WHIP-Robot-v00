/*
  WHIP Hexapod - Control Sketch
  Controller: RTrobot Servo Motor Controller (32 channel)

  Wiring: SoftwareSerial - Arduino pin 9 (RX) = controller TX,
                           Arduino pin 10 (TX) = controller RX
    HC-SR04: TRIG -> pin 7, ECHO -> pin 6
    MPU6050: SDA -> A4, SCL -> A5
    IR Receiver: OUT -> pin 5

  ---------------------------------------------------------------------------
  GAIT DATA
  ---------------------------------------------------------------------------
  The gait tables below are transcribed VERBATIM from the RTrobot XML action
  groups (Forward = group 0, Reset/Shutdown = group 1, Turn Left = group 2,
  Turn Right = group 3). Do NOT "optimize" them by removing channels that
  look redundant. The #nP1500 entries on the stance tripod are the power
  stroke -- deleting them is what made the robot shuffle in place.

  The XML lines end in T500D500. T = move duration, D = dwell after the move
  completes. D is not transmitted here; LINE_MS reproduces it host-side.
  If you change T in the strings, change LINE_MS to match (T + D + margin).

  Backward has no XML group -- it is Forward with the coxa channels
  (1,4,7 / 24,27,30) mirrored around 1500. Test it with the feet off the
  ground before trusting it.

  ---------------------------------------------------------------------------
  STRUCTURE
  ---------------------------------------------------------------------------
  Each gait is its own function and plays one complete cycle.
  requestMove() routes every change of movement through the standing pose.
  gaitWait() replaces delay() inside gaits: it keeps polling IR, tilt and
  the distance sensor, and returns false when the move should be dropped.
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>

#define DECODE_NEC              // must precede the IRremote include
#include <IRremote.hpp>         // IRremote 4.x

SoftwareSerial controllerSerial(9, 10); // RX, TX

// ---------------------------------------------------------------------------
// Pins / addresses
// ---------------------------------------------------------------------------
const int TRIG_PIN = 7;
const int ECHO_PIN = 6;
const int IRR_PIN  = 5;
const int MPU_ADDR = 0x68;

// ---------------------------------------------------------------------------
// IR codes
// ---------------------------------------------------------------------------
#define IR_FORWARD          0x74
#define IR_BACKWARD         0x75
#define IR_LEFT             0x34
#define IR_RIGHT            0x33
#define IR_OK               0x65
#define IR_MODE_IRControl   0x0   // "1" -> IR remote control
#define IR_MODE_OBSTACLE    0x1   // "2" -> obstacle avoidance

const unsigned long IR_CMD_TIMEOUT_MS = 350;

// ---------------------------------------------------------------------------
// Timing -- matches the XML's T500D500 (500ms move + 500ms dwell + margin)
// ---------------------------------------------------------------------------
const unsigned long LINE_MS         = 1020;
const unsigned long STAND_SETTLE_MS = 1020;
const unsigned long STAND_HOLD_MS   = 3000;   // hold the stand pose after hitting a wall

// ---------------------------------------------------------------------------
// Obstacle avoidance tuning
// ---------------------------------------------------------------------------
const long MIN_DISTANCE_CM   = 20;
const long CLEAR_DISTANCE_CM = 35;
const uint8_t OBSTACLE_DEBOUNCE = 3;
const uint8_t AVOID_TURN_LOOPS  = 2;

// ---------------------------------------------------------------------------
// Tilt safety tuning (MPU6050)
// ---------------------------------------------------------------------------
const float TILT_LIMIT_DEG = 10.0;
const float TILT_CLEAR_DEG = 7.0;
const unsigned long TILT_DEBOUNCE_MS = 250;
const float COMPLEMENTARY_ALPHA = 0.98;

// ---------------------------------------------------------------------------
// Poses
// STAND_CMD is line 1 of the XML "Reset - Shut Down" group.
// REST_CMD has no XML equivalent -- verify #7P800 before relying on it.
// ---------------------------------------------------------------------------
const char STAND_CMD[] PROGMEM =
  "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500"
  "#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500\r\n";

const char REST_CMD[] PROGMEM =
  "#2P2500#3P2500#5P2500#6P2500#7P800#8P2500#9P2500"
  "#25P500#26P500#28P500#29P500#31P500#32P500T500\r\n";

// ---------------------------------------------------------------------------
// Forward Gait -- XML group 0, verbatim
// ---------------------------------------------------------------------------
const char FWD_1[] PROGMEM =
  "#1P1100#2P2000#3P1000#4P1500#5P1500#6P1500#7P1100#8P2000#9P1000"
  "#24P1500#25P1500#26P1500#27P1900#28P1000#29P2000#30P1500#31P1500#32P1500T500\r\n";
const char FWD_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500T500\r\n";
const char FWD_3[] PROGMEM =
  "#1P1500#4P1100#5P2000#6P1000#7P1500#24P1900#25P1000#26P2000"
  "#27P1500#30P1900#31P1000#32P2000T500\r\n";
const char FWD_4[] PROGMEM =
  "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500T500\r\n";

const char* const FORWARD_GAIT[] PROGMEM = { FWD_1, FWD_2, FWD_3, FWD_4 };
const uint8_t FORWARD_GAIT_LEN = 4;
// Lines 2 and 4 lower the swing tripod -- body settled, safe to range-find
const bool FORWARD_SCAN_SAFE[] PROGMEM = { false, true, false, true };

// ---------------------------------------------------------------------------
// Backward Gait -- Forward with coxa channels mirrored around 1500.
// NOT from XML. Bench-test before running under load.
// ---------------------------------------------------------------------------
const char BWD_1[] PROGMEM =
  "#1P1900#2P2000#3P1000#4P1500#5P1500#6P1500#7P1900#8P2000#9P1000"
  "#24P1500#25P1500#26P1500#27P1100#28P1000#29P2000#30P1500#31P1500#32P1500T500\r\n";
const char BWD_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500T500\r\n";
const char BWD_3[] PROGMEM =
  "#1P1500#4P1900#5P2000#6P1000#7P1500#24P1100#25P1000#26P2000"
  "#27P1500#30P1100#31P1000#32P2000T500\r\n";
const char BWD_4[] PROGMEM =
  "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500T500\r\n";

const char* const BACKWARD_GAIT[] PROGMEM = { BWD_1, BWD_2, BWD_3, BWD_4 };
const uint8_t BACKWARD_GAIT_LEN = 4;

// ---------------------------------------------------------------------------
// Turn Left Gait -- XML group 2, verbatim
// ---------------------------------------------------------------------------
const char TURNL_1[] PROGMEM =
  "#1P1000#2P2500#3P1100#4P1500#5P1500#6P1500#7P1000#8P2500#9P1100"
  "#24P1500#25P1500#26P1500#27P1000#28P500#29P1900#30P1500#31P1500#32P1500T500\r\n";
const char TURNL_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500T500\r\n";
const char TURNL_3[] PROGMEM =
  "#1P1000#2P1500#3P1500#4P1000#5P2500#6P1100#7P1000#8P1500#9P1500"
  "#24P1000#25P500#26P1900#27P1000#28P1500#29P1500#30P1000#31P500#32P1900T500\r\n";
const char TURNL_4[] PROGMEM =
  "#1P1500#7P1500#27P1500T500\r\n";
const char TURNL_5[] PROGMEM =
  "#1P1500#2P1500#3P1500#5P1500#6P1500#7P1500#8P1500#9P1500"
  "#25P1500#26P1500#27P1500#28P1500#29P1500#31P1500#32P1500T500\r\n";

const char* const TURN_LEFT_GAIT[] PROGMEM = { TURNL_1, TURNL_2, TURNL_3, TURNL_4, TURNL_5 };
const uint8_t TURN_LEFT_GAIT_LEN = 5;

// ---------------------------------------------------------------------------
// Turn Right Gait -- XML group 3, verbatim
// (Asymmetric vs Turn Left: line 3 omits the 1,7,27 hold and line 5 omits
//  the 4,24,30 reset. That is how it is in the XML. Left untouched.)
// ---------------------------------------------------------------------------
const char TURNR_1[] PROGMEM =
  "#1P2000#2P2500#3P1000#4P1500#5P1500#6P1500#7P2000#8P2500#9P1000"
  "#24P1500#25P1500#26P1500#27P2000#28P500#29P2000#30P1500#31P1500#32P1500T500\r\n";
const char TURNR_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500T500\r\n";
const char TURNR_3[] PROGMEM =
  "#4P2000#5P2500#6P1000#24P2000#25P500#26P2000#30P2000#31P500#32P2000T500\r\n";
const char TURNR_4[] PROGMEM =
  "#1P1500#7P1500#27P1500#30P2000T500\r\n";
const char TURNR_5[] PROGMEM =
  "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500T500\r\n";

const char* const TURN_RIGHT_GAIT[] PROGMEM = { TURNR_1, TURNR_2, TURNR_3, TURNR_4, TURNR_5 };
const uint8_t TURN_RIGHT_GAIT_LEN = 5;

// ---------------------------------------------------------------------------
// Shutdown Sequence -- XML group 1, verbatim
// ---------------------------------------------------------------------------
const char SHUT_1[] PROGMEM =
  "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500"
  "#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500\r\n";
const char SHUT_2[] PROGMEM = "#2P2500#3P1500#8P2500#28P500#29P1500T500\r\n";
const char SHUT_3[] PROGMEM = "#3P500#9P500#29P2500T500\r\n";
const char SHUT_4[] PROGMEM = "#3P2500#9P2500#29P500T500\r\n";
const char SHUT_5[] PROGMEM = "#5P1700#6P1200#25P1200#26P1700#31P1200#32P1700T500\r\n";
const char SHUT_6[] PROGMEM = "#5P2500#25P500#26P1700#28P500#31P500T500\r\n";
const char SHUT_7[] PROGMEM = "#6P500#26P2500#32P2500T500\r\n";
const char SHUT_8[] PROGMEM = "#6P2500#26P500#32P500T500\r\n";

const char* const SHUTDOWN_SEQ[] PROGMEM = { SHUT_1, SHUT_2, SHUT_3, SHUT_4, SHUT_5, SHUT_6, SHUT_7, SHUT_8 };
const uint8_t SHUTDOWN_SEQ_LEN = 8;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum SystemState   { SYS_ACTIVE, SYS_TILT_SAFE, SYS_SHUTDOWN };
enum ControlMode   { CTRL_OBSTACLE, CTRL_IR };
enum MovementState { MOVE_STAND, MOVE_WALK, MOVE_BACKWARD, MOVE_TURN_LEFT, MOVE_TURN_RIGHT };

SystemState   sysState    = SYS_ACTIVE;
ControlMode   controlMode = CTRL_OBSTACLE;
ControlMode   controlModeBeforeTilt = CTRL_OBSTACLE;
MovementState currentMove = MOVE_STAND;

bool abortMovement = false;   // cleared at the top of every loop()

uint8_t irActiveCmd      = 0;
uint8_t irCmdAtMoveStart = 0;
unsigned long irLastCmdMillis = 0;

uint8_t obstacleCount    = 0;
bool    obstacleDetected = false;

bool gaitWait(unsigned long ms, bool scanSafe);
long readDistanceCM();
void pollTilt();
void pollIR();

// ---------------------------------------------------------------------------
// Serial send helpers
// ---------------------------------------------------------------------------
char cmdBuf[190];   // longest line is 18 channels, ~145 chars

void sendCmdP(const char* pgmCmd) {
  strncpy_P(cmdBuf, pgmCmd, sizeof(cmdBuf) - 1);
  cmdBuf[sizeof(cmdBuf) - 1] = '\0';
  controllerSerial.print(cmdBuf);
}

void sendTableLine(const char* const table[], uint8_t index) {
  sendCmdP((const char*)pgm_read_ptr(&table[index]));
}

// ---------------------------------------------------------------------------
// MOVEMENT FUNCTIONS -- each plays one full cycle
// ---------------------------------------------------------------------------
bool playGait(const char* const table[], uint8_t len, const bool* scanTable) {
  for (uint8_t i = 0; i < len; i++) {
    sendTableLine(table, i);
    bool scanSafe = scanTable ? (bool)pgm_read_byte(&scanTable[i]) : false;
    if (!gaitWait(LINE_MS, scanSafe)) return false;
  }
  return true;
}

void Stand() {
  sendCmdP(STAND_CMD);
  currentMove = MOVE_STAND;
  Serial.println(F("MOVE -> STAND"));
}

bool MoveForward()  { return playGait(FORWARD_GAIT,    FORWARD_GAIT_LEN,    FORWARD_SCAN_SAFE); }
bool MoveBackward() { return playGait(BACKWARD_GAIT,   BACKWARD_GAIT_LEN,   NULL); }
bool TurnLeft()     { return playGait(TURN_LEFT_GAIT,  TURN_LEFT_GAIT_LEN,  NULL); }
bool TurnRight()    { return playGait(TURN_RIGHT_GAIT, TURN_RIGHT_GAIT_LEN, NULL); }

// ---------------------------------------------------------------------------
// Every movement change passes through the standing pose first
// ---------------------------------------------------------------------------
bool requestMove(MovementState next) {
  if (next == currentMove) return true;

  if (currentMove != MOVE_STAND) {
    Stand();
    if (!gaitWait(STAND_SETTLE_MS, false)) return false;
  }

  if (next == MOVE_STAND) {
    currentMove = MOVE_STAND;
    return true;
  }

  currentMove = next;
  switch (next) {
    case MOVE_WALK:       Serial.println(F("MOVE -> WALK"));       break;
    case MOVE_BACKWARD:   Serial.println(F("MOVE -> BACKWARD"));   break;
    case MOVE_TURN_LEFT:  Serial.println(F("MOVE -> TURN LEFT"));  break;
    case MOVE_TURN_RIGHT: Serial.println(F("MOVE -> TURN RIGHT")); break;
    default: break;
  }
  return true;
}

// ---------------------------------------------------------------------------
// gaitWait -- the only wait used inside a movement
// ---------------------------------------------------------------------------
bool gaitWait(unsigned long ms, bool scanSafe) {
  unsigned long start = millis();
  static unsigned long lastPing = 0;

  while (millis() - start < ms) {
    while (controllerSerial.available()) Serial.write(controllerSerial.read());

    pollTilt();
    pollIR();

    // While walking, watch for a wall on the settled gait frames only
    if (scanSafe && controlMode == CTRL_OBSTACLE && currentMove == MOVE_WALK
        && millis() - lastPing >= 100) {
      lastPing = millis();
      long distance = readDistanceCM();

      if (distance > 0 && distance < MIN_DISTANCE_CM) {
        obstacleCount++;
      } else if (distance < 0 || distance >= CLEAR_DISTANCE_CM) {
        obstacleCount = 0;
      }

      if (obstacleCount >= OBSTACLE_DEBOUNCE) {
        obstacleCount    = 0;
        obstacleDetected = true;
        abortMovement    = true;   // stop mid-cycle, go stand
      }
    }

    if (controlMode == CTRL_IR && irActiveCmd != irCmdAtMoveStart) return false;
    if (abortMovement || sysState != SYS_ACTIVE) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// IR REMOTE MODE
// ---------------------------------------------------------------------------
void runIRMode() {
  if (irActiveCmd != 0 && millis() - irLastCmdMillis > IR_CMD_TIMEOUT_MS) {
    irActiveCmd = 0;
  }

  irCmdAtMoveStart = irActiveCmd;

  if (irActiveCmd == IR_FORWARD) {
    if (requestMove(MOVE_WALK)) MoveForward();
  }
  else if (irActiveCmd == IR_BACKWARD) {
    if (requestMove(MOVE_BACKWARD)) MoveBackward();
  }
  else if (irActiveCmd == IR_LEFT) {
    if (requestMove(MOVE_TURN_LEFT)) TurnLeft();
  }
  else if (irActiveCmd == IR_RIGHT) {
    if (requestMove(MOVE_TURN_RIGHT)) TurnRight();
  }
  else {
    requestMove(MOVE_STAND);   // no button held -> all servos 1500
  }
}

// ---------------------------------------------------------------------------
// OBSTACLE AVOIDANCE MODE
// ---------------------------------------------------------------------------
// Debounced check: is something within MIN_DISTANCE_CM right now?
bool wallAhead() {
  uint8_t hits = 0;
  for (uint8_t i = 0; i < OBSTACLE_DEBOUNCE; i++) {
    long d = readDistanceCM();
    if (d > 0 && d < MIN_DISTANCE_CM) hits++;
    delay(60);   // HC-SR04 needs ~60ms between pings
  }
  return (hits >= OBSTACLE_DEBOUNCE);
}

// Is the path clear enough to walk again?
bool pathClear() {
  uint8_t clearCount = 0;
  for (uint8_t i = 0; i < OBSTACLE_DEBOUNCE; i++) {
    long d = readDistanceCM();
    if (d < 0 || d >= CLEAR_DISTANCE_CM) clearCount++;
    delay(60);
  }
  return (clearCount >= OBSTACLE_DEBOUNCE);
}

/*
  Walk forward on repeat.
  If a wall is in range -> stand (all 1500) for 3 seconds.
  Run the turning loop twice.
  Back to stand.
  Scan: clear -> forward. Still blocked -> turn again.
*/
void runObstacleMode() {
  if (!obstacleDetected) {
    // --- WALK ---
    if (requestMove(MOVE_WALK)) MoveForward();
    return;
  }

  // --- WALL HIT ---
  obstacleDetected = false;
  Serial.println(F("WALL -> stand 3s"));

  Stand();
  if (!gaitWait(STAND_HOLD_MS, false)) return;

  // --- TURN TWICE ---
  Serial.println(F("TURNING x2"));
  currentMove = MOVE_TURN_LEFT;
  for (uint8_t i = 0; i < AVOID_TURN_LOOPS; i++) {
    if (!TurnLeft()) return;
  }

  // --- BACK TO STAND ---
  Stand();
  if (!gaitWait(STAND_SETTLE_MS, false)) return;

  // --- SCAN ---
  if (pathClear()) {
    Serial.println(F("CLEAR -> forward"));
  } else {
    Serial.println(F("STILL BLOCKED -> turn again"));
    obstacleDetected = true;   // next loop pass turns again
  }
}

// ---------------------------------------------------------------------------
// IR polling / dispatch
// ---------------------------------------------------------------------------
void handleIRCommand(uint8_t cmd) {
  if (cmd == IR_MODE_IRControl) {
    if (controlMode != CTRL_IR) {
      controlMode   = CTRL_IR;
      irActiveCmd   = 0;
      abortMovement = true;
      Serial.println(F("CONTROL -> IR REMOTE"));
    }
    return;
  }

  if (cmd == IR_MODE_OBSTACLE) {
    if (controlMode != CTRL_OBSTACLE) {
      controlMode      = CTRL_OBSTACLE;
      obstacleCount    = 0;
      obstacleDetected = false;
      abortMovement    = true;
      Serial.println(F("CONTROL -> OBSTACLE AVOIDANCE"));
    }
    return;
  }

  if (cmd == IR_OK) {
    irActiveCmd   = 0;
    abortMovement = true;
    return;
  }

  if (controlMode != CTRL_IR) return;

  if (cmd == IR_FORWARD || cmd == IR_BACKWARD || cmd == IR_LEFT || cmd == IR_RIGHT) {
    irActiveCmd     = cmd;
    irLastCmdMillis = millis();
  }
}

void pollIR() {
  if (IrReceiver.decode()) {
    bool isRepeat = (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT);
    bool isNoise  = (IrReceiver.decodedIRData.protocol == UNKNOWN);

    if (isRepeat) {
      if (irActiveCmd != 0) irLastCmdMillis = millis();
    } else if (!isNoise) {
      Serial.print(F("IR cmd=0x"));
      Serial.println(IrReceiver.decodedIRData.command, HEX);
      handleIRCommand(IrReceiver.decodedIRData.command);
    } else {
      Serial.println(F("IR noise (UNKNOWN protocol)"));
    }
    IrReceiver.resume();
  }

  if (irActiveCmd != 0 && millis() - irLastCmdMillis > IR_CMD_TIMEOUT_MS) {
    irActiveCmd = 0;
  }
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

// ---------------------------------------------------------------------------
// MPU6050 Complementary Filter
// ---------------------------------------------------------------------------
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;
float gyroXBias = 0, gyroYBias = 0;
float pitchDeg = 0, rollDeg = 0;
float pitchOffsetDeg = 0, rollOffsetDeg = 0;
unsigned long lastTiltUpdate = 0;
bool mpuFound = false;

unsigned long tiltOverSince  = 0;
unsigned long tiltClearSince = 0;

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  mpuFound = (Wire.endTransmission() == 0);
  if (!mpuFound) Serial.println(F("MPU6050 NOT FOUND! Check wiring."));
}

void mpuReadRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  rawAccX = Wire.read() << 8 | Wire.read();
  rawAccY = Wire.read() << 8 | Wire.read();
  rawAccZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();
  rawGyroZ = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
  long sumX = 0, sumY = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    mpuReadRaw();
    sumX += rawGyroX;
    sumY += rawGyroY;
    delay(3);
  }
  gyroXBias = (float)sumX / samples;
  gyroYBias = (float)sumY / samples;
}

void calibrateTiltOffset() {
  float sumPitch = 0, sumRoll = 0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    mpuReadRaw();
    float accXg = rawAccX / 16384.0;
    float accYg = rawAccY / 16384.0;
    float accZg = rawAccZ / 16384.0;
    sumPitch += atan2(-accXg, sqrt(accYg * accYg + accZg * accZg)) * RAD_TO_DEG;
    sumRoll  += atan2(accYg, accZg) * RAD_TO_DEG;
    delay(5);
  }
  pitchOffsetDeg = sumPitch / samples;
  rollOffsetDeg  = sumRoll / samples;
  pitchDeg = pitchOffsetDeg;
  rollDeg  = rollOffsetDeg;
}

void updateTilt() {
  mpuReadRaw();
  unsigned long now = millis();
  float dt = (lastTiltUpdate == 0) ? 0.01 : (now - lastTiltUpdate) / 1000.0;
  lastTiltUpdate = now;

  float accXg = rawAccX / 16384.0;
  float accYg = rawAccY / 16384.0;
  float accZg = rawAccZ / 16384.0;
  float gyroXdps = (rawGyroX - gyroXBias) / 131.0;
  float gyroYdps = (rawGyroY - gyroYBias) / 131.0;

  float pitchAcc = atan2(-accXg, sqrt(accYg * accYg + accZg * accZg)) * RAD_TO_DEG;
  float rollAcc  = atan2(accYg, accZg) * RAD_TO_DEG;

  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gyroXdps * dt) + (1.0 - COMPLEMENTARY_ALPHA) * pitchAcc;
  rollDeg  = COMPLEMENTARY_ALPHA * (rollDeg  + gyroYdps * dt) + (1.0 - COMPLEMENTARY_ALPHA) * rollAcc;
}

void enterTiltSafe() {
  if (sysState != SYS_TILT_SAFE) {
    controlModeBeforeTilt = controlMode;
    sysState      = SYS_TILT_SAFE;
    abortMovement = true;
    sendCmdP(STAND_CMD);
    currentMove = MOVE_STAND;
    Serial.println(F("TILT SAFE - holding stand pose"));
  }
}

void exitTiltSafe() {
  sysState    = SYS_ACTIVE;
  controlMode = controlModeBeforeTilt;
  sendCmdP(STAND_CMD);
  currentMove = MOVE_STAND;
  Serial.println(F("Recovering from tilt"));
  delay(500);

  obstacleCount    = 0;
  obstacleDetected = false;
  irActiveCmd      = 0;
}

void pollTilt() {
  static unsigned long lastTiltCheck = 0;
  if (!mpuFound || millis() - lastTiltCheck < 20) return;
  lastTiltCheck = millis();

  updateTilt();
  float tiltMag = max(abs(pitchDeg - pitchOffsetDeg), abs(rollDeg - rollOffsetDeg));
  unsigned long now = millis();

  if (tiltMag >= TILT_LIMIT_DEG) {
    tiltClearSince = 0;
    if (tiltOverSince == 0) tiltOverSince = now;
    if (now - tiltOverSince >= TILT_DEBOUNCE_MS) enterTiltSafe();
  } else {
    tiltOverSince = 0;
    if (sysState == SYS_TILT_SAFE) {
      if (tiltClearSince == 0) tiltClearSince = now;
      if (tiltMag <= TILT_CLEAR_DEG && now - tiltClearSince >= TILT_DEBOUNCE_MS) exitTiltSafe();
    } else {
      tiltClearSince = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------
void ShutdownSequence() {
  for (uint8_t i = 0; i < SHUTDOWN_SEQ_LEN; i++) {
    sendTableLine(SHUTDOWN_SEQ, i);
    delay(LINE_MS);
  }
  Serial.println(F("Shutdown complete."));
}

// ---------------------------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  controllerSerial.begin(9600);   // must match the controller's configured baud

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();
  mpuInit();

  IrReceiver.begin(IRR_PIN, ENABLE_LED_FEEDBACK);

  delay(2000);

  Serial.println(F("Calibrating gyro..."));
  calibrateGyro();

  sendCmdP(STAND_CMD);
  Serial.println(F("Sent: stand pose"));
  currentMove = MOVE_STAND;
  delay(1500);

  if (mpuFound) calibrateTiltOffset();

  controlMode = CTRL_OBSTACLE;
  Serial.println(F("CONTROL -> OBSTACLE AVOIDANCE"));
}

void loop() {
  abortMovement = false;

  while (controllerSerial.available()) Serial.write(controllerSerial.read());

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') {
      sysState = SYS_SHUTDOWN;
      Serial.println(F("SHUTDOWN"));
    }
  }

  if (sysState == SYS_SHUTDOWN) {
    ShutdownSequence();
    while (true) { delay(1000); }
  }

  pollTilt();
  if (sysState == SYS_TILT_SAFE) return;

  pollIR();

  if (controlMode == CTRL_OBSTACLE) {
    runObstacleMode();
  } else {
    runIRMode();
  }
}
