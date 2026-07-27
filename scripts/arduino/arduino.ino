/*
  WHIP Hexapod - Ultimate Control Sketch (Refactored)
  Controller: RTrobot Servo Motor Controller (32 channel)
  Wiring: SoftwareSerial - Arduino pin 9 (RX) = controller TX,
                           Arduino pin 10 (TX) = controller RX

  Sensor wiring:
    HC-SR04: TRIG -> Arduino pin 7, ECHO -> Arduino pin 6
    MPU6050: SDA -> Arduino A4, SCL -> Arduino A5, VCC -> 5V, GND -> GND
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>

SoftwareSerial controllerSerial(9, 10); // RX, TX

// Non-blocking cycle player for a gait's command lines.
struct GaitPlayer {
  const char* const* table;
  uint8_t numLines;
  uint8_t currentLine;
  unsigned long lineStart;
  static const unsigned long LINE_MS = 1000; // T500 + D500
};

// ---------------------------------------------------------------------------
// Pins / addresses
// ---------------------------------------------------------------------------
const int TRIG_PIN = 7;
const int ECHO_PIN = 6;
const int MPU_ADDR = 0x68;

// ---------------------------------------------------------------------------
// Obstacle avoidance tuning
// ---------------------------------------------------------------------------
const long MIN_DISTANCE_CM   = 20;  // trigger AVOID below this
const long CLEAR_DISTANCE_CM = 35;  // resume WALK above this (hysteresis)
const uint8_t OBSTACLE_DEBOUNCE = 3; // consecutive readings required
const uint8_t AVOID_TURN_LOOPS = 2;  // full turn-gait cycles before pausing to scan

// ---------------------------------------------------------------------------
// Tilt safety tuning (MPU6050)
// ---------------------------------------------------------------------------
const float TILT_LIMIT_DEG  = 10.0; // enter TILT_SAFE at/above this
const float TILT_CLEAR_DEG  = 7.0;  // leave TILT_SAFE below this (hysteresis)
const unsigned long TILT_DEBOUNCE_MS = 250;
const float COMPLEMENTARY_ALPHA = 0.98;

// ---------------------------------------------------------------------------
// Calibrated poses
// ---------------------------------------------------------------------------
const char REST_CMD[] PROGMEM =
  "#2P2500#3P2500#5P2500#6P2500#7P800#8P2500#9P2500"
  "#25P500#26P500#28P500#29P500#31P500#32P500"
  "T500D500\r\n";

const char STAND_CMD[] PROGMEM =
  "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500"
  "#10P1500#11P1500#12P1500#13P1500#14P1500#15P1500#16P1500#17P1500#18P1500"
  "#19P1500#20P1500#21P1500#22P1500#23P1500"
  "#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P500#31P1500#32P1500"
  "T500D500\r\n";

// ---------------------------------------------------------------------------
// Forward tripod gait (gait.xml Group 0 "Forward")
// ---------------------------------------------------------------------------
const char FWD_1[] PROGMEM = "#1P1100#2P2000#3P1000#4P1500#5P1500#6P1500#7P1100#8P2000#9P1000#24P1500#25P1500#26P1500#27P1900#28P1000#29P2000#30P1500#31P1500#32P1500T500D500\r\n";
const char FWD_2[] PROGMEM = "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500T500D500\r\n";
const char FWD_3[] PROGMEM = "#1P1500#4P1100#5P2000#6P1000#7P1500#24P1900#25P1000#26P2000#27P1500#30P1900#31P1000#32P2000T500D500\r\n";
const char FWD_4[] PROGMEM = "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500T500D500\r\n";

const char* const FORWARD_GAIT[] PROGMEM = { FWD_1, FWD_2, FWD_3, FWD_4 };
const uint8_t FORWARD_GAIT_LEN = 4;
const bool FORWARD_SCAN_SAFE[] PROGMEM = { false, true, false, true };

// ---------------------------------------------------------------------------
// Turn Left tripod gait (gait.xml Group 2 "Turn Left")
// ---------------------------------------------------------------------------
const char TURNL_1[] PROGMEM = "#1P1000#2P2500#3P1100#4P1500#5P1500#6P1500#7P1000#8P2500#9P1100#24P1500#25P1500#26P1500#27P1000#28P500#29P1900#30P1500#31P1500#32P1500T500D500\r\n";
const char TURNL_2[] PROGMEM = "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500T500D500\r\n";
const char TURNL_3[] PROGMEM = "#1P1000#2P1500#3P1500#4P1000#5P2500#6P1100#7P1000#8P1500#9P1500#24P1000#25P500#26P1900#27P1000#28P1500#29P1500#30P1000#31P500#32P1900T500D500\r\n";
const char TURNL_4[] PROGMEM = "#1P1500#7P1500#27P1500T500D500\r\n";
const char TURNL_5[] PROGMEM = "#1P1500#2P1500#3P1500#5P1500#6P1500#7P1500#8P1500#9P1500#25P1500#26P1500#27P1500#28P1500#29P1500#31P1500#32P1500T500D500\r\n";

const char* const TURN_LEFT_GAIT[] PROGMEM = { TURNL_1, TURNL_2, TURNL_3, TURNL_4, TURNL_5 };
const uint8_t TURN_LEFT_GAIT_LEN = 5;

// ---------------------------------------------------------------------------
// Turn Right tripod gait (gait.xml Group 3 "Turn Right")
// ---------------------------------------------------------------------------
const char TURNR_1[] PROGMEM = "#1P2000#2P2500#3P1000#4P1500#5P1500#6P1500#7P2000#8P2500#9P1000#24P1500#25P1500#26P1500#27P2000#28P500#29P2000#30P1500#31P1500#32P1500T500D500\r\n";
const char TURNR_2[] PROGMEM = "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500T500D500\r\n";
const char TURNR_3[] PROGMEM = "#4P2000#5P2500#6P1000#24P2000#25P500#26P2000#30P2000#31P500#32P2000T500D500\r\n";
const char TURNR_4[] PROGMEM = "#1P1500#7P1500#27P1500#30P2000T500D500\r\n";
const char TURNR_5[] PROGMEM = "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500T500D500\r\n";

const char* const TURN_RIGHT_GAIT[] PROGMEM = { TURNR_1, TURNR_2, TURNR_3, TURNR_4, TURNR_5 };
const uint8_t TURN_RIGHT_GAIT_LEN = 5;

// ---------------------------------------------------------------------------
// Shutdown sequence
// ---------------------------------------------------------------------------
const char SHUT_1[] PROGMEM = "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500D500\r\n";
const char SHUT_2[] PROGMEM = "#2P2500#3P1500#8P2500#28P500#29P1500T500D500\r\n";
const char SHUT_3[] PROGMEM = "#3P500#9P500#29P2500T500D500\r\n";
const char SHUT_4[] PROGMEM = "#3P2500#9P2500#29P500T500D500\r\n";
const char SHUT_5[] PROGMEM = "#5P1700#6P1200#25P1200#26P1700#31P1200#32P1700T500D500\r\n";
const char SHUT_6[] PROGMEM = "#5P2500#25P500#26P1700#28P500#31P500T500D500\r\n";
const char SHUT_7[] PROGMEM = "#6P500#26P2500#32P2500T500D500\r\n";
const char SHUT_8[] PROGMEM = "#6P2500#26P500#32P500T500D500\r\n";

const char* const SHUTDOWN_SEQ[] PROGMEM = { SHUT_1, SHUT_2, SHUT_3, SHUT_4, SHUT_5, SHUT_6, SHUT_7, SHUT_8 };
const uint8_t SHUTDOWN_SEQ_LEN = 8;

// ---------------------------------------------------------------------------
// Helpers & State
// ---------------------------------------------------------------------------
void sendGaitLine(const char* const table[], uint8_t index) {
  char buf[170];
  strcpy_P(buf, (char*)pgm_read_ptr(&table[index]));
  controllerSerial.print(buf);
}

void sendSimplePose(const char* cmd, const char* label) {
  char buf[300];
  strcpy_P(buf, cmd);
  controllerSerial.print(buf);
  Serial.print(F("Sent: "));
  Serial.println(label);
}

void resetPlayer(GaitPlayer &p) {
  p.currentLine = 0;
  p.lineStart = 0;
}

bool stepPlayer(GaitPlayer &p) {
  unsigned long now = millis();
  if (p.lineStart == 0 || now - p.lineStart >= GaitPlayer::LINE_MS) {
    sendGaitLine(p.table, p.currentLine);
    p.currentLine = (p.currentLine + 1) % p.numLines;
    p.lineStart = now;
    return p.currentLine == 0; // Returns true when a full gait cycle completes
  }
  return false;
}

uint8_t activeLine(const GaitPlayer &p) {
  return (p.currentLine + p.numLines - 1) % p.numLines;
}

GaitPlayer forwardPlayer   = { FORWARD_GAIT,    FORWARD_GAIT_LEN,    0, 0 };
GaitPlayer turnLeftPlayer  = { TURN_LEFT_GAIT,  TURN_LEFT_GAIT_LEN,  0, 0 };
GaitPlayer turnRightPlayer = { TURN_RIGHT_GAIT, TURN_RIGHT_GAIT_LEN, 0, 0 };
GaitPlayer shutdownPlayer  = { SHUTDOWN_SEQ,    SHUTDOWN_SEQ_LEN,    0, 0 };

enum State { BOOT, WALK, AVOID, TILT_SAFE, SHUTDOWN };
State state = BOOT;
State stateBeforeTilt = WALK;

bool walkStopRequested = false; // Flag to wait until stride finishes
uint8_t obstacleCount = 0;
unsigned long tiltOverSince = 0;
unsigned long tiltClearSince = 0;

uint8_t avoidLoopsDone = 0;
bool avoidHolding = false;
GaitPlayer* avoidTurnPlayer = &turnLeftPlayer;

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

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  mpuFound = (Wire.endTransmission() == 0);
  if (!mpuFound) {
    Serial.println(F("MPU6050 NOT FOUND! Check wiring."));
  }
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
  rollOffsetDeg = sumRoll / samples;
  pitchDeg = pitchOffsetDeg;
  rollDeg = rollOffsetDeg;
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

void pickAvoidDirection() {
  avoidTurnPlayer = &turnLeftPlayer;
}

void resetAvoid() {
  avoidLoopsDone = 0;
  avoidHolding = false;
  obstacleCount = 0;
  resetPlayer(*avoidTurnPlayer);
}

void enterTiltSafe() {
  if (state != TILT_SAFE) {
    stateBeforeTilt = state;
    state = TILT_SAFE;
    sendSimplePose(STAND_CMD, "TILT SAFE - holding stand pose");
  }
}

// ---------------------------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  controllerSerial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();
  mpuInit();

  delay(2000);

  Serial.println(F("Calibrating gyro..."));
  calibrateGyro();

  sendSimplePose(REST_CMD, "rest pose");
  delay(1000);
  sendSimplePose(STAND_CMD, "stand pose");
  delay(1000);

  if (mpuFound) {
    calibrateTiltOffset();
  }

  state = WALK;
  resetPlayer(forwardPlayer);
  Serial.println(F("WALK"));
}

void loop() {
  while (controllerSerial.available()) {
    Serial.write(controllerSerial.read());
  }

  if (Serial.available()) {
    char c = Serial.read();
    if ((c == 'x' || c == 'X') && state != SHUTDOWN) {
      state = SHUTDOWN;
      resetPlayer(shutdownPlayer);
      Serial.println(F("SHUTDOWN"));
    }
  }

  if (state == SHUTDOWN) {
    if (shutdownPlayer.currentLine == 0 && shutdownPlayer.lineStart != 0) {
      static bool halted = false;
      if (!halted) {
        halted = true;
        Serial.println(F("Shutdown complete."));
      }
    } else {
      stepPlayer(shutdownPlayer);
    }
    return;
  }

  // --- Tilt Safety Check ---
  static unsigned long lastTiltCheck = 0;
  if (mpuFound && millis() - lastTiltCheck >= 20) {
    lastTiltCheck = millis();
    updateTilt();
    float tiltMag = max(abs(pitchDeg - pitchOffsetDeg), abs(rollDeg - rollOffsetDeg));
    unsigned long now = millis();

    if (tiltMag >= TILT_LIMIT_DEG) {
      tiltClearSince = 0;
      if (tiltOverSince == 0) tiltOverSince = now;
      if (now - tiltOverSince >= TILT_DEBOUNCE_MS) {
        enterTiltSafe();
      }
    } else {
      tiltOverSince = 0;
      if (state == TILT_SAFE) {
        if (tiltClearSince == 0) tiltClearSince = now;
        if (tiltMag <= TILT_CLEAR_DEG && now - tiltClearSince >= TILT_DEBOUNCE_MS) {
          state = stateBeforeTilt;
          sendSimplePose(STAND_CMD, "Recovering from tilt");
          delay(300);
          if (state == WALK) resetPlayer(forwardPlayer);
          if (state == AVOID) resetAvoid();
        }
      } else {
        tiltClearSince = 0;
      }
    }
  }

  if (state == TILT_SAFE) return;

  // --- WALK State ---
  if (state == WALK) {
    bool cycleWrapped = stepPlayer(forwardPlayer);

    // Read distance on settled safe lines (FWD_2 / FWD_4)
    static unsigned long lastWalkDistCheck = 0;
    bool scanSafe = pgm_read_byte(&FORWARD_SCAN_SAFE[activeLine(forwardPlayer)]);
    
    if (scanSafe && millis() - lastWalkDistCheck >= 100) {
      lastWalkDistCheck = millis();
      long distance = readDistanceCM();

      if (distance > 0 && distance < MIN_DISTANCE_CM) {
        obstacleCount++;
      } else if (distance < 0 || distance >= CLEAR_DISTANCE_CM) {
        obstacleCount = 0;
      }

      if (obstacleCount >= OBSTACLE_DEBOUNCE) {
        walkStopRequested = true; // Request a graceful stop at the end of this stride
      }
    }

    // Transition ONLY when full forward stride wraps back around cleanly
    if (walkStopRequested && cycleWrapped) {
      walkStopRequested = false;
      obstacleCount = 0;

      // Bring body to neutral before executing turning sequence
      sendSimplePose(STAND_CMD, "Transitioning WALK -> AVOID");
      delay(300);

      state = AVOID;
      pickAvoidDirection();
      resetAvoid();
      Serial.println(F("OBSTACLE DETECTED -> Entering AVOID mode"));
    }
  } 
  
  // --- AVOID State ---
  else if (state == AVOID) {
    if (!avoidHolding) {
      // Execute turn sequence
      if (stepPlayer(*avoidTurnPlayer)) {
        avoidLoopsDone++;
        if (avoidLoopsDone >= AVOID_TURN_LOOPS) {
          avoidHolding = true;
          // Flatten feet squarely on the ground to take a clear, level reading
          sendSimplePose(STAND_CMD, "AVOID: Grounding for clear scan");
          delay(250);
        }
      }
    } else {
      // Hold position and take sensor reading
      long distance = readDistanceCM();
      bool clear = (distance < 0 || distance >= CLEAR_DISTANCE_CM);

      if (clear) {
        sendSimplePose(STAND_CMD, "Transitioning AVOID -> WALK");
        delay(300);

        state = WALK;
        avoidHolding = false;
        avoidLoopsDone = 0;
        resetPlayer(forwardPlayer);
        Serial.println(F("CLEAR -> Resuming WALK"));
      } else {
        resetAvoid(); // Obstacle still in view - turn again
        Serial.println(F("STILL BLOCKED -> Turning again"));
      }
    }
  }
}