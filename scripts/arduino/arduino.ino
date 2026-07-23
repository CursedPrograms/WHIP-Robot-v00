/*
  WHIP Hexapod - Ultimate Control Sketch
  Controller: RTrobot Servo Motor Controller (32 channel)
  Wiring: SoftwareSerial - Arduino pin 11 (RX) = controller TX,
                            Arduino pin 10 (TX) = controller RX

  Sensor wiring:
    HC-SR04: TRIG -> Arduino pin 7, ECHO -> Arduino pin 6
    MPU6050: SDA -> Arduino A4, SCL -> Arduino A5, VCC -> 5V, GND -> GND

  Servos:

  
    1. L Back Coxa      2. L Back Femur      3. L Back Tibia
    4. L Middle Coxa    5. L Middle Femur    6. L Middle Tibia
    7. L Front Coxa     8. L Front Femur     9. L Front Tibia
   24. R Front Coxa    25. R Front Femur    26. R Front Tibia
   27. R Middle Coxa   28. R Middle Femur   29. R Middle Tibia
   30. R Back Coxa     31. R Back Femur     32. R Back Tibia

  State machine:
    BOOT       -> send Rest, then Stand pose
    WALK       -> loop the recorded Forward tripod gait (gait.xml Group 0).
                  The ultrasonic is only sampled on the settled lines
                  (FWD_2/FWD_4, see FORWARD_SCAN_SAFE) - during FWD_1/FWD_3 a
                  tripod is mid-swing and can pass through the sensor beam.
    AVOID      -> obstacle within MIN_DISTANCE_CM: pickAvoidDirection() picks
                  Turn Left (gait.xml Group 2) or Turn Right (Group 3), plays
                  it for AVOID_TURN_LOOPS full cycles, then freezes on the
                  settled last pose (no servo commands sent while frozen) and
                  takes a single ultrasonic reading. Clear -> WALK; still
                  blocked -> another AVOID_TURN_LOOPS cycles, same direction.
    TILT_SAFE  -> tilt exceeds TILT_LIMIT_DEG: freeze in the Stand pose
                  until level again (highest priority - overrides AVOID/WALK)
    SHUTDOWN   -> triggered by sending 'x' over USB serial: plays the
                  recorded Reset/Shutdown sequence (gait.xml Group 1) once,
                  then halts

  Backward is not recorded yet. Once captured, add its PROGMEM lines the
  same way FORWARD/TURN_LEFT/TURN_RIGHT are defined below.

  pickAvoidDirection() (below the state machine variables) currently always
  picks left - replace its body with real left/right logic (e.g. a second
  ultrasonic sensor comparing clearance on both sides) when ready.
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>

SoftwareSerial controllerSerial(9, 10); // RX, TX

// Non-blocking cycle player for a gait's command lines.
// Defined this early so it's visible to Arduino's auto-generated function
// prototypes, which get inserted near the top of the file, above any
// functions that take GaitPlayer& as a parameter.
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
// NOTE: axis signs below assume a specific MPU6050 mounting orientation.
// If "tipping forward" reads as a negative pitch or the filter drifts the
// wrong way, flip the sign on the pitch/roll accel terms in updateTilt().

// ---------------------------------------------------------------------------
// Calibrated poses (from arduino.ino - confirmed correct on hardware)
// ---------------------------------------------------------------------------
const char REST_CMD[] PROGMEM =
  "#2P2500#3P2500#5P2500#6P2500#7P800#8P2500#9P2500"
  "#25P500#26P500#28P500#29P500#31P500#32P500"
  "T500D500\r\n";

const char STAND_CMD[] PROGMEM =
  "#1P2500#2P1500#3P1500#4P1500#5P1500#6P1500#7P800#8P1500#9P1500"
  "#10P1500#11P1500#12P1500#13P1500#14P1500#15P1500#16P1500#17P1500#18P1500"
  "#19P1500#20P1500#21P1500#22P1500#23P1500"
  "#24P2200#25P1500#26P1500#27P1500#28P1500#29P1500#30P500#31P1500#32P1500"
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

// FWD_1/FWD_3 lift & swing a tripod - a leg can pass through the ultrasonic
// beam and read as a false obstacle. FWD_2/FWD_4 settle both tripods back to
// neutral, so those are the only lines it's safe to trust a distance reading.
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

// TODO: Backward - add here once recorded, same pattern as above.

// ---------------------------------------------------------------------------
// Reset / Shutdown sequence (gait.xml Group 1 "Resr -Shut Down")
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
// Gait line sender (reads a PROGMEM command out of a PROGMEM pointer table)
// ---------------------------------------------------------------------------
void sendGaitLine(const char* const table[], uint8_t index) {
  char buf[170];
  strcpy_P(buf, (char*)pgm_read_ptr(&table[index]));
  controllerSerial.print(buf);
}

void sendSimplePose(const char* cmd, const char* label) {
  char buf[300]; // STAND_CMD covers all 32 channels (~258 bytes) - keep headroom
  strcpy_P(buf, cmd);
  controllerSerial.print(buf);
  Serial.print(F("Sent: "));
  Serial.println(label);
}

void resetPlayer(GaitPlayer &p) {
  p.currentLine = 0;
  p.lineStart = 0;
}

// Returns true when this step just wrapped back to line 0, i.e. a full
// gait cycle finished.
bool stepPlayer(GaitPlayer &p) {
  unsigned long now = millis();
  if (p.lineStart == 0 || now - p.lineStart >= GaitPlayer::LINE_MS) {
    sendGaitLine(p.table, p.currentLine);
    p.currentLine = (p.currentLine + 1) % p.numLines;
    p.lineStart = now;
    return p.currentLine == 0;
  }
  return false;
}

// The line currently being physically executed (stepPlayer already advanced
// currentLine to the *next* line right after sending).
uint8_t activeLine(const GaitPlayer &p) {
  return (p.currentLine + p.numLines - 1) % p.numLines;
}

GaitPlayer forwardPlayer   = { FORWARD_GAIT,    FORWARD_GAIT_LEN,    0, 0 };
GaitPlayer turnLeftPlayer  = { TURN_LEFT_GAIT,  TURN_LEFT_GAIT_LEN,  0, 0 };
GaitPlayer turnRightPlayer = { TURN_RIGHT_GAIT, TURN_RIGHT_GAIT_LEN, 0, 0 };
GaitPlayer shutdownPlayer  = { SHUTDOWN_SEQ,    SHUTDOWN_SEQ_LEN,    0, 0 };

// ---------------------------------------------------------------------------
// Ultrasonic
// ---------------------------------------------------------------------------
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1; // out of range / no echo
  return duration * 0.034 / 2;
}

// ---------------------------------------------------------------------------
// MPU6050 - complementary filter pitch/roll
// ---------------------------------------------------------------------------
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;
float gyroXBias = 0, gyroYBias = 0;
float pitchDeg = 0, rollDeg = 0;
unsigned long lastTiltUpdate = 0;

bool mpuFound = false;

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // wake up
  uint8_t err = Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  mpuFound = (Wire.endTransmission() == 0);
  if (!mpuFound) {
    Serial.print(F("MPU6050 NOT FOUND at 0x"));
    Serial.print(MPU_ADDR, HEX);
    Serial.print(F(" (I2C error "));
    Serial.print(err);
    Serial.println(F(") - check SDA/SCL/VCC/GND wiring and AD0 address pin."));
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
  Wire.read(); Wire.read(); // discard temperature
  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();
  rawGyroZ = Wire.read() << 8 | Wire.read();
}

// Averages the gyro at rest so drift doesn't get integrated forever.
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

float pitchOffsetDeg = 0, rollOffsetDeg = 0;

// The MPU6050's mounting angle (and/or the stand pose itself) isn't
// perfectly level, so raw pitch/roll has a fixed non-zero baseline even
// when the robot is standing normally. Average the accel-derived angle
// once at boot, after the robot is in its stand pose, and use that as the
// "level" reference instead of 0 - only deviation from it should ever
// trip TILT_SAFE.
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
  // Start the complementary filter already converged on the baseline so it
  // doesn't need to drift there over the first second or two of runtime.
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

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum State { BOOT, WALK, AVOID, TILT_SAFE, SHUTDOWN };
State state = BOOT;
State stateBeforeTilt = WALK;

uint8_t obstacleCount = 0;
uint8_t clearCount = 0;
unsigned long tiltOverSince = 0;
unsigned long tiltClearSince = 0;

uint8_t avoidLoopsDone = 0; // full turn-gait cycles completed since entering AVOID
bool avoidHolding = false;  // frozen on the settled last pose, sampling the sensor
GaitPlayer* avoidTurnPlayer = &turnLeftPlayer; // which way this AVOID episode is turning

// TODO: pick a direction from real sensor data (e.g. a second ultrasonic
// comparing left/right clearance) instead of always defaulting to left.
void pickAvoidDirection() {
  avoidTurnPlayer = &turnLeftPlayer;
}

// Resets the turn/scan cycle without changing direction - used both when
// first entering AVOID and when restarting after a failed clearance check.
void resetAvoid() {
  avoidLoopsDone = 0;
  avoidHolding = false;
  obstacleCount = 0;
  clearCount = 0;
  resetPlayer(*avoidTurnPlayer);
}

void enterTiltSafe() {
  if (state != TILT_SAFE) {
    stateBeforeTilt = state;
    state = TILT_SAFE;
    sendSimplePose(STAND_CMD, "TILT SAFE - holding stand pose");
  }
}

void setup() {
  Serial.begin(9600);
  controllerSerial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();
  mpuInit();

  delay(2000); // let the controller finish booting

  Serial.println(F("Calibrating gyro - keep the robot still..."));
  calibrateGyro();
  Serial.println(F("Calibration done."));

  sendSimplePose(REST_CMD, "rest pose");
  delay(1000);
  sendSimplePose(STAND_CMD, "stand pose");
  delay(1000);

  if (mpuFound) {
    calibrateTiltOffset();
    Serial.print(F("Tilt baseline: pitch="));
    Serial.print(pitchOffsetDeg);
    Serial.print(F(" roll="));
    Serial.println(rollOffsetDeg);
  }

  state = WALK;
  resetPlayer(forwardPlayer);
  Serial.println(F("WALK"));
}

void loop() {
  // Echo controller replies
  while (controllerSerial.available()) {
    Serial.write(controllerSerial.read());
  }

  // Manual shutdown trigger over USB serial
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
      // Player wrapped back to line 0 -> full sequence has been sent once.
      static bool halted = false;
      if (!halted) {
        halted = true;
        Serial.println(F("Shutdown complete."));
      }
    } else {
      stepPlayer(shutdownPlayer);
    }
    return; // nothing else matters once shutting down
  }

  // --- Tilt monitoring (highest priority) ---
  static unsigned long lastTiltCheck = 0;
  if (mpuFound && millis() - lastTiltCheck >= 20) {
    lastTiltCheck = millis();
    updateTilt();
    float tiltMag = max(abs(pitchDeg - pitchOffsetDeg), abs(rollDeg - rollOffsetDeg));
    unsigned long now = millis();

    // TEMP DEBUG - remove once tilt behavior is confirmed correct
    static unsigned long lastTiltPrint = 0;
    if (now - lastTiltPrint >= 250) {
      lastTiltPrint = now;
      Serial.print(F("pitch="));
      Serial.print(pitchDeg);
      Serial.print(F(" roll="));
      Serial.print(rollDeg);
      Serial.print(F(" tiltMag="));
      Serial.println(tiltMag);
    }

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
          if (state == WALK) resetPlayer(forwardPlayer);
          if (state == AVOID) resetAvoid(); // restart the turn/scan cycle cleanly
          Serial.println(state == WALK ? F("Level again - WALK") : F("Level again - AVOID"));
        }
      } else {
        tiltClearSince = 0;
      }
    }
  }

  if (state == TILT_SAFE) return; // hold the stand pose, skip walking/obstacle logic

  if (state == WALK) {
    stepPlayer(forwardPlayer);

    // Only trust the ultrasonic while both tripods are settled (FWD_2/FWD_4) -
    // during FWD_1/FWD_3 a leg is swinging through the beam.
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
        state = AVOID;
        pickAvoidDirection();
        resetAvoid();
        Serial.println(avoidTurnPlayer == &turnLeftPlayer
                          ? F("OBSTACLE - AVOID (turn left)")
                          : F("OBSTACLE - AVOID (turn right)"));
      }
    }
  } else if (state == AVOID) {
    if (!avoidHolding) {
      // Turn for AVOID_TURN_LOOPS full cycles, then freeze on the settled
      // last pose (legs down) so the sensor can be trusted.
      if (stepPlayer(*avoidTurnPlayer)) {
        avoidLoopsDone++;
        if (avoidLoopsDone >= AVOID_TURN_LOOPS) avoidHolding = true;
      }
    } else {
      // Reached the settled last pose - take one reading and decide right
      // away. No servo commands are sent while avoidHolding is true, so
      // nothing else moves until this decides WALK or another turn.
      long distance = readDistanceCM();
      bool clear = (distance < 0 || distance >= CLEAR_DISTANCE_CM);

      if (clear) {
        state = WALK;
        avoidHolding = false;
        avoidLoopsDone = 0;
        resetPlayer(forwardPlayer);
        Serial.println(F("CLEAR - WALK"));
      } else {
        resetAvoid(); // still blocked - turn for another AVOID_TURN_LOOPS cycles
        Serial.println(F("STILL BLOCKED - turn again"));
      }
    }
  }
}
