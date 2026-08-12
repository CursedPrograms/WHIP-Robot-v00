/*
  WHIP Hexapod - ESP32 Control Sketch
  Controller: RTrobot Servo Motor Controller (32 channel)

  Ported from scripts/arduino/arduino.ino (left there unmodified) to run on
  the ESP32 board used across the fleet (same module as NORA), adding WiFi
  plus a browser control page on top of the original IR-remote / obstacle-
  avoidance gait logic. The RTrobot serial protocol, gait tables, and safety
  clamps are copied verbatim from the Arduino version -- see arduino.ino's
  header comment for the full provenance notes on the gait data itself (XML
  groups, why the "redundant" lines aren't redundant, etc.). That reasoning
  is not repeated here. Position/channel values are untouched from the XML.

  The one deliberate change from arduino.ino: each gait string used to end
  with a literal "T500\r\n" baked in from the XML. Here that's gone -- every
  string ends right after the last channel value, and sendCmdP() appends
  "T<GAIT_MOVE_MS>\r\n" itself at send time (see "Timing" below). Retuning
  the gait's speed is now a single constant instead of a 28-way find/replace.

  ---------------------------------------------------------------------------
  ESP32 WIRING (differs from the AVR Arduino pinout)
  ---------------------------------------------------------------------------
    RTrobot servo controller : Serial (UART0) - the board's labeled RX/TX
                                pins (GPIO3 RX0 <- controller TX,
                                GPIO1 TX0 -> controller RX). These are the
                                SAME pins the onboard USB-serial bridge uses
                                to flash the board and run the Serial
                                Monitor -- exactly like NORA's ESP32<->Arduino
                                link. DISCONNECT the controller wires before
                                uploading new firmware, then reconnect.
    Debug console (optional)  : Serial2 - GPIO16 (RX2), GPIO17 (TX2). All the
                                boot/status/CLAMP logging that used to go out
                                the USB port now goes here instead, since
                                UART0 is busy with the controller. Hook a
                                USB-TTL adapter to GPIO17 (TX2) if you want to
                                see it; the board runs fine with nothing
                                attached there.
    HC-SR04                   : TRIG -> GPIO12, ECHO -> GPIO13
    MPU6050                    : SDA -> GPIO21, SCL -> GPIO22 (I2C)
    IR Receiver                : OUT -> GPIO14

  ---------------------------------------------------------------------------
  WIFI
  ---------------------------------------------------------------------------
  On boot, WHIP scans for an access point named "NORA". If it's in range,
  WHIP joins it as a station (same password NORA broadcasts with), so both
  robots share one network. If NORA's AP isn't found, WHIP starts its own
  access point instead ("WHIP" / "12345678") so the control page is still
  reachable directly from a phone or laptop. Either way, WHIP's own
  WebServer comes up on port 5005 once the network is ready -- check the
  debug console (Serial2, see above) at boot for the IP to browse to
  (http://<ip>:5005/).

  ---------------------------------------------------------------------------
  CONTROL MODES
  ---------------------------------------------------------------------------
  Obstacle Avoidance and IR Remote behave exactly as in arduino.ino. WEB is
  new: the control page's D-pad drives the robot the same way the IR remote
  does -- each press starts a repeating "still held" signal from the
  browser, and letting go (or losing the connection) drops back to standing
  within WEB_CMD_TIMEOUT_MS. Same fail-safe shape as the existing IR
  timeout, just longer to allow for HTTP round-trip time.

  ---------------------------------------------------------------------------
  RIFT FLEET INTEGRATION
  ---------------------------------------------------------------------------
  RIFT's README assigns every fleet member a fixed port for its own web UI
  (RIFT :5000, DREAM :5001, NORA :5002, KIDA-00 :5003, KIDA-01 :5004,
  WHIP :5005, MILA :5010). WHIP's WebServer below listens on 5005 to match.

  Whenever WHIP is on NORA's network (see WIFI above), it also heartbeats
  itself to NORA's fleet registry (192.168.4.1:5000/register) the same way
  RIFT's Fleet/register.py announces RIFT -- same form fields (name/type/
  capabilities), same protocol NORA's own esp32.ino serves on /register.
  That's what makes WHIP show up in RIFT's dashboard "Registered Fleet"
  list with her IP. In WHIP's-own-AP mode there's no known registry address
  to reach (no NORA network to find one on), so registration is skipped --
  the control page at :5005 still works either way, RIFT just won't list
  WHIP as a fleet member until she's on NORA's network.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <pgmspace.h>

#define RAW_BUFFER_LENGTH 100   // NEC needs far less than the default; saves RAM
#define DECODE_NEC              // must precede the IRremote include
#include <IRremote.hpp>         // IRremote 4.x -- supports ESP32

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
const char* NORA_SSID     = "NORA";
const char* NORA_PASSWORD = "12345678";
const char* AP_SSID       = "WHIP";
const char* AP_PASSWORD   = "12345678";

enum WifiMode { WIFI_JOINED_NORA, WIFI_OWN_AP };
WifiMode wifiMode = WIFI_OWN_AP;

// RIFT's assigned port for WHIP's control page (see "RIFT FLEET
// INTEGRATION" above) -- RIFT :5000, DREAM :5001, NORA :5002, KIDA-00
// :5003, KIDA-01 :5004, WHIP :5005, MILA :5010.
WebServer server(5005);

// ---------------------------------------------------------------------------
// RIFT fleet registry -- only reachable while WHIP is joined to NORA's
// network (see setupWiFi()); NORA is the network's fixed-address node, so
// her /register endpoint is the one address every fleet member can count
// on finding, whether NORA or RIFT (deferred authority) actually answers
// /robots underneath it.
// ---------------------------------------------------------------------------
const char*    FLEET_HOST = "192.168.4.1";   // NORA's fixed WiFi AP gateway address
const uint16_t FLEET_PORT = 5000;
// Well under NORA's FLEET_TTL_MS (20s in her esp32.ino) -- same margin
// RIFT's own Fleet/register.py heartbeats at.
const unsigned long FLEET_HEARTBEAT_MS = 10000;

// ---------------------------------------------------------------------------
// Pins / addresses
// ---------------------------------------------------------------------------
// Controller link runs on Serial (UART0's default pins, GPIO3/GPIO1) --
// no custom pin args needed. Debug console runs on Serial2, GPIO16 (RX2) /
// GPIO17 (TX2).
#define DEBUG_RX_PIN 16
#define DEBUG_TX_PIN 17
const int TRIG_PIN = 12;   // strapping pin (MTDI) -- we drive it, nothing external holds it at boot
const int ECHO_PIN = 13;
const int IRR_PIN  = 14;
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
// Web D-pad codes -- mirror the IR command shape (an "active command" byte
// plus a last-seen timestamp), just driven by HTTP requests instead of an
// IR repeat code. See runWebMode() / pollWeb().
// ---------------------------------------------------------------------------
#define WEB_FORWARD  'F'
#define WEB_BACKWARD 'B'
#define WEB_LEFT     'L'
#define WEB_RIGHT    'R'

const unsigned long WEB_CMD_TIMEOUT_MS = 500;

// ---------------------------------------------------------------------------
// Timing -- change GAIT_MOVE_MS to retune gait speed. It's the only value
// that needs editing: none of the gait strings have a T baked in anymore --
// sendCmdP() appends "T<GAIT_MOVE_MS>\r\n" to every command it sends, and
// LINE_MS/STAND_SETTLE_MS are derived below so they always stay in sync.
// (Originally T500D500 from the XML. GAIT_DWELL_MS was a leftover dead pause
// after each line's move finished -- zeroed out for continuous motion.
// Bench-test with the feet off the ground before trusting a new value.)
//
// Unlike the PC software (which uploads gait.xml as a stored action group
// the controller plays back internally), every frame here is re-sent live
// over the wire, and the controller can't start moving until the whole
// line -- including the trailing T -- has arrived. At 9600 baud an
// 18-channel power-stroke line (~145 bytes) took ~150ms just to transmit,
// which blew straight through the old 20ms margin: WHIP was sending the
// next frame before the servos had actually finished the previous one.
// That's the source of the overlap, not GAIT_MOVE_MS itself.
//
// CONTROLLER_BAUD must be set to match the RTrobot controller's own
// configured UART baud -- that's a setting on the controller itself
// (normally changed once from its PC config software, the same one used
// to build gait.xml), NOT something this sketch can change remotely. Before
// raising this, go set the controller to the same rate and power-cycle it
// to confirm the new rate stuck -- if this doesn't match, WHIP simply loses
// the controller (garbled/no response), it won't damage anything, but stand
// up will do nothing. RTrobot-family 32ch controllers document UART support
// for 4800/9600/19200/38400/57600 (some revisions to 115200); 38400 is a
// safe, widely-supported middle ground -- ~4x less transmission time.
// ---------------------------------------------------------------------------
const unsigned long CONTROLLER_BAUD = 38400;

const uint16_t GAIT_MOVE_MS      = 300;   // transmitted: how long the controller takes to execute a move
const uint16_t GAIT_DWELL_MS     = 0;     // NOT transmitted -- how long we wait after that before the next line
// At CONTROLLER_BAUD, the worst-case 18-channel line (~145 bytes, 10 bits/byte)
// takes roughly (145 * 10 * 1000) / CONTROLLER_BAUD ms to transmit. Margin
// covers that plus a little slop, so even the heaviest line finishes well
// inside LINE_MS.
const uint16_t TIMING_MARGIN_MS  = (uint16_t)((145UL * 10UL * 1000UL) / CONTROLLER_BAUD) + 20;

const unsigned long LINE_MS         = GAIT_MOVE_MS + GAIT_DWELL_MS + TIMING_MARGIN_MS;
const unsigned long STAND_SETTLE_MS = LINE_MS;
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
  "#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500";

const char REST_CMD[] PROGMEM =
  "#2P2500#3P2500#5P2500#6P2500#7P800#8P2500#9P2500"
  "#25P500#26P500#28P500#29P500#31P500#32P500";

// ---------------------------------------------------------------------------
// Forward Gait -- XML group 0, verbatim
// ---------------------------------------------------------------------------
const char FWD_1[] PROGMEM =
  "#1P1100#2P2000#3P1000#4P1500#5P1500#6P1500#7P1100#8P2000#9P1000"
  "#24P1500#25P1500#26P1500#27P1900#28P1000#29P2000#30P1500#31P1500#32P1500";
const char FWD_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500";
const char FWD_3[] PROGMEM =
  "#1P1500#4P1100#5P2000#6P1000#7P1500#24P1900#25P1000#26P2000"
  "#27P1500#30P1900#31P1000#32P2000";
const char FWD_4[] PROGMEM =
  "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500";

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
  "#24P1500#25P1500#26P1500#27P1100#28P1000#29P2000#30P1500#31P1500#32P1500";
const char BWD_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500";
const char BWD_3[] PROGMEM =
  "#1P1500#4P1900#5P2000#6P1000#7P1500#24P1100#25P1000#26P2000"
  "#27P1500#30P1100#31P1000#32P2000";
const char BWD_4[] PROGMEM =
  "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500";

const char* const BACKWARD_GAIT[] PROGMEM = { BWD_1, BWD_2, BWD_3, BWD_4 };
const uint8_t BACKWARD_GAIT_LEN = 4;

// ---------------------------------------------------------------------------
// Turn Left Gait -- XML group 2, verbatim
// ---------------------------------------------------------------------------
const char TURNL_1[] PROGMEM =
  "#1P1000#2P2500#3P1100#4P1500#5P1500#6P1500#7P1000#8P2500#9P1100"
  "#24P1500#25P1500#26P1500#27P1000#28P500#29P1900#30P1500#31P1500#32P1500";
const char TURNL_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500";
const char TURNL_3[] PROGMEM =
  "#1P1000#2P1500#3P1500#4P1000#5P2500#6P1100#7P1000#8P1500#9P1500"
  "#24P1000#25P500#26P1900#27P1000#28P1500#29P1500#30P1000#31P500#32P1900";
const char TURNL_4[] PROGMEM =
  "#1P1500#7P1500#27P1500";
const char TURNL_5[] PROGMEM =
  "#1P1500#2P1500#3P1500#5P1500#6P1500#7P1500#8P1500#9P1500"
  "#25P1500#26P1500#27P1500#28P1500#29P1500#31P1500#32P1500";
const char* const TURN_LEFT_GAIT[] PROGMEM = { TURNL_1, TURNL_2, TURNL_3, TURNL_4, TURNL_5 };
const uint8_t TURN_LEFT_GAIT_LEN = 5;

// ---------------------------------------------------------------------------
// Turn Right Gait -- XML group 3, verbatim
// (Asymmetric vs Turn Left: line 3 omits the 1,7,27 hold and line 5 omits
//  the 4,24,30 reset. That is how it is in the XML. Left untouched.)
// ---------------------------------------------------------------------------
const char TURNR_1[] PROGMEM =
  "#1P2000#2P2500#3P1000#4P1500#5P1500#6P1500#7P2000#8P2500#9P1000"
  "#24P1500#25P1500#26P1500#27P2000#28P500#29P2000#30P1500#31P1500#32P1500";
const char TURNR_2[] PROGMEM =
  "#2P1500#3P1500#8P1500#9P1500#28P1500#29P1500";
const char TURNR_3[] PROGMEM =
  "#4P2000#5P2500#6P1000#24P2000#25P500#26P2000#30P2000#31P500#32P2000";
const char TURNR_4[] PROGMEM =
  "#1P1500#7P1500#27P1500#30P2000";
const char TURNR_5[] PROGMEM =
  "#5P1500#6P1500#25P1500#26P1500#31P1500#32P1500";
const char* const TURN_RIGHT_GAIT[] PROGMEM = { TURNR_1, TURNR_2, TURNR_3, TURNR_4, TURNR_5 };
const uint8_t TURN_RIGHT_GAIT_LEN = 5;

// ---------------------------------------------------------------------------
// Shutdown Sequence -- XML group 1, verbatim
// ---------------------------------------------------------------------------
const char SHUT_1[] PROGMEM =
  "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500"
  "#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500";
const char SHUT_2[] PROGMEM = "#2P2500#3P1500#8P2500#28P500#29P1500";
const char SHUT_3[] PROGMEM = "#3P500#9P500#29P2500";
const char SHUT_4[] PROGMEM = "#3P2500#9P2500#29P500";
const char SHUT_5[] PROGMEM = "#5P1700#6P1200#25P1200#26P1700#31P1200#32P1700";
const char SHUT_6[] PROGMEM = "#5P2500#25P500#26P1700#28P500#31P500";
const char SHUT_7[] PROGMEM = "#6P500#26P2500#32P2500";
const char SHUT_8[] PROGMEM = "#6P2500#26P500#32P500";

const char* const SHUTDOWN_SEQ[] PROGMEM = { SHUT_1, SHUT_2, SHUT_3, SHUT_4, SHUT_5, SHUT_6, SHUT_7, SHUT_8 };
const uint8_t SHUTDOWN_SEQ_LEN = 8;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum SystemState   { SYS_ACTIVE, SYS_TILT_SAFE, SYS_SHUTDOWN };
enum ControlMode   { CTRL_OBSTACLE, CTRL_IR, CTRL_WEB };
enum MovementState { MOVE_STAND, MOVE_WALK, MOVE_BACKWARD, MOVE_TURN_LEFT, MOVE_TURN_RIGHT };

SystemState   sysState    = SYS_ACTIVE;
ControlMode   controlMode = CTRL_OBSTACLE;
ControlMode   controlModeBeforeTilt = CTRL_OBSTACLE;
MovementState currentMove = MOVE_STAND;

bool abortMovement = false;   // cleared at the top of every loop()

uint8_t irActiveCmd      = 0;
uint8_t irCmdAtMoveStart = 0;
unsigned long irLastCmdMillis = 0;

char webActiveCmd      = 0;   // 0 = no button held on the web D-pad
char webCmdAtMoveStart = 0;
unsigned long webLastCmdMillis = 0;

uint8_t obstacleCount    = 0;
bool    obstacleDetected = false;

// MPU6050 state -- declared here (not down by mpuInit()/updateTilt() where
// they're used) because setup() and handleStatus() both reference these
// directly, and Arduino only auto-prototypes functions, not variables; a
// plain global still has to appear before its first use in the file.
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;
float gyroXBias = 0, gyroYBias = 0;
float pitchDeg = 0, rollDeg = 0;
float pitchOffsetDeg = 0, rollOffsetDeg = 0;
unsigned long lastTiltUpdate = 0;
bool mpuFound = false;

unsigned long tiltOverSince  = 0;
unsigned long tiltClearSince = 0;

bool gaitWaitUntil(unsigned long start, unsigned long ms, bool scanSafe);
bool gaitWait(unsigned long ms, bool scanSafe);
long readDistanceCM();
void pollTilt();
void pollIR();
void pollWeb();
void runWebMode();
void setupWiFi();
void setupWebServer();
void pollFleetRegistration();

// ---------------------------------------------------------------------------
// MECHANICAL SAFETY LIMITS
// ---------------------------------------------------------------------------
// Every outgoing command is clamped against these before transmission, so a bad
// gait value, a typo, or a corrupted serial byte can't drive a servo into its
// stop and stall it.
//
// The two sides mirror around 1500: mirror(v) = 3000 - v.
// Confirmed against the gait data -- #2P2000 <-> #28P1000, #1P1100 <-> #27P1900.
//
// So you only measure SIDE A. Side B is derived. Note that mirroring SWAPS the
// bounds: if side A femur is safe over [1200, 2400], side B is [600, 1800].
//
//   Side A: legs 1, 4, 7   -> coxa 1/4/7,    femur 2/5/8,    tibia 3/6/9
//   Side B: legs 24,27,30  -> coxa 24/27/30, femur 25/28/31, tibia 26/29/32
//
// MEASURE THESE with the feet off the ground, one channel at a time, stepping
// 100us out from 1500 until the joint buzzes or hits a stop, then back off 100.
// Left at 500/2500 they clamp nothing.
//
const uint16_t A_COXA_MIN  = 500,  A_COXA_MAX  = 2500;
const uint16_t A_FEMUR_MIN = 500,  A_FEMUR_MAX = 2500;
const uint16_t A_TIBIA_MIN = 500,  A_TIBIA_MAX = 2500;

// Derived - do not edit. Bounds swap under mirroring.
const uint16_t B_COXA_MIN  = 3000 - A_COXA_MAX;
const uint16_t B_COXA_MAX  = 3000 - A_COXA_MIN;
const uint16_t B_FEMUR_MIN = 3000 - A_FEMUR_MAX;
const uint16_t B_FEMUR_MAX = 3000 - A_FEMUR_MIN;
const uint16_t B_TIBIA_MIN = 3000 - A_TIBIA_MAX;
const uint16_t B_TIBIA_MAX = 3000 - A_TIBIA_MIN;

const uint16_t CH_MIN[33] PROGMEM = {
  500,                                                    // 0 unused
  A_COXA_MIN, A_FEMUR_MIN, A_TIBIA_MIN,                   // 1-3   leg 1
  A_COXA_MIN, A_FEMUR_MIN, A_TIBIA_MIN,                   // 4-6   leg 4
  A_COXA_MIN, A_FEMUR_MIN, A_TIBIA_MIN,                   // 7-9   leg 7
  500,500,500,500,500,500,500,500,500,500,500,500,500,500,// 10-23 unused
  B_COXA_MIN, B_FEMUR_MIN, B_TIBIA_MIN,                   // 24-26 leg 24
  B_COXA_MIN, B_FEMUR_MIN, B_TIBIA_MIN,                   // 27-29 leg 27
  B_COXA_MIN, B_FEMUR_MIN, B_TIBIA_MIN                    // 30-32 leg 30
};

const uint16_t CH_MAX[33] PROGMEM = {
  2500,
  A_COXA_MAX, A_FEMUR_MAX, A_TIBIA_MAX,
  A_COXA_MAX, A_FEMUR_MAX, A_TIBIA_MAX,
  A_COXA_MAX, A_FEMUR_MAX, A_TIBIA_MAX,
  2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,2500,
  B_COXA_MAX, B_FEMUR_MAX, B_TIBIA_MAX,
  B_COXA_MAX, B_FEMUR_MAX, B_TIBIA_MAX,
  B_COXA_MAX, B_FEMUR_MAX, B_TIBIA_MAX
};

// ---------------------------------------------------------------------------
// Serial send helpers
// ---------------------------------------------------------------------------
char cmdBuf[190];    // raw command from PROGMEM
char safeBuf[190];   // clamped command actually transmitted

// ---------------------------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial2.begin(115200, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);   // debug console
  Serial2.println();
  Serial2.println(F("WHIP Hexapod - ESP32 boot"));

  Serial.begin(CONTROLLER_BAUD);   // RTrobot controller link -- must match its configured baud

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(21, 22);
  mpuInit();

  IrReceiver.begin(IRR_PIN, DISABLE_LED_FEEDBACK);

  setupWiFi();
  setupWebServer();

  delay(2000);

  auditAllGaits();

  Serial2.println(F("Calibrating gyro..."));
  calibrateGyro();

  sendCmdP(STAND_CMD);
  Serial2.println(F("Sent: stand pose"));
  currentMove = MOVE_STAND;
  delay(1500);

  if (mpuFound) calibrateTiltOffset();

  controlMode = CTRL_OBSTACLE;
  Serial2.println(F("CONTROL -> OBSTACLE AVOIDANCE"));
}

void loop() {
  abortMovement = false;

  while (Serial.available()) Serial2.write(Serial.read());

  if (Serial2.available()) {   // manual shutdown trigger from the debug console
    char c = Serial2.read();
    if (c == 'x' || c == 'X') {
      sysState = SYS_SHUTDOWN;
      Serial2.println(F("SHUTDOWN"));
    }
  }

  server.handleClient();
  pollFleetRegistration();

  if (sysState == SYS_SHUTDOWN) {
    ShutdownSequence();
    while (true) { delay(1000); }
  }

  pollTilt();
  if (sysState == SYS_TILT_SAFE) return;

  pollIR();
  pollWeb();

  if (controlMode == CTRL_OBSTACLE) {
    runObstacleMode();
  } else if (controlMode == CTRL_IR) {
    runIRMode();
  } else {
    runWebMode();
  }
}

// Parses "#<ch>P<val>" tokens and clamps each value. The gait strings no
// longer carry a trailing T/\r\n (sendCmdP appends "T<GAIT_MOVE_MS>\r\n"
// after this runs), so in practice every character here is part of a
// token -- the passthrough branch just protects against a stray or
// malformed byte instead of being load-bearing. Reports anything it had to
// correct.
void clampCommand(const char* in, char* out, size_t outSize) {
  size_t o = 0;
  const char* p = in;

  while (*p && o < outSize - 1) {
    if (*p != '#') { out[o++] = *p++; continue; }

    const char* tokenStart = p;
    p++;                                   // skip '#'

    uint16_t ch = 0;
    if (!isdigit(*p)) { out[o++] = *tokenStart; p = tokenStart + 1; continue; }
    while (isdigit(*p)) ch = ch * 10 + (*p++ - '0');

    if (*p != 'P') {                       // malformed - copy raw and move on
      while (tokenStart < p && o < outSize - 1) out[o++] = *tokenStart++;
      continue;
    }
    p++;                                   // skip 'P'

    uint16_t val = 0;
    while (isdigit(*p)) val = val * 10 + (*p++ - '0');

    uint16_t lo = 500, hi = 2500;
    if (ch <= 32) {
      lo = pgm_read_word(&CH_MIN[ch]);
      hi = pgm_read_word(&CH_MAX[ch]);
    }

    uint16_t clamped = val;
    if (clamped < lo) clamped = lo;
    if (clamped > hi) clamped = hi;

    if (clamped != val) {
      Serial2.print(F("CLAMP ch"));
      Serial2.print(ch);
      Serial2.print(F(": "));
      Serial2.print(val);
      Serial2.print(F(" -> "));
      Serial2.println(clamped);
    }

    o += snprintf(out + o, outSize - o, "#%uP%u", ch, clamped);
  }
  out[o] = '\0';
}

// Runs every stored gait line through the clamp without transmitting, so any
// value that exceeds a mechanical limit is reported at boot rather than
// discovered mid-walk. clampCommand() prints each correction it makes.
void auditTable(const char* const table[], uint8_t len, const __FlashStringHelper* name) {
  Serial2.print(F("  checking "));
  Serial2.println(name);
  for (uint8_t i = 0; i < len; i++) {
    strncpy_P(cmdBuf, (const char*)pgm_read_ptr(&table[i]), sizeof(cmdBuf) - 1);
    cmdBuf[sizeof(cmdBuf) - 1] = '\0';
    clampCommand(cmdBuf, safeBuf, sizeof(safeBuf));
  }
}

void auditAllGaits() {
  Serial2.println(F("Auditing gait tables against servo limits..."));
  auditTable(FORWARD_GAIT,    FORWARD_GAIT_LEN,    F("FORWARD"));
  auditTable(BACKWARD_GAIT,   BACKWARD_GAIT_LEN,   F("BACKWARD"));
  auditTable(TURN_LEFT_GAIT,  TURN_LEFT_GAIT_LEN,  F("TURN LEFT"));
  auditTable(TURN_RIGHT_GAIT, TURN_RIGHT_GAIT_LEN, F("TURN RIGHT"));
  auditTable(SHUTDOWN_SEQ,    SHUTDOWN_SEQ_LEN,    F("SHUTDOWN"));
  strncpy_P(cmdBuf, STAND_CMD, sizeof(cmdBuf) - 1);
  cmdBuf[sizeof(cmdBuf) - 1] = '\0';
  clampCommand(cmdBuf, safeBuf, sizeof(safeBuf));
  Serial2.println(F("Audit done. Any CLAMP lines above are values your gaits"));
  Serial2.println(F("exceed -- fix the XML, or widen the limit if it's safe."));
}

void sendCmdP(const char* pgmCmd) {
  strncpy_P(cmdBuf, pgmCmd, sizeof(cmdBuf) - 1);
  cmdBuf[sizeof(cmdBuf) - 1] = '\0';
  clampCommand(cmdBuf, safeBuf, sizeof(safeBuf));
  Serial.print(safeBuf);
  Serial.print('T');
  Serial.print(GAIT_MOVE_MS);
  Serial.print("\r\n");
}

void sendTableLine(const char* const table[], uint8_t index) {
  sendCmdP((const char*)pgm_read_ptr(&table[index]));
}

// ---------------------------------------------------------------------------
// MOVEMENT FUNCTIONS -- each plays one full cycle
// ---------------------------------------------------------------------------
bool playGait(const char* const table[], uint8_t len, const bool* scanTable) {
  for (uint8_t i = 0; i < len; i++) {
    unsigned long frameStart = millis();      // clock starts BEFORE transmission
    sendTableLine(table, i);
    bool scanSafe = scanTable ? (bool)pgm_read_byte(&scanTable[i]) : false;
    if (!gaitWaitUntil(frameStart, LINE_MS, scanSafe)) return false;
  }
  return true;
}

void Stand() {
  sendCmdP(STAND_CMD);
  currentMove = MOVE_STAND;
  Serial2.println(F("MOVE -> STAND"));
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
    unsigned long frameStart = millis();
    Stand();
    if (!gaitWaitUntil(frameStart, STAND_SETTLE_MS, false)) return false;
  }

  if (next == MOVE_STAND) {
    currentMove = MOVE_STAND;
    return true;
  }

  currentMove = next;
  switch (next) {
    case MOVE_WALK:       Serial2.println(F("MOVE -> WALK"));       break;
    case MOVE_BACKWARD:   Serial2.println(F("MOVE -> BACKWARD"));   break;
    case MOVE_TURN_LEFT:  Serial2.println(F("MOVE -> TURN LEFT"));  break;
    case MOVE_TURN_RIGHT: Serial2.println(F("MOVE -> TURN RIGHT")); break;
    default: break;
  }
  return true;
}

// ---------------------------------------------------------------------------
// gaitWait -- the only wait used inside a movement
// ---------------------------------------------------------------------------
// Waits until `ms` have elapsed since `start`. Because the caller timestamps
// before transmitting, every gait frame occupies exactly LINE_MS regardless of
// how many channels that line contains -- which is how the RTrobot software
// schedules its groups. Returns false when the move should be dropped.
bool gaitWaitUntil(unsigned long start, unsigned long ms, bool scanSafe) {
  static unsigned long lastPing = 0;

  while (millis() - start < ms) {
    while (Serial.available()) Serial2.write(Serial.read());

    server.handleClient();   // keep the control page responsive mid-gait

    pollTilt();
    pollIR();
    pollWeb();

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

    if (controlMode == CTRL_IR  && irActiveCmd  != irCmdAtMoveStart)  return false;
    if (controlMode == CTRL_WEB && webActiveCmd != webCmdAtMoveStart) return false;
    if (abortMovement || sysState != SYS_ACTIVE) return false;
  }
  return true;
}

// Convenience wrapper for waits that don't wrap a transmission.
bool gaitWait(unsigned long ms, bool scanSafe) {
  return gaitWaitUntil(millis(), ms, scanSafe);
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
// WEB CONTROL MODE -- driven by the browser D-pad, same shape as IR mode
// ---------------------------------------------------------------------------
void runWebMode() {
  webCmdAtMoveStart = webActiveCmd;

  if (webActiveCmd == WEB_FORWARD) {
    if (requestMove(MOVE_WALK)) MoveForward();
  }
  else if (webActiveCmd == WEB_BACKWARD) {
    if (requestMove(MOVE_BACKWARD)) MoveBackward();
  }
  else if (webActiveCmd == WEB_LEFT) {
    if (requestMove(MOVE_TURN_LEFT)) TurnLeft();
  }
  else if (webActiveCmd == WEB_RIGHT) {
    if (requestMove(MOVE_TURN_RIGHT)) TurnRight();
  }
  else {
    requestMove(MOVE_STAND);   // no button held / connection lost -> stand
  }
}

// Browser repeats the held command every ~200ms (see INDEX_HTML); if none
// arrives for a while, assume the page was closed or the connection dropped
// and fail safe to "no command held" instead of walking forever.
void pollWeb() {
  if (webActiveCmd != 0 && millis() - webLastCmdMillis > WEB_CMD_TIMEOUT_MS) {
    webActiveCmd = 0;
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
  Serial2.println(F("WALL -> stand 3s"));

  Stand();
  if (!gaitWait(STAND_HOLD_MS, false)) return;

  // --- TURN TWICE ---
  Serial2.println(F("TURNING x2"));
  currentMove = MOVE_TURN_LEFT;
  for (uint8_t i = 0; i < AVOID_TURN_LOOPS; i++) {
    if (!TurnLeft()) return;
  }

  // --- BACK TO STAND ---
  Stand();
  if (!gaitWait(STAND_SETTLE_MS, false)) return;

  // --- SCAN ---
  if (pathClear()) {
    Serial2.println(F("CLEAR -> forward"));
  } else {
    Serial2.println(F("STILL BLOCKED -> turn again"));
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
      Serial2.println(F("CONTROL -> IR REMOTE"));
    }
    return;
  }

  if (cmd == IR_MODE_OBSTACLE) {
    if (controlMode != CTRL_OBSTACLE) {
      controlMode      = CTRL_OBSTACLE;
      obstacleCount    = 0;
      obstacleDetected = false;
      abortMovement    = true;
      Serial2.println(F("CONTROL -> OBSTACLE AVOIDANCE"));
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
      Serial2.print(F("IR cmd=0x"));
      Serial2.println(IrReceiver.decodedIRData.command, HEX);
      handleIRCommand(IrReceiver.decodedIRData.command);
    } else {
      Serial2.println(F("IR noise (UNKNOWN protocol)"));
    }
    IrReceiver.resume();
  }

  if (irActiveCmd != 0 && millis() - irLastCmdMillis > IR_CMD_TIMEOUT_MS) {
    irActiveCmd = 0;
  }
}

// ---------------------------------------------------------------------------
// WIFI + WEB SERVER
// ---------------------------------------------------------------------------
void setupWiFi() {
  Serial2.println(F("Scanning for NORA..."));
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  bool noraFound = false;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == NORA_SSID) { noraFound = true; break; }
  }
  WiFi.scanDelete();

  if (noraFound) {
    Serial2.println(F("NORA found -> joining her network"));
    WiFi.begin(NORA_SSID, NORA_PASSWORD);
    unsigned long attemptStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - attemptStart < 15000) {
      delay(250);
      Serial2.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiMode = WIFI_JOINED_NORA;
      Serial2.println();
      Serial2.print(F("Joined NORA. IP: "));
      Serial2.println(WiFi.localIP());
      return;
    }
    Serial2.println();
    Serial2.println(F("Join attempt failed -> falling back to own AP"));
  } else {
    Serial2.println(F("NORA not in range -> starting own AP"));
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  wifiMode = WIFI_OWN_AP;
  Serial2.print(F("AP \""));
  Serial2.print(AP_SSID);
  Serial2.print(F("\" up. IP: "));
  Serial2.println(WiFi.softAPIP());
}

// Same wire format NORA's /register expects (esp32.ino, setupFleetServer())
// and RIFT's own Fleet/register.py sends when it announces itself: a plain
// form POST with name/type/capabilities. Blocks for up to ~2s on a slow or
// dropped connection, which is why this is only ever called from loop()
// and never from inside a gait's busy-wait -- stalling there would throw
// off LINE_MS-timed servo frames.
void registerWithFleet() {
  HTTPClient http;
  http.begin(String("http://") + FLEET_HOST + ":" + FLEET_PORT + "/register");
  http.setTimeout(2000);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.POST("name=WHIP&type=hexapod&capabilities=obstacle_avoidance,ir_remote,web_control");
  http.end();
}

void pollFleetRegistration() {
  if (wifiMode != WIFI_JOINED_NORA) return;   // no known registry address without NORA's network

  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat < FLEET_HEARTBEAT_MS) return;
  lastHeartbeat = millis();

  registerWithFleet();
}

void handleWebCmd() {
  if (!server.hasArg("cmd")) { server.send(400, "text/plain", "missing 'cmd'"); return; }
  String cmd = server.arg("cmd");

  if      (cmd == "fwd")   { webActiveCmd = WEB_FORWARD;  webLastCmdMillis = millis(); }
  else if (cmd == "bwd")   { webActiveCmd = WEB_BACKWARD; webLastCmdMillis = millis(); }
  else if (cmd == "left")  { webActiveCmd = WEB_LEFT;     webLastCmdMillis = millis(); }
  else if (cmd == "right") { webActiveCmd = WEB_RIGHT;    webLastCmdMillis = millis(); }
  else if (cmd == "stop")  { webActiveCmd = 0; abortMovement = true; }
  else { server.send(400, "text/plain", "bad cmd"); return; }

  server.send(200, "text/plain", "OK");
}

void handleModeCmd() {
  if (!server.hasArg("m")) { server.send(400, "text/plain", "missing 'm'"); return; }
  String m = server.arg("m");

  if (m == "obstacle") {
    controlMode      = CTRL_OBSTACLE;
    obstacleCount    = 0;
    obstacleDetected = false;
    abortMovement    = true;
    Serial2.println(F("CONTROL -> OBSTACLE AVOIDANCE"));
  } else if (m == "ir") {
    controlMode   = CTRL_IR;
    irActiveCmd   = 0;
    abortMovement = true;
    Serial2.println(F("CONTROL -> IR REMOTE"));
  } else if (m == "web") {
    controlMode   = CTRL_WEB;
    webActiveCmd  = 0;
    abortMovement = true;
    Serial2.println(F("CONTROL -> WEB"));
  } else {
    server.send(400, "text/plain", "bad mode");
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleShutdownCmd() {
  sysState = SYS_SHUTDOWN;
  Serial2.println(F("SHUTDOWN (web)"));
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  long distance = readDistanceCM();

  String json = "{";
  json += "\"distance\":"    + String(distance) + ",";
  json += "\"pitch\":"       + String(pitchDeg - pitchOffsetDeg, 1) + ",";
  json += "\"roll\":"        + String(rollDeg - rollOffsetDeg, 1) + ",";
  json += "\"sysState\":"    + String((int)sysState) + ",";
  json += "\"controlMode\":" + String((int)controlMode) + ",";
  json += "\"move\":"        + String((int)currentMove) + ",";
  json += "\"wifiMode\":"    + String((int)wifiMode) + ",";
  json += "\"mpuFound\":"    + String(mpuFound ? 1 : 0);
  json += "}";

  server.send(200, "application/json", json);
}

void setupWebServer() {
  server.on("/",         handleRoot);
  server.on("/web",      handleWebCmd);
  server.on("/mode",     handleModeCmd);
  server.on("/shutdown", handleShutdownCmd);
  server.on("/status",   handleStatus);
  server.begin();
  Serial2.println(F("Web control server started on port 5005."));
}

// =====================
// WEB PAGE (stored in flash, not RAM)
// =====================
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>WHIP Control</title>
  <style>
    :root {
      /* same palette as the rest of the fleet's HUDs -- one look throughout */
      --bg:          #07090f;
      --panel:       #0a0e1a;
      --border:      #1e3458;
      --accent:      #6496e6;
      --accent-rgb:  100,150,230;
      --text:        #a0b8e8;
      --text-dim:    #4a5f80;
      --text-val:    #d7e6ff;
      --green:       #46d764;
      --green-rgb:   70,215,100;
      --orange:      #ffbe50;
      --orange-rgb:  255,190,80;
      --red:         #c84040;
      --red-rgb:     200,64,64;
      --radius:      6px;
      --font:        'Courier New', Courier, monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: var(--bg); color: var(--text);
      font-family: var(--font);
      display: flex; flex-direction: column; align-items: center;
      min-height: 100vh; padding: 20px; gap: 16px;
    }
    h1 { font-size: 1.6rem; letter-spacing: 3px; color: var(--accent); }
    h2 { font-size: 0.8rem; color: var(--text-dim); letter-spacing: 1px; }

    #sensors { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; width: 100%; max-width: 340px; }
    .sensor-box {
      background: var(--panel); border: 2px solid var(--border); border-radius: var(--radius);
      padding: 8px 12px; font-size: 0.85rem;
      display: flex; flex-direction: column; gap: 4px;
    }
    .sensor-box span { color: var(--accent); font-weight: bold; }

    #modeRow { display: flex; gap: 8px; flex-wrap: wrap; justify-content: center; }
    .mode-btn {
      padding: 10px 18px; border: 2px solid var(--border); border-radius: var(--radius);
      font-size: 0.85rem; font-weight: bold; cursor: pointer;
      background: var(--panel); color: var(--text-dim);
    }
    .mode-btn.active { border-color: var(--accent); color: var(--accent); background: rgba(var(--accent-rgb), 0.12); }

    .dpad {
      display: grid;
      grid-template-columns: repeat(3, 80px);
      grid-template-rows: repeat(3, 80px);
      gap: 8px;
    }
    .btn {
      background: var(--panel); border: 2px solid var(--border); border-radius: var(--radius);
      color: var(--text-val); font-size: 1.5rem; cursor: pointer; user-select: none;
      display: flex; align-items: center; justify-content: center;
      -webkit-tap-highlight-color: transparent; transition: opacity 0.2s;
      touch-action: none;
    }
    .btn.disabled { opacity: 0.25; pointer-events: none; }
    .btn.pressed  { background: rgba(var(--accent-rgb), 0.25); border-color: var(--accent); }
    .btn.stop-btn { background: rgba(var(--red-rgb), 0.15); border-color: var(--red); font-size: 0.8rem; font-weight: bold; color: var(--red); }
    .btn.stop-btn.pressed { background: rgba(var(--red-rgb), 0.3); }
    .fw    { grid-column: 2; grid-row: 1; }
    .turnL { grid-column: 1; grid-row: 2; }
    .left  { grid-column: 1; grid-row: 3; }
    .stop  { grid-column: 2; grid-row: 2; }
    .bw    { grid-column: 2; grid-row: 3; }
    .turnR { grid-column: 3; grid-row: 2; }
    .right { grid-column: 3; grid-row: 3; }
    #status { font-size: 0.8rem; color: var(--text-dim); text-align: center; }

    #shutdownBtn {
      padding: 9px 20px; border: 2px solid var(--red); border-radius: var(--radius);
      font-size: 0.85rem; font-weight: bold; cursor: pointer;
      background: rgba(var(--red-rgb), 0.12); color: var(--red);
    }
  </style>
</head>
<body>
  <h1>WHIP</h1>
  <h2>Hexapod Control</h2>

  <div id="sensors">
    <div class="sensor-box">
      <div>Distance <span id="sDist">--</span> cm</div>
    </div>
    <div class="sensor-box">
      <div>Tilt <span id="sTilt">--</span> deg</div>
    </div>
  </div>

  <div id="modeRow">
    <div class="mode-btn" id="modeObstacle" onclick="setMode('obstacle')">OBSTACLE</div>
    <div class="mode-btn" id="modeIr"       onclick="setMode('ir')">IR REMOTE</div>
    <div class="mode-btn" id="modeWeb"      onclick="setMode('web')">WEB</div>
  </div>

  <div class="dpad">
    <div class="btn fw"    id="btnFw">&#8593;</div>
    <div class="btn turnL" id="btnTurnL">&#8630;</div>
    <div class="btn stop stop-btn" id="btnStop">STOP</div>
    <div class="btn turnR" id="btnTurnR">&#8631;</div>
    <div class="btn bw"    id="btnBw">&#8595;</div>
  </div>

  <div id="status">mode: -- | move: -- | wifi: --</div>

  <div id="shutdownBtn" onclick="doShutdown()">SHUTDOWN</div>

  <script>
    let repeatTimer = null;
    let curMode = 'obstacle';

    function sendCmd(c) {
      fetch('/web?cmd=' + c).catch(function(){});
    }

    function press(c, el) {
      if (curMode !== 'web') return;
      el.classList.add('pressed');
      sendCmd(c);
      if (repeatTimer) clearInterval(repeatTimer);
      repeatTimer = setInterval(function () { sendCmd(c); }, 200);
    }

    function release(el) {
      if (repeatTimer) { clearInterval(repeatTimer); repeatTimer = null; }
      if (el) el.classList.remove('pressed');
      if (curMode === 'web') sendCmd('stop');
    }

    function bindHold(id, cmd) {
      const el = document.getElementById(id);
      el.addEventListener('mousedown',  function () { press(cmd, el); });
      el.addEventListener('touchstart', function (e) { e.preventDefault(); press(cmd, el); });
      el.addEventListener('mouseup',    function () { release(el); });
      el.addEventListener('mouseleave', function () { release(el); });
      el.addEventListener('touchend',   function (e) { e.preventDefault(); release(el); });
    }

    bindHold('btnFw',    'fwd');
    bindHold('btnBw',    'bwd');
    bindHold('btnTurnL', 'left');
    bindHold('btnTurnR', 'right');

    document.getElementById('btnStop').addEventListener('click', function () {
      release(null);
      sendCmd('stop');
    });

    function setMode(m) {
      curMode = m;
      fetch('/mode?m=' + m).catch(function(){});
      document.getElementById('modeObstacle').classList.toggle('active', m === 'obstacle');
      document.getElementById('modeIr').classList.toggle('active', m === 'ir');
      document.getElementById('modeWeb').classList.toggle('active', m === 'web');
      document.querySelectorAll('.dpad .btn').forEach(function (b) {
        b.classList.toggle('disabled', m !== 'web');
      });
    }

    function doShutdown() {
      if (confirm('Shutdown WHIP? This cannot be undone without a power cycle.')) {
        fetch('/shutdown').catch(function(){});
      }
    }

    const MODE_NAMES  = ['OBSTACLE', 'IR', 'WEB'];
    const MOVE_NAMES  = ['STAND', 'WALK', 'BACKWARD', 'TURN L', 'TURN R'];
    const WIFI_NAMES  = ['JOINED NORA', 'OWN AP (WHIP)'];

    function updateStatus() {
      fetch('/status').then(function (r) { return r.json(); }).then(function (j) {
        document.getElementById('sDist').textContent = j.distance >= 0 ? j.distance : '--';
        document.getElementById('sTilt').textContent = Math.max(Math.abs(j.pitch), Math.abs(j.roll)).toFixed(1);
        document.getElementById('status').textContent =
          'mode: ' + MODE_NAMES[j.controlMode] +
          ' | move: ' + MOVE_NAMES[j.move] +
          ' | wifi: ' + WIFI_NAMES[j.wifiMode];
      }).catch(function(){});
    }

    setMode('obstacle');
    updateStatus();
    setInterval(updateStatus, 500);
  </script>
</body>
</html>
)rawhtml";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
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
// (state variables declared up near the other globals -- see the comment
// there for why)
// ---------------------------------------------------------------------------
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  mpuFound = (Wire.endTransmission() == 0);
  if (!mpuFound) Serial2.println(F("MPU6050 NOT FOUND! Check wiring."));
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
    Serial2.println(F("TILT SAFE - holding stand pose"));
  }
}

void exitTiltSafe() {
  sysState    = SYS_ACTIVE;
  controlMode = controlModeBeforeTilt;
  sendCmdP(STAND_CMD);
  currentMove = MOVE_STAND;
  Serial2.println(F("Recovering from tilt"));
  delay(500);

  obstacleCount    = 0;
  obstacleDetected = false;
  irActiveCmd      = 0;
  webActiveCmd     = 0;
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
  Serial2.println(F("Shutdown complete."));
}
