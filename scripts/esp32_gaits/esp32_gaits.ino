/*
  WHIP Hexapod - ESP32 Control Sketch: the gaits version (testing; flash_gaits.bat).
  Tripod / ripple / wave (gaits.h), the 4x4 LED grid (leds.h) and the buzzer
  (buzzer.h) on top of scripts/esp32. When it walks well it moves over there.
  Controller: RTrobot Servo Motor Controller (32 channel)

  WHIP walks by inverse kinematics (ik_gait.h): every joint is worked out
  from where each foot should be, in millimetres, using the leg maths and
  tripod gait of Jakob Leander's hexapod (same chassis; MIT, see ik_gait.h).
  The recorded XML gaits it used to play are gone. Each frame goes to the
  RTrobot controller as one "#<ch>P<us>...T<ms>" line; servo trims, channels
  and directions are set on the /calib page and kept in flash.

  Everything around the walking is as it was: WiFi and the browser control
  page, IR remote, obstacle avoidance, MPU6050 tilt safety, the fleet (RIFT,
  ESP-NOW / BLE beacons, radar) and the control PIN.

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
    4x4 RGB grid (WS2812)      : DIN -> GPIO18, 5V, GND (leds.h)
    Buzzer (passive)           : + -> GPIO19 (~100 ohm), - -> GND (buzzer.h)
    Build switch               : GPIO33 -> switch -> GND (on: every servo at 1500 us)

  ---------------------------------------------------------------------------
  WIFI
  ---------------------------------------------------------------------------
  The fleet's ladder (see "WiFi" below): at boot WHIP joins the highest fleet
  network he sees above him (RIFT, DREAM, NINA, TINA, the controllers, NORA);
  with none, he hosts "WHIP" (the fleet's password, secrets.h) and the fleet
  list on :5000, so the control page is still reachable directly from a phone
  or laptop. He checks every minute and moves up when he can. Either way, WHIP's own
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
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <pgmspace.h>

#define RAW_BUFFER_LENGTH 100   // NEC needs far less than the default; saves RAM
#define DECODE_NEC              // must precede the IRremote include
#define DECODE_SAMSUNG          // NORA's fleet link (see FLEET LINK below)
#include "avatar.h"           // WHIP's avatar as a JPEG byte array (served at /avatar.jpg)
#include "fleet_near.h"       // ESP-NOW "I'm here" beacons: which robots are nearby, from signal strength
#include "fleet_near_ble.h"   // ...and the same beacon over Bluetooth LE, so she and the Pi robots (KIDAs) hear each other
#include <IRremote.hpp>         // IRremote 4.x -- supports ESP32
#include "ik_gait.h"            // walking by inverse kinematics (Jakob Leander's hexapod, same chassis)
#include "gaits.h"              // tripod / ripple / wave: every leg on its own phase, smooth throughout
#include "leds.h"               // the 4x4 RGB grid on GPIO18: his mode, his feet as he walks, walls, tilt
#include "buzzer.h"             // a buzzer on GPIO19: chirps for walls, tilt, gait and mode changes
#include "talk_bf.h"            // the fleet's Brainfuck phrasebook (generated by RIFT/Fleet/brainfuck_talk.py)

// TALKING: the fleet's Brainfuck phrases out loud (RIFT's conversations: /chirp?u=<n>), one tone per
// symbol in his own lower voice, each symbol on his grid as it's beeped. Never blocks: it runs from
// the same place as the buzzer, and he walks on while he talks. Muted, only the grid talks.
const uint8_t TALK_VOICE_PCT = 80;
char talkSym = 0;   // the symbol he's beeping now (0: quiet)
#include "ps2.h"                // a PS2 pad's receiver on GPIO 25 / 26 / 27 / 32 (3.3 V): drives him directly
#include "lights_page.h"        // /lights: every LED look's colour and effect, colour pickers
#include "keys_page.h"           // /keys: every key and button (generated from scripts/keymap.py)
#include "calib_page.h"         // /calib: trim each servo, check its channel, pick the gait

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
// The fleet's ladder (fleet TODO, section 2):
//   RIFT > DREAM > NINA > TINA                                  the hub, the virtual bots
//   > NORACTL > WHIPCTL > KIDA01CTL > KIDA00CTL > ARMCTL        the PC controllers, in their bots' order
//   > NORA > WHIP > KIDA01 > KIDA00 > ARM > MILA                the real bots
// WHIP joins the highest network he sees above him; with none he hosts "WHIP" (192.168.4.1, channel 1)
// and the fleet list on :5000, so KIDA01, KIDA00, ARM and MILA find each other through him. Every
// LADDER_CHECK_MS he looks again: still connected? Does the host's fleet list still take his /register?
// Is a higher one back? Starting over (climb, else host) is a restart, only while he stands.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "No secrets.h: building with secrets.example.h's placeholders (copy it to secrets.h and fill it in)"
#include "secrets.example.h"
#endif
const char* const ABOVE_WHIP[] = { FLEET_SSID, "DREAM", "NINA", "TINA",
                                   "NORACTL", "WHIPCTL", "KIDA01CTL", "KIDA00CTL", "ARMCTL", "NORA" };   // highest first
const int ABOVE_N = sizeof(ABOVE_WHIP) / sizeof(ABOVE_WHIP[0]);
const char* AP_SSID       = "WHIP";
const char* AP_PASSWORD   = NORA_PASSWORD;   // the fleet's fallback password, like every robot's own AP
const unsigned long WIFI_JOIN_MS    = 15000;   // per network, at boot
const unsigned long LADDER_CHECK_MS = 60000;   // not on RIFT: look for a higher network this often
const unsigned long NET_LOST_MS     = 60000;   // his network, or its fleet list, silent this long: start over

enum WifiMode { WIFI_JOINED, WIFI_OWN_AP };
WifiMode wifiMode = WIFI_OWN_AP;
int joinedRank = -1;                 // which network in ABOVE_WHIP he's on; -1 = hosting his own
bool higherSeen = false;             // the last scan saw a network above his
unsigned long netGoneSince = 0;      // his network (or its fleet list) silent since; 0 = fine
unsigned long joinFailedMs[ABOVE_N] = { 0 };   // when joining it failed at boot: not "higher" again for a while
const unsigned long JOIN_RETRY_MS = 600000;    // ...10 minutes, so a network he can't join doesn't restart him every minute
bool wifiBooting = false;            // still working through the networks he saw at boot (wifiBootLoop())
bool bootSeen[ABOVE_N] = { false };
int bootTry = -1;                    // the one he's joining now
unsigned long bootTryStart = 0;
WebServer fleetServer(5000);         // the fleet list, while he hosts

// RIFT's assigned port for WHIP's control page (see "RIFT FLEET
// INTEGRATION" above) -- RIFT :5000, DREAM :5001, NORA :5002, KIDA-00
// :5003, KIDA-01 :5004, WHIP :5005, MILA :5010.
WebServer server(5005);

// === RADAR (passive imposter spotter) ===
// WHIP listens to the WiFi around her and flags anything that is not one of
// our own networks as an "imposter". On-demand only (a scan channel-hops and
// would disturb her link / ESP-NOW): she scans when /radar is opened. Passive
// -- she never connects to or touches a network that is not hers.
const char* const FLEET_SSIDS[] = { "RIFT", "DREAM", "NINA", "TINA", "NORACTL", "WHIPCTL", "KIDA01CTL", "KIDA00CTL",
                                    "ARMCTL", "NORA", "WHIP", "KIDA01", "KIDA00", "ARM", "MILA", "NIKA" };
const unsigned long RADAR_MIN_GAP_MS = 10000;
String radarJson = "{\"scanning\":false,\"nets\":[]}";
unsigned long lastRadarScan = 0;
bool radarInFlight = false;
bool radarRequested = false;

// === CONTROL LOCK ===
// A device must unlock with the PIN before it can drive WHIP. Unlocked devices
// are remembered by IP, up to MAX_CONTROLLERS at once (the PC controller, his page
// on a phone, ...): a new one takes the oldest one's place. Looking (page, /status,
// /radar) is always allowed; driving is not. Servo Studio outranks them all: while
// it holds him (/studio), only the studio drives him.
// (radar + lock helper functions are defined lower down, after the type
// declarations, so the auto-generated prototypes land after those types.)
const char* const CONTROL_PIN = "1234";
const uint8_t MAX_CONTROLLERS = 4;
IPAddress controllerIPs[MAX_CONTROLLERS];
uint8_t controllerCount = 0, controllerNext = 0;
// Servo Studio: it checks in on /studio about every second while it's connected. No word
// for STUDIO_TIMEOUT_MS (closed, crashed, out of range) and he's his own again. It's known by
// its session id (sid=, on every request it makes), not its IP: the controller and his page
// are often on the same PC as the studio, and they mustn't drive him while it holds him.
const unsigned long STUDIO_TIMEOUT_MS = 5000;
String studioSid;
IPAddress studioIP(0, 0, 0, 0);
unsigned long studioSeen = 0;
bool studioActive = false;
// The mind host: a PC controller that can run the LLM (mind.py can_think()) unlocks with
// /unlock?pw=&ctl=1 and checks in on every /status?ctl=1. MIND mode needs it there: the
// button (controller or web page) only works while it is, and only it drives /mind.
IPAddress mindHostIP(0, 0, 0, 0);
unsigned long mindHostSeen = 0;
const unsigned long MIND_HOST_TTL_MS = 5000;   // silent this long: gone, and MIND mode ends

// ---------------------------------------------------------------------------
// The fleet list he registers with: whoever hosts the network he joined (its gateway: RIFT, DREAM,
// NORA... 192.168.4.1, or a Windows PC's hotspot at 192.168.137.1). While he hosts, he keeps the
// list himself (setupFleetServer()).
// ---------------------------------------------------------------------------
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
const int CENTRE_PIN = 33;  // build switch to GND (internal pull-up): on, every servo to 1500 us
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
// The fleet's remote (every robot's but NORA's): the digits count up from 0x0, so "3" = 0x2
// (as MILA, NIKA and IDA use it) and "4"-"7" = 0x3-0x6. The debug console prints each code ("IR cmd=0x..")
#define IR_GAIT             0x2   // "3" -> next gait
#define IR_HELLO            0x3   // "4" -> wave hello
#define IR_DANCE            0x4   // "5" -> a little dance
#define IR_FOLLOW           0x5   // "6" -> follow-me mode
#define IR_REST             0x6   // "7" -> lie down / get up

const unsigned long IR_CMD_TIMEOUT_MS = 350;
unsigned long irTimeoutMs = IR_CMD_TIMEOUT_MS;   // release timeout for the current drive (remote or link)

// FLEET LINK (commands relayed by NORA). NORA's web page, Python controller
// and Bluetooth link can drive WHIP through NORA's IR transmitter: Samsung-
// format frames at WHIP_LINK_ADDRESS, with the command codes the whole fleet
// link uses (IDA 0x0DA1, MILA 0x0DA2, KIDA-01 0x0DA4). Drive commands repeat
// every 150 ms while held; each is run as the matching remote button.
#define WHIP_LINK_ADDRESS 0x0DA3
#define LINK_FORWARD   0x48
#define LINK_BACKWARD  0x49
#define LINK_LEFT      0x4A
#define LINK_RIGHT     0x4B
#define LINK_STOP      0x4C
#define LINK_OBSTACLE  0x4D
#define LINK_MANUAL    0x4E   // IR remote control mode
#define LINK_SPEED     0x4F   // WHIP has one gait speed: ignored
#define LINK_UNLOCK    0x50   // NORA frees his legs
#define LINK_LOCK      0x51   // NORA locks them: he stands, and walks for nobody until LINK_UNLOCK
bool linkLocked = false;      // unlocked at boot
const unsigned long LINK_HOLD_MS = 600;   // longer than the remote: frames can land mid-step

// ---------------------------------------------------------------------------
// Web D-pad codes -- mirror the IR command shape (an "active command" byte
// plus a last-seen timestamp), just driven by HTTP requests instead of an
// IR repeat code. See runWebMode() / pollWeb().
// ---------------------------------------------------------------------------
#define WEB_FORWARD  'F'
#define WEB_BACKWARD 'B'
#define WEB_LEFT     'L'
#define WEB_RIGHT    'R'
#define WEB_SLEFT    'Q'   // sideways (crab) left / right
#define WEB_SRIGHT   'E'
#define WEB_DRIVE    'D'   // analog: driveFwd / driveLeft / driveTurn (the controller's sticks)

const unsigned long WEB_CMD_TIMEOUT_MS = 500;

// ---------------------------------------------------------------------------
// Timing -- the controller can't start a move until the whole line has
// arrived. At CONTROLLER_BAUD an 18-channel IK line (~135 bytes, 10 bits/byte)
// takes (145 * 10 * 1000) / CONTROLLER_BAUD ms to send: every step waits its
// move time (ik::stepMs(), set by speed on /calib) plus that margin.
//
// CONTROLLER_BAUD must match the RTrobot controller's own configured UART
// baud -- a setting on the controller itself (changed from its PC config
// software), NOT something this sketch can change. If it doesn't match, WHIP
// loses the controller (garbled/no response) and stand-up does nothing;
// nothing gets damaged. RTrobot-family 32ch controllers document
// 4800/9600/19200/38400/57600 (some revisions 115200). WHIP's board is set to
// 115200 (since 2026-10-24, was 9600): a line takes ~12 ms instead of ~150 ms.
// Don't change this unless the board itself was reconfigured to match.
// ---------------------------------------------------------------------------
const unsigned long CONTROLLER_BAUD = 115200;

const uint16_t TIMING_MARGIN_MS  = (uint16_t)((145UL * 10UL * 1000UL) / CONTROLLER_BAUD) + 20;
// Walking sends the next line while the last move runs: each move lasts as long as a
// line takes to send, so the next one arrives just as it ends and the legs never stop.
const uint16_t LINE_MS = (uint16_t)((145UL * 10UL * 1000UL) / CONTROLLER_BAUD) + 5;
const unsigned long STAND_SETTLE_MS = TIMING_MARGIN_MS;   // Stand() itself waits out its move
const unsigned long STAND_HOLD_MS   = 800;    // hold the stand pose after hitting a wall (was 3 s: too long)

// Boot: he stands, holds still for the gyro, and walks off in obstacle avoidance
// WALK_AFTER_BOOT_MS after that (red on the grid meanwhile), whatever the WiFi is doing (it joins in the background)
const unsigned long CONTROLLER_BOOT_MS = 2000;   // the servo controller boots too: no pose before this
const unsigned long BOOT_SETTLE_MS     = 1500;   // stand pose sent -> legs still -> calibrate the MPU
const unsigned long WALK_AFTER_BOOT_MS = 5000;
unsigned long readyAt = 0;   // when setup finished (standing, gyro calibrated): the 5 s count from here

// ---------------------------------------------------------------------------
// Obstacle avoidance tuning
// ---------------------------------------------------------------------------
const long MIN_DISTANCE_CM   = 35;   // was 20: he walked into the closet door
const long CLEAR_DISTANCE_CM = 50;
// WEAVE (/gait?weave=1, off again at every boot): his feet stick out past the sonar's cone (about +-15 degrees) until
// about 46 cm ahead, so a chair leg off to one side wasn't seen. Weaving, he swings his heading
// left and right as he walks (the beam sweeps), and stops further back, where the cone is wider.
const long WEAVE_DISTANCE_CM = 48;
const float   WEAVE_TURN   = 0.5;   // how hard he steers each way (-1..1, shared with walking forward)
const uint8_t WEAVE_CYCLES = 8;     // gait cycles for one left-right swing (about +-8 degrees)
const uint8_t OBSTACLE_DEBOUNCE = 3;
const uint8_t AVOID_LOOK_TURNS  = 1;    // at a wall he looks this many turn cycles left, then right
const uint8_t AVOID_MAX_TURNS   = 12;   // turning the clearer way, still blocked after this many: start over

// Follow me: keep a hand FOLLOW_CM in front of the sonar
const long FOLLOW_CM          = 30;
const long FOLLOW_DEADBAND_CM = 4;    // close enough: stand
const long FOLLOW_LOST_CM     = 80;   // nothing nearer than this: nobody to follow, stand

// Rest: standing still this long (IR, web, MIND, follow) he lies down; any move gets him up.
// He lowers his belly to the floor with his feet where they stand, legs spread out. No folding
// them up under him (gait.xml's "Reset - Shut Down", knees and feet right up, until 2026-10-27):
// the user didn't want the tarantula death curl.
const unsigned long REST_AFTER_MS = 60000;
const float REST_Z        = -75;   // belly on the floor (mm): raise if the feet still carry him
const uint8_t TRIPOD_A = (1 << ik::LF) | (1 << ik::LB) | (1 << ik::RM);
const uint8_t TRIPOD_B = (1 << ik::LM) | (1 << ik::RF) | (1 << ik::RB);

// ---------------------------------------------------------------------------
// Tilt safety tuning (MPU6050)
// ---------------------------------------------------------------------------
// was 10 / 7 / 250: his own footsteps rocked him past it and he stood for nothing
const float TILT_LIMIT_DEG = 20.0;
const float TILT_CLEAR_DEG = 12.0;
const unsigned long TILT_DEBOUNCE_MS = 500;
const float COMPLEMENTARY_ALPHA = 0.98;
// Picked up: the MPU feels more than HELD_G_HIGH or less than HELD_G_LOW for HELD_MS (a lift,
// a drop), or he's upside down. His legs stop until he's been level and still for HELD_DOWN_MS.
const float HELD_G_HIGH = 1.4;
const float HELD_G_LOW  = 0.55;
const unsigned long HELD_MS = 120;
const unsigned long HELD_DOWN_MS = 2000;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum SystemState   { SYS_ACTIVE, SYS_TILT_SAFE, SYS_SHUTDOWN, SYS_HELD };
enum ControlMode   { CTRL_OBSTACLE, CTRL_IR, CTRL_WEB, CTRL_MIND, CTRL_FOLLOW };
enum MovementState { MOVE_STAND, MOVE_WALK, MOVE_BACKWARD, MOVE_TURN_LEFT, MOVE_TURN_RIGHT };

SystemState   sysState    = SYS_ACTIVE;
ControlMode   controlMode = CTRL_OBSTACLE;
ControlMode   controlModeBeforeTilt = CTRL_OBSTACLE;
MovementState currentMove = MOVE_STAND;

bool abortMovement = false;   // cleared at the top of every loop()
ik::Frame ikNow;          // where the feet are (the last frame sent)
bool calibrating = false; // /calib is open: WHIP stands still and takes no drive commands

// Lying down (REST, idle, shutdown): belly on the floor, legs spread out. Every move gets him
// up first (wakeUp()).
bool resting = false;
bool centred = false;     // the build switch is on: every servo at 1500 us, nothing moves him
bool awaiting = false;    // AUTO-START off: booted with every servo at 1500 us, standing up at the first input
bool testGait = false;    // TEST-GAIT on (/testgait, not saved): hips and thighs only, see testCycle()
bool testPosed = false;   // his legs are in TEST-GAIT's joint angles (ikNow just says home meanwhile)
uint16_t testK = 0;       // TEST-GAIT: ticks of the half-cycle under way sent (0: none)
// where he stops for a wall / walks on again (further with WEAVE on)
long stopCm()  { return ik::weave ? WEAVE_DISTANCE_CM : MIN_DISTANCE_CM; }
long clearCm() { return ik::weave ? WEAVE_DISTANCE_CM + 15 : CLEAR_DISTANCE_CM; }
unsigned long stoodSince = 0;             // standing still since (he lies down after REST_AFTER_MS)
bool dancing = false;
// One thing to do between moves (web, keys, IR): h(ello) d(ance) r(est) w(ake) s(tand)
char pendingAct = 0;
// Analog driving (web cmd=drive: the controller's sticks), each -1..1
float driveFwd = 0, driveLeft = 0, driveTurn = 0;
// Body IK (ik_gait.h bodyFrame / fitPose): while he stands, his body slides and tilts over planted
// feet. Asked for on /body (controller, Servo Studio) or by the PS2 pad's BODY mode (R3); fitted so
// every foot reaches (bodyK: how much of the asked-for pose that is). Walking starts from neutral.
ik::BodyPose bodyNow;        // the pose he holds (fitted)
float bodyK = 1;
bool bodyMode = false;       // the PS2 pad's BODY mode: its sticks move the body, not his feet
ik::Frame standFrame() { return ik::bodyFrame(ik::home(), bodyNow); }
float followFwd = 0;   // FOLLOW: towards (+) / away from (-) the hand
long  seenFar = 0;     // the furthest the sonar saw while he looked around (cm)
float gravX = 0, gravY = 0, gravZ = 1;   // which way down was when he stood at boot (picked-up check)

// MIND mode: the controller's LLM (mind.py) drives him one short move at a time
// (/mind?act=fwd&steps=2), and he stands while it thinks. Only his PC controller
// can switch it on or drive it (not his web page); a wall ahead ends a forward move.
const uint8_t MIND_MAX_STEPS = 4;   // gait cycles per /mind command
uint8_t mindAct     = 0;            // WEB_FORWARD ... WEB_RIGHT, 0 = stand
uint8_t mindSteps   = 0;            // cycles left
bool    mindBlocked = false;        // the last forward move stopped at a wall
bool    manualBlocked = false;      // driven by hand (IR / web) and there's a wall: he won't walk on

// Driven by hand, like NORA (MANUAL_STOP_CM), MILA and IDA (guardDist): forward is refused while
// something is within MIN_DISTANCE_CM. One ping per 100 ms at most (HC-SR04 needs ~60 ms between).
long readDistanceCM();
bool wallInWay() {
  static unsigned long lastPing = 0;
  static long d = -1;
  if (millis() - lastPing >= 100) {
    lastPing = millis();
    d = readDistanceCM();
  }
  manualBlocked = d > 0 && d < stopCm();
  return manualBlocked;
}

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
// Tilt safety can be switched off (web TILT button, /tilt?on=0, key T): he then walks on
// whatever the MPU says. On again at every boot.
bool tiltGuard = true;

unsigned long tiltOverSince  = 0;
unsigned long tiltClearSince = 0;

bool gaitWaitUntil(unsigned long start, unsigned long ms, bool scanSafe);
bool gaitWait(unsigned long ms, bool scanSafe);
bool testCycle(const float& fwd, const float& turn, bool scanSafe);
bool testSettle();
void leaveTestPose();
long readDistanceCM();
void pollTilt();
void pollIR();
void pollWeb();
void pollPS2();
void setBody(ik::BodyPose b);
void applyBody();
void runWebMode();
void setupWiFi();
void tryNextBootNetwork();
void wifiBootLoop();
void hostOwnAP();
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
  pinMode(CENTRE_PIN, INPUT_PULLUP);

  leds::begin();
  buzzer::begin();
  Wire.begin(21, 22);
  mpuInit();

  IrReceiver.begin(IRR_PIN, DISABLE_LED_FEEDBACK);

  fleetNearBleBegin("WHIP");   // Bluetooth doesn't wait for the WiFi
  setupWiFi();                 // scans (~2 s) and starts joining; wifiBootLoop() finishes it
  setupWebServer();

  while (millis() < CONTROLLER_BOOT_MS) delay(10);

  ik::load();
  if (ik::gait >= ik::GAITS) ik::gait = ik::GAIT_TRIPOD;   // a gait taken out since (jakob, 6 until 2026-10-28)
  ps2::begin();   // nothing plugged in: it just finds no pad
  ikNow = ik::home();
  Serial2.println(ik::calibrated ? F("IK: calibration loaded") : F("IK: default calibration (fine-tune on /calib)"));
  if (digitalRead(CENTRE_PIN) == LOW) {   // build switch on: straight to the middle, no standing up
    centred = true;
    centreAll();
    Serial2.println(F("Sent: every servo at 1500 us (build switch)"));
  } else if (!ik::autoStart) {   // AUTO-START off: stay at the middle until someone gives him input
    awaiting = true;
    centreAll();         // the board holds 1500 us from power-up anyway; this covers a reset without one
    Serial2.println(F("Sent: every servo at 1500 us (AUTO-START off: waiting for input)"));
  } else {
    standFromCentre();   // the servo board holds them all at 1500 us when it powers up
    Serial2.println(F("Sent: stand pose"));
  }
  currentMove = MOVE_STAND;
  delay(BOOT_SETTLE_MS);

  Serial2.println(F("Calibrating gyro..."));   // standing still
  calibrateGyro();
  if (mpuFound) calibrateTiltOffset();

  if (ik::autoStart) {
    controlMode = CTRL_OBSTACLE;
    Serial2.print(F("CONTROL -> OBSTACLE AVOIDANCE, walking at "));
    Serial2.print(WALK_AFTER_BOOT_MS / 1000);
    Serial2.println(F(" s"));
  } else {
    controlMode = CTRL_WEB;
    Serial2.println(F("CONTROL -> WEB, every servo at 1500 us until the first input"));
  }
  buzzer::play(buzzer::BOOT);
  readyAt = millis();
}

void loop() {
  // what she tells the other robots: walking by herself, driven by you, or standing
  fleetNearSetState(controlMode == CTRL_OBSTACLE ? FLEET_DRIVING : currentMove == MOVE_STAND ? FLEET_PARKED : FLEET_USER);
  fleetNearLoop();
  fleetNearBleLoop();
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
  radarLoop();
  wifiBootLoop();
  pollFleetRegistration();
  wifiLadderLoop();

  if (sysState == SYS_SHUTDOWN) {
    buzzer::playNow(buzzer::SHUTDOWN);
    ShutdownSequence();
    outputsTick(true);   // grid off
    while (true) { delay(1000); }
  }
  outputsTick(false);

  if (centreSwitchLoop()) return;   // the build switch holds every servo at 1500 us
  if (studioLoop()) return;         // Servo Studio holds him: it moves the servos, nothing else
  if (testPosed && !testGait && currentMove == MOVE_STAND) leaveTestPose();   // TEST-GAIT switched off
  pollTilt();
  levelTick();
  if (sysState == SYS_TILT_SAFE || sysState == SYS_HELD) return;

  pollIR();
  pollWeb();
  pollPS2();
  if (awaiting && !awaitLoop()) return;   // AUTO-START off: at 1500 us until the first input

  if (calibrating) return;   // /calib moves the servos itself; WHIP doesn't walk meanwhile
  if (linkLocked) {          // NORA locked his legs: he crouches, sulking
    requestMove(MOVE_STAND);
    if (!resting) ikGlide(crouchPose(), 15, 80);
    return;
  }
  if (pendingAct) {          // hello, dance, rest, wake: once, between moves
    const char act = pendingAct;
    pendingAct = 0;
    runAct(act);
    stoodSince = millis();
    return;
  }
  if (controlMode == CTRL_MIND && !mindHostAlive()) {    // the mind's controller went quiet
    controlMode = CTRL_WEB;
    webActiveCmd = 0;
    mindSteps = 0;
    Serial2.println(F("MIND: controller gone -> WEB (standing)"));
  }
  if (millis() - readyAt < WALK_AFTER_BOOT_MS) return;            // just booted: standing until it's time

  if (controlMode == CTRL_OBSTACLE) {
    runObstacleMode();
  } else if (controlMode == CTRL_IR) {
    runIRMode();
  } else if (controlMode == CTRL_MIND) {
    runMindMode();
  } else if (controlMode == CTRL_FOLLOW) {
    runFollowMode();
  } else {
    runWebMode();
  }

  // Standing still a while (driven by hand, MIND or follow, nothing to do): lie down
  if (currentMove != MOVE_STAND || controlMode == CTRL_OBSTACLE || resting) stoodSince = millis();
  else if (millis() - stoodSince > REST_AFTER_MS) restDown();
}

// Parses "#<ch>P<val>" tokens and clamps each value to the channel's limits
// (MECHANICAL SAFETY LIMITS), so a bad trim or a foot asked for out of range
// can't drive a servo into its stop. Reports anything it had to correct.
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

// ---------------------------------------------------------------------------
// MOVEMENT -- inverse kinematics (ik_gait.h). Each Move*() plays one gait cycle.
// ---------------------------------------------------------------------------

// One frame to the controller, moving over `ms`. False if a foot is out of reach.
bool ikSend(const ik::Frame& f, uint16_t ms) { return ikSendLead(f, ms, nullptr, 0); }

bool ikSendLead(const ik::Frame& f, uint16_t ms, const ik::Frame* lead, uint8_t leadMask) {
  if (centred) return false;   // the build switch holds every servo at 1500 us
  if (awaiting) wakeFromCentre();   // AUTO-START off and this is the first move: up from 1500 us first, gently
  if (testPosed) leaveTestPose();   // out of TEST-GAIT's pose into the stand first, smoothly
  if (!ik::line(f, cmdBuf, sizeof(cmdBuf), lead, leadMask)) {
    Serial2.println(F("IK: foot out of reach, frame skipped"));
    return false;
  }
  clampCommand(cmdBuf, safeBuf, sizeof(safeBuf));
  Serial.print(safeBuf);
  Serial.print('T');
  Serial.print(ms);
  Serial.print("\r\n");
  ikNow = f;
  return true;
}

// Waits like gaitWaitUntil, but a new drive command doesn't cut it short: only
// tilt safety or being picked up does. For standing up, so feet always land first
// (and for lying down at shutdown).
bool finishWaitUntil(unsigned long start, unsigned long ms) {
  while (millis() - start < ms) {
    while (Serial.available()) Serial2.write(Serial.read());
    outputsTick(false);
    server.handleClient();
    pollTilt();
    pollIR();
    pollWeb();
    pollPS2();
    if (sysState == SYS_TILT_SAFE || sysState == SYS_HELD) return false;
  }
  return true;
}

// From where the feet are to `target`, in ik::STEPS small moves (the reference's
// interpolation). Each step occupies stepMs + the line's transmit time.
// finish: don't stop for a new command (standing up between gaits).
bool ikMoveTo(const ik::Frame& target, bool scanSafe, bool finish) {
  if (resting && !wakeUp()) return false;
  const ik::Frame from = ikNow;
  const uint16_t ms = ik::stepMs();
  for (uint8_t i = 1; i <= ik::STEPS; i++) {
    unsigned long frameStart = millis();
    if (!ikSend(ik::lerp(from, target, ik::ease((float)i / ik::STEPS)), ms)) return false;
    // feet all down at the end of this key frame: safe to range-find
    bool waited = finish ? finishWaitUntil(frameStart, ms + TIMING_MARGIN_MS)
                         : gaitWaitUntil(frameStart, ms + TIMING_MARGIN_MS, scanSafe && i == ik::STEPS);
    if (!waited) return false;
  }
  return true;
}

ik::Gait gaitNow;          // the gait engine: where every foot is in its cycle
// The next walking frame: with FOOT LEAD, the stepping feet's servos run ahead (gaits.h)
bool ikSendGait(const ik::Frame& f, uint16_t ms) { return ikSendLead(f, ms, &gaitNow.lead, gaitNow.leadMask); }
bool gaitSettle();
bool watchClear = false;  // turning away from a wall: count clear pings while he turns
uint8_t clearPings = 0;

// The LED grid and the buzzer: what he's doing now (leds.h, buzzer.h). Called from
// loop() and every wait. The buzzer chirps when something changes.
void outputsTick(bool force) {
  static bool started = false, wasWall = false, wasTiltStop = false, wasTiltGuard = true, wasLocked = false, wasHeld = false;
  static uint8_t wasMode = 0, wasGait = 0;
  static unsigned long lastAlarm = 0;
  const bool wall = obstacleDetected || manualBlocked || mindBlocked || obstacleCount >= OBSTACLE_DEBOUNCE;
  const bool tiltStop = sysState == SYS_TILT_SAFE;
  if (started) {
    if (tiltStop && (!wasTiltStop || (!buzzer::busy() && millis() - lastAlarm > 1500))) {
      buzzer::play(buzzer::TILT);   // and again every 1.5 s while he's held
      lastAlarm = millis();
    }
    else if (sysState == SYS_HELD && !wasHeld) buzzer::play(buzzer::HELD);
    else if (linkLocked != wasLocked)          buzzer::play(linkLocked ? buzzer::LOCK : buzzer::ON);
    else if (tiltGuard != wasTiltGuard)        buzzer::play(tiltGuard ? buzzer::ON : buzzer::OFF);
    else if (ik::gait != wasGait)              buzzer::play(buzzer::GAIT_BEEPS[ik::gait % 6]);
    else if ((uint8_t)controlMode != wasMode)  buzzer::play(buzzer::MODE);
    else if (wall && !wasWall)                 buzzer::play(buzzer::WALL);
  }
  started = true;
  wasWall = wall; wasTiltStop = tiltStop; wasTiltGuard = tiltGuard; wasLocked = linkLocked; wasHeld = sysState == SYS_HELD;
  wasMode = (uint8_t)controlMode; wasGait = ik::gait;
  buzzer::update();
  talkTick();

  leds::State st;
  if (sysState == SYS_SHUTDOWN)       st.look = leds::SHUTDOWN;
  else if (sysState == SYS_TILT_SAFE) st.look = leds::TILT_STOP;
  else if (sysState == SYS_HELD)      st.look = leds::HELD;
  else if (linkLocked)                st.look = leds::LOCKED;
  else if (bodyMode)                  st.look = leds::BODY;
  else if (controlMode == CTRL_OBSTACLE && readyAt && millis() - readyAt < WALK_AFTER_BOOT_MS) st.look = leds::STARTING;   // red, 5 s
  else if (controlMode == CTRL_IR)    st.look = leds::IR;
  else if (controlMode == CTRL_WEB)   st.look = leds::WEB;
  else if (controlMode == CTRL_MIND)  st.look = leds::MIND;
  else if (controlMode == CTRL_FOLLOW) st.look = leds::FOLLOW;
  else                                st.look = leds::OBSTACLE;
  st.walking = gaitNow.running;
  st.gait    = &gaitNow;
  st.gaitId  = ik::gait;
  st.wall    = wall || obstacleCount > 0;
  st.tiltOff = !tiltGuard;
  st.resting = resting;
  st.dancing = dancing;
  st.talkSym = talkSym;
  st.startFrac = constrain((float)(millis() - readyAt) / WALK_AFTER_BOOT_MS, 0.0f, 1.0f);
  leds::update(st, force);
}

// One step of TALKING (top of the file)
void talkTick() {
  if (talkUtterance < 0 || (long)(millis() - talkNextMs) < 0) return;
  talkSym = talkSymbol(talkUtterance, talkPos++);
  if (!talkSym) { talkUtterance = -1; buzzer::tone(0); return; }
  buzzer::tone((uint32_t)bfToneHz(talkSym) * TALK_VOICE_PCT / 100);
  talkNextMs = millis() + bfToneMs(talkSym) + TALK_GAP_MS;
}

// Ticks a foot spends in the air: a fast link has time for every move (smoother),
// a slow one drops moves to keep up
uint16_t airTicks() { return 2 * (LINE_MS > ik::stepMs() ? ik::walkSteps() : ik::STEPS); }

// One cycle of the chosen gait (ik::gait): every leg steps once. fwd / left / turn
// (anticlockwise) -1..1. Back to back the cycles run on without a pause; the engine
// starts from wherever the feet are if something else moved them meanwhile.
// The steering is read on every move (references), so analog driving and follow-me change
// it mid-cycle.
bool gaitCycle(const float& fwd, const float& left, const float& turn, bool scanSafe) {
  if (resting && !wakeUp()) return false;
  if (testGait) return testCycle(fwd, turn, scanSafe);
  // a new gait picked mid-walk: step to a stop in the old one, start off in the new one
  // (ANTI-SLIP switched mid-walk too: the cycle is a different length)
  if (gaitNow.running && (gaitNow.id != ik::gait || gaitNow.plant != ik::antiSlip) && !gaitSettle()) return false;
  if (!gaitNow.running || ik::maxOffset(gaitNow.last, ikNow) > 0.01f) {
    // crouched, just put down, after a hello: up to the stand pose first, smoothly
    if (ik::maxOffset(ikNow, ik::home()) > 5 && !ikMoveTo(ik::home(), false, true)) return false;
    bodyNow = ik::BodyPose();   // walking: no body pose (he stands back into it from neutral)
    gaitNow.start(ikNow, ik::gait, airTicks());
  }
  const uint16_t ms = max(ik::stepMs(), LINE_MS);
  for (uint16_t i = 0; i < gaitNow.cycleTicks; i++) {
    unsigned long frameStart = millis();
    if (!ikSendGait(gaitNow.tick(fwd, left, turn, false), ms)) { gaitNow.running = false; return false; }
    // range-find on every move (gaitWaitUntil pings every 100 ms)
    if (!gaitWaitUntil(frameStart, ms, scanSafe)) return false;
  }
  return true;
}

// Bring the gait to a stop by stepping: he slows down over one swing, then the feet
// walk home in two tripods. Only tilt safety cuts it short. False if it didn't finish.
bool gaitSettle() {
  if (testPosed) return testSettle();
  if (!gaitNow.running || ik::maxOffset(gaitNow.last, ikNow) > 0.01f) { gaitNow.running = false; return true; }
  const uint16_t ms = max(ik::stepMs(), LINE_MS);
  for (uint16_t i = 0; i < 4 * gaitNow.cycleTicks && !gaitNow.settled(); i++) {
    unsigned long frameStart = millis();
    if (!ikSendGait(gaitNow.tick(0, 0, 0, true), ms)) break;
    if (!finishWaitUntil(frameStart, ms)) { gaitNow.running = false; return false; }
  }
  gaitNow.running = false;
  return true;
}

// Stand at once (tilt safety, boot): one move, no interpolation
void ikSnapHome() { gaitNow.running = false; ikSend(ik::home(), 300); }

// The stand pose right now (tilt safety, boot): no interpolation. Not while he lies
// down: the snap would slam his body up off the floor.
void standNow() { if (!resting && !awaiting) ikSnapHome(); }

// Every change of gait passes through here, so all six feet are down before the
// next one starts. Walking: the feet step home; anything else: a smooth move.
void Stand() {
  if (gaitSettle() && ik::maxOffset(ikNow, standFrame()) > ik::HOME_MM) ikMoveTo(standFrame(), false, true);
  currentMove = MOVE_STAND;
  Serial2.println(F("MOVE -> STAND"));
}

// The build switch (CENTRE_PIN to GND): every servo to its plain middle, the servo type's
// centre (1500 us) with no trim and no direction, so a horn goes on straight while he's built.
// He holds it while the switch is on, nothing drives him, and when it goes off he stands up
// in WEB mode (not walking off by himself).
bool centreSwitchOn() {
  static bool last = false;
  static unsigned long since = 0;
  const bool now = digitalRead(CENTRE_PIN) == LOW;
  if (now != last) { last = now; since = millis(); }
  return millis() - since >= 30 ? now : centred;   // 30 ms to settle: a switch bounces
}

void centreAll() {
  size_t used = 0;
  cmdBuf[0] = '\0';
  for (uint8_t l = 0; l < ik::LEGS; l++)
    for (uint8_t j = 0; j < ik::JOINTS; j++)
      used += snprintf(cmdBuf + used, sizeof(cmdBuf) - used, "#%uP%d", ik::cal[l][j].ch, servocfg::JOINT[l][j].usCenter);
  clampCommand(cmdBuf, safeBuf, sizeof(safeBuf));
  Serial.print(safeBuf); Serial.print("T800\r\n");
}

// From every servo at its middle (the build switch, the servo board's own pulse at power-up) up
// into the stand pose, slowly and eased in joint angles. One move there in 300-800 ms had all 18
// servos pull at full speed at once: he sagged, then stood up, then walked off.
const uint8_t  CENTRE_GLIDE_MOVES = 12;        // 12 x 160 ms: about 2 s
const uint16_t CENTRE_GLIDE_MS    = LINE_MS + 4;
void standFromCentre() {
  float to[ik::LEGS][ik::JOINTS], a[ik::LEGS][ik::JOINTS];
  if (!ik::anglesOf(ik::home(), to)) { standNow(); return; }
  gaitNow.running = false; resting = false;
  for (uint8_t i = 1; i <= CENTRE_GLIDE_MOVES; i++) {
    const unsigned long start = millis();
    const float t = ik::ease((float)i / CENTRE_GLIDE_MOVES);
    for (uint8_t l = 0; l < ik::LEGS; l++)
      for (uint8_t j = 0; j < ik::JOINTS; j++) a[l][j] = to[l][j] * t;   // from 0 degrees
    anglesSend(a, CENTRE_GLIDE_MS);
    while (millis() - start < CENTRE_GLIDE_MS) { server.handleClient(); buzzer::update(); }
    if (digitalRead(CENTRE_PIN) == LOW) return;   // the switch went back on: centre again
  }
  ikNow = ik::home();
}

// true while the switch holds him at the middle (loop() does nothing else then)
bool centreSwitchLoop() {
  const bool on = centreSwitchOn();
  if (on && !centred) {
    centred = true;
    abortMovement = true; webActiveCmd = 0; gaitNow.running = false;
    resting = false; currentMove = MOVE_STAND; testPosed = false; testK = 0;
    centreAll();
    buzzer::play(buzzer::LOCK);
    Serial2.println(F("BUILD SWITCH ON: every servo at 1500 us"));
  } else if (!on && centred) {
    centred = false;
    awaiting = false;
    controlMode = CTRL_WEB; webActiveCmd = 0;
    buzzer::play(buzzer::WAKE);
    standFromCentre();
    stoodSince = millis();
    Serial2.println(F("BUILD SWITCH OFF: standing (web mode)"));
  }
  return centred;
}

// AUTO-START off: he booted at 1500 us. The first input (a drive key, a mode, IR, the pad, a body
// move...) stands him up from there, then does what it asked.
void wakeFromCentre() {
  if (!awaiting) return;
  awaiting = false;
  Serial2.println(F("INPUT: standing up from 1500 us"));
  buzzer::play(buzzer::WAKE);
  standFromCentre();
  currentMove = MOVE_STAND;
  stoodSince = millis();
  readyAt = millis() - WALK_AFTER_BOOT_MS;   // no boot wait on top: he's been ready all along
}

// While he waits: true once there's input (and he has stood up), false to keep waiting.
bool awaitLoop() {
  const bool input = controlMode != CTRL_WEB || webActiveCmd != 0 || irActiveCmd != 0 || pendingAct != 0 ||
                     driveFwd != 0 || driveLeft != 0 || driveTurn != 0 || testGait;
  if (!input) { stoodSince = millis(); return false; }
  wakeFromCentre();
  return true;
}

bool MoveForward()   { return gaitCycle(1, 0, 0, true); }
// Forward, steering a little left and right in turn (WEAVE): one swing every WEAVE_CYCLES cycles
bool WeaveForward() {
  static uint8_t n = 0;
  const float turn = WEAVE_TURN * sinf(2 * M_PI * n / WEAVE_CYCLES);
  n = (n + 1) % WEAVE_CYCLES;
  return gaitCycle(1, 0, turn, true);
}
bool StrafeLeft()    { return gaitCycle(0, 1, 0, false); }
bool StrafeRight()   { return gaitCycle(0, -1, 0, false); }

// ---------------------------------------------------------------------------
// Body moves: rest, wave hello, dance, crouch
// ---------------------------------------------------------------------------
// A slow, smooth move of every foot to `to` (body moves), n moves of ms each.
// Only tilt safety or being picked up cuts it short.
bool ikGlide(const ik::Frame& to, uint8_t n, uint16_t ms) {
  if (resting && !wakeUp()) return false;
  gaitNow.running = false;
  const ik::Frame from = ikNow;
  if (ik::maxOffset(from, to) < 0.5f) return true;
  for (uint8_t i = 1; i <= n; i++) {
    unsigned long start = millis();
    if (!ikSend(ik::lerp(from, to, ik::ease((float)i / n)), ms)) return false;
    if (!finishWaitUntil(start, ms)) return false;
  }
  return true;
}

ik::Frame restLow() {       // standing pose, belly on the floor
  ik::Frame f = ik::home();
  for (uint8_t l = 0; l < ik::LEGS; l++) f.leg[l].z = REST_Z;
  return f;
}

ik::Frame crouchPose() {    // his lowest stance
  ik::Frame f = ik::home();
  for (uint8_t l = 0; l < ik::LEGS; l++) f.leg[l].z = ik::HOME_Z_MIN;
  return f;
}

void anglesSend(const float a[ik::LEGS][ik::JOINTS], uint16_t ms) {
  if (centred) return;
  ik::anglesLine(a, cmdBuf, sizeof(cmdBuf));
  clampCommand(cmdBuf, safeBuf, sizeof(safeBuf));
  Serial.print(safeBuf);
  Serial.print('T');
  Serial.print(ms);
  Serial.print("\r\n");
}

// Lie down: step to a stop, then lower the belly to the floor on all six legs, feet
// where they stand, so nothing drops and no leg curls up.
bool restDown() {
  if (resting) return true;
  Serial2.println(F("REST: lying down"));
  buzzer::play(buzzer::REST);
  Stand();
  if (!ikGlide(restLow(), 20, 100)) return false;
  resting = true;   // from here every move gets him up first
  return true;
}

// Get up: lift the body off the floor back into the stand.
bool wakeUp() {
  if (!resting) return true;
  Serial2.println(F("REST: getting up"));
  buzzer::play(buzzer::WAKE);
  resting = false;
  ikNow = restLow();
  gaitNow.running = false;
  currentMove = MOVE_STAND;
  return ikGlide(ik::home(), 20, 100);
}

// ---------------------------------------------------------------------------
// TEST-GAIT: a tripod walk on the hips and thighs alone, no IK. On the ground every thigh and
// lower leg is at 0 degrees (1500 us + trim, his build pose) and only the hips sweep; a stepping
// leg lifts on its thigh, the lower leg stays at 1500 throughout. Turns too; no crabbing (the
// hips can't). /testgait?on=1 swaps it in for whatever gait is picked, until on=0 or a reboot.
// ---------------------------------------------------------------------------
const float TEST_HIP_DEG  = 15;   // each hip sweeps this far either side of straight out (~75 mm a step)
const float TEST_LIFT_DEG = 30;   // a stepping thigh lifts this far at the top of the step
float   testHip[ik::LEGS];        // where each hip is now (degrees, the IK's direction)
float   testFrom[ik::LEGS], testTo[ik::LEGS];
uint8_t testSwing = TRIPOD_B;     // the tripod stepping in this half (the first half steps A)

void testSendTick(uint16_t k, uint16_t n, uint16_t ms) {
  float a[ik::LEGS][ik::JOINTS];
  const float t = ik::ease((float)k / n);
  for (uint8_t l = 0; l < ik::LEGS; l++) {
    testHip[l] = testFrom[l] + (testTo[l] - testFrom[l]) * t;
    a[l][ik::HIP]  = testHip[l];
    a[l][ik::KNEE] = (testSwing >> l & 1) ? TEST_LIFT_DEG * sinf(M_PI * k / n) : 0;
    a[l][ik::FOOT] = 0;
  }
  anglesSend(a, ms);
}

// The next half: the other tripod steps to where it will push from, the feet down slide back.
// fwd / turn -1..1 (turn anticlockwise); stop: everything to straight out.
void testHalfStart(float fwd, float turn, bool stop) {
  const float n = fabsf(fwd) + fabsf(turn);
  if (n > 1) { fwd /= n; turn /= n; }
  testSwing = testSwing == TRIPOD_A ? TRIPOD_B : TRIPOD_A;
  for (uint8_t l = 0; l < ik::LEGS; l++) {
    // a left foot forward is a clockwise hip (minus), a right one anticlockwise
    const float side = l <= ik::LB ? -1 : 1;
    const float reach = stop ? 0 : TEST_HIP_DEG * (fwd * side + turn);
    testFrom[l] = testHip[l];
    testTo[l] = (testSwing >> l & 1) ? reach : -reach;
  }
  testK = 0;
}

// From the stand (or wherever the IK left him) into the build pose: every joint at 0 degrees
void enterTestPose() {
  float from[ik::LEGS][ik::JOINTS], a[ik::LEGS][ik::JOINTS];
  if (!ik::anglesOf(ikNow, from)) memset(from, 0, sizeof(from));
  for (uint8_t i = 1; i <= CENTRE_GLIDE_MOVES; i++) {
    const unsigned long start = millis();
    const float t = 1 - ik::ease((float)i / CENTRE_GLIDE_MOVES);
    for (uint8_t l = 0; l < ik::LEGS; l++)
      for (uint8_t j = 0; j < ik::JOINTS; j++) a[l][j] = from[l][j] * t;
    anglesSend(a, CENTRE_GLIDE_MS);
    finishWaitUntil(start, CENTRE_GLIDE_MS);
  }
  for (uint8_t l = 0; l < ik::LEGS; l++) testHip[l] = 0;
  testSwing = TRIPOD_B; testK = 0;
  gaitNow.running = false;
  bodyNow = ik::BodyPose();
  ikNow = ik::home();
  testPosed = true;
  Serial2.println(F("TEST-GAIT: in the build pose"));
}

// One cycle: both tripods step once. A new command doesn't lose the step: the half under way
// carries on from where it was on the next call.
bool testCycle(const float& fwd, const float& turn, bool scanSafe) {
  if (!testPosed) enterTestPose();
  const uint16_t n = airTicks(), ms = max(ik::stepMs(), LINE_MS);
  for (uint8_t half = 0; half < 2; half++) {
    if (testK == 0) testHalfStart(fwd, turn, false);
    while (testK < n) {
      const unsigned long frameStart = millis();
      testSendTick(++testK, n, ms);
      if (!gaitWaitUntil(frameStart, ms, scanSafe)) return false;
    }
    testK = 0;
  }
  return true;
}

// Stop: finish the step under way, then one more half with every hip going straight out.
// He stays in the build pose (every joint at 1500).
bool testSettle() {
  if (!testPosed) return true;
  const uint16_t n = airTicks(), ms = max(ik::stepMs(), LINE_MS);
  auto finishHalf = [&]() {
    while (testK < n) {
      const unsigned long frameStart = millis();
      testSendTick(++testK, n, ms);
      if (!finishWaitUntil(frameStart, ms)) return false;
    }
    testK = 0;
    return true;
  };
  if (testK > 0 && !finishHalf()) return false;
  bool still = true;
  for (uint8_t l = 0; l < ik::LEGS; l++) still &= fabsf(testHip[l]) < 0.5f;
  if (still) return true;
  testHalfStart(0, 0, true);
  return finishHalf();
}

// Back to the IK: stop, then glide from the build pose into the stand
void leaveTestPose() {
  if (!testPosed) return;
  testSettle();
  testPosed = false;   // before standFromCentre: it sends angles, never ikSend
  standFromCentre();
  Serial2.println(F("TEST-GAIT: back in the stand"));
}

// Wave hello: weight back and off the front left leg, lift it high and wave, put it down.
bool waveHello() {
  Serial2.println(F("HELLO"));
  Stand();
  ik::Frame base = ik::home();
  for (uint8_t l = 0; l < ik::LEGS; l++) { base.leg[l].x += 20; base.leg[l].y += 15; }   // body back and right
  if (!ikGlide(base, 10, 80)) return false;
  ik::Frame up = base;
  up.leg[ik::LF] = { base.leg[ik::LF].x + 30, base.leg[ik::LF].y + 20, ik::homeZ() + 120 };
  buzzer::play(buzzer::HELLO);
  if (!ikGlide(up, 10, 60)) return false;
  for (uint8_t k = 0; k < 3; k++) {
    ik::Frame a = up, b = up;
    a.leg[ik::LF].y += 35;
    b.leg[ik::LF].y -= 35;
    if (!ikGlide(a, 5, 50) || !ikGlide(b, 7, 50)) return false;
  }
  return ikGlide(up, 4, 50) && ikGlide(base, 10, 60) && ikGlide(ik::home(), 10, 80);
}

// A little dance: the body circles over its feet twice, bobbing, the grid in colours.
bool dance() {
  Serial2.println(F("DANCE"));
  Stand();
  dancing = true;
  buzzer::play(buzzer::DANCE);
  const ik::Frame h = ik::home();
  const float R = 25, BOB = 15;
  const uint8_t N = 48;
  bool ok = true;
  for (uint8_t i = 0; i <= N && ok; i++) {
    const float a = 4 * PI * i / N;   // two turns
    const float k = i < 6 ? ik::ease(i / 6.0f) : i > N - 6 ? ik::ease((N - i) / 6.0f) : 1;   // ease in and out
    ik::Frame f = h;
    for (uint8_t l = 0; l < ik::LEGS; l++) {   // the feet stay put: the body moves over them
      f.leg[l].x -= k * R * cosf(a);
      f.leg[l].y -= k * R * sinf(a);
      f.leg[l].z += k * BOB * (1 + sinf(2 * a)) / 2;   // bob down only: at full height the legs are already stretched
    }
    unsigned long start = millis();
    ok = ikSend(f, 70) && finishWaitUntil(start, 70);
  }
  dancing = false;
  return ok && ikGlide(h, 6, 60);
}

void runAct(char a) {
  if      (a == 'r') {   // lying down means staying down: out of walking by himself, into WEB mode
    if (controlMode == CTRL_OBSTACLE) { controlMode = CTRL_WEB; webActiveCmd = 0; Serial2.println(F("CONTROL -> WEB (rest)")); }
    restDown();
  }
  else if (a == 'w') wakeUp();
  else if (a == 'h') waveHello();
  else if (a == 'd') dance();
  else if (a == 's') Stand();
}
bool MoveBackward()  { return gaitCycle(-1, 0, 0, false); }
bool TurnLeft()      { return gaitCycle(0, 0, 1, false); }
bool TurnRight()     { return gaitCycle(0, 0, -1, false); }

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
// before transmitting, every step takes the same time however long its line is. Returns false when the move should be dropped.
bool gaitWaitUntil(unsigned long start, unsigned long ms, bool scanSafe) {
  static unsigned long lastPing = 0;

  while (millis() - start < ms) {
    while (Serial.available()) Serial2.write(Serial.read());
    outputsTick(false);

    server.handleClient();   // keep the control page responsive mid-gait

    pollTilt();
    pollIR();
    pollWeb();
    pollPS2();

    // Turning away from a wall: is the way ahead clear yet? (counted while he turns,
    // so he doesn't have to stop and look)
    if (watchClear && millis() - lastPing >= 100) {
      lastPing = millis();
      long d = readDistanceCM();
      clearPings = (d < 0 || d >= clearCm()) ? min(clearPings + 1, 255) : 0;
      seenFar = max(seenFar, d < 0 ? 400L : d);   // no echo: nothing in range
    }

    // FOLLOW: steer by the hand's distance as he walks
    if (controlMode == CTRL_FOLLOW && millis() - lastPing >= 100) {
      lastPing = millis();
      followFwd = followSpeed(readDistanceCM());
    }

    // While walking, watch for a wall
    if (scanSafe && currentMove == MOVE_WALK   // every mode: obstacle, mind, and driven by hand

        && millis() - lastPing >= 100) {
      lastPing = millis();
      long distance = readDistanceCM();

      if (distance > 0 && distance < stopCm()) {
        obstacleCount++;
      } else if (distance < 0 || distance >= clearCm()) {
        obstacleCount = 0;
      }

      if (obstacleCount >= OBSTACLE_DEBOUNCE) {
        obstacleCount    = 0;
        if (controlMode == CTRL_MIND) { mindSteps = 0; mindBlocked = true; }
        else if (controlMode == CTRL_OBSTACLE) obstacleDetected = true;
        else manualBlocked = true;   // IR / web: stand; wallInWay() keeps him from walking on
        abortMovement    = true;   // stop mid-cycle, go stand
      }
    }

    if (controlMode == CTRL_IR  && irActiveCmd  != irCmdAtMoveStart)  return false;
    if (controlMode == CTRL_WEB && webActiveCmd != webCmdAtMoveStart) return false;
    if (abortMovement || linkLocked || sysState != SYS_ACTIVE || centreSwitchOn()) return false;
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
  if (irActiveCmd != 0 && millis() - irLastCmdMillis > irTimeoutMs) {
    irActiveCmd = 0;
  }

  irCmdAtMoveStart = irActiveCmd;

  if (irActiveCmd == IR_FORWARD) {
    if (wallInWay()) requestMove(MOVE_STAND);   // driven by hand: no walking into a wall
    else if (requestMove(MOVE_WALK)) MoveForward();
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
const float NO_FWD = 0;

void runWebMode() {
  webCmdAtMoveStart = webActiveCmd;

  if (webActiveCmd == WEB_DRIVE) {
    // a wall ahead: he can still turn, crab and back off, just not walk on
    const bool blocked = driveFwd > 0.05f && wallInWay();
    if (requestMove(MOVE_WALK)) {
      if (blocked) gaitCycle(NO_FWD, driveLeft, driveTurn, false);
      else gaitCycle(driveFwd, driveLeft, driveTurn, driveFwd > 0.2f);
    }
    return;
  }
  if (webActiveCmd == WEB_SLEFT || webActiveCmd == WEB_SRIGHT) {
    if (requestMove(MOVE_WALK)) webActiveCmd == WEB_SLEFT ? StrafeLeft() : StrafeRight();
    return;
  }

  if (webActiveCmd == WEB_FORWARD) {
    if (wallInWay()) requestMove(MOVE_STAND);   // driven by hand: no walking into a wall
    else if (requestMove(MOVE_WALK)) MoveForward();
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
    applyBody();               // ...holding the body pose, which may have changed
  }
}

// Ask for a body pose: fitted to what his feet can reach, then he moves to it (standing, WEB mode)
void setBody(ik::BodyPose b) {
  bodyNow = ik::fitPose(ik::home(), ik::clampPose(b), &bodyK);
}

// Standing: move to the body pose if it changed (a short, smooth move)
void applyBody() {
  if (resting || currentMove != MOVE_STAND || gaitNow.running) return;   // lying down: stays down till you drive him
  const ik::Frame f = standFrame();
  if (ik::maxOffset(ikNow, f) > 0.5f) ikMoveTo(f, false, true);
}

// LEVEL (/gait?level=1, off at every boot): standing on a slope he tilts his body against it, a little
// at a time, feet planted, as far as BODY_TILT_MAX. The signs are worked out from the maths, not tried
// on him yet: if the slope reading gets worse instead of better he gives up, goes neutral and says so.
const unsigned long LEVEL_EVERY_MS = 100;
const float LEVEL_GAIN        = 0.08f;   // of the error, each time: slow, so it can't oscillate
const float LEVEL_DEADBAND    = 1.5f;    // degrees: near enough
const float LEVEL_WORSE_DEG   = 4.0f;    // this much worse than where he started: wrong way round
bool  levelOn = false;
float levelStart = -1;

void stopLevel() {
  levelOn = false;
  levelStart = -1;
  setBody(ik::BodyPose());
  applyBody();
}

void levelTick() {
  static unsigned long last = 0;
  if (!levelOn || millis() - last < LEVEL_EVERY_MS) return;
  last = millis();
  if (resting || bodyMode || sysState != SYS_ACTIVE || currentMove != MOVE_STAND || gaitNow.running) return;
  const float err = max(fabsf(pitchDeg), fabsf(rollDeg));
  if (levelStart < 0) levelStart = err;
  if (err > levelStart + LEVEL_WORSE_DEG) {
    Serial2.println(F("LEVEL: getting worse, off"));
    buzzer::play(buzzer::OFF);
    stopLevel();
    return;
  }
  if (err < LEVEL_DEADBAND) return;
  ik::BodyPose b = bodyNow;
  if (fabsf(pitchDeg) >= LEVEL_DEADBAND) b.pitch -= LEVEL_GAIN * pitchDeg;
  if (fabsf(rollDeg) >= LEVEL_DEADBAND)  b.roll  -= LEVEL_GAIN * rollDeg;
  setBody(b);
  applyBody();
}

// ---------------------------------------------------------------------------
// MIND MODE -- the steps the last /mind command asked for, then stand
// ---------------------------------------------------------------------------
void runMindMode() {
  if (mindSteps == 0) { requestMove(MOVE_STAND); return; }

  bool done = false;
  if      (mindAct == WEB_FORWARD)  { if (requestMove(MOVE_WALK))       done = MoveForward(); }
  else if (mindAct == WEB_BACKWARD) { if (requestMove(MOVE_BACKWARD))   done = MoveBackward(); }
  else if (mindAct == WEB_LEFT)     { if (requestMove(MOVE_TURN_LEFT))  done = TurnLeft(); }
  else if (mindAct == WEB_RIGHT)    { if (requestMove(MOVE_TURN_RIGHT)) done = TurnRight(); }
  if (done && mindSteps) mindSteps--;
  else if (!done) mindSteps = 0;   // cut short (wall, tilt, stop, mode change): stand and report
}

// ---------------------------------------------------------------------------
// FOLLOW MODE -- keep a hand FOLLOW_CM in front of his sonar
// ---------------------------------------------------------------------------
float followSpeed(long d) {
  if (d <= 0 || d > FOLLOW_LOST_CM) return 0;   // nobody there
  const long e = d - FOLLOW_CM;
  if (labs(e) <= FOLLOW_DEADBAND_CM) return 0;
  return constrain(e / 20.0f, -1.0f, 1.0f);     // full speed 20 cm off
}

void runFollowMode() {
  followFwd = followSpeed(readDistanceCM());
  if (followFwd == 0) { requestMove(MOVE_STAND); return; }
  if (requestMove(MOVE_WALK)) gaitCycle(followFwd, NO_FWD, NO_FWD, false);   // followFwd is updated as he walks
}

void setFollowMode() {
  if (controlMode == CTRL_FOLLOW) return;
  controlMode   = CTRL_FOLLOW;
  followFwd     = 0;
  abortMovement = true;
  Serial2.println(F("CONTROL -> FOLLOW ME"));
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
// PS2 PAD (ps2.h) -- a pad's receiver on WHIP himself. The sticks and D-pad take over (WEB mode)
// and drive him like the controller's analog drive (cmd=drive): left stick walks and turns, right
// stick crabs; let go and he stands. Buttons as on the PC controller's pad. Polled with pollWeb(),
// so the sticks steer mid-step too.
// ---------------------------------------------------------------------------
bool ps2Driving = false;

// 1: also print the PS2 receiver's raw answer on USB serial twice a second, to debug the pad with only
// a USB cable. That line goes to the servo board too, so ONLY with the servo board switched off. Keep 0.
#define PS2_DEBUG_USB 0

void pollPS2() {
  using namespace ps2;
  const bool was = pad.connected;
  const bool ans = poll();
#if PS2_DEBUG_USB
  static unsigned long lastUsb = 0;
  if (millis() - lastUsb > 500) {
    lastUsb = millis();
    Serial.print(F("PS2 raw:"));
    for (uint8_t i = 0; i < 9; i++) Serial.printf(" %02x", lastRaw[i]);
    Serial.printf("  buttons %04x  L %.2f %.2f  R %.2f %.2f  | %s\r\n", pad.buttons, pad.lx, pad.ly, pad.rx, pad.ry, diagnosis());
  }
#endif
  if (!ans) {
    static unsigned long lastSay = 0;   // the debug console says why, every few seconds
    if (millis() - lastSay > 4000) {
      lastSay = millis();
      Serial2.print(F("PS2: ")); Serial2.println(diagnosis());
    }
    if (was) {
      Serial2.println(F("PS2: pad gone"));
      if (ps2Driving) { ps2Driving = false; webActiveCmd = 0; abortMovement = true; }
    }
    return;
  }
  if (!was) { Serial2.println(F("PS2: pad connected")); buzzer::play(buzzer::MODE); }

  if (pressed(BTN_R3)) {   // BODY mode on / off: the sticks move his body over planted feet
    bodyMode = !bodyMode;
    if (!bodyMode) setBody(ik::BodyPose());
    controlMode = CTRL_WEB; webActiveCmd = 0; abortMovement = true;
    buzzer::play(bodyMode ? buzzer::ON : buzzer::OFF);
    Serial2.println(bodyMode ? F("PS2: BODY mode") : F("PS2: walking"));
  }
  if (bodyMode) {          // left stick slides, right stick tilts, L1 / R1 twist, D-pad up / down raise / lower
    static float yaw = 0, z = 0;
    if (held(BTN_L1)) yaw = min(yaw + 0.5f, ik::BODY_TILT_MAX);
    if (held(BTN_R1)) yaw = max(yaw - 0.5f, -ik::BODY_TILT_MAX);
    if (held(BTN_UP)) z = min(z + 1.0f, ik::BODY_SLIDE_MAX);
    if (held(BTN_DOWN)) z = max(z - 1.0f, -ik::BODY_SLIDE_MAX);
    ik::BodyPose b;
    b.x = pad.ly * ik::BODY_SLIDE_MAX; b.y = pad.lx * ik::BODY_SLIDE_MAX; b.z = z;
    b.roll = -pad.rx * ik::BODY_TILT_MAX; b.pitch = -pad.ry * ik::BODY_TILT_MAX; b.yaw = yaw;
    setBody(b);
    if (pressed(BTN_CROSS) || pressed(BTN_TRIANGLE)) { yaw = 0; z = 0; setBody(ik::BodyPose()); }
    return;                // the rest of the pad walks him: not in BODY mode
  }
  float f = pad.ly, turn = pad.lx, left = pad.rx;
  if (held(BTN_UP)) f = 1;
  if (held(BTN_DOWN)) f = -1;
  if (held(BTN_LEFT)) turn = 1;
  if (held(BTN_RIGHT)) turn = -1;
  if ((f != 0 || turn != 0 || left != 0) && !linkLocked) {
    if (controlMode != CTRL_WEB) {   // he was walking by himself / on the remote: the pad takes over
      controlMode = CTRL_WEB;
      abortMovement = true;
      Serial2.println(F("CONTROL -> WEB (PS2 pad)"));
    }
    driveFwd = f; driveLeft = left; driveTurn = turn;
    webActiveCmd = WEB_DRIVE;
    webLastCmdMillis = millis();
    ps2Driving = true;
  } else if (ps2Driving) {           // let go: stand
    ps2Driving = false;
    webActiveCmd = 0;
    abortMovement = true;
  }

  if (pressed(BTN_CROSS) || pressed(BTN_TRIANGLE)) {   // stand, in WEB mode
    controlMode = CTRL_WEB; webActiveCmd = 0; abortMovement = true;
  }
  if (pressed(BTN_CIRCLE)) {
    controlMode = CTRL_OBSTACLE; obstacleCount = 0; obstacleDetected = false; abortMovement = true;
    Serial2.println(F("CONTROL -> OBSTACLE AVOIDANCE (PS2 pad)"));
  }
  if (pressed(BTN_SQUARE)) setFollowMode();
  if (pressed(BTN_R1)) { ik::gait = (ik::gait + 1) % ik::GAITS; ik::save(); }
  if (pressed(BTN_L1)) { ik::gait = (ik::gait + ik::GAITS - 1) % ik::GAITS; ik::save(); }
  if (pressed(BTN_L2)) pendingAct = 'h';
  if (pressed(BTN_R2)) pendingAct = 'd';
  if (pressed(BTN_START)) pendingAct = resting ? 'w' : 'r';
  if (pressed(BTN_SELECT)) tiltGuard = !tiltGuard;
  if (pressed(BTN_L3)) buzzer::setMuted(!buzzer::muted);
}

// ---------------------------------------------------------------------------
// OBSTACLE AVOIDANCE MODE
// ---------------------------------------------------------------------------
// Debounced check: is something within MIN_DISTANCE_CM right now?
bool wallAhead() {
  uint8_t hits = 0;
  for (uint8_t i = 0; i < OBSTACLE_DEBOUNCE; i++) {
    long d = readDistanceCM();
    if (d > 0 && d < stopCm()) hits++;
    delay(60);   // HC-SR04 needs ~60ms between pings
  }
  return (hits >= OBSTACLE_DEBOUNCE);
}

/*
  Walk forward on repeat.
  If a wall is in range -> stand for a moment.
  Look around: turn a little left, then right past the middle, noting how far the
  sonar sees each way. Turn the clearer way, looking ahead as he turns.
  Clear -> stand, forward. Still blocked after AVOID_MAX_TURNS -> start over.
*/
void runObstacleMode() {
  // Another robot close by and coming or very near, with right of way
  // (fleet_near.h): stand still, then turn away from it before walking on.
  FleetYieldPhase yieldNow = fleetGiveWay(true);
  if (yieldNow == FLEET_WAIT) {
    requestMove(MOVE_STAND);
    return;
  }
  if (yieldNow == FLEET_TURN) {
    if (requestMove(MOVE_TURN_LEFT)) TurnLeft();
    return;
  }

  if (!obstacleDetected) {
    // --- WALK --- (weaving, if it's on)
    if (requestMove(MOVE_WALK)) ik::weave ? WeaveForward() : MoveForward();
    return;
  }

  // --- WALL HIT ---
  obstacleDetected = false;
  Serial2.println(F("WALL -> stand, look around"));

  Stand();
  if (!gaitWait(STAND_HOLD_MS, false)) return;

  // --- LOOK AROUND: a little left, then right past the middle, noting how far he sees ---
  watchClear = true;
  seenFar = 0;
  currentMove = MOVE_TURN_LEFT;
  for (uint8_t i = 0; i < AVOID_LOOK_TURNS; i++) if (!TurnLeft()) { watchClear = false; return; }
  const long farLeft = seenFar;
  seenFar = 0;
  clearPings = 0;
  currentMove = MOVE_TURN_RIGHT;
  for (uint8_t i = 0; i < 2 * AVOID_LOOK_TURNS; i++) if (!TurnRight()) { watchClear = false; return; }
  const long farRight = seenFar;
  const bool right = farRight >= farLeft;
  Serial2.print(F("LOOKED: left ")); Serial2.print(farLeft);
  Serial2.print(F(" cm, right ")); Serial2.print(farRight);
  Serial2.println(right ? F(" cm -> right") : F(" cm -> left"));

  // --- TURN the clearer way until the way ahead is clear (looking as he turns) ---
  bool clear = right && clearPings >= OBSTACLE_DEBOUNCE;   // already looking at it
  clearPings = 0;
  currentMove = right ? MOVE_TURN_RIGHT : MOVE_TURN_LEFT;
  for (uint8_t i = 1; i <= AVOID_MAX_TURNS && !clear; i++) {
    if (!(right ? TurnRight() : TurnLeft())) { watchClear = false; return; }
    clear = clearPings >= OBSTACLE_DEBOUNCE;
  }
  watchClear = false;
  Stand();

  if (clear) {
    Serial2.println(F("CLEAR -> forward"));
    if (ik::wallGait) {   // GAIT PER WALL: walks on in the next gait (all six feet are down now)
      ik::gait = (ik::gait + 1) % ik::GAITS;
      Serial2.print(F("GAIT PER WALL -> ")); Serial2.println(ik::GAIT_NAMES[ik::gait]);
    }
  } else {
    Serial2.println(F("STILL BLOCKED -> start over"));
    obstacleDetected = true;   // next loop pass turns again (the other way)
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

  if (cmd == IR_GAIT) {
    ik::gait = (ik::gait + 1) % ik::GAITS;
    ik::save();
    Serial2.print(F("GAIT -> ")); Serial2.println(ik::GAIT_NAMES[ik::gait]);
    return;
  }
  if (cmd == IR_HELLO)  { pendingAct = 'h'; return; }
  if (cmd == IR_DANCE)  { pendingAct = 'd'; return; }
  if (cmd == IR_REST)   { pendingAct = resting ? 'w' : 'r'; return; }
  if (cmd == IR_FOLLOW) { setFollowMode(); return; }

  if (controlMode != CTRL_IR) return;

  if (cmd == IR_FORWARD || cmd == IR_BACKWARD || cmd == IR_LEFT || cmd == IR_RIGHT) {
    irActiveCmd     = cmd;
    irLastCmdMillis = millis();
  }
}

// A command relayed by NORA over the fleet link, run as the matching remote
// button. Driving switches WHIP into IR control first, so one press is enough.
void runLinkCommand(uint8_t c) {
  switch (c) {
    case LINK_FORWARD: case LINK_BACKWARD: case LINK_LEFT: case LINK_RIGHT:
      handleIRCommand(IR_MODE_IRControl);
      irTimeoutMs = LINK_HOLD_MS;
      handleIRCommand(c == LINK_FORWARD ? IR_FORWARD : c == LINK_BACKWARD ? IR_BACKWARD
                    : c == LINK_LEFT ? IR_LEFT : IR_RIGHT);
      break;
    case LINK_STOP:     handleIRCommand(IR_MODE_IRControl); handleIRCommand(IR_OK); break;
    case LINK_OBSTACLE: handleIRCommand(IR_MODE_OBSTACLE);  break;
    case LINK_MANUAL:   handleIRCommand(IR_MODE_IRControl); break;
    case LINK_UNLOCK:   linkLocked = false; break;
    case LINK_LOCK:     linkLocked = true; irActiveCmd = 0; webActiveCmd = 0; abortMovement = true; break;
  }
}

void pollIR() {
  if (IrReceiver.decode()) {
    bool isRepeat = (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT);
    bool isNoise  = (IrReceiver.decodedIRData.protocol == UNKNOWN);

    if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == WHIP_LINK_ADDRESS) {
      Serial2.print(F("LINK cmd=0x"));
      Serial2.println(IrReceiver.decodedIRData.command, HEX);
      runLinkCommand(IrReceiver.decodedIRData.command);
    } else if (isRepeat) {
      if (irActiveCmd != 0) irLastCmdMillis = millis();
    } else if (!isNoise) {
      Serial2.print(F("IR cmd=0x"));
      Serial2.println(IrReceiver.decodedIRData.command, HEX);
      irTimeoutMs = IR_CMD_TIMEOUT_MS;
      handleIRCommand(IrReceiver.decodedIRData.command);
    } else {
      Serial2.println(F("IR noise (UNKNOWN protocol)"));
    }
    IrReceiver.resume();
  }

  if (irActiveCmd != 0 && millis() - irLastCmdMillis > irTimeoutMs) {
    irActiveCmd = 0;
  }
}

// ---------------------------------------------------------------------------
// WIFI + WEB SERVER
// ---------------------------------------------------------------------------
void setupWiFi() {
  Serial2.println(F("Looking for the fleet's networks..."));
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    int r = ladderRank(WiFi.SSID(i));
    if (r >= 0) bootSeen[r] = true;
  }
  WiFi.scanDelete();

  wifiBooting = true;
  bootTry = -1;
  tryNextBootNetwork();
}

// The highest network he saw at boot that he hasn't tried yet; none left -> he hosts
void tryNextBootNetwork() {
  for (int r = bootTry + 1; r < ABOVE_N; r++) {
    if (!bootSeen[r]) continue;
    bootTry = r;
    bootTryStart = millis();
    Serial2.print(F("Joining "));
    Serial2.println(ABOVE_WHIP[r]);
    WiFi.begin(ABOVE_WHIP[r], r == 0 ? FLEET_PASSWORD : NORA_PASSWORD);
    return;
  }
  hostOwnAP();
}

// Joining in the background, so he walks while it connects (WIFI_JOIN_MS per network)
void wifiBootLoop() {
  if (!wifiBooting) return;
  if (WiFi.status() == WL_CONNECTED) {
    wifiBooting = false;
    wifiMode = WIFI_JOINED;
    joinedRank = bootTry;
    Serial2.print(F("Joined "));
    Serial2.print(ABOVE_WHIP[bootTry]);
    Serial2.print(F(". IP: "));
    Serial2.println(WiFi.localIP());
    fleetNearBegin("WHIP", WIFI_IF_STA);   // on that network's channel, so the fleet hears him
    return;
  }
  if (millis() - bootTryStart < WIFI_JOIN_MS) return;
  WiFi.disconnect(false);
  joinFailedMs[bootTry] = millis();
  Serial2.println(F("  couldn't join (password? full?) -> trying the next one"));
  tryNextBootNetwork();
}

void hostOwnAP() {
  wifiBooting = false;
  Serial2.println(F("Nobody above him around -> hosting \"WHIP\""));
  WiFi.mode(WIFI_AP_STA);                        // AP for the robots below; STA so he can still scan
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 8);    // channel 1 like every host, up to 8 robots
  wifiMode = WIFI_OWN_AP;
  joinedRank = -1;
  setupFleetServer();
  fleetNearBegin("WHIP", WIFI_IF_AP);
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
int registerWithFleet() {
  HTTPClient http;
  http.begin(String("http://") + WiFi.gatewayIP().toString() + ":" + FLEET_PORT + "/register");
  http.setTimeout(2000);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST("name=WHIP&type=hexapod&capabilities=obstacle_avoidance,ir_remote,web_control,web:5005,talk:5005");
  http.end();
  return code;
}

// The heartbeat doubles as the "still connected?" check: no WiFi, or a host whose fleet list
// doesn't take it, and netGoneSince starts counting (wifiLadderLoop() acts on it).
void pollFleetRegistration() {
  if (wifiMode != WIFI_JOINED) return;   // hosting: he is the list

  if (WiFi.status() != WL_CONNECTED) {
    if (!netGoneSince) netGoneSince = millis();
    return;
  }
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat < FLEET_HEARTBEAT_MS) return;
  lastHeartbeat = millis();

  if (registerWithFleet() == 200) netGoneSince = 0;
  else if (!netGoneSince) netGoneSince = millis();
}

// Climb the ladder: look around now and then (through the radar's scan, so the two never collide)
// and start over when a higher network is up or his own has gone quiet. Only while he stands.
void wifiLadderLoop() {
  if (wifiBooting) return;   // still joining
  if (wifiMode == WIFI_OWN_AP) fleetServer.handleClient();

  static unsigned long lastLook = 0;
  bool onTop = wifiMode == WIFI_JOINED && joinedRank == 0;   // on RIFT's network: nothing higher
  if (!onTop && millis() - lastLook >= LADDER_CHECK_MS) {
    lastLook = millis();
    radarRequested = true;
  }

  bool lost = netGoneSince && millis() - netGoneSince > NET_LOST_MS;
  if ((higherSeen || lost) && currentMove == MOVE_STAND) {
    Serial2.println(higherSeen ? F("LADDER: a higher network is up -> restart to join it")
                               : F("LADDER: my network went quiet -> restart and climb again"));
    delay(100);
    ESP.restart();
  }
}

int ladderRank(const String& ssid) {   // its place in ABOVE_WHIP, or -1 if it isn't above him
  for (int i = 0; i < ABOVE_N; i++) if (ssid == ABOVE_WHIP[i]) return i;
  return -1;
}

// ---- The fleet list, while he hosts: the same wire format as RIFT's and NORA's ----
#define FLEET_MAX 8
#define FLEET_TTL_MS 20000   // drop a robot after this long without a heartbeat
String fleetName[FLEET_MAX], fleetType[FLEET_MAX], fleetCaps[FLEET_MAX];
IPAddress fleetIp[FLEET_MAX];
unsigned long fleetSeen[FLEET_MAX] = { 0 };

void setupFleetServer() {
  fleetServer.on("/ping", []() { fleetServer.send(200, "text/plain", "WHIP alive"); });

  fleetServer.on("/register", HTTP_POST, []() {
    String name = fleetServer.arg("name");
    if (!name.length()) { fleetServer.send(400, "text/plain", "missing 'name'"); return; }
    int slot = -1;
    for (int i = 0; i < FLEET_MAX; i++) if (fleetSeen[i] && fleetName[i] == name) slot = i;
    for (int i = 0; i < FLEET_MAX && slot < 0; i++)
      if (!fleetSeen[i] || millis() - fleetSeen[i] > FLEET_TTL_MS) slot = i;
    if (slot < 0) { fleetServer.send(503, "text/plain", "fleet list full"); return; }
    fleetName[slot] = name;
    fleetType[slot] = fleetServer.hasArg("type") ? fleetServer.arg("type") : "unknown";
    fleetCaps[slot] = fleetServer.arg("capabilities");
    fleetIp[slot]   = fleetServer.client().remoteIP();
    fleetSeen[slot] = millis();
    fleetServer.send(200, "text/plain", "OK");
  });

  fleetServer.on("/robots", []() {
    String j = "{\"authority\":\"WHIP\",\"robots\":[";
    j += "{\"name\":\"WHIP\",\"ip\":\"" + WiFi.softAPIP().toString() +
         "\",\"type\":\"hexapod\",\"capabilities\":[\"obstacle_avoidance\",\"ir_remote\",\"web_control\",\"web:5005\"]}";
    for (int i = 0; i < FLEET_MAX; i++) {
      if (!fleetSeen[i] || millis() - fleetSeen[i] > FLEET_TTL_MS) continue;
      j += ",{\"name\":\"" + radarEscape(fleetName[i]) + "\",\"ip\":\"" + fleetIp[i].toString() +
           "\",\"type\":\"" + radarEscape(fleetType[i]) + "\",\"capabilities\":[";
      String caps = fleetCaps[i];
      int start = 0, n = 0;
      while (start < (int)caps.length()) {
        int comma = caps.indexOf(',', start);
        if (comma < 0) comma = caps.length();
        if (comma > start) j += String(n++ ? "," : "") + "\"" + radarEscape(caps.substring(start, comma)) + "\"";
        start = comma + 1;
      }
      j += "]}";
    }
    j += "]}";
    fleetServer.send(200, "application/json", j);
  });

  fleetServer.begin();
  Serial2.println(F("Fleet list on :5000 (he hosts)"));
}

bool isFleetSsid(const String& s) {
  for (unsigned i = 0; i < sizeof(FLEET_SSIDS) / sizeof(FLEET_SSIDS[0]); i++) {
    if (s == FLEET_SSIDS[i]) return true;
  }
  return false;
}

String radarEscape(const String& in) {
  String out;
  for (unsigned i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n' || c == '\r' || c == '\t') { out += ' '; }
    else out += c;
  }
  return out;
}

void radarLoop() {
  if (wifiBooting) return;   // a scan now would get in the way of joining
  if (!radarInFlight && radarRequested && millis() - lastRadarScan >= RADAR_MIN_GAP_MS) {
    radarRequested = false;
    WiFi.scanNetworks(true, true);   // async, include hidden
    radarInFlight = true;
    lastRadarScan = millis();
    return;
  }
  if (radarInFlight) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) return;
    if (n == WIFI_SCAN_FAILED)  { radarInFlight = false; return; }
    int imposters = 0;
    String j = "{\"scanning\":false,\"nets\":[";
    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      String shown = ssid.length() ? radarEscape(ssid) : "(hidden)";
      bool fleet = ssid.length() && isFleetSsid(ssid);
      if (!fleet) imposters++;
      int r = ladderRank(ssid);   // the ladder: is a network above his own up?
      if (r >= 0 && (wifiMode == WIFI_OWN_AP || r < joinedRank)
          && (!joinFailedMs[r] || millis() - joinFailedMs[r] > JOIN_RETRY_MS)) higherSeen = true;
      if (i) j += ",";
      j += "{\"ssid\":\"" + shown + "\",\"rssi\":" + String(WiFi.RSSI(i)) +
           ",\"imposter\":" + String(fleet ? "false" : "true") + "}";
    }
    j += "],\"imposters\":" + String(imposters) + "}";
    radarJson = j;
    WiFi.scanDelete();
    radarInFlight = false;
  }
}

bool studioHolds() { return studioActive && millis() - studioSeen < STUDIO_TIMEOUT_MS; }

bool isController() {
  const IPAddress ip = server.client().remoteIP();
  if (studioHolds()) return server.arg("sid") == studioSid;   // the testing software outranks everyone
  for (uint8_t i = 0; i < controllerCount; i++) if (controllerIPs[i] == ip) return true;
  return false;
}

void addController(IPAddress ip) {
  for (uint8_t i = 0; i < controllerCount; i++) if (controllerIPs[i] == ip) return;
  if (controllerCount < MAX_CONTROLLERS) { controllerIPs[controllerCount++] = ip; return; }
  controllerIPs[controllerNext] = ip;   // full: the oldest one makes room
  controllerNext = (controllerNext + 1) % MAX_CONTROLLERS;
}

// He's his own again: the studio let go or went quiet. He stands in WEB mode, not walking off.
void studioRelease() {
  if (!studioActive) return;
  studioActive = false;
  calibrating = false;
  controlMode = CTRL_WEB; webActiveCmd = 0;
  ikNow = ik::home();
  standNow();
  stoodSince = millis();
  Serial2.println(F("STUDIO: let go, standing (web mode)"));
}

// true while Servo Studio holds him: loop() does nothing else, so no remote, pad, tilt
// safety or walk moves a servo under the studio's hands
bool studioLoop() {
  if (studioActive && !studioHolds()) studioRelease();   // it went quiet
  return studioActive;
}

// Every saved setting in one number (FNV-1a): lights, gait, height, speed, tilt safety,
// sound, body, calibration. /status carries it, so every page and program sees at once that
// something changed somewhere, and fetches what it shows again.
uint32_t settingsRev() {
  uint32_t h = 2166136261u;
  auto mix = [&h](const void* p, size_t n) {
    const uint8_t* b = (const uint8_t*)p;
    while (n--) { h ^= *b++; h *= 16777619u; }
  };
  mix(leds::cfg, sizeof leds::cfg); mix(&leds::brightness, sizeof leds::brightness);
  mix(&ik::gait, 1); mix(&ik::height, 1); mix(&ik::speed, 1); mix(&ik::weave, 1); mix(&ik::antiSlip, 1); mix(&ik::autoStart, 1); mix(&ik::footLead, 1); mix(&ik::wallGait, 1); mix(&ik::crabBack, 1);
  mix(&tiltGuard, sizeof tiltGuard); mix(&buzzer::muted, sizeof buzzer::muted);
  mix(&bodyK, sizeof bodyK); mix(&ik::calibrated, sizeof ik::calibrated);
  for (uint8_t l = 0; l < ik::LEGS; l++)
    for (uint8_t j = 0; j < ik::JOINTS; j++) {
      const ik::ServoCal& c = ik::cal[l][j];
      mix(&c.ch, sizeof c.ch); mix(&c.rev, sizeof c.rev); mix(&c.trim, sizeof c.trim);
    }
  return h;
}

void handleWebCmd() {
  if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
  if (!server.hasArg("cmd")) { server.send(400, "text/plain", "missing 'cmd'"); return; }
  String cmd = server.arg("cmd");

  if      (cmd == "fwd")   { webActiveCmd = WEB_FORWARD;  webLastCmdMillis = millis(); }
  else if (cmd == "bwd")   { webActiveCmd = WEB_BACKWARD; webLastCmdMillis = millis(); }
  else if (cmd == "left")  { webActiveCmd = WEB_LEFT;     webLastCmdMillis = millis(); }
  else if (cmd == "right") { webActiveCmd = WEB_RIGHT;    webLastCmdMillis = millis(); }
  else if (cmd == "sleft") { webActiveCmd = WEB_SLEFT;    webLastCmdMillis = millis(); }
  else if (cmd == "sright"){ webActiveCmd = WEB_SRIGHT;   webLastCmdMillis = millis(); }
  else if (cmd == "drive") {   // &f= forward &l= left (sideways) &t= turn (anticlockwise), each -1..1
    driveFwd  = constrain(server.arg("f").toFloat(), -1.0f, 1.0f);
    driveLeft = constrain(server.arg("l").toFloat(), -1.0f, 1.0f);
    driveTurn = constrain(server.arg("t").toFloat(), -1.0f, 1.0f);
    webActiveCmd = WEB_DRIVE;
    webLastCmdMillis = millis();
  }
  else if (cmd == "stop")  { webActiveCmd = 0; abortMovement = true; }
  else { server.send(400, "text/plain", "bad cmd"); return; }

  server.send(200, "text/plain", "OK");
}

// /mind?act=fwd|bwd|left|right|stop&steps=1-4 (MIND mode only). A forward move
// with a wall within MIN_DISTANCE_CM is refused: {"ok":false,"blocked":true}.
bool mindHostAlive() { return mindHostSeen && millis() - mindHostSeen < MIND_HOST_TTL_MS; }

void handleMindCmd() {
  if (studioHolds() || !mindHostAlive() || server.client().remoteIP() != mindHostIP) { server.send(403, "text/plain", "LOCKED"); return; }
  if (controlMode != CTRL_MIND) { server.send(409, "application/json", "{\"ok\":false,\"error\":\"not in mind mode\"}"); return; }
  String act = server.arg("act");
  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 1;
  if (steps < 1) steps = 1;
  if (steps > MIND_MAX_STEPS) steps = MIND_MAX_STEPS;

  uint8_t a = 0;
  if      (act == "fwd")   a = WEB_FORWARD;
  else if (act == "bwd")   a = WEB_BACKWARD;
  else if (act == "left")  a = WEB_LEFT;
  else if (act == "right") a = WEB_RIGHT;
  else if (act == "stop")  { mindSteps = 0; abortMovement = true; server.send(200, "application/json", "{\"ok\":true}"); return; }
  else { server.send(400, "application/json", "{\"ok\":false,\"error\":\"bad act\"}"); return; }

  mindBlocked = false;
  if (a == WEB_FORWARD) {
    long d = readDistanceCM();
    if (d > 0 && d < stopCm()) {
      mindBlocked = true;
      server.send(200, "application/json", "{\"ok\":false,\"blocked\":true,\"distance\":" + String(d) + "}");
      return;
    }
  }
  mindAct   = a;
  mindSteps = steps;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleModeCmd() {
  if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
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
  } else if (m == "follow") {
    setFollowMode();
  } else if (m == "mind") {
    if (!mindHostAlive()) { server.send(409, "text/plain", "no mind controller running"); return; }
    controlMode   = CTRL_MIND;
    mindSteps     = 0;
    mindBlocked   = false;
    abortMovement = true;
    Serial2.println(F("CONTROL -> MIND"));
  } else {
    server.send(400, "text/plain", "bad mode");
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleShutdownCmd() {
  if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
  sysState = SYS_SHUTDOWN;
  Serial2.println(F("SHUTDOWN (web)"));
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  if (server.arg("ctl") == "1" && mindHostSeen && server.client().remoteIP() == mindHostIP) mindHostSeen = millis();
  long distance = readDistanceCM();

  String json = "{";
  json += "\"distance\":"    + String(distance) + ",";
  json += "\"pitch\":"       + String(pitchDeg - pitchOffsetDeg, 1) + ",";
  json += "\"roll\":"        + String(rollDeg - rollOffsetDeg, 1) + ",";
  json += "\"sysState\":"    + String((int)sysState) + ",";
  json += "\"controlMode\":" + String((int)controlMode) + ",";
  json += "\"move\":"        + String((int)currentMove) + ",";
  json += "\"wifiMode\":"    + String((int)wifiMode) + ",";
  json += "\"network\":\""    + String(wifiMode == WIFI_JOINED ? ABOVE_WHIP[joinedRank] : AP_SSID) + "\",";
  json += "\"mpuFound\":"    + String(mpuFound ? 1 : 0) + ",";
  json += "\"calibrated\":"  + String(ik::calibrated ? 1 : 0) + ",";
  json += "\"calibrating\":" + String(calibrating ? 1 : 0) + ",";
  json += "\"locked\":"      + String(linkLocked ? 1 : 0) + ",";
  json += "\"mindHost\":"    + String(mindHostAlive() ? 1 : 0) + ",";
  json += "\"mindSteps\":"   + String(mindSteps) + ",";
  json += "\"blocked\":"     + String(mindBlocked || manualBlocked ? 1 : 0) + ",";
  json += "\"height\":"      + String(ik::height) + ",";
  json += "\"speed\":"       + String(ik::speed) + ",";
  json += "\"gait\":"        + String(ik::gait) + ",";
  json += "\"tiltGuard\":"   + String(tiltGuard ? 1 : 0) + ",";
  json += "\"sound\":"       + String(buzzer::muted ? 0 : 1) + ",";
  json += "\"resting\":"     + String(resting ? 1 : 0) + ",";
  json += "\"ps2\":"         + String(ps2::pad.connected ? 1 : 0) + ",";
  json += "\"centred\":"     + String(centred ? 1 : 0) + ",";
  json += "\"testGait\":"    + String(testGait ? 1 : 0) + ",";
  json += "\"weave\":"       + String(ik::weave ? 1 : 0) + ",";
  json += "\"antiSlip\":"    + String(ik::antiSlip ? 1 : 0) + ",";
  json += "\"autoStart\":"   + String(ik::autoStart ? 1 : 0) + ",";
  json += "\"footLead\":"    + String(ik::footLead) + ",";
  json += "\"crabBack\":"    + String(ik::crabBack) + ",";
  json += "\"level\":"       + String(levelOn ? 1 : 0) + ",";
  json += "\"wallGait\":"    + String(ik::wallGait ? 1 : 0) + ",";
  json += "\"awaiting\":"    + String(awaiting ? 1 : 0) + ",";
  json += "\"studio\":"      + String(studioHolds() ? 1 : 0) + ",";
  json += "\"rev\":"         + String(settingsRev()) + ",";
  json += "\"bodyMode\":"    + String(bodyMode ? 1 : 0) + ",";
  json += "\"bodyK\":"       + String(bodyK, 2) + ",";
  json += "\"body\":["       + String(bodyNow.x, 1) + "," + String(bodyNow.y, 1) + "," + String(bodyNow.z, 1) + "," +
          String(bodyNow.roll, 1) + "," + String(bodyNow.pitch, 1) + "," + String(bodyNow.yaw, 1) + "]";
  json += "}";

  server.send(200, "application/json", json);
}

void setupWebServer() {
  server.on("/near",     []() { server.send(200, "application/json", fleetNearStatusJson()); });   // nearby robots, nearest first
  server.on("/radar",    []() { radarRequested = true; server.send(200, "application/json", radarJson); });
  // His leg lock over WiFi, as NORA's wheel lock (RIFT's Lock all / Unlock all): the same lock as
  // NORA's IR LOCK / UNLOCK. Locking is free, unlocking needs his PIN.
  server.on("/motorlockOn", []() {
    linkLocked = true; irActiveCmd = 0; webActiveCmd = 0; abortMovement = true;
    server.send(200, "text/plain", "OK");
  });
  server.on("/motorlockOff", []() {
    if (server.arg("pw") != CONTROL_PIN) { server.send(403, "text/plain", "DENIED"); return; }
    linkLocked = false;
    server.send(200, "text/plain", "OK");
  });
  server.on("/unlock",   []() {
    if (server.hasArg("pw") && server.arg("pw") == CONTROL_PIN) {
      const IPAddress ip = server.client().remoteIP();
      addController(ip);
      if (server.arg("ctl") == "1") { mindHostIP = ip; mindHostSeen = millis(); }
      server.send(200, "application/json", "{\"ok\":true,\"controller\":\"" + ip.toString() + "\",\"studio\":" +
                  String(studioHolds() && server.arg("sid") != studioSid ? 1 : 0) + "}");
    } else {
      server.send(403, "application/json", "{\"ok\":false,\"error\":\"bad pin\"}");
    }
  });
  server.on("/keys",     []() { server.send_P(200, "text/html", KEYS_HTML); });
  // Servo Studio: ?pw=<PIN>&sid=<its session>&on=1 takes him and checks in (about every second),
  // on=0 lets go.
  // While it holds him nobody else drives him (403), and IR, the PS2 pad and tilt safety rest.
  server.on("/studio",   []() {
    const IPAddress ip = server.client().remoteIP();
    if (server.arg("on") == "0") {
      if (studioActive && server.arg("sid") == studioSid) studioRelease();
      server.send(200, "application/json", "{\"ok\":true,\"studio\":0}");
      return;
    }
    if (server.arg("pw") != CONTROL_PIN) { server.send(403, "application/json", "{\"ok\":false,\"error\":\"bad pin\"}"); return; }
    const String sid = server.arg("sid");
    if (sid.length() < 4) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"sid= missing\"}"); return; }
    if (studioHolds() && sid != studioSid) {
      server.send(409, "application/json", "{\"ok\":false,\"error\":\"another Servo Studio has him: " + studioIP.toString() + "\"}");
      return;
    }
    if (!studioActive) {
      Serial2.print(F("STUDIO: ")); Serial2.print(ip); Serial2.println(F(" has him"));
      abortMovement = true; webActiveCmd = 0; irActiveCmd = 0; gaitNow.running = false;
      currentMove = MOVE_STAND; calibrating = true;
      testPosed = false; testK = 0;   // the studio moves them now: no gliding back from a stale pose
      wakeUp();
    }
    studioSid = sid; studioIP = ip; studioSeen = millis(); studioActive = true;
    addController(ip);
    server.send(200, "application/json", "{\"ok\":true,\"studio\":1,\"rev\":" + String(settingsRev()) + "}");
  });
  // Restart his ESP32 (the controller's / the studio's REBOOT): the reply goes out first
  server.on("/reboot",   []() {
    if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
    server.send(200, "text/plain", "OK");
    Serial2.println(F("REBOOT (web)"));
    delay(300);
    ESP.restart();
  });
  server.on("/",         handleRoot);
  server.on("/web",      handleWebCmd);
  server.on("/mode",     handleModeCmd);
  server.on("/mind",     handleMindCmd);
  server.on("/shutdown", handleShutdownCmd);
  server.on("/status",   handleStatus);
  server.on("/ps2",      []() {                 // the PS2 receiver's last answer, raw, and what it means
    String j = "{\"connected\":" + String(ps2::pad.connected ? 1 : 0) + ",\"raw\":\"";
    for (uint8_t i = 0; i < 9; i++) { char h[4]; snprintf(h, sizeof h, "%02x ", ps2::lastRaw[i]); j += h; }
    j.trim();
    j += "\",\"buttons\":" + String(ps2::pad.buttons) + ",\"lx\":" + String(ps2::pad.lx, 2) + ",\"ly\":" +
         String(ps2::pad.ly, 2) + ",\"rx\":" + String(ps2::pad.rx, 2) + ",\"ry\":" + String(ps2::pad.ry, 2) +
         ",\"pins\":\"DAT 27, CMD 25, ATT 26, CLK 32, VCC 3.3 V\",\"says\":\"" + ps2::diagnosis() + "\"}";
    server.send(200, "application/json", j);
  });
  server.on("/leds",     []() {                 // the lights: GET = every look; ?look=&rgb=rrggbb&fx= / ?bright= / ?reset=1
    if (server.args() && !isController()) { server.send(403, "text/plain", "LOCKED"); return; }
    if (server.hasArg("reset")) { memcpy(leds::cfg, leds::DEFAULT_CFG, sizeof(leds::cfg)); leds::brightness = 40; leds::save(); }
    if (server.hasArg("bright")) { leds::brightness = constrain(server.arg("bright").toInt(), 0, 255); leds::save(); }
    if (server.hasArg("look")) {
      for (uint8_t l = 0; l < leds::LOOKS; l++) if (server.arg("look") == leds::LOOK_NAMES[l]) {
        if (server.hasArg("rgb")) {
          const long v = strtol(server.arg("rgb").c_str(), nullptr, 16);
          leds::cfg[l].r = (v >> 16) & 255; leds::cfg[l].g = (v >> 8) & 255; leds::cfg[l].b = v & 255;
        }
        if (server.hasArg("fx")) {
          for (uint8_t f = 0; f < leds::FXS; f++) if (server.arg("fx") == leds::FX_NAMES[f] || server.arg("fx") == String(f)) leds::cfg[l].fx = f;
        }
        leds::save();
      }
    }
    String j = "{\"bright\":" + String(leds::brightness) + ",\"looks\":{";
    for (uint8_t l = 0; l < leds::LOOKS; l++) {
      char hex[8]; snprintf(hex, sizeof hex, "%02x%02x%02x", leds::cfg[l].r, leds::cfg[l].g, leds::cfg[l].b);
      j += String(l ? "," : "") + "\"" + leds::LOOK_NAMES[l] + "\":{\"rgb\":\"" + hex + "\",\"fx\":\"" + leds::FX_NAMES[leds::cfg[l].fx] + "\"}";
    }
    server.send(200, "application/json", j + "}}");
  });
  server.on("/lights",   []() { server.send_P(200, "text/html", LIGHTS_HTML); });
  server.on("/pose",     []() {                 // his joint angles now (deg, LF..RB x hip knee foot) + body pose
    float a[ik::LEGS][ik::JOINTS];
    String j = "{\"a\":[";
    const bool ok = ik::anglesOf(ikNow, a);
    for (uint8_t l = 0; l < ik::LEGS; l++) {
      j += String(l ? ",[" : "[");
      for (uint8_t k = 0; k < ik::JOINTS; k++) j += String(k ? "," : "") + String(ok ? a[l][k] : 0.0f, 1);
      j += "]";
    }
    j += "],\"b\":[" + String(bodyNow.x, 1) + "," + String(bodyNow.y, 1) + "," + String(bodyNow.z, 1) + "," +
         String(bodyNow.roll, 1) + "," + String(bodyNow.pitch, 1) + "," + String(bodyNow.yaw, 1) + "]";
    j += ",\"walk\":" + String(gaitNow.running ? 1 : 0) + ",\"bm\":" + String(bodyMode ? 1 : 0) + "}";
    server.send(200, "application/json", j);
  });
  server.on("/body",     []() {                 // ?x=&y=&z= mm &roll=&pitch=&yaw= deg: his body over planted feet
    if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
    ik::BodyPose b;
    b.x = server.arg("x").toFloat(); b.y = server.arg("y").toFloat(); b.z = server.arg("z").toFloat();
    b.roll = server.arg("roll").toFloat(); b.pitch = server.arg("pitch").toFloat(); b.yaw = server.arg("yaw").toFloat();
    if (controlMode != CTRL_WEB) { controlMode = CTRL_WEB; abortMovement = true; }
    webActiveCmd = 0;
    setBody(b);
    server.send(200, "application/json", String("{\"ok\":true,\"k\":") + String(bodyK, 2) + "}");
  });
  server.on("/chirp",    []() {                 // ?u=<n>: say a phrase (RIFT's conversations, as every robot's)
    const int u = server.hasArg("u") ? server.arg("u").toInt() : -1;
    if (u < 0 || u >= TALK_UTTERANCES) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"use /chirp?u=0-" + String(TALK_UTTERANCES - 1) + "\"}");
      return;
    }
    talkStart(u);
    server.send(200, "application/json", u < 14 ? "{\"ok\":true,\"text\":\"" + String(TALK_TEXT[u]) + "\"}" : String("{\"ok\":true}"));
  });
  server.on("/act",      []() {                 // ?do=hello|dance|rest|wake: once, between moves
    if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
    const String a = server.arg("do");
    if      (a == "hello") pendingAct = 'h';
    else if (a == "dance") pendingAct = 'd';
    else if (a == "rest")  pendingAct = resting ? 'w' : 'r';   // REST again gets him up
    else if (a == "wake")  pendingAct = 'w';
    else { server.send(400, "text/plain", "bad act"); return; }
    abortMovement = true;
    server.send(200, "text/plain", "OK");
  });
  server.on("/buzzer",   []() {                 // ?on=0|1: mute / sound (on again at boot)
    if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
    if (server.hasArg("on")) buzzer::setMuted(server.arg("on") == "0");
    if (!buzzer::muted) buzzer::play(buzzer::MODE);
    server.send(200, "application/json", String("{\"sound\":") + (buzzer::muted ? 0 : 1) + "}");
  });
  server.on("/tilt",     []() {                 // ?on=0|1: tilt safety off / on (back on at boot)
    if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
    if (server.hasArg("on")) tiltGuard = server.arg("on") != "0";
    Serial2.println(tiltGuard ? F("TILT SAFETY ON") : F("TILT SAFETY OFF"));
    server.send(200, "application/json", String("{\"tiltGuard\":") + (tiltGuard ? 1 : 0) + "}");
  });
  server.on("/testgait", []() {                 // ?on=0|1: TEST-GAIT (hips and thighs only) off / on (off at boot)
    if (!isController()) { server.send(403, "text/plain", "LOCKED"); return; }
    if (server.hasArg("on")) testGait = server.arg("on") != "0";
    Serial2.println(testGait ? F("TEST-GAIT ON") : F("TEST-GAIT OFF"));
    server.send(200, "application/json", String("{\"testGait\":") + (testGait ? 1 : 0) + "}");
  });
  setupCalibRoutes();
  server.on("/avatar.jpg",  handleAvatar);
  server.on("/favicon.ico", handleAvatar);
  server.begin();
  if (MDNS.begin("whip")) MDNS.addService("http", "tcp", 5005);   // whip.local, as MILA and NIKA have
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
  <link rel="icon" href="/avatar.jpg">
  <style>
    :root {
      /* his colour_scheme.xml (sampled from his avatar: red and blue spider on night black) */
      --bg:          #080A12;
      --panel:       #0E1120;
      --border:      #1D223F;
      --accent:      #D34859;
      --accent-rgb:  211,72,89;
      --accent2:     #1A7BD9;
      --text:        #A89497;
      --text-dim:    #6B575A;
      --text-val:    #F2E9EA;
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
    #avatar { width: 96px; height: 96px; border-radius: 50%; border: 2px solid var(--accent); object-fit: cover; }
    h1 { font-size: 1.6rem; letter-spacing: 3px; color: var(--accent); }
    h2 { font-size: 0.8rem; color: var(--text-dim); letter-spacing: 1px; }

    #sensors { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; width: 100%; max-width: 340px; }
    .sensor-box {
      background: var(--panel); border: 2px solid var(--border); border-radius: var(--radius);
      padding: 8px 12px; font-size: 0.85rem;
      display: flex; flex-direction: column; gap: 4px;
    }
    .sensor-box span { color: var(--accent); font-weight: bold; }

    #modeRow { display: flex; gap: 8px; flex-wrap: wrap; justify-content: center; width: 100%; max-width: 380px; }
    .mode-btn {
      padding: 10px 18px; border: 2px solid var(--border); border-radius: var(--radius);
      font-size: 0.85rem; font-weight: bold; cursor: pointer;
      background: var(--panel); color: var(--text-dim);
    }
    .mode-btn.active { border-color: var(--accent); color: var(--accent); background: rgba(var(--accent-rgb), 0.12); }
    .keys-link { text-decoration: none; }

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

    #gaitBox { width: 100%; max-width: 340px; }
    #actRow { display: flex; gap: 8px; justify-content: center; }
    .gait-row { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
    #gaitSel {
      background: var(--panel); color: var(--accent); border: 2px solid var(--border); border-radius: var(--radius);
      font-family: var(--font); font-size: 0.85rem; font-weight: bold; padding: 6px 8px; cursor: pointer;
    }
    #gaitAnim { width: 100%; height: auto; display: block; }
    #gaitNote { font-size: 0.75rem; color: var(--text-dim); }
    #tiltBtn, #soundBtn { padding: 6px 10px; }
    #soundBtn.active { border-color: var(--accent); color: var(--accent); }
    #tiltBtn.active { border-color: var(--green); color: var(--green); background: rgba(var(--green-rgb), 0.12); }
    #tiltBtn.off { border-color: var(--orange); color: var(--orange); background: rgba(var(--orange-rgb), 0.12); }

    #shutdownBtn {
      padding: 9px 20px; border: 2px solid var(--red); border-radius: var(--radius);
      font-size: 0.85rem; font-weight: bold; cursor: pointer;
      background: rgba(var(--red-rgb), 0.12); color: var(--red);
    }
  </style>
</head>
<body>
  <div id="lockOverlay" style="position:fixed;inset:0;background:rgba(5,8,16,0.96);z-index:9999;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:16px;color:#e6edf3;font-family:inherit">
    <div style="font-size:1.3rem;letter-spacing:3px">&#128274; WHIP LOCKED</div>
    <div style="opacity:0.7;font-size:0.85rem">Enter controller PIN to drive</div>
    <input id="pinInput" type="password" inputmode="numeric" maxlength="12" placeholder="PIN"
           style="font-size:1.3rem;text-align:center;padding:10px 16px;width:160px;background:#0b1020;border:1px solid #4a90d9;color:#e6edf3;border-radius:8px;letter-spacing:6px">
    <button id="pinBtn" style="font-size:1rem;padding:10px 24px;background:#4a90d9;color:#041018;border:0;border-radius:8px;cursor:pointer">UNLOCK</button>
    <div id="pinMsg" style="color:#e5534b;font-size:0.8rem;min-height:1em"></div>
  </div>
  <img id="avatar" src="/avatar.jpg" alt="WHIP avatar">
  <h1>WHIP</h1>
  <h2>Hexapod Control</h2>
  <a class="mode-btn keys-link" href="/keys" target="whip_keys">KEYS</a>

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
    <div class="mode-btn" id="modeFollow"   onclick="setMode('follow')" title="F: he keeps your hand 30 cm from his sonar">FOLLOW</div>
    <div class="mode-btn" id="modeMind"     onclick="setMode('mind')" style="display:none" title="the LLM on his PC controller drives him">MIND</div>
  </div>

  <div id="gaitBox" class="sensor-box">
    <div class="gait-row">
      <label>Gait <select id="gaitSel" title="G: next gait, 1-6: tripod / ripple / wave / quad / amble / metachronal"></select></label>
      <div class="mode-btn active" id="tiltBtn" title="T: tilt safety on / off (on again at every boot)">TILT ON</div>
      <div class="mode-btn active" id="soundBtn" title="B: buzzer on / off (on again at every boot)">SOUND</div>
      <div class="mode-btn" id="weaveBtn" title="WEAVE: in obstacle mode he weaves left and right as he walks, so the sonar sees past his legs, and stops at 48 cm (off again at every boot)">WEAVE</div>
      <div class="mode-btn" id="antiSlipBtn" title="ANTI-SLIP: a foot that lands takes his weight a moment before the next ones lift (tripod: all six down), same stride, slower (saved)">ANTI-SLIP</div>
      <label title="FOOT LEAD: while a leg steps, its foot servo runs ahead of the thigh and gets there first. 0 = pure IK, 9 = the foot there at about half the step (saved)">Foot lead <select id="footLeadSel"></select></label>
      <div class="mode-btn" id="wallGaitBtn" title="GAIT PER WALL: in obstacle mode he walks on in the next gait after every wall (saved)">GAIT PER WALL</div>
      <div class="mode-btn" id="autoStartBtn" title="AUTO-START: at boot he stands up and walks off in obstacle avoidance. Off: every servo stays at 1500 us until you give him input (saved, from the next boot)">AUTO-START</div>
      <div class="mode-btn" id="levelBtn" title="LEVEL: standing on a slope he tilts his body against it, feet planted (off again at every boot)">LEVEL</div>
      <div class="mode-btn" id="testBtn" title="TEST-GAIT: walk on hips and thighs only, every joint at 1500 us on the ground (off again at every boot)">TEST-GAIT</div>
    </div>
    <div class="gait-row" title="CRAB OFFSET: while he crabs, his back feet land this many degrees in on their hips, towards the middle legs. Walking forward they stay where they are, further back to hold his weight (saved)">
      <label for="crabBackRange">Crab offset</label>
      <input type="range" id="crabBackRange" min="0" max="15" step="1" style="flex:1">
      <span id="crabBackVal"></span>
    </div>
    <canvas id="gaitAnim" width="300" height="120"></canvas>
    <div id="gaitNote"></div>
  </div>

  <div id="body3dBox" class="sensor-box" title="WHIP as he stands right now: drag to turn the view">
    <div class="gait-row"><span>WHIP in 3D</span><span id="body3dInfo" style="font-size:0.75rem"></span></div>
    <canvas id="body3d" width="300" height="230" style="width:100%;max-width:300px;touch-action:none;cursor:grab"></canvas>
  </div>

  <div id="actRow">
    <div class="mode-btn" id="actHello" title="H: he waves hello">HELLO</div>
    <div class="mode-btn" id="actDance" title="N: a little dance">DANCE</div>
    <div class="mode-btn" id="actRest"  title="R: lie down / get up (he lies down by himself after a minute standing still)">REST</div>
  </div>

  <div class="dpad">
    <div class="btn fw"    id="btnFw">&#8593;</div>
    <div class="btn turnL" id="btnTurnL">&#8630;</div>
    <div class="btn stop stop-btn" id="btnStop">STOP</div>
    <div class="btn turnR" id="btnTurnR">&#8631;</div>
    <div class="btn bw"    id="btnBw">&#8595;</div>
    <div class="btn left"  id="btnSL" title="crab left">&#8676;</div>
    <div class="btn right" id="btnSR" title="crab right">&#8677;</div>
  </div>

  <div id="status">mode: -- | move: -- | wifi: --</div>

  <a href="/calib" style="color:var(--accent);font-size:0.85rem">CALIBRATION</a>
  <div id="shutdownBtn" onclick="doShutdown()">SHUTDOWN</div>

  <script>
    // --- Control lock: PIN unlock before driving ---
    var unlocked = false;
    function doUnlock() {
      var pin = document.getElementById('pinInput').value;
      fetch('/unlock?pw=' + encodeURIComponent(pin))
        .then(function (r) { if (!r.ok) throw new Error('bad'); return r.json(); })
        .then(function () {
          unlocked = true;
          document.getElementById('lockOverlay').style.display = 'none';
          document.getElementById('pinMsg').textContent = '';
        })
        .catch(function () { document.getElementById('pinMsg').textContent = 'Wrong PIN'; });
    }
    function showLock() {
      unlocked = false;
      var ov = document.getElementById('lockOverlay');
      if (ov) ov.style.display = 'flex';
    }
    // 403 from a driving endpoint means the server locked us out: pop the PIN.
    function driveFetch(url) {
      return fetch(url).then(function (r) { if (r.status === 403) showLock(); return r; })
                       .catch(function () {});
    }
    document.getElementById('pinBtn').addEventListener('click', doUnlock);
    document.getElementById('pinInput').addEventListener('keydown', function (e) {
      if (e.key === 'Enter') doUnlock();
    });

    let repeatTimer = null;
    let curMode = 'obstacle';

    function sendCmd(c) {
      driveFetch('/web?cmd=' + c);
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
    bindHold('btnSL',    'sleft');
    bindHold('btnSR',    'sright');

    document.getElementById('btnStop').addEventListener('click', function () {
      release(null);
      sendCmd('stop');
    });

    function setMode(m) {
      driveFetch('/mode?m=' + m);
      showMode(m);
    }

    function showMode(m) {
      curMode = m;
      document.getElementById('modeObstacle').classList.toggle('active', m === 'obstacle');
      document.getElementById('modeIr').classList.toggle('active', m === 'ir');
      document.getElementById('modeWeb').classList.toggle('active', m === 'web');
      document.getElementById('modeMind').classList.toggle('active', m === 'mind');
      document.getElementById('modeFollow').classList.toggle('active', m === 'follow');      document.querySelectorAll('.dpad .btn').forEach(function (b) {
        b.classList.toggle('disabled', m !== 'web');
      });
    }

    function doShutdown() {
      if (confirm('Shutdown WHIP? This cannot be undone without a power cycle.')) {
        driveFetch('/shutdown');
      }
    }

    const MODE_NAMES  = ['OBSTACLE', 'IR', 'WEB', 'MIND', 'FOLLOW'];
    const MODE_KEYS   = ['obstacle', 'ir', 'web', 'mind', 'follow'];
    const MOVE_NAMES  = ['STAND', 'WALK', 'BACKWARD', 'TURN L', 'TURN R'];
    const WIFI_NAMES  = ['JOINED', 'HOSTING'];


    // Servo Studio holds him: say so (everything here is refused until it lets go)
    var studioBar = document.createElement('div');
    studioBar.textContent = 'SERVO STUDIO HAS HIM: everything else waits until it lets go';
    studioBar.style.cssText = 'display:none;position:fixed;top:0;left:0;right:0;z-index:10000;padding:8px;' +
      'text-align:center;font-weight:bold;letter-spacing:1px;background:#b3262e;color:#fff';
    document.body.appendChild(studioBar);
    function studioNote(on) { studioBar.style.display = on ? '' : 'none'; }

    function updateStatus() {
      fetch('/status').then(function (r) { return r.json(); }).then(function (j) {
        studioNote(j.studio);
        document.getElementById('sDist').textContent = j.distance >= 0 ? j.distance : '--';
        document.getElementById('sTilt').textContent = Math.max(Math.abs(j.pitch), Math.abs(j.roll)).toFixed(1);
        document.getElementById('modeMind').style.display = j.mindHost ? '' : 'none';   // only while his controller can think
        if (j.gait !== undefined && j.gait !== gaitNow && document.activeElement !== gaitSel) showGait(j.gait);
        if (j.speed !== undefined) speedNow = j.speed;
        if (j.tiltGuard !== undefined && !!j.tiltGuard !== tiltOn) showTilt(!!j.tiltGuard);
        if (j.sound !== undefined && !!j.sound !== soundOn) showSound(!!j.sound);
        if (j.testGait !== undefined && !!j.testGait !== testOn) showTest(!!j.testGait);
        if (j.weave !== undefined && !!j.weave !== weaveOn) showWeave(!!j.weave);
        if (j.level !== undefined) document.getElementById('levelBtn').classList.toggle('active', !!j.level);
        if (j.antiSlip !== undefined && !!j.antiSlip !== antiSlipOn) showAntiSlip(!!j.antiSlip);
        if (j.autoStart !== undefined && !!j.autoStart !== autoStartOn) showAutoStart(!!j.autoStart);
        if (j.footLead !== undefined && document.activeElement !== footLeadSel) footLeadSel.value = j.footLead;
        if (j.crabBack !== undefined && document.activeElement !== crabBackRange) showCrabBack(j.crabBack);
        if (j.wallGait !== undefined && !!j.wallGait !== wallGaitOn) showWallGait(!!j.wallGait);
        document.getElementById('status').textContent =
          'mode: ' + MODE_NAMES[j.controlMode] +
          ' | move: ' + MOVE_NAMES[j.move] +
          ' | wifi: ' + WIFI_NAMES[j.wifiMode] + ' ' + j.network +
          (j.sysState === 3 ? ' | PICKED UP' : '') + (j.resting ? ' | resting' : '');
        document.getElementById('actRest').textContent = j.resting ? 'WAKE' : 'REST';
        document.getElementById('actRest').classList.toggle('active', !!j.resting);
        if (MODE_KEYS[j.controlMode] && MODE_KEYS[j.controlMode] !== curMode) showMode(MODE_KEYS[j.controlMode]);
      }).catch(function(){});
    }

    // --- Gaits: the dropdown, a little animation of the one picked, keys G / 1 2 3, and T for tilt ---
    const GAITS = [   // as gaits.h: sixths of the cycle a foot is in the air, and when each leg lifts (LF LM LB RF RM RB)
      { name: 'TRIPOD', air: 3, lift: [0, 3, 0, 3, 0, 3], note: '3 feet down, two groups of 3 by turns: fastest' },
      { name: 'RIPPLE', air: 2, lift: [4, 2, 0, 1, 5, 3], note: '4 feet down, a step runs front to back each side' },
      { name: 'WAVE',   air: 1, lift: [2, 1, 0, 5, 4, 3], note: '5 feet down, one leg at a time: slowest, easiest on the servos' },
      { name: 'QUAD',   air: 2, lift: [4, 2, 0, 2, 0, 4], note: '4 feet down, 3 pairs step by turns: between tripod and ripple' },
      { name: 'AMBLE',  air: 2, lift: [4, 0, 2, 2, 4, 0], note: '4 feet down, diagonal pairs (none straight across): spreads the load' },
      { name: 'METACHRONAL', air: 1, lift: [2, 1, 0, 3, 4, 5], note: '5 feet down, one leg at a time round the body: a Mexican wave' }
    ];
    const LEG_NAMES = ['LF', 'LM', 'LB', 'RF', 'RM', 'RB'];
    const LEG_HOME  = [[150, 123], [0, 177], [-150, 123], [150, -123], [0, -177], [-150, -123]];   // mm, x forward, y left
    const LEG_HIP   = [[74, 39], [0, 64], [-74, 39], [74, -39], [0, -64], [-74, -39]];
    const BODY      = [[74, 39], [0, 64], [-74, 39], [-74, -39], [0, -64], [74, -39]];
    const STRIDE    = 45;
    let gaitNow = 0, speedNow = 5, tiltOn = true;

    const gaitSel = document.getElementById('gaitSel');
    GAITS.forEach(function (g, i) {
      const o = document.createElement('option');
      o.value = i; o.textContent = g.name;
      gaitSel.appendChild(o);
    });
    function showGait(i) {
      gaitNow = i;
      gaitSel.value = i;
      document.getElementById('gaitNote').textContent = GAITS[i].note;
    }
    function setGait(i) {
      showGait((i + GAITS.length) % GAITS.length);
      driveFetch('/gait?gait=' + gaitNow);
    }
    gaitSel.addEventListener('change', function () { setGait(+gaitSel.value); });

    function showTilt(on) {
      tiltOn = on;
      const b = document.getElementById('tiltBtn');
      b.textContent = on ? 'TILT ON' : 'TILT OFF';
      b.classList.toggle('active', on);
      b.classList.toggle('off', !on);
    }
    function toggleTilt() {
      showTilt(!tiltOn);
      driveFetch('/tilt?on=' + (tiltOn ? 1 : 0));
    }
    document.getElementById('tiltBtn').addEventListener('click', toggleTilt);

    let soundOn = true;
    function showSound(on) {
      soundOn = on;
      const b = document.getElementById('soundBtn');
      b.textContent = on ? 'SOUND' : 'MUTED';
      b.classList.toggle('active', on);
    }
    function toggleSound() {
      showSound(!soundOn);
      driveFetch('/buzzer?on=' + (soundOn ? 1 : 0));
    }
    document.getElementById('soundBtn').addEventListener('click', toggleSound);

    document.getElementById('levelBtn').addEventListener('click', function () {
      const on = !this.classList.contains('active');
      this.classList.toggle('active', on);
      driveFetch('/gait?level=' + (on ? 1 : 0));
    });

    let weaveOn = false;
    function showWeave(on) {
      weaveOn = on;
      document.getElementById('weaveBtn').classList.toggle('active', on);
    }
    document.getElementById('weaveBtn').addEventListener('click', function () {
      showWeave(!weaveOn);
      driveFetch('/gait?weave=' + (weaveOn ? 1 : 0));
    });

    let antiSlipOn = false;
    function showAntiSlip(on) {
      antiSlipOn = on;
      document.getElementById('antiSlipBtn').classList.toggle('active', on);
    }
    document.getElementById('antiSlipBtn').addEventListener('click', function () {
      showAntiSlip(!antiSlipOn);
      driveFetch('/gait?antislip=' + (antiSlipOn ? 1 : 0));
    });

    const footLeadSel = document.getElementById('footLeadSel');
    for (let i = 0; i <= 9; i++) footLeadSel.add(new Option(i === 0 ? '0 (IK)' : String(i), i));
    footLeadSel.addEventListener('change', function () { driveFetch('/gait?footlead=' + footLeadSel.value); });

    const crabBackRange = document.getElementById('crabBackRange');
    function showCrabBack(v) {
      crabBackRange.value = v;
      document.getElementById('crabBackVal').textContent = v + '°';
    }
    crabBackRange.addEventListener('input', function () { showCrabBack(crabBackRange.value); });
    crabBackRange.addEventListener('change', function () {
      driveFetch('/gait?crabback=' + crabBackRange.value);
      crabBackRange.blur();   // /status keeps it in step again, from any controller
    });

    let wallGaitOn = false;
    function showWallGait(on) {
      wallGaitOn = on;
      document.getElementById('wallGaitBtn').classList.toggle('active', on);
    }
    document.getElementById('wallGaitBtn').addEventListener('click', function () {
      showWallGait(!wallGaitOn);
      driveFetch('/gait?wallgait=' + (wallGaitOn ? 1 : 0));
    });

    let autoStartOn = false;
    function showAutoStart(on) {
      autoStartOn = on;
      document.getElementById('autoStartBtn').classList.toggle('active', on);
    }
    document.getElementById('autoStartBtn').addEventListener('click', function () {
      showAutoStart(!autoStartOn);
      driveFetch('/gait?autostart=' + (autoStartOn ? 1 : 0));
    });

    let testOn = false;
    function showTest(on) {
      testOn = on;
      document.getElementById('testBtn').classList.toggle('active', on);
      if (on) document.getElementById('gaitNote').textContent = 'TEST-GAIT: hips and thighs only, every joint at 1500 us on the ground';
      else showGait(gaitNow);
    }
    document.getElementById('testBtn').addEventListener('click', function () {
      showTest(!testOn);
      driveFetch('/testgait?on=' + (testOn ? 1 : 0));
    });

    function act(a) { driveFetch('/act?do=' + a); }
    document.getElementById('actHello').addEventListener('click', function () { act('hello'); });
    document.getElementById('actDance').addEventListener('click', function () { act('dance'); });
    document.getElementById('actRest').addEventListener('click', function () { act('rest'); });

    document.addEventListener('keydown', function (e) {
      if (e.repeat || e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
      const k = e.key.toLowerCase();
      if (k === 'g') setGait(gaitNow + 1);
      else if (k >= '1' && k <= '6') setGait(+k - 1);
      else if (k === 't') toggleTilt();
      else if (k === 'b') toggleSound();
      else if (k === 'h') act('hello');
      else if (k === 'n') act('dance');
      else if (k === 'r') act('rest');
      else if (k === 'f') setMode('follow');
    });

    // Where a foot is, phase 0..1 through the cycle: [mm ahead of home, how high 0..1]
    function legAt(g, leg, ph) {
      const air = g.air / 6, k = ((ph - g.lift[leg] / 6) % 1 + 1) % 1;
      if (k < air) {
        const u = k / air;
        return [-STRIDE + 2 * STRIDE * u * u * (3 - 2 * u), Math.sin(Math.PI * u)];
      }
      return [STRIDE - 2 * STRIDE * (k - air) / (1 - air), 0];
    }

    const cv = document.getElementById('gaitAnim'), ctx = cv.getContext('2d');
    const css = getComputedStyle(document.documentElement);
    const C = {};
    ['--accent', '--border', '--text-dim', '--text-val', '--green'].forEach(function (n) { C[n] = css.getPropertyValue(n).trim(); });

    function drawGait(ts) {
      const g = GAITS[gaitNow];
      const periodMs = (60 / g.air) * (120 - speedNow * 10);   // his cycle: ticks * ms per tick
      const ph = (ts % periodMs) / periodMs;
      const W = cv.width, H = cv.height;
      ctx.clearRect(0, 0, W, H);

      // WHIP from above, head up: lifted feet light up and grow
      const bx = 62, by = H / 2, sc = 0.26;
      function P(x, y) { return [bx - y * sc, by - x * sc]; }
      ctx.fillStyle = C['--border'];
      ctx.beginPath();
      BODY.forEach(function (p, i) { const q = P(p[0], p[1]); if (i) ctx.lineTo(q[0], q[1]); else ctx.moveTo(q[0], q[1]); });
      ctx.closePath(); ctx.fill();
      const head = P(92, 0);
      ctx.fillStyle = C['--accent'];
      ctx.beginPath(); ctx.arc(head[0], head[1], 4, 0, 2 * Math.PI); ctx.fill();
      for (let i = 0; i < 6; i++) {
        const s = legAt(g, i, ph), hip = P(LEG_HIP[i][0], LEG_HIP[i][1]), foot = P(LEG_HOME[i][0] + s[0], LEG_HOME[i][1]);
        ctx.strokeStyle = s[1] > 0 ? C['--accent'] : C['--text-dim'];
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(hip[0], hip[1]); ctx.lineTo(foot[0], foot[1]); ctx.stroke();
        ctx.fillStyle = s[1] > 0 ? C['--green'] : C['--text-dim'];
        ctx.beginPath(); ctx.arc(foot[0], foot[1], 3 + 3 * s[1], 0, 2 * Math.PI); ctx.fill();
      }

      // the step chart: one row a leg, a bar while that foot is in the air
      const x0 = 150, x1 = W - 6, top = 6, rowH = (H - 12) / 6;
      ctx.font = '10px Courier New'; ctx.textBaseline = 'middle';
      for (let i = 0; i < 6; i++) {
        const y = top + i * rowH;
        ctx.fillStyle = C['--text-dim'];
        ctx.fillText(LEG_NAMES[i], x0 - 22, y + rowH / 2);
        ctx.strokeStyle = C['--border']; ctx.lineWidth = 1;
        ctx.strokeRect(x0, y + 2, x1 - x0, rowH - 4);
        ctx.fillStyle = legAt(g, i, ph)[1] > 0 ? C['--green'] : C['--accent'];
        for (let w = -1; w <= 0; w++) {   // the bar can wrap round the end of the cycle
          const a = g.lift[i] / 6 + w, b = a + g.air / 6;
          const ca = Math.max(a, 0), cb = Math.min(b, 1);
          if (cb > ca) ctx.fillRect(x0 + ca * (x1 - x0), y + 3, (cb - ca) * (x1 - x0), rowH - 6);
        }
      }
      ctx.strokeStyle = C['--text-val']; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(x0 + ph * (x1 - x0), top); ctx.lineTo(x0 + ph * (x1 - x0), H - top); ctx.stroke();
      requestAnimationFrame(drawGait);
    }
    showGait(0);
    requestAnimationFrame(drawGait);

    // --- WHIP in 3D: his joint angles (/pose) through his own leg geometry (ik_gait.h, the reverse of
    // solve: as whip_body.joints), drawn from a camera you turn by dragging; the ground, the support
    // polygon under the feet that are down, and his centre over it. Plain canvas, no libraries.
    var B3 = { HK: 28, TH: 84, FT: 127, TIP: 14,
               ANCHOR: [[74, 39], [0, 64], [-74, 39], [74, -39], [0, -64], [-74, -39]],
               MOUNT: [27.8, 90, 152.2, -27.8, -90, -152.2],   // each leg at 1500 us (ik_gait.h MOUNT_DEG, Jakob's)
               BODY: [[74, 39], [0, 64], [-74, 39], [-74, -39], [0, -64], [74, -39]],
               az: -0.65, el: 0.55, pose: null };
    var c3 = document.getElementById('body3d'), g3 = c3.getContext('2d');
    function legPoints(l, hip, knee, foot) {
      var ax = B3.ANCHOR[l][0], ay = B3.ANCHOR[l][1], R = Math.PI / 180;
      var yaw = (B3.MOUNT[l] + hip) * R, c = Math.cos(yaw), s = Math.sin(yaw);
      var kx = ax + B3.HK * c, ky = ay + B3.HK * s, k = knee * R;
      var ah = B3.TH * Math.cos(k), az = B3.TH * Math.sin(k);
      var f = k + Math.PI + (foot + 90 + B3.TIP) * R;
      var fh = ah + B3.FT * Math.cos(f), fz = az + B3.FT * Math.sin(f);
      return [[ax, ay, 0], [kx, ky, 0], [kx + ah * c, ky + ah * s, az], [kx + fh * c, ky + fh * s, fz]];
    }
    function toWorld(p, b) {   // R p + t, R = Rz(yaw) Ry(pitch) Rx(roll)
      var R = Math.PI / 180, cr = Math.cos(b[3] * R), sr = Math.sin(b[3] * R), cp = Math.cos(b[4] * R),
          sp = Math.sin(b[4] * R), cy = Math.cos(b[5] * R), sy = Math.sin(b[5] * R), x = p[0], y = p[1], z = p[2];
      return [cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z + b[0],
              sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z + b[1],
              -sp * x + cp * sr * y + cp * cr * z + b[2]];
    }
    function proj(p) {
      var ca = Math.cos(B3.az), sa = Math.sin(B3.az), ce = Math.cos(B3.el), se = Math.sin(B3.el);
      var xr = p[0] * ca - p[1] * sa, yr = p[0] * sa + p[1] * ca, k = 0.62;
      return [c3.width / 2 - yr * k, c3.height * 0.42 - (p[2] * ce + xr * se) * k];
    }
    function hull(pts) {
      pts = pts.slice().sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
      if (pts.length < 3) return pts;
      var cr = function (o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); };
      var lo = [], up = [];
      pts.forEach(function (q) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); });
      pts.slice().reverse().forEach(function (q) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); });
      return lo.slice(0, -1).concat(up.slice(0, -1));
    }
    function draw3d() {
      var P = B3.pose, css = getComputedStyle(document.documentElement);
      var acc = css.getPropertyValue('--accent').trim(), acc2 = css.getPropertyValue('--accent2').trim() || '#1A7BD9';
      g3.clearRect(0, 0, c3.width, c3.height);
      if (!P) return;
      var legs = P.a.map(function (a, l) { return legPoints(l, a[0], a[1], a[2]).map(function (q) { return toWorld(q, P.b); }); });
      var gz = Math.min.apply(null, legs.map(function (pts) { return pts[3][2]; }));
      g3.strokeStyle = 'rgba(160,160,180,0.12)'; g3.lineWidth = 1;
      for (var k = -5; k <= 5; k++) {
        [[[k * 50, -250, gz], [k * 50, 250, gz]], [[-250, k * 50, gz], [250, k * 50, gz]]].forEach(function (ln) {
          var a = proj(ln[0]), b = proj(ln[1]); g3.beginPath(); g3.moveTo(a[0], a[1]); g3.lineTo(b[0], b[1]); g3.stroke();
        });
      }
      var down = legs.filter(function (pts) { return pts[3][2] <= gz + 4; }).map(function (pts) { return [pts[3][0], pts[3][1]]; });
      var h = hull(down);
      if (h.length >= 3) {
        g3.beginPath(); h.forEach(function (q, i) { var s = proj([q[0], q[1], gz]); i ? g3.lineTo(s[0], s[1]) : g3.moveTo(s[0], s[1]); });
        g3.closePath(); g3.fillStyle = 'rgba(26,123,217,0.22)'; g3.fill(); g3.strokeStyle = acc2; g3.stroke();
      }
      var cog = proj([P.b[0], P.b[1], gz]), mid = proj(toWorld([0, 0, 0], P.b));
      g3.strokeStyle = acc2; g3.beginPath(); g3.moveTo(mid[0], mid[1]); g3.lineTo(cog[0], cog[1]); g3.stroke();
      g3.fillStyle = acc2; g3.beginPath(); g3.arc(cog[0], cog[1], 4, 0, 7); g3.fill();
      g3.beginPath();
      B3.BODY.forEach(function (q, i) { var s = proj(toWorld([q[0], q[1], 0], P.b)); i ? g3.lineTo(s[0], s[1]) : g3.moveTo(s[0], s[1]); });
      g3.closePath(); g3.fillStyle = 'rgba(' + css.getPropertyValue('--accent-rgb').trim() + ',0.3)'; g3.fill();
      g3.strokeStyle = acc; g3.lineWidth = 2; g3.stroke();
      legs.forEach(function (pts) {
        g3.strokeStyle = acc; g3.lineWidth = 3; g3.beginPath();
        pts.forEach(function (q, i) { var s = proj(q); i ? g3.lineTo(s[0], s[1]) : g3.moveTo(s[0], s[1]); });
        g3.stroke();
        pts.forEach(function (q, i) { var s = proj(q); g3.fillStyle = i == 3 ? '#F2E9EA' : acc; g3.beginPath(); g3.arc(s[0], s[1], i == 3 ? 3 : 2.5, 0, 7); g3.fill(); });
      });
      document.getElementById('body3dInfo').textContent = P.bm ? 'BODY mode' : (P.walk ? 'walking' : 'standing');
    }
    (function () {   // drag to turn the view
      var drag = null;
      c3.addEventListener('pointerdown', function (e) { drag = [e.clientX, e.clientY, B3.az, B3.el]; c3.setPointerCapture(e.pointerId); });
      c3.addEventListener('pointermove', function (e) {
        if (!drag) return;
        B3.az = drag[2] + (e.clientX - drag[0]) * 0.01;
        B3.el = Math.max(0.1, Math.min(1.45, drag[3] + (e.clientY - drag[1]) * 0.01));
        draw3d();
      });
      c3.addEventListener('pointerup', function () { drag = null; });
    })();
    function updatePose() {
      fetch('/pose').then(function (r) { return r.json(); }).then(function (j) { B3.pose = j; draw3d(); }).catch(function () {});
    }
    updatePose();
    setInterval(updatePose, 300);

    showMode('obstacle');   // the status poll shows his real mode
    updateStatus();
    setInterval(updateStatus, 500);
  </script>
</body>
</html>
)rawhtml";

// ---------------------------------------------------------------------------
// /calib -- line the IK up with WHIP's servos (page in calib_page.h)
// ---------------------------------------------------------------------------
char calibPose = 'h';   // the pose /calib is showing: z(ero) / h(ome) / s(leep)

void calibShow(char pose) {
  wakeUp();
  calibPose = pose;
  if (pose == 'z') {
    ik::zeroLine(cmdBuf, sizeof(cmdBuf));
    clampCommand(cmdBuf, safeBuf, sizeof(safeBuf));
    Serial.print(safeBuf); Serial.print("T600\r\n");
  } else {
    ikSend(pose == 's' ? ik::sleepPose() : ik::home(), 600);
  }
}

bool calibArgs(uint8_t& leg, uint8_t& joint) {
  leg = server.arg("leg").toInt();
  joint = server.arg("joint").toInt();
  return server.hasArg("leg") && server.hasArg("joint") && leg < ik::LEGS && joint < ik::JOINTS;
}

// Every /calib action drives servos: only the unlocked controller
bool calibAllowed() {
  if (isController()) return true;
  server.send(403, "application/json", "{\"ok\":false,\"error\":\"unlock first\"}");
  return false;
}

void calibState() {
  String j = "{\"calibrating\":" + String(calibrating ? 1 : 0) + ",\"calibrated\":" + String(ik::calibrated ? 1 : 0) +
             ",\"height\":" + String(ik::height) + ",\"speed\":" + String(ik::speed) + ",\"gait\":" + String(ik::gait) +
             ",\"crabBack\":" + String(ik::crabBack) +
             ",\"pose\":\"" + String(calibPose) + "\",\"legs\":[";
  for (uint8_t l = 0; l < ik::LEGS; l++) {
    j += String(l ? "," : "") + "{\"name\":\"" + ik::LEG_NAMES[l] + "\",\"servos\":[";
    for (uint8_t k = 0; k < ik::JOINTS; k++) {
      const ik::ServoCal& c = ik::cal[l][k];
      j += String(k ? "," : "") + "{\"joint\":\"" + ik::JOINT_NAMES[k] + "\",\"ch\":" + c.ch +
           ",\"rev\":" + (c.rev ? 1 : 0) + ",\"trim\":" + c.trim + "}";
    }
    j += "]}";
  }
  server.send(200, "application/json", j + "]}");
}

void setupCalibRoutes() {
  server.on("/calib", []() { server.send_P(200, "text/html", CALIB_HTML); });
  server.on("/calib/state", calibState);
  server.on("/calib/start", []() {            // stop walking and hold still
    if (!calibAllowed()) return;
    calibrating = true;
    abortMovement = true;
    webActiveCmd = 0;
    currentMove = MOVE_STAND;
    calibShow('h');
    calibState();
  });
  server.on("/calib/done", []() {
    if (!calibAllowed()) return;
    calibrating = false;
    ikNow = ik::home();
    standNow();
    calibState();
  });
  server.on("/calib/pose", []() {             // ?p=zero|home|sleep
    if (!calibAllowed()) return;
    if (!calibrating) { server.send(409, "application/json", "{\"ok\":false,\"error\":\"press START first\"}"); return; }
    String p = server.arg("p");
    calibShow(p == "zero" ? 'z' : p == "sleep" ? 's' : 'h');
    calibState();
  });
  server.on("/calib/set", []() {              // ?leg=&joint=&ch=&rev=&trim=, shown at once
    if (!calibAllowed()) return;
    uint8_t leg, joint;
    if (!calibrating || !calibArgs(leg, joint)) { server.send(400, "application/json", "{\"ok\":false}"); return; }
    ik::ServoCal& c = ik::cal[leg][joint];
    if (server.hasArg("ch"))   c.ch   = constrain(server.arg("ch").toInt(), 0, 32);
    if (server.hasArg("rev"))  c.rev  = server.arg("rev") == "1";
    if (server.hasArg("trim")) c.trim = constrain(server.arg("trim").toInt(), -400, 400);
    if (!studioHolds()) calibShow(calibPose);   // the studio moves the servos itself
    calibState();
  });
  // /servo?l=<line>: a line straight to the servo controller (Servo Studio): "#1P1500#2P1600T300",
  // a stored group "G1F3", or "~ST" / "~RE" (RTrobot's commands). He holds still meanwhile (as on
  // /calib: DONE there, or /calib/done, gives him back). Pulses are clamped as everything he sends.
  server.on("/servo", []() {
    if (!calibAllowed()) return;
    const String l = server.arg("l");
    if (!(l.startsWith("#") || l.startsWith("G") || l == "~ST" || l == "~RE")) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"use l=#1P1500T300, G1F1, ~ST or ~RE\"}");
      return;
    }
    calibrating = true; abortMovement = true; webActiveCmd = 0; currentMove = MOVE_STAND;
    static char out[320];
    clampCommand(l.c_str(), out, sizeof out);
    Serial.print(out);
    Serial.print("\r\n");
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/calib/wiggle", []() {           // that one servo 15 degrees out and back: which leg is it?
    if (!calibAllowed()) return;
    uint8_t leg, joint;
    if (!calibrating || !calibArgs(leg, joint)) { server.send(400, "application/json", "{\"ok\":false}"); return; }
    const ik::ServoCal& c = ik::cal[leg][joint];
    for (int d : { 15, 0, 15, 0 }) {
      Serial.printf("#%uP%dT250\r\n", c.ch, constrain(ik::toUs(leg, joint, d), 500, 2500));
      delay(300);
    }
    calibShow(calibPose);
    calibState();
  });
  server.on("/calib/save", []() {
    if (!calibAllowed()) return;
    ik::calibrated = true;
    ik::save();
    calibState();
  });
  server.on("/calib/defaults", []() {
    if (!calibAllowed()) return;
    ik::loadDefaults();
    if (calibrating && !studioHolds()) calibShow(calibPose);
    calibState();
  });
  server.on("/gait", []() {                    // ?gait=0-5 (tripod / ripple / wave / quad / amble / metachronal) &height=0-9 &speed=0-9 &weave=0|1 &antislip=0|1 &autostart=0|1 &footlead=0-9 &wallgait=0|1 &crabback=0-15 (or &crabstep=+-1), saved
    if (!calibAllowed()) return;
    if (server.hasArg("weave")) {   // obstacle mode weaving: no re-stand for that
      ik::weave = server.arg("weave") != "0";
      Serial2.println(ik::weave ? F("WEAVE ON") : F("WEAVE OFF"));
      if (!server.hasArg("gait") && !server.hasArg("height") && !server.hasArg("speed")) { ik::save(); calibState(); return; }
    }
    if (server.hasArg("antislip")) {   // feet planted before the next lift: walking, he steps to a stop and on again
      ik::antiSlip = server.arg("antislip") != "0";
      Serial2.println(ik::antiSlip ? F("ANTI-SLIP ON") : F("ANTI-SLIP OFF"));
      if (!server.hasArg("gait") && !server.hasArg("height") && !server.hasArg("speed")) { ik::save(); calibState(); return; }
    }
    if (server.hasArg("wallgait")) {   // obstacle mode: the next gait after every wall
      ik::wallGait = server.arg("wallgait") != "0";
      Serial2.println(ik::wallGait ? F("GAIT PER WALL ON") : F("GAIT PER WALL OFF"));
      ik::save(); calibState(); return;
    }
    if (server.hasArg("level")) {   // keep his body level on a slope, standing (off at every boot)
      if (server.arg("level") != "0") { levelOn = true; levelStart = -1; }
      else stopLevel();
      Serial2.println(levelOn ? F("LEVEL ON") : F("LEVEL OFF"));
      calibState();
      return;
    }
    if (server.hasArg("crabback") || server.hasArg("crabstep")) {   // back feet in while he crabs (a step up or down: RITA's buttons)
      const int v = server.hasArg("crabback") ? server.arg("crabback").toInt() : ik::crabBack + server.arg("crabstep").toInt();
      ik::crabBack = constrain(v, 0, ik::CRAB_BACK_MAX);
      Serial2.print(F("CRAB BACK IN -> ")); Serial2.println(ik::crabBack);
      ik::save(); calibState(); return;
    }
    if (server.hasArg("footlead")) {   // a stepping foot's servo ahead of its thigh: from the next step
      ik::footLead = constrain(server.arg("footlead").toInt(), 0, 9);
      Serial2.print(F("FOOT LEAD -> ")); Serial2.println(ik::footLead);
      if (!server.hasArg("gait") && !server.hasArg("height") && !server.hasArg("speed") && !server.hasArg("autostart")) { ik::save(); calibState(); return; }
    }
    if (server.hasArg("autostart")) {   // takes effect at the next boot
      ik::autoStart = server.arg("autostart") != "0";
      Serial2.println(ik::autoStart ? F("AUTO-START ON (next boot)") : F("AUTO-START OFF (next boot)"));
      if (!server.hasArg("gait") && !server.hasArg("height") && !server.hasArg("speed")) { ik::save(); calibState(); return; }
    }
    if (server.hasArg("gait")) {
      // he changes over while walking (gaitCycle steps to a stop in the old gait first)
      ik::gait = constrain(server.arg("gait").toInt(), 0, ik::GAITS - 1);
      Serial2.print(F("GAIT -> ")); Serial2.println(ik::GAIT_NAMES[ik::gait]);
      if (!server.hasArg("height") && !server.hasArg("speed")) { ik::save(); calibState(); return; }
    }
    if (server.hasArg("height")) ik::height = constrain(server.arg("height").toInt(), 0, 9);
    if (server.hasArg("speed"))  ik::speed  = constrain(server.arg("speed").toInt(), 0, 9);
    ik::save();
    abortMovement = true;            // re-stand at the new height
    currentMove = MOVE_STAND;
    if (calibrating && calibPose == 'h') calibShow('h');
    else if (!calibrating) { ikNow = ik::home(); standNow(); }
    calibState();
  });
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// The avatar lives in its own route so the page stays plain text (one place to swap it for an animation later)
void handleAvatar() {
  server.sendHeader("Cache-Control", "max-age=86400");
  server.send_P(200, "image/jpeg", (PGM_P)AVATAR_JPG, AVATAR_JPG_LEN);
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

  // DLPF ~21 Hz (CONFIG = 4): the servos' buzz and every footfall no longer read as tilt
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(4);
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
  // which way is down, standing (the picked-up check compares against it)
  mpuReadRaw();
  const float ax = rawAccX / 16384.0, ay = rawAccY / 16384.0, az = rawAccZ / 16384.0;
  const float g = sqrtf(ax * ax + ay * ay + az * az);
  if (g > 0.5f) { gravX = ax / g; gravY = ay / g; gravZ = az / g; }
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

  // pitch turns about Y (accX), roll about X (accY): each integrates its own gyro axis
  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gyroYdps * dt) + (1.0 - COMPLEMENTARY_ALPHA) * pitchAcc;
  rollDeg  = COMPLEMENTARY_ALPHA * (rollDeg  + gyroXdps * dt) + (1.0 - COMPLEMENTARY_ALPHA) * rollAcc;
}

// Picked up: his legs stop where they are (no thrashing in your hand) until he's been
// put down, level and still, for HELD_DOWN_MS. Then he stands up.
void enterHeld() {
  if (sysState == SYS_HELD) return;
  if (sysState == SYS_ACTIVE) controlModeBeforeTilt = controlMode;
  sysState      = SYS_HELD;
  abortMovement = true;
  gaitNow.running = false;
  Serial2.println(F("PICKED UP - legs still"));
}

void exitHeld() {
  sysState    = SYS_ACTIVE;
  controlMode = controlModeBeforeTilt;
  currentMove = MOVE_STAND;
  obstacleCount = 0;
  obstacleDetected = false;
  irActiveCmd  = 0;
  webActiveCmd = 0;
  if (!resting) pendingAct = 's';   // stand up, smoothly
  Serial2.println(F("PUT DOWN"));
}

void enterTiltSafe() {
  if (sysState != SYS_TILT_SAFE) {
    if (sysState == SYS_ACTIVE) controlModeBeforeTilt = controlMode;
    sysState      = SYS_TILT_SAFE;
    abortMovement = true;
    standNow();
    currentMove = MOVE_STAND;
    Serial2.println(F("TILT SAFE - holding stand pose"));
  }
}

void exitTiltSafe() {
  sysState    = SYS_ACTIVE;
  controlMode = controlModeBeforeTilt;
  standNow();
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

  updateTilt();   // keeps the angles (and the web page's tilt) current even when it's off
  if (!tiltGuard) {
    tiltOverSince = tiltClearSince = 0;
    if (sysState == SYS_TILT_SAFE) exitTiltSafe();
    if (sysState == SYS_HELD) exitHeld();
    return;
  }

  // Picked up? A hard lift or drop (far from 1 g), or upside down
  {
    static unsigned long oddSince = 0, calmSince = 0;
    const float ax = rawAccX / 16384.0, ay = rawAccY / 16384.0, az = rawAccZ / 16384.0;
    const float g = sqrtf(ax * ax + ay * ay + az * az);
    const float upright = g > 0.1f ? (ax * gravX + ay * gravY + az * gravZ) / g : 1;   // cos(angle to boot's down)
    const unsigned long t = millis();
    if (sysState != SYS_HELD) {
      const bool odd = g > HELD_G_HIGH || g < HELD_G_LOW || upright < 0;
      if (!odd) oddSince = 0;
      else if (!oddSince) oddSince = t;
      else if (t - oddSince >= HELD_MS && !calibrating) { enterHeld(); calmSince = 0; }
    } else {
      const bool calm = g > 0.85f && g < 1.15f && upright > 0.95f;
      if (!calm) calmSince = 0;
      else if (!calmSince) calmSince = t;
      else if (t - calmSince >= HELD_DOWN_MS) { exitHeld(); oddSince = 0; }
      return;   // held: the tilt check waits
    }
  }
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
void ShutdownSequence() {        // lie down, belly on the floor, legs spread (restDown())
  restDown();
  Serial2.println(F("Shutdown complete."));
}
