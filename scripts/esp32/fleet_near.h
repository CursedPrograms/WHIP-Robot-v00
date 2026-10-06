// fleet_near.h - which robots are nearby, from ESP-NOW signal strength.
//
// Every robot broadcasts a tiny "I'm here" beacon (DRFL + its name) twice a
// second over ESP-NOW, on the same WiFi channel as NORA's network, and listens
// for everyone else's. The signal strength (RSSI) of each robot's beacons,
// smoothed, says roughly how close it is:
//
//     above -45 dBm  very close      -45 .. -60  near
//     -60 .. -75     medium          below -75   far
//
// It's coarse: walls, bodies and antenna angle all change RSSI, so it's for
// "who's around and roughly how close", not distance. Precise collision
// avoidance stays with the ultrasonic / ToF sensors.
//
// Two radios feed the same list:
//   ESP-NOW   NORA, WHIP and MILA (MILA's ESP8266 only sends)
//   BLE       fleet_near_ble.h: NORA and WHIP beacon and scan over Bluetooth LE,
//             which is how they hear the Pi robots (KIDA-00, KIDA-01) and the
//             KIDAs hear them. Heard on both = a surer "it's really nearby".
//
//   fleetNearBegin("NORA", WIFI_IF_AP);   after WiFi is up (AP or STA interface)
//   fleetNearLoop();                      every loop
//   fleetNearJson()                       -> [{"name":"WHIP","rssi":-52,"zone":"near","via":"espnow+ble","age_ms":120}, ...]
//                                            nearest first
//   fleetNearStatusJson()                 -> {"self":"NORA","advice":"caution","yielding_to":"","robots":[...]}
//   fleetNearAdvice() / fleetGiveWay()    -> avoiding each other in self-driving modes (below)

#pragma once
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define FLEET_NEAR_MAX 12
#define FLEET_NEAR_BEACON_MS 500
#define FLEET_NEAR_FORGET_MS 5000

// What a robot is doing, sent in every beacon. RSSI rising looks the same from
// both sides ("approaching") even when only one of them moves, so the robots
// tell each other: one that's parked isn't the one closing in.
enum FleetState : uint8_t { FLEET_PARKED = 0, FLEET_DRIVING = 1, FLEET_YIELDING = 2, FLEET_USER = 3 };
static const char* const FLEET_STATE_NAMES[] = { "parked", "driving", "yielding", "user" };

// ...and over ESP-NOW they say it in Brainfuck, like the fleet's conversations:
// each beacon carries a program that prints the word, and the receiver runs it.
// RIFT decodes the same programs from the robots' /near. Made with RIFT's
// Fleet/brainfuck_talk.py compile_text(). (BLE adverts are too small for a
// program, so BLE sends the state as one byte - see fleet_near_ble.h.)
static const char* const FLEET_STATE_WORDS[] = { "park", "go", "wait", "hand" };
static const char* const FLEET_STATE_BF[] = {
  "++++++++[>++++++++++++++<-]>.---------------.<++++[>++++<-]>+.-------.",                 // park
  "++++++++[>+++++++++++++<-]>-.++++++++.",                                                // go
  "++++++++++[>++++++++++++<-]>-.<+++[>-------<-]>-.++++++++.+++++++++++.",                 // wait
  "++++++++[>+++++++++++++<-]>.-------.+++++++++++++.----------.",                          // hand
};
#define FLEET_BF_MAX 72

// A small Brainfuck interpreter: runs `prog`, writes what it prints to `out`.
// Bounded (tape, steps, output) so a garbled packet can't hang the robot.
static void fleetBfRun(const char* prog, char* out, int outLen) {
  uint8_t tape[32] = {0};
  int ptr = 0, o = 0, steps = 0, len = strnlen(prog, FLEET_BF_MAX);
  for (int pc = 0; pc < len && steps < 4000 && o < outLen - 1; pc++, steps++) {
    switch (prog[pc]) {
      case '+': tape[ptr]++; break;
      case '-': tape[ptr]--; break;
      case '>': ptr = (ptr + 1) & 31; break;
      case '<': ptr = (ptr + 31) & 31; break;
      case '.': out[o++] = (char)tape[ptr]; break;
      case '[':
        if (!tape[ptr]) for (int depth = 1; depth && ++pc < len;) depth += (prog[pc] == '[') - (prog[pc] == ']');
        break;
      case ']':
        if (tape[ptr]) for (int depth = 1; depth && --pc >= 0;) depth += (prog[pc] == ']') - (prog[pc] == '[');
        break;
    }
  }
  out[o] = 0;
}

static uint8_t fleetStateFromWord(const char* w) {
  for (uint8_t i = 0; i < 4; i++)
    if (strcmp(w, FLEET_STATE_WORDS[i]) == 0) return i;
  return FLEET_DRIVING;              // unknown word: assume it's moving, the careful choice
}

struct __attribute__((packed)) FleetBeacon {
  char magic[4];      // "DRFL"
  uint8_t version;    // 1
  char name[12];      // NUL-padded robot name
  uint16_t seq;
  char bf[FLEET_BF_MAX];   // NUL-terminated Brainfuck that prints its state word
};

#define FLEET_VIA_ESPNOW 1
#define FLEET_VIA_BLE 2

struct FleetPeer {
  bool used;
  char name[12];
  float rssi;         // smoothed, dBm
  unsigned long seenMs;
  unsigned long espnowMs, bleMs;   // when each radio last heard it (0 = never)
  uint8_t state;                   // what it says it's doing (FleetState)
  char bf[FLEET_BF_MAX];           // the program it last sent over ESP-NOW ("" if only heard on BLE)
  float rate;                      // how fast its RSSI changes, dB/s: + coming closer, - going away
  // per radio (0 ESP-NOW, 1 BLE): the two read a few dB apart, so each keeps its
  // own trend - mixing them would look like movement
  float viaRssi[2], viaRssiThen[2], viaRate[2];
  unsigned long viaThenMs[2];
};

// Coming or going: the signal getting stronger over time means it's closer.
// Above +1.5 dB/s (smoothed) = approaching, below -1.5 = leaving.
#define FLEET_TREND_DBS 1.5f

static const char* fleetTrend(const FleetPeer& p) {
  if (p.rate > FLEET_TREND_DBS) return "approaching";
  if (p.rate < -FLEET_TREND_DBS) return "leaving";
  return "steady";
}

static FleetPeer fleetPeers[FLEET_NEAR_MAX];
static char fleetSelf[12] = "";
static FleetState fleetSelfState = FLEET_PARKED;
enum FleetYieldPhase { FLEET_GO, FLEET_WAIT, FLEET_TURN };
static FleetYieldPhase fleetYieldPhase = FLEET_GO;

// What this robot tells the others: yielding while it gives way, otherwise what it's set to.
static FleetState fleetNearMyState() { return fleetYieldPhase != FLEET_GO ? FLEET_YIELDING : fleetSelfState; }

// Tell the others what this robot is doing (call whenever it changes; yielding is set automatically).
static void fleetNearSetState(FleetState s) { fleetSelfState = s; }
static uint16_t fleetSeq = 0;
static unsigned long fleetNextBeaconMs = 0;
static bool fleetNearOk = false;
static portMUX_TYPE fleetMux = portMUX_INITIALIZER_UNLOCKED;
static const uint8_t FLEET_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Optional: called for every beacon heard (name, raw rssi). The KIDA bridge prints these.
static void (*fleetNearOnBeacon)(const char* name, int rssi) = nullptr;

static const char* fleetZone(float rssi) {
  if (rssi > -45) return "very close";
  if (rssi > -60) return "near";
  if (rssi > -75) return "medium";
  return "far";
}

// One beacon heard, by either radio.
static void fleetNearRecord(const char* name, int rssi, int via, uint8_t state, const char* bf = nullptr) {
  if (!name[0] || strcmp(name, fleetSelf) == 0) return;
  portENTER_CRITICAL(&fleetMux);
  int slot = -1, freeSlot = -1;
  for (int i = 0; i < FLEET_NEAR_MAX; i++) {
    if (fleetPeers[i].used && strcmp(fleetPeers[i].name, name) == 0) { slot = i; break; }
    if (!fleetPeers[i].used && freeSlot < 0) freeSlot = i;
  }
  unsigned long t = millis();
  int v = via == FLEET_VIA_BLE ? 1 : 0;
  if (slot < 0 && freeSlot >= 0) {
    slot = freeSlot;
    FleetPeer& p = fleetPeers[slot];
    p.used = true;
    strcpy(p.name, name);
    p.rssi = rssi;
    p.rate = 0;
    for (int i = 0; i < 2; i++) { p.viaRssi[i] = p.viaRssiThen[i] = rssi; p.viaRate[i] = 0; p.viaThenMs[i] = 0; }
  }
  if (slot >= 0) {
    FleetPeer& p = fleetPeers[slot];
    p.rssi = 0.7f * p.rssi + 0.3f * rssi;                   // the zone: smoothed, both radios
    bool fresh = p.viaThenMs[v] == 0 || t - (v ? p.bleMs : p.espnowMs) > FLEET_NEAR_FORGET_MS;
    p.viaRssi[v] = fresh ? rssi : 0.7f * p.viaRssi[v] + 0.3f * rssi;
    if (fresh) {                                            // first reading from this radio (or back after a gap)
      p.viaRssiThen[v] = p.viaRssi[v];
      p.viaThenMs[v] = t;
      p.viaRate[v] = 0;
    } else if (t - p.viaThenMs[v] >= 1000) {                // the trend: dB per second over ~1 s, smoothed again
      float inst = (p.viaRssi[v] - p.viaRssiThen[v]) * 1000.0f / (t - p.viaThenMs[v]);
      p.viaRate[v] = 0.7f * p.viaRate[v] + 0.3f * inst;
      p.viaRssiThen[v] = p.viaRssi[v];
      p.viaThenMs[v] = t;
    }
    p.rate = p.viaRate[v];                                  // the radio that heard it last speaks for the trend
    p.state = state <= FLEET_USER ? state : FLEET_DRIVING;
    if (fresh && v == 0) p.bf[0] = 0;
    if (bf) { strncpy(p.bf, bf, FLEET_BF_MAX - 1); p.bf[FLEET_BF_MAX - 1] = 0; }
  }
  if (slot >= 0) {
    unsigned long now = millis();
    if (freeSlot == slot) fleetPeers[slot].espnowMs = fleetPeers[slot].bleMs = 0;
    fleetPeers[slot].seenMs = now;
    if (via == FLEET_VIA_ESPNOW) fleetPeers[slot].espnowMs = now;
    else fleetPeers[slot].bleMs = now;
  }
  portEXIT_CRITICAL(&fleetMux);
  if (fleetNearOnBeacon) fleetNearOnBeacon(name, rssi);
}

static void fleetNearRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(FleetBeacon)) return;
  const FleetBeacon* b = (const FleetBeacon*)data;
  if (memcmp(b->magic, "DRFL", 4) != 0 || b->version != 1) return;
  char name[12], bf[FLEET_BF_MAX], word[8];
  memcpy(name, b->name, 11);
  name[11] = 0;
  memcpy(bf, b->bf, FLEET_BF_MAX - 1);
  bf[FLEET_BF_MAX - 1] = 0;
  fleetBfRun(bf, word, sizeof(word));       // what it's saying, in Brainfuck
  fleetNearRecord(name, info->rx_ctrl ? info->rx_ctrl->rssi : -100, FLEET_VIA_ESPNOW, fleetStateFromWord(word), bf);
}

// ifx: WIFI_IF_AP when this board hosts the network (NORA), WIFI_IF_STA when it joined it.
static bool fleetNearBegin(const char* selfName, wifi_interface_t ifx) {
  strncpy(fleetSelf, selfName, sizeof(fleetSelf) - 1);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(fleetNearRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, FLEET_BROADCAST, 6);
  peer.channel = 0;                 // whatever channel the WiFi is on (NORA's)
  peer.ifidx = ifx;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(FLEET_BROADCAST) && esp_now_add_peer(&peer) != ESP_OK) return false;
  fleetNearOk = true;
  return true;
}

static void fleetNearLoop() {
  if (!fleetNearOk) return;
  unsigned long now = millis();
  if (now >= fleetNextBeaconMs) {
    fleetNextBeaconMs = now + FLEET_NEAR_BEACON_MS + random(0, 60);   // a little jitter so beacons don't collide
    FleetBeacon b = {};
    memcpy(b.magic, "DRFL", 4);
    b.version = 1;
    strncpy(b.name, fleetSelf, sizeof(b.name) - 1);
    strncpy(b.bf, FLEET_STATE_BF[fleetNearMyState()], FLEET_BF_MAX - 1);
    b.seq = fleetSeq++;
    esp_now_send(FLEET_BROADCAST, (const uint8_t*)&b, sizeof(b));
  }
  portENTER_CRITICAL(&fleetMux);
  for (int i = 0; i < FLEET_NEAR_MAX; i++)
    if (fleetPeers[i].used && now - fleetPeers[i].seenMs > FLEET_NEAR_FORGET_MS) fleetPeers[i].used = false;
  portEXIT_CRITICAL(&fleetMux);
}

// ---------------------------------------------------------------------------
// Avoiding each other. RSSI says how close another robot is, not which way it
// is, so this changes *how* a self-driving robot moves and the ultrasonic
// sensors still decide *where*:
//   caution  another robot is near (> -60 dBm), or coming closer from medium
//            range: slower, wider margins
//   yield    one is very close (> -45 dBm) - or near and still coming closer -
//            and it has right of way: stop, wait, turn away, carry on. Not if
//            it's already leaving.
// Right of way: the name earlier in the alphabet goes first (KIDA00 > KIDA01 >
// NORA > WHIP), except that everyone gives way to MILA: her ESP8266 can't hear
// the others, so she can't yield herself. Every robot applies the same rule, so
// two robots never both freeze or both push on. Only for modes where the robot
// drives itself.
#define FLEET_CAUTION_RSSI -60
#define FLEET_YIELD_RSSI -45
#define FLEET_FRESH_MS 2000

enum FleetAdvice { FLEET_CLEAR, FLEET_CAUTION, FLEET_YIELD };
static const char* const FLEET_ADVICE_NAMES[] = { "clear", "caution", "yield" };

// The nearest robot heard in the last 2 s decides. `who` (12 chars) gets its name.
static FleetAdvice fleetNearAdvice(char* who = nullptr) {
  unsigned long now = millis();
  int best = -1;
  portENTER_CRITICAL(&fleetMux);
  for (int i = 0; i < FLEET_NEAR_MAX; i++)
    if (fleetPeers[i].used && now - fleetPeers[i].seenMs < FLEET_FRESH_MS && (best < 0 || fleetPeers[i].rssi > fleetPeers[best].rssi))
      best = i;
  FleetAdvice a = FLEET_CLEAR;
  if (best >= 0) {
    const FleetPeer& p = fleetPeers[best];
    float r = p.rssi;
    bool coming = p.rate > FLEET_TREND_DBS, going = p.rate < -FLEET_TREND_DBS;
    // Who acts? The beacons say what each robot is doing:
    //   it's parked      -> I'm the one closing in, so I steer away
    //   it's yielding    -> it's making way for me: carry on (carefully)
    //   a human drives it -> unpredictable: I make way
    //   it drives itself -> the alphabet decides; MILA can't hear us, so she always gets the way
    bool myTurnToWait;
    if (p.state == FLEET_YIELDING) myTurnToWait = false;
    else if (p.state == FLEET_PARKED || p.state == FLEET_USER) myTurnToWait = true;
    else myTurnToWait = strcmp(p.name, "MILA") == 0 || strcmp(fleetSelf, p.name) > 0;
    // coming closer: act sooner (yield from "near", be careful from "medium"); going away: no need to yield
    float yieldAt = coming ? FLEET_CAUTION_RSSI : FLEET_YIELD_RSSI;
    float cautionAt = coming ? -75 : FLEET_CAUTION_RSSI;
    if (r > yieldAt && myTurnToWait && !going) a = FLEET_YIELD;
    else if (r > cautionAt) a = FLEET_CAUTION;
    if (who) strcpy(who, fleetPeers[best].name);
  }
  portEXIT_CRITICAL(&fleetMux);
  return a;
}

static unsigned long fleetYieldUntil = 0, fleetYieldQuietUntil = 0;
static char fleetYieldTo[12] = "";

// Call every control step of a self-driving mode; do what it says:
//   FLEET_WAIT  stop      FLEET_TURN  turn away (only if canTurn)      FLEET_GO  drive as usual
// After giving way it won't yield again for a few seconds, so two robots that
// stay close keep moving (carefully) instead of taking turns forever.
static FleetYieldPhase fleetGiveWay(bool canTurn) {
  unsigned long now = millis();
  if (fleetYieldPhase == FLEET_WAIT && now >= fleetYieldUntil) {
    if (canTurn) {
      fleetYieldPhase = FLEET_TURN;
      fleetYieldUntil = now + 600;
    } else {
      fleetYieldPhase = FLEET_GO;
      fleetYieldQuietUntil = now + 4000;
    }
  } else if (fleetYieldPhase == FLEET_TURN && now >= fleetYieldUntil) {
    fleetYieldPhase = FLEET_GO;
    fleetYieldQuietUntil = now + 5000;
  }
  if (fleetYieldPhase == FLEET_GO && now >= fleetYieldQuietUntil && fleetNearAdvice(fleetYieldTo) == FLEET_YIELD) {
    fleetYieldPhase = FLEET_WAIT;
    fleetYieldUntil = now + 2000;
  }
  return fleetYieldPhase;
}

// Nearest first.
static String fleetNearJson() {
  FleetPeer copy[FLEET_NEAR_MAX];
  portENTER_CRITICAL(&fleetMux);
  memcpy(copy, fleetPeers, sizeof(copy));
  portEXIT_CRITICAL(&fleetMux);
  int order[FLEET_NEAR_MAX], n = 0;
  for (int i = 0; i < FLEET_NEAR_MAX; i++)
    if (copy[i].used) order[n++] = i;
  for (int a = 1; a < n; a++)       // insertion sort by rssi, strongest first
    for (int b = a; b > 0 && copy[order[b]].rssi > copy[order[b - 1]].rssi; b--) {
      int t = order[b]; order[b] = order[b - 1]; order[b - 1] = t;
    }
  unsigned long now = millis();
  String j = "[";
  for (int k = 0; k < n; k++) {
    const FleetPeer& p = copy[order[k]];
    if (k) j += ",";
    bool en = p.espnowMs && now - p.espnowMs < FLEET_NEAR_FORGET_MS, bl = p.bleMs && now - p.bleMs < FLEET_NEAR_FORGET_MS;
    j += "{\"name\":\"" + String(p.name) + "\",\"rssi\":" + String((int)roundf(p.rssi)) + ",\"zone\":\"" +
         fleetZone(p.rssi) + "\",\"state\":\"" + FLEET_STATE_NAMES[p.state <= FLEET_USER ? p.state : 1] + "\",\"bf\":\"" + String(p.bf) + "\",\"trend\":\"" + fleetTrend(p) + "\",\"rate_dbs\":" + String(p.rate, 1) +
         ",\"via\":\"" + (en && bl ? "espnow+ble" : en ? "espnow" : "ble") + "\",\"age_ms\":" + String(now - p.seenMs) + "}";
  }
  return j + "]";
}

// What /near serves: who this is, what it's doing about the others, and the list.
static String fleetNearStatusJson() {
  char who[12] = "";
  FleetAdvice a = fleetNearAdvice(who);
  FleetState me = fleetNearMyState();
  return String("{\"self\":\"") + fleetSelf + "\",\"state\":\"" + FLEET_STATE_NAMES[me] + "\",\"advice\":\"" + FLEET_ADVICE_NAMES[a] + "\",\"nearest\":\"" + who +
         "\",\"yielding_to\":\"" + (fleetYieldPhase != FLEET_GO ? fleetYieldTo : "") + "\",\"robots\":" + fleetNearJson() + "}";
}
