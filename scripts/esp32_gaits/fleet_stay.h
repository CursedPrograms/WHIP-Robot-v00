// fleet_stay.h - STAY CLOSE: keep near RITA, who's in your hand. The same file on NORA and WHIP.
//
// RITA's beacon says how close she is, not which way, so this finds her the way a bacterium finds
// food: walk a while, and if her signal got stronger keep going; if not, turn and try another way.
// Close enough, stop and wait. She goes quiet: stop. Each robot's own sensor still decides what's in
// the way: `blocked` turns it, and the wheel lock still holds.
#pragma once

#include <stdint.h>

#define STAY_PEER "RITA"
#define STAY_ARRIVED_RSSI -50.0f     // about a metre: stop here
#define STAY_LEAVE_RSSI -57.0f       // and set off again only below this, so it doesn't twitch
#define STAY_LOST_MS 2000            // no beacon this long: stop
#define STAY_TRY_MS 4000             // how long each try walks before comparing the signal
#define STAY_BETTER_DB 0.5f          // the smallest rise that counts as getting closer
#define STAY_TURN_MS 800             // a turn after a bad try, longer after each one
#define STAY_TURNS_MAX 3

enum StayMove : uint8_t { STAY_STOP, STAY_WALK, STAY_LEFT, STAY_RIGHT };

struct Stay {
  bool arrived, turning, walking;
  int8_t side;                       // 1 left, -1 right: the next turn's way
  uint8_t misses;                    // bad tries in a row
  unsigned long since, turnFor;
  float from;                        // the signal when this try began
};

static inline void stayStart(Stay& s) {
  s.arrived = s.turning = s.walking = false;
  s.side = 1;
  s.misses = 0;
}

static inline void stayTurn(Stay& s, unsigned long now) {
  s.turning = true;
  s.walking = false;
  s.since = now;
  const uint8_t turns = s.misses < 1 ? 1 : s.misses < STAY_TURNS_MAX ? s.misses : STAY_TURNS_MAX;
  s.turnFor = STAY_TURN_MS * turns;
}

// One step. heard: RITA's been heard lately; rssi: how strong; blocked: something's right in front
static inline StayMove stayStep(Stay& s, bool heard, float rssi, bool blocked, unsigned long now) {
  if (!heard) {
    stayStart(s);
    return STAY_STOP;
  }
  if (s.arrived) {
    if (rssi >= STAY_LEAVE_RSSI) return STAY_STOP;
    stayStart(s);
  } else if (rssi >= STAY_ARRIVED_RSSI) {
    stayStart(s);
    s.arrived = true;
    return STAY_STOP;
  }
  if (s.turning) {
    if (now - s.since < s.turnFor) return s.side > 0 ? STAY_LEFT : STAY_RIGHT;
    s.turning = false;
    s.side = -s.side;
  }
  if (blocked) {                     // the turn's the same whether it's a wall or a bad try
    stayTurn(s, now);
    return s.side > 0 ? STAY_LEFT : STAY_RIGHT;
  }
  if (!s.walking) {
    s.walking = true;
    s.since = now;
    s.from = rssi;
  } else if (now - s.since >= STAY_TRY_MS) {
    if (rssi - s.from >= STAY_BETTER_DB) {
      s.misses = 0;
      s.since = now;
      s.from = rssi;
    } else {
      s.misses++;
      stayTurn(s, now);
      return s.side > 0 ? STAY_LEFT : STAY_RIGHT;
    }
  }
  return STAY_WALK;
}
