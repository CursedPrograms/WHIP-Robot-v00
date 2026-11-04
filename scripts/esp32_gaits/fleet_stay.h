// fleet_stay.h - STAY CLOSE: keep near RITA, who's in your hand. The same file on NORA and WHIP.
//
// RITA's beacon says how close she is, not which way, so this finds her the way a bacterium finds
// food: walk a while, and if her signal got stronger keep going; if not, turn and try another way.
// Close enough, stop and wait. She goes quiet: stop. The wheel lock still holds.
//
// On the way it avoids things the smart way, with whatever the robot can sense and do (StaySight):
//   - something ahead (SLOW_CM): it creeps instead of walking;
//   - something in the way (WALL_CM): a robot that can move sideways (NORA's mecanum wheels, WHIP's
//     crab) sidesteps toward the side with more room, still facing her signal, carries on a little past
//     the edge, then walks on; one that can't turns toward the roomier side. Going round something
//     isn't a bad guess about where RITA is, so it doesn't count against the search;
//   - the sidestep doesn't clear it: the other side, then back up and turn;
//   - boxed in (ahead and both sides close): back up, then turn.
//
//   StaySight v = { front_cm, left_cm, right_cm, canSlide };   // -1: that side isn't sensed
//   switch (stayStepSmart(stay, heard, rssi, v, millis())) { case STAY_WALK: ... }
//
// stayStep(s, heard, rssi, blocked, now) is the simple form (only "is something in front").
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

#define STAY_WALL_CM 30.0f           // in the way
#define STAY_SLOW_CM 60.0f           // close: creep
#define STAY_SIDE_CM 25.0f           // a side this close counts as closed
#define STAY_SLIDE_MAX_MS 2500       // sidestepping this long without clearing it: the other side
#define STAY_SLIDE_PAST_MS 600       // once clear, a little more, to clear its edge
#define STAY_BACK_MS 900

enum StayMove : uint8_t { STAY_STOP, STAY_WALK, STAY_LEFT, STAY_RIGHT,
                          STAY_CREEP, STAY_SLIDE_LEFT, STAY_SLIDE_RIGHT, STAY_BACK };

struct StaySight { float front, left, right; bool canSlide; };   // cm; -1: not sensed

enum StayAvoid : uint8_t { AVOID_NONE, AVOID_SLIDE, AVOID_PAST, AVOID_BACK };

struct Stay {
  bool arrived, turning, walking;
  int8_t side;                       // 1 left, -1 right: the next turn's way
  uint8_t misses;                    // bad tries in a row
  unsigned long since, turnFor;
  float from;                        // the signal when this try began
  StayAvoid avoid;                   // going round something
  int8_t avoidSide;                  // 1 left, -1 right
  uint8_t avoidTries;                // sides tried for this obstacle
  unsigned long avoidSince;
};

static inline void stayStart(Stay& s) {
  s.arrived = s.turning = s.walking = false;
  s.side = 1;
  s.misses = 0;
  s.avoid = AVOID_NONE;
  s.avoidTries = 0;
}

static inline void stayTurn(Stay& s, unsigned long now) {
  s.turning = true;
  s.walking = false;
  s.since = now;
  const uint8_t turns = s.misses < 1 ? 1 : s.misses < STAY_TURNS_MAX ? s.misses : STAY_TURNS_MAX;
  s.turnFor = STAY_TURN_MS * turns;
}

static inline bool stayNear(float cm, float limit) { return cm > 0 && cm < limit; }

// Which side has more room: 1 left, -1 right, 0 can't tell
static inline int8_t stayRoomier(const StaySight& v) {
  if (v.left < 0 || v.right < 0) return 0;
  if (v.left > v.right + 5) return 1;
  if (v.right > v.left + 5) return -1;
  return 0;
}

static inline StayMove stayTurnMove(const Stay& s) { return s.side > 0 ? STAY_LEFT : STAY_RIGHT; }

// One step. heard: RITA's been heard lately; rssi: how strong; v: what the robot senses and can do
static inline StayMove stayStepSmart(Stay& s, bool heard, float rssi, const StaySight& v, unsigned long now) {
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

  const bool blocked = stayNear(v.front, STAY_WALL_CM);

  // ---- going round something: its own little plan, before the search ----
  if (s.avoid == AVOID_BACK) {
    if (now - s.avoidSince < STAY_BACK_MS) return STAY_BACK;
    s.avoid = AVOID_NONE;
    s.avoidTries = 0;
    const int8_t room = stayRoomier(v);
    if (room) s.side = room;
    stayTurn(s, now);
    return stayTurnMove(s);
  }
  if (s.avoid == AVOID_SLIDE) {
    const bool sideShut = stayNear(s.avoidSide > 0 ? v.left : v.right, STAY_SIDE_CM);
    if (!blocked) {                                   // clear of it: a little more, then on
      s.avoid = AVOID_PAST;
      s.avoidSince = now;
    } else if (sideShut || now - s.avoidSince >= STAY_SLIDE_MAX_MS) {
      if (++s.avoidTries < 2) {                       // that way didn't do it: the other side
        s.avoidSide = -s.avoidSide;
        s.avoidSince = now;
      } else {                                        // neither: back up, then turn
        s.avoid = AVOID_BACK;
        s.avoidSince = now;
        return STAY_BACK;
      }
    }
    if (s.avoid == AVOID_SLIDE) return s.avoidSide > 0 ? STAY_SLIDE_LEFT : STAY_SLIDE_RIGHT;
  }
  if (s.avoid == AVOID_PAST) {
    if (now - s.avoidSince < STAY_SLIDE_PAST_MS && !blocked) return s.avoidSide > 0 ? STAY_SLIDE_LEFT : STAY_SLIDE_RIGHT;
    s.avoid = AVOID_NONE;
    s.avoidTries = 0;
    s.walking = false;                                // a fresh try from here: going round isn't a miss
  }

  if (s.turning) {
    if (now - s.since < s.turnFor) return stayTurnMove(s);
    s.turning = false;
    s.side = -s.side;
  }

  if (blocked) {
    const bool boxed = stayNear(v.left, STAY_SIDE_CM) && stayNear(v.right, STAY_SIDE_CM);
    if (boxed) {
      s.avoid = AVOID_BACK;
      s.avoidSince = now;
      return STAY_BACK;
    }
    const int8_t room = stayRoomier(v);
    if (v.canSlide) {                                 // step round it, still facing her
      s.avoid = AVOID_SLIDE;
      s.avoidSide = room ? room : s.side;
      s.avoidTries = 0;
      s.avoidSince = now;
      return s.avoidSide > 0 ? STAY_SLIDE_LEFT : STAY_SLIDE_RIGHT;
    }
    if (room) s.side = room;                          // turn toward the room, not just the next side
    stayTurn(s, now);
    return stayTurnMove(s);
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
      return stayTurnMove(s);
    }
  }
  return stayNear(v.front, STAY_SLOW_CM) ? STAY_CREEP : STAY_WALK;
}

// The simple form: only whether something's right in front
static inline StayMove stayStep(Stay& s, bool heard, float rssi, bool blocked, unsigned long now) {
  StaySight v = { blocked ? 1.0f : -1.0f, -1.0f, -1.0f, false };
  return stayStepSmart(s, heard, rssi, v, now);
}
