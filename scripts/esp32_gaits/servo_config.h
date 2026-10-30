// servo_config.h - written by scripts/servo_config.py from servo_config.json: edit the JSON (or SERVO
// TYPE in Servo Studio), not this. Each joint's servo: us per degree, the pulse at 0 degrees, the
// lowest / highest pulse, the most it turns either way. ik_gait.h's toUs() uses it.
#pragma once
#include <stdint.h>

namespace servocfg {

struct Servo { float usPerDeg; int16_t usCenter; uint16_t usMin, usMax; float degMax; };

const Servo JOINT[6][3] = {   // [leg LF LM LB RF RM RB][joint hip knee foot]
  { {10.49f, 1500, 500, 2500, 90.0f}, {7.41f, 1500, 500, 2500, 135.0f}, {10.49f, 1500, 500, 2500, 90.0f} },   // LF: MG995, DS3218 270, MG995
  { {10.49f, 1500, 500, 2500, 90.0f}, {7.41f, 1500, 500, 2500, 135.0f}, {10.49f, 1500, 500, 2500, 90.0f} },   // LM: MG995, DS3218 270, MG995
  { {10.49f, 1500, 500, 2500, 90.0f}, {7.41f, 1500, 500, 2500, 135.0f}, {10.49f, 1500, 500, 2500, 90.0f} },   // LB: MG995, DS3218 270, MG995
  { {10.49f, 1500, 500, 2500, 90.0f}, {7.41f, 1500, 500, 2500, 135.0f}, {10.49f, 1500, 500, 2500, 90.0f} },   // RF: MG995, DS3218 270, MG995
  { {10.49f, 1500, 500, 2500, 90.0f}, {7.41f, 1500, 500, 2500, 135.0f}, {10.49f, 1500, 500, 2500, 90.0f} },   // RM: MG995, DS3218 270, MG995
  { {10.49f, 1500, 500, 2500, 90.0f}, {7.41f, 1500, 500, 2500, 135.0f}, {10.49f, 1500, 500, 2500, 90.0f} },   // RB: MG995, DS3218 270, MG995
};

}  // namespace servocfg
