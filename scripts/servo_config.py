"""servo_config.py - WHIP's servo types: servo_config.json -> the firmware's servo_config.h.

servo_config.json is the one place his servos are described: each servo type's pulses (us per
degree, the middle, the limits, how far it turns), which type each joint is (hip / knee / foot), and
per leg where one leg differs ("legs": {"LF": {"knee": "DS3218 180"}}). Change a servo, change it
here (or SERVO TYPE in Servo Studio), and everything follows:

    python scripts/servo_config.py        writes scripts/esp32_gaits/servo_config.h (only if it changed,
                                          so the build stays incremental); flash_gaits does it first
    from servo_config import model        model(leg, joint) -> that servo's numbers, for the PC side
                                          (whip_body.py, Servo Studio, the emulator)

The firmware's IK (ik_gait.h toUs) uses the header, the emulator (serial_emu) is built from the same
header, and the PC side reads the JSON: they can't disagree.
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
JSON = os.path.join(REPO, "servo_config.json")
HEADER = os.path.join(HERE, "esp32_gaits", "servo_config.h")
LEGS = ("LF", "LM", "LB", "RF", "RM", "RB")
JOINTS = ("hip", "knee", "foot")
FALLBACK = {"us_per_deg": 10.49, "us_center": 1500, "us_min": 500, "us_max": 2500, "deg_max": 90}


def load():
    try:
        with open(JSON, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"models": {"MG995": dict(FALLBACK)}, "joints": {j: "MG995" for j in JOINTS}, "legs": {}}


def save(cfg):
    with open(JSON, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")


def model_name(cfg, leg, joint):
    """The servo type on this leg's joint (leg / joint: index or name)."""
    leg = LEGS[leg] if isinstance(leg, int) else leg
    joint = JOINTS[joint] if isinstance(joint, int) else joint
    return cfg.get("legs", {}).get(leg, {}).get(joint) or cfg.get("joints", {}).get(joint, "MG995")


def model(leg, joint, cfg=None):
    cfg = cfg or load()
    return {**FALLBACK, **cfg.get("models", {}).get(model_name(cfg, leg, joint), {})}


def header_text(cfg):
    rows = []
    for l, leg in enumerate(LEGS):
        cells = []
        for j, joint in enumerate(JOINTS):
            m = model(l, j, cfg)
            cells.append(f"{{{m['us_per_deg']}f, {int(m['us_center'])}, {int(m['us_min'])}, {int(m['us_max'])}, "
                         f"{float(m['deg_max'])}f}}")
        names = ", ".join(model_name(cfg, l, j) for j in range(3))
        rows.append(f"  {{ {', '.join(cells)} }},   // {leg}: {names}")
    return ("// servo_config.h - written by scripts/servo_config.py from servo_config.json: edit the JSON (or SERVO\n"
            "// TYPE in Servo Studio), not this. Each joint's servo: us per degree, the pulse at 0 degrees, the\n"
            "// lowest / highest pulse, the most it turns either way. ik_gait.h's toUs() uses it.\n"
            "#pragma once\n#include <stdint.h>\n\nnamespace servocfg {\n\n"
            "struct Servo { float usPerDeg; int16_t usCenter; uint16_t usMin, usMax; float degMax; };\n\n"
            "const Servo JOINT[6][3] = {   // [leg LF LM LB RF RM RB][joint hip knee foot]\n"
            + "\n".join(rows) + "\n};\n\n}  // namespace servocfg\n")


def write_header(cfg=None):
    """Write servo_config.h if it changed. True if it did."""
    text = header_text(cfg or load())
    try:
        with open(HEADER, encoding="utf-8") as f:
            if f.read() == text:
                return False
    except OSError:
        pass
    with open(HEADER, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return True


if __name__ == "__main__":
    changed = write_header()
    print(f"{os.path.relpath(HEADER, REPO)} {'written' if changed else 'already up to date'}")
    sys.exit(0)
