"""ik_values.py - read and write WHIP's IK calibration, and put it into his RTrobot gait XML.

    python scripts/ik_values.py show              his live calibration + the IK constants in ik_gait.h
    python scripts/ik_values.py get [file]        save his calibration (default: whip_calibration.json)
    python scripts/ik_values.py set [file]        send a calibration file to him and SAVE it in his flash
    python scripts/ik_values.py xml [file]        gait.xml -> gait_calibrated.xml with every servo's trim
                                                  added, plus a "Zero (calibrated)" group (1500 + trim)

Options:  --url http://192.168.4.1:5005   where he is (default: WHIP_URL, then NORA's fleet list, then
                                          his own AP)      --pin 1234   his PIN (set only)
          --out FILE                      the XML to write (xml; never gait.xml itself)

The calibration file is his /calib/state as it is (every leg: each joint's channel, rev and trim in
us; plus height, speed, gait), so edit it by hand and `set` it back, or change it on /calib and `get`.

Why only the trim goes into the XML: a servo's pulse is 1500 + trim + (rev ? -angle : angle) * 10.49
us (ik_gait.h). The XML's positions were recorded on the real servos, already turning the right way,
so "rev" is the IK's business, not theirs; the trim is what lines each horn up, so every #<ch>P<us> in
the XML gets its channel's trim added (kept inside 500-2500 us).
"""

import json
import os
import re
import sys

import requests

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
IK_H = os.path.join(HERE, "esp32_gaits", "ik_gait.h")
GAIT_XML = os.path.join(REPO, "gait.xml")
DEFAULT_JSON = os.path.join(REPO, "whip_calibration.json")
DEFAULT_OUT = os.path.join(REPO, "gait_calibrated.xml")
CONTROL_PIN = "1234"   # esp32.ino's CONTROL_PIN (a safety catch, not a secret)
US_MIN, US_MAX = 500, 2500


def arg(name, default=None):
    if name in sys.argv:
        i = sys.argv.index(name)
        value = sys.argv[i + 1]
        del sys.argv[i:i + 2]
        return value
    return default


def find_whip(url):
    """His control page's URL: given, WHIP_URL, NORA's fleet list, or his own AP."""
    for u in (url, os.environ.get("WHIP_URL")):
        if u:
            return u.rstrip("/")
    try:
        robots = requests.get("http://192.168.4.1:5000/robots", timeout=1.5).json().get("robots", [])
        for r in robots:
            if r.get("name") == "WHIP" and r.get("ip"):
                return f"http://{r['ip']}:5005"
    except (requests.RequestException, ValueError):
        pass
    return "http://192.168.4.1:5005"


def get_state(base):
    r = requests.get(f"{base}/calib/state", timeout=3)
    r.raise_for_status()
    return r.json()


def ik_constants():
    """The numeric `const` values in ik_gait.h (the IK's fixed numbers: heights, strides, ...)."""
    try:
        with open(IK_H, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return []
    return re.findall(r"^const\s+(?:float|int|uint8_t)\s+(\w+)\s*=\s*([-\d.]+)\s*;\s*(?://\s*(.*))?$", text, re.M)


def print_state(state):
    print(f"calibrated: {'yes (saved on /calib)' if state.get('calibrated') else 'no (defaults)'}   "
          f"height {state.get('height')}   speed {state.get('speed')}   gait {state.get('gait')}")
    print(f"{'leg':4} {'joint':6} {'ch':>3} {'rev':>4} {'trim us':>8} {'~deg':>6}")
    for leg in state.get("legs", []):
        for s in leg["servos"]:
            print(f"{leg['name']:4} {s['joint']:6} {s['ch']:>3} {'yes' if s['rev'] else '':>4} "
                  f"{s['trim']:>8} {s['trim'] / 10.49:>6.1f}")


def trims_by_channel(state):
    return {s["ch"]: s["trim"] for leg in state["legs"] for s in leg["servos"]}


def calibrated_xml(xml, trims):
    """gait.xml's text with each #<ch>P<us> moved by that channel's trim, and a Zero group added."""
    def move(m):
        ch, us = int(m.group(1)), int(m.group(2))
        return f"#{ch}P{max(US_MIN, min(US_MAX, us + trims.get(ch, 0)))}"

    out = re.sub(r"#(\d+)P(\d+)", move, xml)
    zero = "".join(f"#{ch}P{max(US_MIN, min(US_MAX, 1500 + t))}" for ch, t in sorted(trims.items())) + "T500D500"
    # the first empty group slot gets the calibrated zero pose (every servo in its middle)
    empty = re.search(r'<Group id="(\d+)" alias="">\s*<value></value>', out)
    if empty:
        out = (out[:empty.start()] + f'<Group id="{empty.group(1)}" alias="Zero (calibrated)">\n'
               f"        <value>{zero}</value>" + out[empty.end():])
    return out


def main():
    url, pin, out = arg("--url"), arg("--pin", CONTROL_PIN), arg("--out", DEFAULT_OUT)
    cmd = sys.argv[1] if len(sys.argv) > 1 else "show"
    path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_JSON

    if cmd == "show":
        base = find_whip(url)
        try:
            print(f"WHIP at {base}")
            print_state(get_state(base))
        except requests.RequestException as e:
            print(f"Can't reach WHIP at {base} ({e.__class__.__name__}). Use --url, or EMULATE on the controller.")
        print("\nIK constants (scripts/esp32_gaits/ik_gait.h; change them there and reflash):")
        for name, value, note in ik_constants():
            print(f"  {name:18} {value:>8}   {note or ''}")

    elif cmd == "get":
        base = find_whip(url)
        state = get_state(base)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(state, f, indent=2)
        print_state(state)
        print(f"\nSaved to {path}")

    elif cmd == "set":
        base = find_whip(url)
        with open(path, encoding="utf-8") as f:
            state = json.load(f)
        s = requests.Session()
        if s.get(f"{base}/unlock", params={"pw": pin}, timeout=3).status_code != 200:
            sys.exit("Wrong PIN (--pin).")
        s.get(f"{base}/calib/start", timeout=3).raise_for_status()   # he stops and holds still
        try:
            for li, leg in enumerate(state["legs"]):
                for ji, sv in enumerate(leg["servos"]):
                    s.get(f"{base}/calib/set", timeout=3, params={
                        "leg": li, "joint": ji, "ch": sv["ch"], "rev": int(bool(sv["rev"])), "trim": sv["trim"],
                    }).raise_for_status()
            s.get(f"{base}/gait", timeout=3, params={k: state[k] for k in ("gait", "height", "speed") if k in state})
            s.get(f"{base}/calib/save", timeout=3).raise_for_status()
        finally:
            s.get(f"{base}/calib/done", timeout=3)   # stands up again
        print_state(get_state(base))
        print("\nSent and saved in his flash.")

    elif cmd == "xml":
        if os.path.abspath(out) == os.path.abspath(GAIT_XML):
            sys.exit("Not over gait.xml itself: pick another --out.")
        with open(path, encoding="utf-8") as f:
            state = json.load(f)
        with open(GAIT_XML, encoding="utf-8") as f:
            xml = f.read()
        trims = trims_by_channel(state)
        with open(out, "w", encoding="utf-8", newline="") as f:
            f.write(calibrated_xml(xml, trims))
        moved = {ch: t for ch, t in trims.items() if t}
        print(f"Wrote {out}: {len(moved)} channels trimmed "
              f"({', '.join(f'#{ch} {t:+d}' for ch, t in sorted(moved.items())) or 'all trims are 0'}), "
              f"plus a 'Zero (calibrated)' group.")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
