#!/usr/bin/env python3
"""servo_studio.py - WHIP Servo Studio: his servos as sliders, as linkages, and as WHIP himself.

What RTrobot's ServoController program does, as our own (from its manual and the commands his
firmware already speaks, nothing taken from their program), plus WHIP on screen:

  ROBOT    WHIP from above and from the side, drawn from the servo pulses with his own IK run
           backwards (whip_body.py): click a joint for its servo, wheel / drag to move it, or drag a
           foot and his IK sets all three of that leg's servos.
  LAYOUT   your own linkage drawing: servo dials anywhere, joined by links, each horn turning with
           its servo. Starts out laid out like WHIP; EDIT to move / link / add / remove (kept in
           servo_layout.json), so it works for ARM or anything else on the same board.
  SLIDERS  all 32 channels, to play with.

  Groups   255 of them, each a list of poses ("#1P1500#2P1600T300D0"): ADD the sliders as a pose,
           INSERT / UPDATE / DELETE / UP / DOWN, RUN, LOOP, STOP; open and save RTrobot's XML (gait.xml,
           gait_emulated.xml, gait_real.xml; gait.xml itself is saved as gait_studio.xml).
  Connect  EMULATE (just the drawing) / SERIAL (the servo board's own USB, 115200 fixed; or a USB-TTL
           on its UART at the baud set on the board: 4800-115200, 9600 out of the box) / WHIP WIFI
           (through WHIP: his /servo route; he holds still meanwhile, STAND gives him back).
  Board    RTrobot's commands: G<n>F<times> runs a group stored on the board, ~ST stops it, ~RE
           restarts it; replies ("OK") show under the connection. The board's own settings (its UART
           baud, start positions, deviation, buzzer, auto run) and Download / Read / Erase have no
           published commands: see SNIFF in the README to learn them from RTrobot's program.

    python scripts/servo_studio.py [--file gait_emulated.xml]

Keys: wheel / + - move the picked servo 10 us (Shift: 1), Left / Right pick the previous / next one,
Space runs / stops the group, Ctrl+S saves, Esc quits (or leaves a text field).
"""

import argparse
import json
import math
import os
import re
import sys
import threading
import time
from xml.sax.saxutils import quoteattr

import pygame
import requests

import controller_kit as kit
import servo_config as sc
import whip_body as wb

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
LAYOUT_JSON = os.path.join(HERE, "servo_layout.json")
CAL_JSON = os.path.join(REPO, "whip_calibration.json")
W, H = 1280, 820
CHANNELS = list(range(1, 33))
GROUPS = 255
BAUDS = [115200, 57600, 38400, 19200, 9600, 4800]
LIVE_T = 100        # ms: a slider move sent live takes this long
LIVE_EVERY = 0.06   # s between live sends
CONTROL_PIN = "1234"
POSE_RE = re.compile(r"#(\d+)P(\d+)")


# ---- colours: his colour_scheme.xml (controller_kit's themes) ----
def apply_theme(c):
    globals().update(BG=c["background"], PANEL=c["panel"], BORDER=c["border"], DIV=c["divider"], TEXT=c["text"],
                     DIM=c["text_sec"], ACC=c["accent"], ACC2=c["accent2"], ACC3=c["accent3"], OK=c["accent_hover"],
                     SEL=c["button_hover_bg"], DANGER=c["danger_bg"])


SETTINGS = kit.Settings()
apply_theme(SETTINGS.colours())


# ---- his calibration: which channel is which joint ----
def load_cal():
    try:
        with open(CAL_JSON, encoding="utf-8") as f:
            st = json.load(f)
        return [[(s["ch"], bool(s["rev"]), int(s["trim"])) for s in leg["servos"]] for leg in st["legs"]]
    except (OSError, ValueError, KeyError):
        return [list(leg) for leg in wb.DEFAULT_CAL]


CAL = load_cal()
CFG = sc.load()   # servo_config.json: each joint's servo type
JOINT_OF = {}


def servo_of(l, j):
    return sc.model(l, j, CFG)


def remap():
    """JOINT_OF from CAL: board channel -> (leg, joint, rev, trim)."""
    JOINT_OF.clear()
    JOINT_OF.update({ch: (l, j, rev, trim) for l, leg in enumerate(CAL) for j, (ch, rev, trim) in enumerate(leg)})


remap()


def save_cal():
    """CAL into whip_calibration.json (his /calib/state's format: ik_values.py get / set read it)."""
    try:
        with open(CAL_JSON, encoding="utf-8") as f:
            st = json.load(f)
    except (OSError, ValueError):
        st = {"calibrated": 0, "height": 5, "speed": 5, "gait": 0}
    st["legs"] = [{"name": wb.LEGS[l], "servos": [{"joint": wb.JOINTS[j], "ch": ch, "rev": int(rev), "trim": trim}
                                                   for j, (ch, rev, trim) in enumerate(leg)]} for l, leg in enumerate(CAL)]
    with open(CAL_JSON, "w", encoding="utf-8") as f:
        json.dump(st, f, indent=2)


def name_of(ch):
    if ch in JOINT_OF:
        l, j, _, _ = JOINT_OF[ch]
        return f"{wb.LEGS[l]} {wb.JOINTS[j]}"
    return ""


def deg_of(ch, us):
    if ch in JOINT_OF:
        l, j, rev, trim = JOINT_OF[ch]
        return wb.to_deg(us, trim, rev, servo_of(l, j))
    return (us - 1500) / wb.US_PER_DEG


def us_of(ch, deg):
    if ch in JOINT_OF:
        l, j, rev, trim = JOINT_OF[ch]
        return wb.to_us(deg, trim, rev, servo_of(l, j))
    return round(1500 + deg * wb.US_PER_DEG)


def stand_pose(z=wb.HOME_Z):
    pose = {}
    for l, (x, y) in enumerate(wb.HOME):
        a = wb.angles(l, (x, y, z))
        for j, (ch, rev, trim) in enumerate(CAL[l]):
            pose[ch] = wb.to_us(a[j], trim, rev, servo_of(l, j))
    return pose


def zero_pose():
    return {ch: us_of(ch, 0) for ch in JOINT_OF}


# ---- lines and files ----
def parse(line):
    """'#1P1500#2P1600T300D100' -> ({1: 1500, 2: 1600}, 300, 100)"""
    pos = {int(c): int(p) for c, p in POSE_RE.findall(line)}
    t = re.search(r"T(\d+)", line)
    d = re.search(r"D(\d+)", line)
    return pos, int(t.group(1)) if t else 0, int(d.group(1)) if d else 0


def fmt(pos, t, d=0):
    return "".join(f"#{c}P{max(500, min(2500, int(pos[c])))}" for c in sorted(pos)) + f"T{int(t)}D{int(d)}"


def read_xml(path):
    groups = [["", []] for _ in range(GROUPS)]
    with open(path, encoding="utf-8") as f:
        text = f.read()
    for gid, alias, body in re.findall(r'<Group id="(\d+)" alias="([^"]*)">\s*<value>(.*?)</value>', text, re.S):
        g = int(gid)
        if g < GROUPS:
            groups[g] = [alias.replace("&amp;", "&").replace("&quot;", '"'),
                         [l.strip() for l in body.splitlines() if POSE_RE.search(l)]]
    return groups


def write_xml(path, groups):
    out = ['<?xml version="1.0" encoding="UTF-8"?>', "<RTrobot>"]
    for i, (alias, lines) in enumerate(groups):
        out.append(f"    <Group id=\"{i}\" alias={quoteattr(alias)}>")
        out.append(f"        <value>{chr(10).join(lines)}</value>")
        out.append("    </Group>")
    out.append("</RTrobot>")
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("\n".join(out) + "\n")


def xml_files():
    return sorted(f for f in os.listdir(REPO) if f.endswith(".xml") and f.startswith("gait"))


# ---- connections ----
class Emulated:
    name = "EMULATE"
    connected = True
    reply = "nothing is sent: only the drawing moves"

    def send(self, line):
        pass

    def close(self):
        pass


class SerialLink:
    """The servo board over a serial port (its own USB, or a USB-TTL on its UART)."""

    def __init__(self, port, baud):
        import serial
        self.name = f"SERIAL {port} @ {baud}"
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, baud, 0.2
        self.s.dtr = self.s.rts = False
        self.s.open()
        self.connected, self.reply = True, "connected"
        self._on = True
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while self._on:
            try:
                line = self.s.readline().decode("ascii", "replace").strip()
                if line:
                    self.reply = line
            except Exception:
                self.connected, self.reply = False, "the port went away"
                break

    def send(self, line):
        try:
            self.s.write((line + "\r\n").encode("ascii"))
        except Exception as e:
            self.connected, self.reply = False, f"can't write: {e}"

    def close(self):
        self._on = False
        try:
            self.s.close()
        except Exception:
            pass


class WhipWifi:
    """Through WHIP: his /servo route (he holds still; /calib/done gives him back)."""

    def __init__(self, url):
        self.base, self.name = url.rstrip("/"), f"WHIP WIFI {url}"
        self.session = requests.Session()
        r = self.session.get(f"{self.base}/unlock", params={"pw": CONTROL_PIN}, timeout=2)
        self.connected = r.status_code == 200
        self.reply = "unlocked: he holds still while the studio drives" if self.connected else "wrong PIN"
        self._q, self._wake = [], threading.Event()
        threading.Thread(target=self._sender, daemon=True).start()

    def _sender(self):
        while True:
            self._wake.wait()
            self._wake.clear()
            while self._q:
                line = self._q.pop(0)
                try:
                    r = self.session.get(f"{self.base}/servo", params={"l": line}, timeout=2)
                    self.reply = "OK" if r.status_code == 200 else f"{r.status_code} {r.text[:40]}"
                except requests.RequestException as e:
                    self.reply = f"no answer: {e.__class__.__name__}"

    def send(self, line):
        if len(self._q) > 4:      # live slider moves: only the newest matters
            self._q = self._q[-1:]
        self._q.append(line)
        self._wake.set()

    def set_channel(self, leg, joint, ch):
        """His own channel map too (/calib/set while he holds still, then /calib/save)."""
        try:
            self.session.get(f"{self.base}/calib/start", timeout=2)
            self.session.get(f"{self.base}/calib/set", params={"leg": leg, "joint": joint, "ch": ch}, timeout=2)
            self.session.get(f"{self.base}/calib/save", timeout=2)
            self.reply = f"his {wb.LEGS[leg]} {wb.JOINTS[joint]} is on pin {ch} now (saved on him)"
        except requests.RequestException as e:
            self.reply = f"couldn't tell him: {e.__class__.__name__}"

    def stand(self):
        try:
            self.session.get(f"{self.base}/calib/done", timeout=2)
            self.reply = "STAND: he's his own again"
        except requests.RequestException:
            pass

    def close(self):
        self.stand()


def serial_ports():
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()] or ["(no ports)"]
    except ImportError:
        return ["(pip install pyserial)"]


# ---- the studio ----
class Studio:
    def __init__(self, path):
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("WHIP Servo Studio")
        try:
            pygame.display.set_icon(pygame.image.load(os.path.join(REPO, "images", "whip-icon.png")))
        except (pygame.error, OSError):
            pass
        self.f_big = pygame.font.SysFont("consolas", 22, bold=True)
        self.f = pygame.font.SysFont("consolas", 15)
        self.f_s = pygame.font.SysFont("consolas", 12)
        self.clock = pygame.time.Clock()

        self.target = {ch: 1500 for ch in CHANNELS}
        self.target.update(stand_pose())
        self.shown = {ch: float(v) for ch, v in self.target.items()}
        self.anim = {}                    # ch -> (from, to, t0, ms)
        self.sel = CAL[0][0][0]           # the picked channel
        self.view = "ROBOT"
        self.link = Emulated()
        self.mode, self.ports, self.port_i, self.baud_i = "EMULATE", serial_ports(), 0, 0
        self.wifi_url = os.environ.get("WHIP_URL", "http://192.168.4.1:5005")
        self.live = True
        self.changed, self.last_live = set(), 0.0
        self.path = path
        self.groups = read_xml(path) if path and os.path.exists(path) else [["", []] for _ in range(GROUPS)]
        self.gi, self.fi, self.scroll = 0, -1, 0
        self.t_ms, self.d_ms = 500, 0
        self.focus, self.text = None, ""   # a text field being typed into: "alias" / "T" / "D" / "url"
        self.loop, self.playing, self._stop = False, False, threading.Event()
        self.note = f"opened {os.path.basename(path)}" if path and os.path.exists(path) else "new groups"
        self.edit = False
        self.layout = self._load_layout()
        self.drag = None                  # what the mouse is dragging
        self.hits = []
        # the first group with poses
        for i, (_, lines) in enumerate(self.groups):
            if lines:
                self.gi = i
                break

    # -- servo values --
    def set_us(self, ch, us, send=True):
        us = max(500, min(2500, int(us)))
        self.target[ch] = us
        self.shown[ch] = float(us)
        self.anim.pop(ch, None)
        if send:
            self.changed.add(ch)

    def move_to(self, pos, ms):
        now = time.monotonic()
        for ch, us in pos.items():
            self.target[ch] = us
            self.anim[ch] = (self.shown.get(ch, 1500.0), float(us), now, max(1, ms))

    def tick_anim(self):
        now = time.monotonic()
        for ch, (a, b, t0, ms) in list(self.anim.items()):
            u = min(1.0, (now - t0) * 1000 / ms)
            self.shown[ch] = a + (b - a) * u
            if u >= 1:
                del self.anim[ch]

    def send_pose(self, pos, ms, d=0):
        line = fmt(pos, ms, d)
        self.link.send(line)
        self.move_to(pos, ms)
        return line

    def tick_live(self):
        if self.live and self.changed and time.monotonic() - self.last_live > LIVE_EVERY and not self.playing:
            self.link.send(fmt({ch: self.target[ch] for ch in self.changed}, LIVE_T))
            self.changed.clear()
            self.last_live = time.monotonic()

    # -- connection --
    def connect(self):
        self.link.close()
        try:
            if self.mode == "SERIAL":
                self.link = SerialLink(self.ports[self.port_i], BAUDS[self.baud_i])
            elif self.mode == "WHIP WIFI":
                self.link = WhipWifi(self.wifi_url)
            else:
                self.link = Emulated()
        except Exception as e:
            self.link = Emulated()
            self.link.reply = f"couldn't connect: {e}"[:90]

    # -- groups --
    @property
    def frames(self):
        return self.groups[self.gi][1]

    def current_line(self):
        return fmt({ch: self.target[ch] for ch in self._pose_channels()}, self.t_ms, self.d_ms)

    def _pose_channels(self):
        chans = {w["ch"] for w in self.layout} | set(JOINT_OF)
        return sorted(chans)

    def frame_op(self, op):
        fr = self.frames
        if op == "ADD":
            fr.append(self.current_line()); self.fi = len(fr) - 1
        elif op == "INSERT":
            i = max(0, self.fi)
            fr.insert(i, self.current_line()); self.fi = i
        elif op == "UPDATE" and 0 <= self.fi < len(fr):
            fr[self.fi] = self.current_line()
        elif op == "DELETE" and 0 <= self.fi < len(fr):
            del fr[self.fi]; self.fi = min(self.fi, len(fr) - 1)
        elif op == "UP" and 0 < self.fi < len(fr):
            fr[self.fi - 1], fr[self.fi] = fr[self.fi], fr[self.fi - 1]; self.fi -= 1
        elif op == "DOWN" and 0 <= self.fi < len(fr) - 1:
            fr[self.fi + 1], fr[self.fi] = fr[self.fi], fr[self.fi + 1]; self.fi += 1

    def pick_frame(self, i):
        self.fi = i
        pos, t, d = parse(self.frames[i])
        self.t_ms, self.d_ms = t or self.t_ms, d
        if self.live:
            self.send_pose(pos, t or 300)
        else:
            self.move_to(pos, t or 300)

    def run(self):
        if self.playing:
            self._stop.set()
            return
        if not self.frames:
            self.note = "this group has no poses yet: ADD some"
            return
        self._stop.clear()
        self.playing = True
        threading.Thread(target=self._play, daemon=True).start()

    def _play(self):
        try:
            while not self._stop.is_set():
                for i, line in enumerate(list(self.frames)):
                    if self._stop.is_set():
                        break
                    self.fi = i
                    pos, t, d = parse(line)
                    self.link.send(line)
                    self.move_to(pos, t or 1)
                    self._stop.wait((t + d) / 1000)
                if not self.loop:
                    break
        finally:
            self.playing = False

    def save(self):
        path = self.path or os.path.join(REPO, "gait_studio.xml")
        if os.path.basename(path) == "gait.xml":   # the hand-recorded original stays as it is
            path = os.path.join(REPO, "gait_studio.xml")
        write_xml(path, self.groups)
        self.path = path
        self.note = f"saved {os.path.basename(path)}"

    def open_next(self):
        files = xml_files()
        if not files:
            return
        cur = os.path.basename(self.path) if self.path else ""
        nxt = files[(files.index(cur) + 1) % len(files)] if cur in files else files[0]
        self.path = os.path.join(REPO, nxt)
        self.groups, self.fi, self.scroll = read_xml(self.path), -1, 0
        self.gi = next((i for i, (_, l) in enumerate(self.groups) if l), 0)
        self.note = f"opened {nxt}"

    # -- servo types (servo_config.json) and the firmware --
    def next_servo_type(self, joint):
        """The next servo type for this joint on every leg (a per-leg one: "legs" in the JSON)."""
        names = list(CFG["models"])
        cur = CFG["joints"].get(wb.JOINTS[joint], names[0])
        CFG["joints"][wb.JOINTS[joint]] = names[(names.index(cur) + 1) % len(names)] if cur in names else names[0]
        sc.save(CFG)
        self.note = f"every {wb.JOINTS[joint]}: {CFG['joints'][wb.JOINTS[joint]]} (APPLY TO FIRMWARE to flash it)"

    def apply_firmware(self):
        """servo_config.json -> the firmware's servo_config.h, then flash_gaits in its own window
        (it compiles, flashes and talks you through it: unplug the servo controller's RX / TX first)."""
        sc.write_header(CFG)
        try:
            if sys.platform == "win32":
                import subprocess
                subprocess.Popen(["cmd", "/c", "start", "WHIP flash", "cmd", "/k", os.path.join(REPO, "flash_gaits.bat")],
                                 cwd=REPO)
            else:
                import shutil, subprocess
                term = shutil.which("x-terminal-emulator") or shutil.which("gnome-terminal") or shutil.which("xterm")
                cmd = [os.path.join(REPO, "flash_gaits.sh")]
                subprocess.Popen([term, "-e"] + cmd if term else cmd, cwd=REPO)
            self.note = "servo_config.h written; flashing WHIP in its own window"
        except OSError as e:
            self.note = f"servo_config.h written; couldn't start the flash: {e}"

    # -- moving a servo to another pin on the board --
    def move_pin(self, step):
        """The picked servo onto the next / previous board pin. A servo already there swaps with it,
        the poses in every group follow (so they still move the same joint), the layout too, and the
        map is kept in whip_calibration.json (and on WHIP over WHIP WIFI)."""
        old = self.sel
        new = (old - 1 + step) % 32 + 1
        for leg in CAL:                                   # his joints
            for j, (ch, rev, trim) in enumerate(leg):
                if ch == old:
                    leg[j] = (new, rev, trim)
                elif ch == new:
                    leg[j] = (old, rev, trim)
        remap()
        for w in self.layout:                             # the linkage drawing
            for key in ("ch", "parent"):
                if w.get(key) == old:
                    w[key] = new
                elif w.get(key) == new:
                    w[key] = old
        swap = {old: new, new: old}
        for g in self.groups:                             # every pose in every group
            g[1] = [POSE_RE.sub(lambda m: f"#{swap.get(int(m.group(1)), int(m.group(1)))}P{m.group(2)}", line)
                    for line in g[1]]
        self.target[old], self.target[new] = self.target[new], self.target[old]
        self.shown[old], self.shown[new] = self.shown[new], self.shown[old]
        self.sel = new
        save_cal()
        self.save_layout()
        if new in JOINT_OF and hasattr(self.link, "set_channel"):
            l, j, _, _ = JOINT_OF[new]
            self.link.set_channel(l, j, new)
            if old in JOINT_OF:
                l2, j2, _, _ = JOINT_OF[old]
                self.link.set_channel(l2, j2, old)
        what = name_of(new) or f"channel {old}"
        self.note = f"{what}: pin {old} -> {new}" + (f" (swapped with {name_of(old)})" if name_of(old) else "")

    # -- layout (LAYOUT view) --
    def _load_layout(self):
        try:
            with open(LAYOUT_JSON, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return self._default_layout()

    def _default_layout(self):
        """WHIP from above: each leg's three servos where they are, linked hip -> knee -> foot."""
        cx, cy, s = 590, 400, 1.55
        out = []
        pose = stand_pose()
        for l, leg in enumerate(CAL):
            pts = wb.joints(l, *(deg_of(ch, pose[ch]) for ch, _, _ in leg))
            for j, (ch, _, _) in enumerate(leg):
                x, y = pts[j][0], pts[j][1] + (0 if j == 0 else (14 if pts[j][1] > 0 else -14) * j)
                out.append({"ch": ch, "x": int(cx - y * s), "y": int(cy - x * s),
                            "parent": leg[j - 1][0] if j else None, "name": name_of(ch)})
        return out

    def save_layout(self):
        with open(LAYOUT_JSON, "w", encoding="utf-8") as f:
            json.dump(self.layout, f, indent=1)

    # ---- drawing helpers ----
    def btn(self, rect, label, action, on=False, color=None, fill=None):
        rect = pygame.Rect(rect)
        col = color or ACC
        pygame.draw.rect(self.screen, fill or (SEL if on else PANEL), rect, border_radius=6)
        pygame.draw.rect(self.screen, col if on else BORDER, rect, 2, border_radius=6)
        t = self.f_s.render(kit.fit(label, self.f_s, rect.w - 8), True, col if on else TEXT)
        self.screen.blit(t, t.get_rect(center=rect.center))
        self.hits.append((rect, action))

    def field(self, rect, key, value, label):
        rect = pygame.Rect(rect)
        on = self.focus == key
        pygame.draw.rect(self.screen, SEL if on else PANEL, rect, border_radius=5)
        pygame.draw.rect(self.screen, ACC if on else BORDER, rect, 2, border_radius=5)
        shown = (self.text + "_") if on else str(value)
        self.screen.blit(self.f_s.render(label, True, DIM), (rect.x, rect.y - 14))
        self.screen.blit(self.f_s.render(kit.fit(shown, self.f_s, rect.w - 8), True, TEXT), (rect.x + 5, rect.y + 5))
        self.hits.append((rect, lambda: self._focus(key, value)))

    def _focus(self, key, value):
        self.focus, self.text = key, str(value)

    def label(self, text, pos, color=None, font=None, width=None):
        font = font or self.f_s
        self.screen.blit(font.render(kit.fit(text, font, width) if width else text, True, color or DIM), pos)

    # ---- the three views ----
    VIEW = pygame.Rect(256, 54, 664, 646)

    def legs_now(self):
        legs = []
        for l, leg in enumerate(CAL):
            degs = [deg_of(ch, self.shown.get(ch, 1500)) for ch, _, _ in leg]
            legs.append(wb.joints(l, *degs))
        return legs

    def draw_robot(self):
        v = self.VIEW
        cx, cy, s = v.x + 330, v.y + 215, 0.9
        top = lambda p: (int(cx - p[1] * s), int(cy - p[0] * s))
        sx, sy, ss = v.x + 330, v.bottom - 175, 0.9        # side view: x across, z up (feet reach z -180)
        side = lambda p: (int(sx + p[0] * ss), int(sy - p[2] * ss))
        self.label("from above (head up): click a joint for its servo, drag a foot", (v.x + 8, v.y + 4))
        pygame.draw.polygon(self.screen, PANEL, [top(p) for p in wb.BODY])
        pygame.draw.polygon(self.screen, BORDER, [top(p) for p in wb.BODY], 2)
        pygame.draw.circle(self.screen, ACC2, top((92, 0)), 4)
        self.label("from the left (ground at his stance height)", (v.x + 8, sy - 62))
        pygame.draw.line(self.screen, DIV, (v.x + 40, side((0, 0, wb.HOME_Z))[1]), (v.right - 40, side((0, 0, wb.HOME_Z))[1]), 1)
        self.joint_hits = []
        for l, pts in enumerate(self.legs_now()):
            left = l < 3
            col = ACC if left else ACC3
            for proj, width in ((side, 3 if left else 2), (top, 4)):
                if proj is side and not left:
                    col2 = kit.mix(ACC3, BG, 0.45)
                else:
                    col2 = col
                ps = [proj(p) for p in pts]
                pygame.draw.lines(self.screen, col2, False, ps, width)
                for j in range(3):
                    ch = CAL[l][j][0]
                    r = 9 if ch == self.sel else 6
                    pygame.draw.circle(self.screen, OK if ch == self.sel else PANEL, ps[j], r)
                    pygame.draw.circle(self.screen, col2, ps[j], r, 2)
                    if proj is top:
                        self.joint_hits.append((ps[j], ch))
                pygame.draw.circle(self.screen, col2, ps[3], 5)
                if proj is top:
                    self.joint_hits.append((ps[3], ("foot", l, "top")))
                elif left:
                    self.joint_hits.append((ps[3], ("foot", l, "side")))
                else:
                    self.joint_hits.append((ps[3], ("foot", l, "side")))
            self.label(wb.LEGS[l], (top(pts[3])[0] + 8, top(pts[3])[1] - 6), col)
        self._top, self._side = (cx, cy, s), (sx, sy, ss)

    def draw_layout(self):
        v = self.VIEW
        self.label("your linkage drawing: EDIT to move (drag), link (right-drag), +SERVO / Del", (v.x + 8, v.y + 4))
        byc = {w["ch"]: w for w in self.layout}
        for w in self.layout:
            if w.get("parent") in byc:
                p = byc[w["parent"]]
                pygame.draw.line(self.screen, BORDER, (p["x"], p["y"]), (w["x"], w["y"]), 4)
        self.joint_hits = []
        for w in self.layout:
            ch, x, y = w["ch"], w["x"], w["y"]
            sel = ch == self.sel
            pygame.draw.circle(self.screen, SEL if sel else PANEL, (x, y), 20)
            pygame.draw.circle(self.screen, OK if sel else ACC, (x, y), 20, 2)
            a = math.radians(deg_of(ch, self.shown.get(ch, 1500)) - 90)   # the horn: up at 0 degrees
            pygame.draw.line(self.screen, OK if sel else ACC2, (x, y), (x + 17 * math.cos(a), y + 17 * math.sin(a)), 4)
            t = self.f_s.render(str(ch), True, TEXT)
            self.screen.blit(t, t.get_rect(center=(x, y + 30)))
            self.joint_hits.append(((x, y), ch))

    def draw_sliders(self):
        v = self.VIEW
        self.label("all 32 channels: drag, or the wheel over one", (v.x + 8, v.y + 4))
        self.slider_rects = []
        for i, ch in enumerate(CHANNELS):
            row, col = divmod(i, 16)
            x, y = v.x + 18 + col * 40, v.y + 34 + row * 300
            r = pygame.Rect(x, y + 18, 18, 230)
            us = self.shown.get(ch, 1500)
            pygame.draw.rect(self.screen, PANEL, r, border_radius=6)
            pygame.draw.rect(self.screen, OK if ch == self.sel else BORDER, r, 2, border_radius=6)
            ky = r.bottom - (us - 500) / 2000 * r.h
            pygame.draw.rect(self.screen, ACC if ch in JOINT_OF else ACC3, (r.x - 3, ky - 5, r.w + 6, 10), border_radius=4)
            self.label(str(ch), (x + 1, y), TEXT)
            self.label(str(int(us)), (x - 4, r.bottom + 4))
            nm = name_of(ch)
            if nm:
                self.label(nm.split()[0], (x - 2, r.bottom + 18), DIM)
            self.slider_rects.append((r, ch))

    # ---- panels ----
    def draw_left(self):
        p = pygame.Rect(8, 54, 240, H - 62)
        pygame.draw.rect(self.screen, PANEL, p, border_radius=10)
        x, y = p.x + 10, p.y + 10
        self.label("CONNECTION", (x, y), DIM); y += 18
        modes = ["EMULATE", "SERIAL", "WHIP WIFI"]
        self.btn((x, y, 220, 26), self.mode, lambda: setattr(self, "mode", modes[(modes.index(self.mode) + 1) % 3]), True); y += 32
        if self.mode == "SERIAL":
            self.btn((x, y, 140, 26), f"PORT {self.ports[self.port_i]}",
                     lambda: setattr(self, "port_i", (self.port_i + 1) % len(self.ports)))
            self.btn((x + 146, y, 74, 26), "REFRESH", lambda: (setattr(self, "ports", serial_ports()), setattr(self, "port_i", 0))); y += 32
            self.btn((x, y, 220, 26), f"BAUD {BAUDS[self.baud_i]}  (USB: 115200)",
                     lambda: setattr(self, "baud_i", (self.baud_i + 1) % len(BAUDS))); y += 32
        elif self.mode == "WHIP WIFI":
            y += 14
            self.field((x, y, 220, 24), "url", self.wifi_url, "WHIP's address"); y += 30
        self.btn((x, y, 220, 30), f"CONNECT ({self.mode})" if self.link.name != self.mode and not self.link.name.startswith(self.mode) else "RECONNECT",
                 self.connect, color=OK); y += 36
        self.label(kit.fit(self.link.name, self.f_s, 220), (x, y), OK if self.link.connected else ACC3); y += 15
        self.label(kit.fit(f"> {self.link.reply}", self.f_s, 220), (x, y)); y += 22
        self.btn((x, y, 220, 26), "LIVE: sliders move him" if self.live else "LIVE OFF: drawing only",
                 lambda: setattr(self, "live", not self.live), self.live); y += 38

        self.label("THE BOARD (RTrobot's commands)", (x, y)); y += 18
        self.btn((x, y, 108, 26), f"RUN G{self.gi} F1", lambda: self.link.send(f"G{self.gi}F1"))
        self.btn((x + 112, y, 108, 26), "STOP  ~ST", lambda: self.link.send("~ST")); y += 32
        self.btn((x, y, 108, 26), "RESTART ~RE", lambda: self.link.send("~RE"))
        self.btn((x + 112, y, 108, 26), "STAND (WHIP)", lambda: getattr(self.link, "stand", lambda: None)()); y += 38

        self.label("POSES", (x, y)); y += 18
        self.btn((x, y, 108, 26), "STANCE", lambda: self.send_pose(stand_pose(), 600))
        self.btn((x + 112, y, 108, 26), "ZERO 1500+trim", lambda: self.send_pose(zero_pose(), 600)); y += 38

        self.label("FILE", (x, y)); y += 18
        self.btn((x, y, 150, 26), f"OPEN > {os.path.basename(self.path) if self.path else '-'}", self.open_next)
        self.btn((x + 156, y, 64, 26), "SAVE", self.save); y += 32
        self.label(kit.fit(self.note, self.f_s, 220), (x, y), OK); y += 30
        self.btn((x, y, 220, 26), f"THEME: {SETTINGS.theme}", lambda: apply_theme(SETTINGS.next_theme())); y += 40
        for line in ("wheel / + -: move the picked servo", "Shift: 1 us at a time", "Left / Right: pick a servo",
                     "PIN < >: put it on another board pin",
                     "Space: run / stop the group", "Ctrl+S: save"):
            self.label(line, (x, y), width=220); y += 15

    def draw_right(self):
        p = pygame.Rect(928, 54, 344, H - 62)
        pygame.draw.rect(self.screen, PANEL, p, border_radius=10)
        x, y = p.x + 10, p.y + 10
        self.label("GROUP", (x, y)); y += 16
        self.btn((x, y, 30, 26), "<", lambda: self._group(-1))
        self.label(f"{self.gi:3d}", (x + 36, y + 4), TEXT, self.f)
        self.btn((x + 70, y, 30, 26), ">", lambda: self._group(1))
        self.field((x + 108, y, 216, 26), "alias", self.groups[self.gi][0], ""); y += 36
        box = pygame.Rect(x, y, 324, 400)
        pygame.draw.rect(self.screen, BG, box, border_radius=6)
        rows = (box.h - 8) // 18
        fr = self.frames
        if self.fi >= 0:
            self.scroll = max(min(self.scroll, self.fi), self.fi - rows + 1)
        self.scroll = max(0, min(self.scroll, max(0, len(fr) - rows)))
        for k in range(rows):
            i = self.scroll + k
            if i >= len(fr):
                break
            r = pygame.Rect(box.x + 4, box.y + 4 + k * 18, box.w - 8, 18)
            if i == self.fi:
                pygame.draw.rect(self.screen, SEL, r, border_radius=4)
            pos, t, d = parse(fr[i])
            txt = f"{i + 1:3d}  {len(pos):2d} servos  T{t} D{d}  " + " ".join(f"{c}:{u}" for c, u in sorted(pos.items()))
            self.label(kit.fit(txt, self.f_s, r.w - 6), (r.x + 3, r.y + 2), TEXT if i == self.fi else DIM)
            self.hits.append((r, lambda i=i: self.pick_frame(i)))
        if not fr:
            self.label("no poses: set the servos, then ADD", (box.x + 8, box.y + 8))
        y = box.bottom + 10
        for k, op in enumerate(("ADD", "INSERT", "UPDATE", "DELETE", "UP", "DOWN")):
            self.btn((x + (k % 3) * 110, y + (k // 3) * 32, 104, 26), op, lambda op=op: self.frame_op(op))
        y += 70
        self.field((x, y + 14, 104, 26), "T", self.t_ms, "move time T ms")
        self.field((x + 110, y + 14, 104, 26), "D", self.d_ms, "then wait D ms")
        y += 50
        self.btn((x, y, 104, 30), "STOP" if self.playing else "RUN", self.run, self.playing, OK)
        self.btn((x + 110, y, 104, 30), "LOOP ON" if self.loop else "LOOP OFF", lambda: setattr(self, "loop", not self.loop), self.loop)
        self.label(f"{len(fr)} poses", (x + 224, y + 8))

    def _group(self, step):
        self.gi = (self.gi + step) % GROUPS
        self.fi, self.scroll = -1, 0

    def draw_selected(self):
        r = pygame.Rect(self.VIEW.x, self.VIEW.bottom + 8, self.VIEW.w, 96)
        pygame.draw.rect(self.screen, PANEL, r, border_radius=8)
        us = int(self.shown.get(self.sel, 1500))
        self.label(f"CH {self.sel:2d}  {name_of(self.sel) or '-':10}  {us} us  ({deg_of(self.sel, us):+.1f} deg)",
                   (r.x + 10, r.y + 6), TEXT, self.f)
        self.label("on board pin", (r.right - 196, r.y + 8))   # move it to another pin on the board
        self.btn((r.right - 100, r.y + 4, 40, 22), "<", lambda: self.move_pin(-1))
        self.btn((r.right - 54, r.y + 4, 40, 22), ">", lambda: self.move_pin(1))
        bar = pygame.Rect(r.x + 10, r.y + 30, r.w - 20, 10)
        pygame.draw.rect(self.screen, DIV, bar, border_radius=5)
        kx = bar.x + (us - 500) / 2000 * bar.w
        pygame.draw.circle(self.screen, ACC, (int(kx), bar.centery), 9)
        self.sel_bar = bar
        if self.sel in JOINT_OF:   # its servo type (servo_config.json): for that joint on every leg
            l, j, _, _ = JOINT_OF[self.sel]
            name = sc.model_name(CFG, l, j)
            self.label(f"servo type, every {wb.JOINTS[j]}:", (r.x + 10, r.y + 60))
            self.btn((r.x + 190, r.y + 54, 170, 26), f"{name}  >", lambda: self.next_servo_type(j))
            self.label(kit.fit(CFG["models"].get(name, {}).get("note", ""), self.f_s, 280), (r.x + 10, r.y + 82), DIM)
        self.btn((r.right - 196, r.y + 54, 186, 30), "APPLY TO FIRMWARE", self.apply_firmware, color=OK)

    def draw(self):
        self.hits = []
        self.screen.fill(BG)
        self.label("WHIP", (12, 12), ACC, self.f_big)
        self.label("Servo Studio", (78, 12), TEXT, self.f_big)
        for k, v in enumerate(("ROBOT", "LAYOUT", "SLIDERS")):
            self.btn((256 + k * 110, 14, 104, 28), v, lambda v=v: setattr(self, "view", v), self.view == v)
        if self.view == "LAYOUT":
            self.btn((596, 14, 90, 28), "EDIT ON" if self.edit else "EDIT", lambda: setattr(self, "edit", not self.edit), self.edit)
            self.btn((690, 14, 90, 28), "+SERVO", self._add_widget)
            self.btn((784, 14, 90, 28), "RESET", self._reset_layout)
        pygame.draw.rect(self.screen, kit.mix(BG, PANEL, 0.5), self.VIEW, border_radius=10)
        {"ROBOT": self.draw_robot, "LAYOUT": self.draw_layout, "SLIDERS": self.draw_sliders}[self.view]()
        self.draw_selected()
        self.draw_left()
        self.draw_right()
        pygame.display.flip()

    def _add_widget(self):
        used = {w["ch"] for w in self.layout}
        ch = next((c for c in CHANNELS if c not in used), None)
        if ch:
            self.layout.append({"ch": ch, "x": self.VIEW.centerx, "y": self.VIEW.centery, "parent": None, "name": name_of(ch)})
            self.sel = ch
            self.save_layout()

    def _reset_layout(self):
        self.layout = self._default_layout()
        self.save_layout()

    # ---- input ----
    def nearest(self, pos, radius=14):
        best, bd = None, radius
        for (x, y), what in self.joint_hits:
            d = math.hypot(pos[0] - x, pos[1] - y)
            if d <= bd:
                best, bd = what, d
        return best

    def foot_to(self, leg, where, mouse):
        """Drag a foot: from above (x, y follow, z kept) or from the side (z follows)."""
        cur = self.legs_now()[leg][3]
        if where == "top":
            cx, cy, s = self._top
            p = ((cy - mouse[1]) / s, (cx - mouse[0]) / s, cur[2])
        else:
            sx, sy, ss = self._side
            p = (cur[0], cur[1], (sy - mouse[1]) / ss)
        a = wb.angles(leg, p)
        if a is None:
            return
        for j, (ch, rev, trim) in enumerate(CAL[leg]):
            self.set_us(ch, wb.to_us(a[j], trim, rev, servo_of(leg, j)))

    def on_event(self, ev):
        if ev.type == pygame.QUIT:
            return False
        if ev.type == pygame.KEYDOWN:
            if self.focus:
                return self._type(ev)
            mods = pygame.key.get_mods()
            step = 1 if mods & pygame.KMOD_SHIFT else 10
            if ev.key == pygame.K_ESCAPE:
                return False
            if ev.key == pygame.K_s and mods & pygame.KMOD_CTRL:
                self.save()
            elif ev.key == pygame.K_SPACE:
                self.run()
            elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.set_us(self.sel, self.target[self.sel] + step)
            elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.set_us(self.sel, self.target[self.sel] - step)
            elif ev.key in (pygame.K_LEFT, pygame.K_RIGHT):
                self.sel = CHANNELS[(CHANNELS.index(self.sel) + (1 if ev.key == pygame.K_RIGHT else -1)) % 32]
            elif ev.key == pygame.K_DELETE and self.view == "LAYOUT" and self.edit:
                self.layout = [w for w in self.layout if w["ch"] != self.sel]
                for w in self.layout:
                    if w.get("parent") == self.sel:
                        w["parent"] = None
                self.save_layout()
        elif ev.type == pygame.MOUSEWHEEL:
            step = (1 if pygame.key.get_mods() & pygame.KMOD_SHIFT else 10) * ev.y
            over = self._over(pygame.mouse.get_pos())
            ch = over if isinstance(over, int) else self.sel
            self.sel = ch
            self.set_us(ch, self.target[ch] + step)
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button in (1, 3):
            if self.focus:
                self._commit()
            for rect, action in self.hits:
                if rect.collidepoint(ev.pos):
                    action()
                    return True
            if getattr(self, "sel_bar", None) and self.sel_bar.inflate(0, 20).collidepoint(ev.pos):
                self.drag = ("bar",)
                self._drag_to(ev.pos)
                return True
            over = self._over(ev.pos)
            if isinstance(over, tuple) and over[0] == "foot":
                self.drag = over
            elif isinstance(over, tuple) and over[0] == "slider":
                self.sel = over[1]
                self.drag = over
                self._drag_to(ev.pos)
            elif isinstance(over, int):
                self.sel = over
                if self.view == "LAYOUT" and self.edit:
                    self.drag = ("move", over) if ev.button == 1 else ("link", over)
                else:
                    self.drag = ("turn", over, ev.pos[1], self.target[over])
        elif ev.type == pygame.MOUSEMOTION and self.drag:
            self._drag_to(ev.pos)
        elif ev.type == pygame.MOUSEBUTTONUP:
            if self.drag and self.drag[0] == "link":
                to = self._over(ev.pos)
                if isinstance(to, int) and to != self.drag[1]:
                    w = next(w for w in self.layout if w["ch"] == to)
                    w["parent"] = None if w.get("parent") == self.drag[1] else self.drag[1]
                    self.save_layout()
            if self.drag and self.drag[0] == "move":
                self.save_layout()
            self.drag = None
        return True

    def _over(self, pos):
        if self.view == "SLIDERS":
            for r, ch in getattr(self, "slider_rects", []):
                if r.inflate(10, 10).collidepoint(pos):
                    return ("slider", ch)
            return None
        return self.nearest(pos)

    def _drag_to(self, pos):
        d = self.drag
        if d[0] == "bar":
            b = self.sel_bar
            self.set_us(self.sel, 500 + (pos[0] - b.x) / b.w * 2000)
        elif d[0] == "slider":
            r = next(r for r, ch in self.slider_rects if ch == d[1])
            self.set_us(d[1], 500 + (r.bottom - pos[1]) / r.h * 2000)
        elif d[0] == "foot":
            self.foot_to(d[1], d[2], pos)
        elif d[0] == "turn":
            self.set_us(d[1], d[3] + (d[2] - pos[1]) * 4)
        elif d[0] == "move":
            w = next(w for w in self.layout if w["ch"] == d[1])
            w["x"], w["y"] = max(self.VIEW.x + 20, min(self.VIEW.right - 20, pos[0])), max(self.VIEW.y + 30, min(self.VIEW.bottom - 20, pos[1]))

    def _type(self, ev):
        if ev.key == pygame.K_ESCAPE:
            self.focus = None
        elif ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            self._commit()
        elif ev.key == pygame.K_BACKSPACE:
            self.text = self.text[:-1]
        elif ev.unicode and ev.unicode.isprintable():
            if self.focus in ("T", "D") and not ev.unicode.isdigit():
                return True
            self.text += ev.unicode
        return True

    def _commit(self):
        k, t = self.focus, self.text
        if k == "alias":
            self.groups[self.gi][0] = t
        elif k in ("T", "D") and t.isdigit():
            v = max(0, min(9999, int(t)))
            if k == "T":
                self.t_ms = v
            else:
                self.d_ms = v
        elif k == "url" and t:
            self.wifi_url = t if t.startswith("http") else "http://" + t
        self.focus = None

    def loop_forever(self):
        running = True
        while running:
            for ev in pygame.event.get():
                running = self.on_event(ev) and running
            self.tick_anim()
            self.tick_live()
            self.draw()
            self.clock.tick(60)
        self._stop.set()
        self.link.close()
        pygame.quit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default=None, help="an RTrobot XML to open (default: gait_emulated.xml, else gait.xml)")
    args = ap.parse_args()
    path = args.file
    if not path:
        for f in ("gait_emulated.xml", "gait.xml"):
            if os.path.exists(os.path.join(REPO, f)):
                path = os.path.join(REPO, f)
                break
    elif not os.path.isabs(path):
        path = os.path.join(REPO, path)
    Studio(path).loop_forever()


if __name__ == "__main__":
    main()
