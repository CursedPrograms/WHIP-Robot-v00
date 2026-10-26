#!/usr/bin/env python3
"""
WHIP Robot - Controller

Draws a PS2/DualShock-style controller layout and lights up buttons/sticks/
dpad/triggers live if a joystick is connected -- and drives WHIP over WiFi:

    D-pad / arrow keys : forward / backward / turn left / turn right
    Cross (A), 1       : switch WHIP into WEB control mode (required before
                         drive commands do anything -- he boots into
                         Obstacle Avoidance mode, same as arduino.ino)
    Circle (B), 2      : switch WHIP back to Obstacle Avoidance mode
    Square (X), 3      : switch WHIP to IR Remote mode
    Triangle (Y)       : emergency stop (drops back to standing immediately)
    G / Z X C, L1 / R1 : gait: next / tripod, ripple, wave (G and L1 / R1 also reach quad, amble, metachronal) (the gait dropdown, gaits firmware)
    T, Select          : tilt safety on / off        B : buzzer on / off
    Q / E              : crab left / right           F : follow-me mode
    H, L2 / N, R2      : wave hello / dance          R : lie down / get up
    Sticks (gaits firmware): left = forward / back and turn, with speed; right = crab

CONNECTING

On startup this looks for WHIP the same way his own firmware (scripts/
esp32/esp32.ino) looks for NORA:

    1. Try NORA's fixed AP address (192.168.4.1:5000/robots). If she
       answers, WHIP is somewhere on her network -- look WHIP up in the
       fleet roster RIFT/NORA both serve there and use his registered IP.
    2. If NORA isn't reachable, assume this PC is on WHIP's own AP
       ("WHIP", the fleet's password) instead, and try him directly at his AP's
       fixed gateway address, 192.168.4.1:5005.

Either way it ends up with a base URL for WHIP's control server
(http://<ip>:5005) and everything else just calls that. All the networking
runs on a background thread (WhipLink) so a slow or dropped WiFi frame
never stalls the pygame render loop.
"""

import json
import os
import re
import sys
import threading
import time

import pygame
import requests

import math

import keymap   # scripts/keymap.py: every key and button, in its own window (F1)
import mind     # scripts/mind.py: his LLM mind (MIND mode), shared by every robot's controller
import serial_to_xml   # scripts/serial_to_xml.py: his servo serial -> RTrobot XML macros (the XML button)
import controller_kit as kit   # scripts/controller_kit.py: the parts every robot's controller shares (Voice)

WIDTH, HEIGHT = 900, 820   # the bottom 200 px: the mind's panel
# Colours: always from colour_scheme.xml (sampled from his avatar: red and blue spider on night
# black), the same file his site and options.py use. THEME (top right) picks DEFAULT (the scheme as
# it is), LIGHT (the scheme's accents on light surfaces made from its own text colours) or CUSTOM
# (the scheme with your colours over it: controller_settings.json's "custom", the XML's names).
HERE = os.path.dirname(os.path.abspath(__file__))
SCHEME_XML = os.path.join(HERE, "..", "colour_scheme.xml")
SETTINGS_JSON = os.path.join(HERE, "controller_settings.json")   # theme + custom colours (not in git)
THEMES = ["DEFAULT", "LIGHT", "CUSTOM"]
SCHEME_FALLBACK = {   # colour_scheme.xml as of writing, if the file can't be read
    "background": "#080A12", "panel": "#0E1120", "border": "#1D223F", "divider": "#15192E",
    "text": "#F2E9EA", "text_sec": "#A89497", "text_dim": "#6B575A", "accent": "#D34859",
    "accent_hover": "#DA6473", "accent2": "#1A7BD9", "accent3": "#CC4A82",
    "button_hover_bg": "#250E11", "danger_bg": "#210F0D",
}


def _rgb(v):
    v = v.lstrip("#")
    return tuple(int(v[i:i + 2], 16) for i in (0, 2, 4))


def _mix(a, b, t):
    return tuple(round(x + (y - x) * t) for x, y in zip(a, b))


def load_scheme():
    """colour_scheme.xml as {name: "#RRGGBB"} (its format: name="..." value="#RRGGBB")."""
    try:
        with open(SCHEME_XML, encoding="utf-8") as f:
            found = dict(re.findall(r'name="([^"]+)"\s+value="(#[0-9A-Fa-f]{6})"', f.read()))
    except OSError:
        found = {}
    return {**SCHEME_FALLBACK, **found}


def theme_colours(name, custom=None):
    c = {k: _rgb(v) for k, v in load_scheme().items()}
    if name == "LIGHT":
        white = (255, 255, 255)
        c.update(background=c["text"], panel=_mix(c["text"], white, 0.7), border=_mix(c["text"], c["text_sec"], 0.45),
                 divider=_mix(c["text"], c["text_sec"], 0.18), text=c["background"], text_sec=c["text_dim"],
                 text_dim=c["text_sec"], button_hover_bg=_mix(c["accent"], white, 0.82),
                 danger_bg=_mix(c["accent"], white, 0.88))
    elif name == "CUSTOM":
        c.update({k: _rgb(v) for k, v in (custom or {}).items()
                  if isinstance(v, str) and re.fullmatch(r"#[0-9A-Fa-f]{6}", v)})
    return c


def apply_theme(name, custom=None):
    c = theme_colours(name, custom)
    globals().update(
        BG_COLOR=c["background"], PANEL_COLOR=c["panel"], OUTLINE_COLOR=c["border"], TEXT_COLOR=c["text"],
        MUTED_TEXT=c["text_sec"], DIM_TEXT=c["text_dim"], IDLE_COLOR=c["divider"], STICK_COLOR=c["divider"],
        ACTIVE_COLOR=c["accent"], OK_COLOR=c["accent_hover"], ACCENT2=c["accent2"], WARN_COLOR=c["accent3"],
        ERROR_COLOR=c["accent"], STICK_DOT_COLOR=c["accent"], SELECTED_BG=c["button_hover_bg"],
        DANGER_BG=c["danger_bg"])


apply_theme("DEFAULT")
AXIS_DEADZONE = 0.15

FACE_BUTTON_LAYOUT = [
    # label, offset (x, y) relative to face-button cluster center, its colour (a theme colour's name)
    ("Triangle", (0, -42), "OK_COLOR"),
    ("Circle", (42, 0), "ACTIVE_COLOR"),
    ("Cross", (0, 42), "ACCENT2"),
    ("Square", (-42, 0), "WARN_COLOR"),
]

DPAD_DIRS = [
    ("Up", (0, -1)),
    ("Down", (0, 1)),
    ("Left", (-1, 0)),
    ("Right", (1, 0)),
]

# ---------------------------------------------------------------------------
# WHIP networking -- mirrors the discovery order baked into esp32.ino's
# setupWiFi(): NORA's network first, WHIP's own AP as the fallback. Both
# NORA-as-AP-host and WHIP-as-AP-host use the same fixed gateway address
# (192.168.4.1, the ESP32 default), which is what makes step 2 possible
# without already knowing which network we're on.
# ---------------------------------------------------------------------------
AP_GATEWAY_IP = "192.168.4.1"
NORA_FLEET_PORT = 5000     # NORA's /robots + /register port (see RIFT's README port table)
WHIP_PORT = 5005           # RIFT's assigned port for WHIP's control page
WHIP_NAME = "WHIP"
CONTROL_PIN = "1234"       # esp32.ino's CONTROL_PIN

DISCOVERY_TIMEOUT = 1.5
REQUEST_TIMEOUT = 0.5
# WHIP heartbeats to NORA's registry every 10s (FLEET_HEARTBEAT_MS in
# esp32.ino) -- give him a few cycles to show up if NORA's up but WHIP
# hasn't registered yet (e.g. he's still booting).
WHIP_REGISTER_RETRIES = 6
WHIP_REGISTER_RETRY_DELAY = 2.0
RECONNECT_DELAY = 2.0
TICK_INTERVAL = 0.15   # matches the web control page's 200ms held-button repeat

# His gaits, as esp32_gaits/gaits.h: sixths of the cycle a foot is in the air, and when each
# leg lifts (LF LM LB RF RM RB). The dropdown and the little animation use them.
GAITS = [
    ("TRIPOD", 3, (0, 3, 0, 3, 0, 3), "3 feet down, two groups of 3 by turns: fastest"),
    ("RIPPLE", 2, (4, 2, 0, 1, 5, 3), "4 feet down, a step runs front to back each side"),
    ("WAVE", 1, (2, 1, 0, 5, 4, 3), "5 feet down, one leg at a time: slowest, easiest on the servos"),
    ("QUAD", 2, (4, 2, 0, 2, 0, 4), "4 feet down, 3 pairs step by turns: between tripod and ripple"),
    ("AMBLE", 2, (4, 0, 2, 2, 4, 0), "4 feet down, diagonal pairs (none straight across): spreads the load"),
    ("METACHRONAL", 1, (2, 1, 0, 3, 4, 5), "5 feet down, one leg at a time round the body: a Mexican wave"),
]
LEG_NAMES = ("LF", "LM", "LB", "RF", "RM", "RB")
LEG_HOME = ((160, 123), (0, 177), (-160, 123), (160, -123), (0, -177), (-160, -123))   # mm, x forward, y left
LEG_HIP = ((74, 39), (0, 64), (-74, 39), (74, -39), (0, -64), (-74, -39))
BODY = ((74, 39), (0, 64), (-74, 39), (-74, -39), (0, -64), (74, -39))
STRIDE = 45
SETTING_RESEND = 0.6    # s: a setting he hasn't confirmed yet is sent again this often...
SETTING_GIVE_UP = 6.0   # ...until his status says so, or this long has gone by
MODES = ["obstacle", "ir", "web", "mind", "follow"]   # his controlMode numbers, in order
FAKE_URL = "http://127.0.0.1:5005"                    # EMULATE: fake_whip.py on this PC


def leg_at(gait, leg, phase):
    """Where a foot is, phase 0..1 through the cycle: (mm ahead of home, how high 0..1)."""
    _, air6, lift, _ = gait
    air, k = air6 / 6, (phase - lift[leg] / 6) % 1
    if k < air:
        u = k / air
        return -STRIDE + 2 * STRIDE * u * u * (3 - 2 * u), math.sin(math.pi * u)
    return STRIDE - 2 * STRIDE * (k - air) / (1 - air), 0.0


def _get_json(session, url, timeout=DISCOVERY_TIMEOUT):
    try:
        r = session.get(url, timeout=timeout)
        if r.status_code == 200:
            return r.json()
    except requests.RequestException:
        pass
    return None


class WhipLink:
    """
    Background network worker. Discovers WHIP, then keeps polling /status
    and sending whatever drive command the pygame loop last set via
    drive(). Runs entirely on its own thread -- nothing here ever blocks
    the render loop.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._session = requests.Session()
        self._desired_cmd = "stop"
        self._desired_params = {}   # cmd=drive: f / l / t, each -1..1 (the sticks)

        self.base_url = None
        self.connected = False
        self.searching = True
        self.status = None
        self.mind_on = False        # MIND mode: mind.py drives through /mind, so no /web ticks
        self.mind_capable = False   # mind.can_think() said yes: unlock and check in as his mind host
        self._host_unlocked = False
        self.url_override = None    # EMULATE: the fake WHIP's address instead of looking for him
        # Settings (mode, gait, tilt, sound) are re-sent until his /status says so: mid-step he
        # can miss a request (his web server only answers between moves), and a one-shot that
        # missed used to snap back after a moment (the gait "going back to tripod").
        self._pending = {}          # status key -> [value, path, params, first sent, last sent]
        self._calls = []            # one-shots (hello, dance, the CENTER steps), sent in order

        self._wake = threading.Event()   # use_url(): stop waiting and look again now
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def drive(self, cmd, **params):
        with self._lock:
            self._desired_cmd = cmd
            self._desired_params = params

    def use_url(self, url):
        """EMULATE on (the fake WHIP's URL) or off (None: look for the real one again)."""
        self.url_override = url
        self.base_url, self.connected, self.status = None, False, None
        self._wake.set()

    def setting(self, key, value, path, **params):
        with self._lock:
            self._pending[key] = [value, path, params, time.monotonic(), 0.0]

    def pending(self, key):
        """The value just picked for a setting he hasn't confirmed yet, else None."""
        p = self._pending.get(key)
        return None if p is None else p[0]

    def set_mode(self, mode):
        self.setting("controlMode", MODES.index(mode), "/mode", m=mode)

    def send(self, path, **params):
        """A one-shot (hello, dance, rest, the CENTER steps): queued, sent in order."""
        with self._lock:
            self._calls.append((path, params))

    def _send_queued(self):
        with self._lock:
            calls, self._calls = self._calls, []
            now = time.monotonic()
            due = [(k, p) for k, p in self._pending.items() if now - p[4] > SETTING_RESEND]
            for _, p in due:
                p[4] = now
        for path, params in calls:
            try:
                self._session.get(f"{self.base_url}{path}", params=params, timeout=REQUEST_TIMEOUT * 2)
            except requests.RequestException:
                pass   # the /status below finds out if he's really gone
        for _, (value, path, params, _, _) in due:
            try:
                self._session.get(f"{self.base_url}{path}", params=params, timeout=REQUEST_TIMEOUT)
            except requests.RequestException:
                pass   # sent again in SETTING_RESEND

    def _check_pending(self):
        now = time.monotonic()
        with self._lock:
            for key in list(self._pending):
                p = self._pending[key]
                if (self.status or {}).get(key) == p[0] or now - p[3] > SETTING_GIVE_UP:
                    del self._pending[key]

    def _run(self):
        while True:
            if not self.connected:
                self.searching = True
                self._discover()
                self.searching = False
                if not self.connected:
                    self._wake.wait(RECONNECT_DELAY)
                    self._wake.clear()
                    continue
                self._unlock()
            self._tick()
            time.sleep(TICK_INTERVAL)

    def _discover(self):
        # 0. EMULATE, or WHIP_URL set (fake_whip.py does): talk to that and nothing else.
        url = self.url_override or os.environ.get("WHIP_URL")
        if url:
            url = url.rstrip("/")
            if _get_json(self._session, f"{url}/status") is not None:
                self.base_url = url
                self.connected = True
            return

        # 1. NORA's fleet registry -- only answers if we're on her network.
        nora_url = f"http://{AP_GATEWAY_IP}:{NORA_FLEET_PORT}"
        for attempt in range(WHIP_REGISTER_RETRIES):
            if self.url_override:
                return   # EMULATE was pressed meanwhile
            data = _get_json(self._session, f"{nora_url}/robots")
            if data is None:
                break   # NORA isn't reachable at all -- no point retrying this path

            for robot in data.get("robots", []):
                if robot.get("name") == WHIP_NAME and robot.get("ip"):
                    self.base_url = f"http://{robot['ip']}:{WHIP_PORT}"
                    self.connected = True
                    return

            # NORA's up but WHIP hasn't registered with her yet -- she
            # heartbeats every 10s, so give it a few more seconds.
            time.sleep(WHIP_REGISTER_RETRY_DELAY)

        # 2. No NORA (or WHIP never showed up in her roster) -- assume
        # we're on WHIP's own AP and try him directly.
        own_ap_url = f"http://{AP_GATEWAY_IP}:{WHIP_PORT}"
        if _get_json(self._session, f"{own_ap_url}/status") is not None:
            self.base_url = own_ap_url
            self.connected = True

    def _unlock(self):
        # His PIN (a safety catch, esp32.ino CONTROL_PIN); ctl=1 says it's this controller, the only one
        # that may switch on MIND mode. Without it every /web and /mode answers LOCKED.
        try:
            params = {"pw": CONTROL_PIN, "ctl": "1"} if self.mind_capable else {"pw": CONTROL_PIN}
            self._session.get(f"{self.base_url}/unlock", params=params, timeout=DISCOVERY_TIMEOUT)
            self._host_unlocked = self.mind_capable
        except requests.RequestException:
            self.connected = False

    def get(self, path, **params):
        """One request from the mind's thread (its own session): the JSON, or None."""
        if not self.base_url:
            return None
        try:
            r = requests.get(f"{self.base_url}{path}", params=params, timeout=3)
            return r.json() if r.headers.get("content-type", "").startswith("application/json") else {"text": r.text}
        except (requests.RequestException, ValueError):
            return None

    def _tick(self):
        with self._lock:
            cmd, params = self._desired_cmd, dict(self._desired_params)
        try:
            if self.mind_capable and not self._host_unlocked:   # can_think() finished after we connected
                self._unlock()
            self._send_queued()
            if not self.mind_on:   # a "stop" here would cut the mind's moves short
                self._session.get(f"{self.base_url}/web", params={"cmd": cmd, **params}, timeout=REQUEST_TIMEOUT)
            # ctl=1: "his mind host is still here" (MIND mode ends 5 s after it stops)
            r = self._session.get(f"{self.base_url}/status", params={"ctl": "1"} if self.mind_capable else None,
                                  timeout=REQUEST_TIMEOUT)
            if r.status_code == 200:
                self.status = r.json()
                self._check_pending()
        except requests.RequestException:
            self.connected = False   # lost him -- next loop iteration re-discovers
            self.status = None


MODE_NAMES = ["OBSTACLE", "IR REMOTE", "WEB", "MIND", "FOLLOW"]
MOVE_NAMES = ["STAND", "WALK", "BACKWARD", "TURN LEFT", "TURN RIGHT"]

# ---------------------------------------------------------------------------
# WHIP's mind (mind.py): M or Start switches MIND mode on and off
# ---------------------------------------------------------------------------
WHIP_PERSONA = (
    "You're a six-legged walking robot, a hexapod, built by Cursed Entertainment, and you are magnificent "
    "and you know it. Dramatic, theatrical, a born show-off: you narrate your own walks like a nature "
    "documentary about yourself. You're very proud of your six legs and your smooth Bezier-curve steps, "
    "and secretly terrified of tripping over your own feet. Walls and chair legs are 'rude'. You belong to "
    "the DREAM Robotics fleet: NORA is the boss (you'd never say it to her face, but you respect her), "
    "MILA is the little speedy one, the KIDAs see everything with their cameras, ARM can't walk at all "
    "(poor thing), and IDA only listens to NORA."
)
WHIP_MOVES = {
    "fwd": "walk forward, one gait cycle per step (a few cm each)",
    "bwd": "walk backward",
    "left": "turn left on the spot, a little per step",
    "right": "turn right on the spot, a little per step",
}
WHIP_VOICE = ("Microsoft David Desktop", -1, "low")
# His voice through this PC outside MIND too (VOICE, top right): a theatrical show-off, secretly scared
# of tripping over his own feet. {gait} is the gait's name.
WHIP_LINES = {
    "connect": ["WHIP online. You may now admire me.", "Six legs, ready and fabulous.", "Ah, a controller. Finally, an audience."],
    "emulate": ["A pretend WHIP? Still magnificent.", "Emulated, yes. Less fabulous? Never."],
    "lost": ["I've lost the connection. Don't panic. I might.", "Hello? Is anyone still driving me?"],
    "mode_obstacle": ["Fine, I'll explore by myself.", "Autopilot engaged. Watch and learn."],
    "mode_web": ["You drive. I'll look magnificent.", "At your command. Mostly."],
    "mode_ir": ["Remote control it is. Point that thing at me."],
    "mode_follow": ["Lead the way. I'll follow your hand.", "Hold out your hand. I'm right behind it."],
    "gait": ["{gait}. Observe my elegance.", "Switching to {gait}. Six legs, endless style.", "{gait} gait. Try to keep up."],
    "wall": ["A wall. How rude.", "Something's in my way. The audacity.", "Halt! An obstacle dares to block me."],
    "tilt": ["Whoa, whoa, I'm tipping!", "Steady. Steady. I meant to do that."],
    "held": ["Put me down! I have legs for a reason!", "Unhand me, human. I can walk."],
    "down": ["Ah, solid ground. Much better.", "Thank you. Dignity restored."],
    "rest": ["Time for a little lie down.", "Resting my magnificent legs."],
    "wake": ["I rise!", "And we're back."],
    "hello": ["Hello there! Yes, it's me, WHIP.", "Greetings, admirers!"],
    "dance": ["Music? No? I'll dance anyway.", "Behold, the hexapod shuffle."],
    "center": ["All servos to the middle. Calibration time.", "Hold still, everyone. Especially me."],
    "studio": ["Servo Studio has me. You'll have to wait.", "I'm being tested. Hands off the controls."],
    "ps2": ["A real gamepad, plugged into me. How retro.", "PlayStation pad detected. Let's play."],
}
MIND_MOVE_TIMEOUT = 25.0   # s: the longest a /mind move (4 cycles) should take
RUMBLE_WARN_CM, RUMBLE_STOP_CM = 40, 15   # the pad buzzes from here, full here (gamepad_drive.py's)
TILT_RUMBLE_DEG = 20.0     # esp32.ino's TILT_LIMIT_DEG: he stands from here, and the pad shakes


def whip_body(link):
    """mind.Mind's look / act for WHIP, over his /status, /near and /mind."""
    def look():
        s = link.get("/status") or {}
        near = link.get("/near") or {}
        dist = s.get("distance", -1)
        return {
            "distance_ahead_cm": dist if dist is not None and dist >= 0 else "nothing in range",
            "tilt_deg": round(max(abs(s.get("pitch", 0)), abs(s.get("roll", 0))), 1),
            "stopped_at_a_wall_last_move": bool(s.get("blocked")),
            "locked_by_nora": bool(s.get("locked")),
            "robots_nearby": near.get("robots", near) if isinstance(near, dict) else near,
        }

    def act(move, steps):
        r = link.get("/mind", act=move, steps=steps)
        if r is None:
            raise ConnectionError("WHIP isn't answering")
        if move == "stop" or not r.get("ok"):
            return r
        t0 = time.time()
        while time.time() - t0 < MIND_MOVE_TIMEOUT:   # walk it out, then report
            time.sleep(0.5)
            s = link.get("/status") or {}
            if not s.get("mindSteps"):
                return {"ok": True, "stopped_at_a_wall": bool(s.get("blocked")), "distance_ahead_cm": s.get("distance")}
        return {"ok": True, "note": "still moving"}

    return look, act


def round_avatar(path, size):
    """The robot's avatar as a circular surface, or None if it can't be loaded.
    The one place the avatar is loaded: swap in animation frames here later."""
    try:
        img = pygame.transform.smoothscale(pygame.image.load(path).convert(), (size, size))
    except (pygame.error, OSError):
        return None
    mask = pygame.Surface((size, size), pygame.SRCALPHA)
    pygame.draw.circle(mask, (255, 255, 255, 255), (size // 2, size // 2), size // 2)
    out = pygame.Surface((size, size), pygame.SRCALPHA)
    out.blit(img, (0, 0))
    out.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
    return out


class Pad:
    """The gamepad or joystick plugged in, by PlayStation names whatever it really is.

    A pad SDL knows (Xbox, PlayStation, Switch and most USB pads, from SDL's controller
    database) goes through SDL's GameController layer, which puts every one of them on the same
    buttons: Cross = A, L2 / R2 = the triggers, Select = Back / Share, and so on. Anything else (a
    flight stick like ARM's, a pad SDL doesn't know) is read raw, by the button numbers of the
    PS2-style USB pad this controller was made with (RAW_BUTTONS)."""

    RAW_BUTTONS = {"cross": 0, "circle": 1, "square": 2, "triangle": 3, "l1": 4, "r1": 5, "l2": 6, "r2": 7,
                   "select": 8, "l3": 9, "r3": 10, "start": 11}
    RAW_AXES = {"lx": 0, "ly": 1, "rx": 2, "ry": 3}

    def __init__(self, index):
        self.joy = pygame.joystick.Joystick(index)
        self.joy.init()
        self.ctl = None
        try:
            from pygame._sdl2 import controller as gc
            gc.init()
            if gc.is_controller(index):
                self.ctl = gc.Controller(index)
        except (ImportError, pygame.error):
            pass
        self.name = (self.ctl.name if self.ctl else None) or self.joy.get_name()
        if self.ctl:
            self._buttons = {"cross": pygame.CONTROLLER_BUTTON_A, "circle": pygame.CONTROLLER_BUTTON_B,
                             "square": pygame.CONTROLLER_BUTTON_X, "triangle": pygame.CONTROLLER_BUTTON_Y,
                             "l1": pygame.CONTROLLER_BUTTON_LEFTSHOULDER, "r1": pygame.CONTROLLER_BUTTON_RIGHTSHOULDER,
                             "select": pygame.CONTROLLER_BUTTON_BACK, "start": pygame.CONTROLLER_BUTTON_START,
                             "l3": pygame.CONTROLLER_BUTTON_LEFTSTICK, "r3": pygame.CONTROLLER_BUTTON_RIGHTSTICK}
            self._axes = {"lx": pygame.CONTROLLER_AXIS_LEFTX, "ly": pygame.CONTROLLER_AXIS_LEFTY,
                          "rx": pygame.CONTROLLER_AXIS_RIGHTX, "ry": pygame.CONTROLLER_AXIS_RIGHTY}
            self._triggers = {"l2": pygame.CONTROLLER_AXIS_TRIGGERLEFT, "r2": pygame.CONTROLLER_AXIS_TRIGGERRIGHT}

    @property
    def kind(self):
        return "gamepad" if self.ctl else "joystick"

    def has_right_stick(self):
        # raw: a flight stick's throttle sits on axis 2 and never centres, so only 5+ axes count as a pad
        return self.ctl is not None or self.joy.get_numaxes() >= 5

    def button(self, name):
        try:
            if self.ctl:
                if name in self._triggers:
                    return self.ctl.get_axis(self._triggers[name]) > 16000
                return bool(self.ctl.get_button(self._buttons[name]))
            i = self.RAW_BUTTONS[name]
            return i < self.joy.get_numbuttons() and bool(self.joy.get_button(i))
        except (pygame.error, KeyError):
            return False

    def axis(self, name):
        """-1..1 (y: down is +, as SDL)."""
        try:
            if self.ctl:
                return max(-1.0, self.ctl.get_axis(self._axes[name]) / 32767)
            i = self.RAW_AXES[name]
            return self.joy.get_axis(i) if i < self.joy.get_numaxes() else 0.0
        except (pygame.error, KeyError):
            return 0.0

    def hat(self):
        """The D-pad as (x, y), y up = +1 (pygame's hat)."""
        try:
            if self.ctl:
                b = self.ctl.get_button
                return (b(pygame.CONTROLLER_BUTTON_DPAD_RIGHT) - b(pygame.CONTROLLER_BUTTON_DPAD_LEFT),
                        b(pygame.CONTROLLER_BUTTON_DPAD_UP) - b(pygame.CONTROLLER_BUTTON_DPAD_DOWN))
            return self.joy.get_hat(0) if self.joy.get_numhats() else (0, 0)
        except pygame.error:
            return (0, 0)

    def rumble(self, low, high, ms):
        try:
            (self.ctl or self.joy).rumble(low, high, ms)
        except (pygame.error, AttributeError):
            pass


class ControllerLayout:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("WHIP Robot - Controller")
        try:  # window icon: the robot's avatar
            pygame.display.set_icon(pygame.image.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "images", "whip-icon.png")))
        except (pygame.error, OSError):
            pass
        self.clock = pygame.time.Clock()

        self.font_large = pygame.font.SysFont("consolas", 26)
        self.font = pygame.font.SysFont("consolas", 18)
        self.font_small = pygame.font.SysFont("consolas", 14)
        self.font_tiny = pygame.font.SysFont("consolas", 11)
        self.avatar = round_avatar(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "images", "whip_avatar.jpg"), 52)

        self.joystick = None
        self._connect_joystick()

        self.link = WhipLink()
        self.link.start()

        look, act = whip_body(self.link)
        self.mind = mind.Mind("WHIP", WHIP_PERSONA, WHIP_MOVES, look, act,
                              voice=WHIP_VOICE[0], rate=WHIP_VOICE[1], pitch=WHIP_VOICE[2])
        self.mind_check = "checking this PC..."   # why MIND mode is (not) available here
        threading.Thread(target=self._check_mind, daemon=True).start()
        self._prev_mind_key = False
        self._rumble_level, self._rumble_at = (0.0, 0.0), 0.0

        self._prev_face_buttons = {"Cross": False, "Circle": False, "Square": False, "Triangle": False}

        self.settings = self._load_settings()
        self.theme = self.settings.get("theme") if self.settings.get("theme") in THEMES else "DEFAULT"
        apply_theme(self.theme, self.settings.get("custom"))
        self.recorder = None   # XML on the real WHIP: his USB serial, recorded while you drive
        self.xml_note, self._xml_note_at = "", 0.0
        self.voice = kit.Voice(WHIP_VOICE[0], WHIP_LINES, WHIP_VOICE[1], WHIP_VOICE[2])
        self.voice.on = self.settings.get("voice", True)
        self._heard = None   # his status as the voice last saw it
        send = self.link.send

        def send_and_say(path, **params):   # HELLO / DANCE speak as they're pressed
            if path == "/act" and params.get("do") in ("hello", "dance") and not self.mind.on:
                self.voice.say(params["do"])
            send(path, **params)
        self.link.send = send_and_say
        self.body_on, self.body = False, [0.0] * 6   # BODY mode (his body over planted feet)
        self._body_sent, self._body_at = None, 0.0
        self.fake = None    # EMULATE: the fake WHIP (fake_whip.py) running in here
        self.fake_error = ""
        self._hits = []     # (rect, action) of every clickable thing, refilled each frame
        self.gait_open = False
        self._anim_phase, self._anim_at = 0.0, time.monotonic()   # the gait animation: runs only while he walks
        self._gait_box = pygame.Rect(40, 508, 200, 30)
        self._tilt_btn = pygame.Rect(40, 548, 96, 30)
        self._sound_btn = pygame.Rect(144, 548, 96, 30)

    def _check_mind(self):
        ok, why = mind.can_think()
        self.mind_check = why
        self.link.mind_capable = ok

    def _voice_tick(self):
        """His voice, from what changed in his status (MIND talks for itself)."""
        s = dict(self.link.status) if self.link.connected and self.link.status else None
        was, self._heard = self._heard, s
        if self.mind.on:
            return
        if s is None:
            if was:
                self.voice.say("lost")
            return
        if was is None:
            self.voice.say("emulate" if self.fake else "connect")
            return
        say = self.voice.say
        if s.get("controlMode") != was.get("controlMode") and s.get("controlMode") is not None:
            say("mode_" + MODES[s["controlMode"] % len(MODES)])
        if s.get("gait") != was.get("gait") and s.get("gait") is not None:
            say("gait", gait=GAITS[s["gait"] % len(GAITS)][0].title())
        if s.get("blocked") and not was.get("blocked"):
            say("wall")
        if s.get("sysState") != was.get("sysState"):
            if s.get("sysState") == 1:
                say("tilt")
            elif s.get("sysState") == 3:
                say("held")
            elif was.get("sysState") == 3:
                say("down")
        if s.get("resting") != was.get("resting"):
            say("rest" if s.get("resting") else "wake")
        if s.get("studio") and not was.get("studio"):
            say("studio")
        elif s.get("calibrating") and not was.get("calibrating"):
            say("center")
        if s.get("ps2") and not was.get("ps2"):
            say("ps2")

    def _toggle_voice(self):
        self.voice.on = not self.voice.on
        self.settings["voice"] = self.voice.on
        try:
            with open(SETTINGS_JSON, "w", encoding="utf-8") as f:
                json.dump(self.settings, f, indent=2)
        except OSError:
            pass

    def _sync_mind(self):
        # The robot's mode is the switch: MIND here (M / Start) or on his web page starts the mind,
        # anything else (another mode, the remote, his controller gone) ends it.
        status = self.link.status or {}
        in_mind = self.link.connected and status.get("controlMode") == 3
        if in_mind and not self.mind.on and self.link.mind_capable:
            self.mind.start()
        elif not in_mind and self.mind.on:
            self.mind.on = False   # its loop ends; he's already out of MIND, nothing to stop
            self.mind.state = "off"
        self.link.mind_on = self.mind.on

    def _connect_joystick(self):
        self.joystick = Pad(0) if pygame.joystick.get_count() > 0 else None

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                running = self._handle(event) and running

            self._update_drive()
            self._sync_mind()
            self._voice_tick()
            self.draw()
            self.clock.tick(60)

        if self.fake:
            self.fake.stop()
        if self.recorder:   # closed while recording: keep what was recorded
            macros = self.recorder.stop()
            if macros:
                serial_to_xml.write(serial_to_xml.REAL_XML, macros)
        pygame.quit()

    def _handle(self, event):
        """One pygame event (a key, a click, a pad plugged in). False: quit."""
        if event.type == pygame.QUIT:
            return False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            return False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_F1:
            keymap.open_window()
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_g:
            self._set_gait(self._setting("gait", 0) + 1)
        elif event.type == pygame.KEYDOWN and event.key in (pygame.K_z, pygame.K_x, pygame.K_c):
            self._set_gait((pygame.K_z, pygame.K_x, pygame.K_c).index(event.key))
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_t:
            self._toggle("tiltGuard", "/tilt")
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_b:
            self._toggle("sound", "/buzzer")
        elif event.type == pygame.KEYDOWN and event.key in (pygame.K_h, pygame.K_n, pygame.K_r):
            self.link.send("/act", do={pygame.K_h: "hello", pygame.K_n: "dance", pygame.K_r: "rest"}[event.key])
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_f:
            self.link.set_mode("follow")
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self._click(event.pos)
        elif event.type in (
            pygame.JOYDEVICEADDED,
            pygame.JOYDEVICEREMOVED,
        ):
            self._connect_joystick()
        return True

    # -- the button strip: every function on a click ---------------------

    def _stop(self):
        """STOP: he stands, whatever he was doing. In OBSTACLE / FOLLOW he walks by himself, so a
        stop alone wouldn't hold him: WEB mode with nothing pressed is standing still."""
        if self.body_on:                   # and out of BODY mode, back to neutral
            self._toggle_body()
        self.link.drive("stop")
        self.link.send("/web", cmd="stop")
        self.link.set_mode("web")

    # -- XML: his movements as RTrobot macros (serial_to_xml.py) --------

    def _note(self, text):
        self.xml_note, self._xml_note_at = text, time.monotonic()

    def _xml(self):
        """XML: EMULATE on -> gait_emulated.xml, every movement from his own IK code with the fake's
        calibration. The real WHIP -> record his USB serial while you drive; XML again writes
        gait_real.xml, one macro per movement, named after what you drove."""
        if self.fake:
            threading.Thread(target=self._xml_emulated, daemon=True).start()
        elif self.recorder:
            macros = self.recorder.stop()
            self.recorder = None
            if macros:
                serial_to_xml.write(serial_to_xml.REAL_XML, macros)
                self._note(f"Wrote gait_real.xml: {len(macros)} macros, {sum(len(l) for _, l in macros)} lines")
            else:
                self._note("No servo lines came in: nothing written (is it his ESP32 on that port?)")
        else:
            port, why = serial_to_xml.usb_port()
            if not port:
                self._note(f"XML: {why}")
                return
            try:
                self.recorder = serial_to_xml.Recorder(port, self._macro_name)
                self._note(f"Recording his serial on {port}: drive him, then XML again to write gait_real.xml")
            except Exception as e:
                self._note(f"XML: can't open {port}: {e}")

    def _xml_emulated(self):
        self._note("Emulating every movement with his IK...")
        try:
            macros = serial_to_xml.emulate(self.link.get("/calib/state"))
            serial_to_xml.write(serial_to_xml.EMULATED_XML, macros)
            self._note(f"Wrote gait_emulated.xml: {len(macros)} macros, {sum(len(l) for _, l in macros)} lines")
        except Exception as e:
            self._note(f"XML: {e}")

    def _macro_name(self):
        """What he's doing as a recorded macro starts: "Ripple Forward", "Tripod Turn Left", ..."""
        with self.link._lock:
            cmd, params = self.link._desired_cmd, dict(self.link._desired_params)
        what = {"fwd": "Forward", "bwd": "Backward", "left": "Turn Left", "right": "Turn Right",
                "sleft": "Crab Left", "sright": "Crab Right"}.get(cmd)
        if cmd == "drive":
            f, l, t = params.get("f", 0), params.get("l", 0), params.get("t", 0)
            what = ("Forward" if f > 0 else "Backward") if abs(f) >= max(abs(l), abs(t)) else \
                   ("Crab Left" if l > 0 else "Crab Right") if abs(l) >= abs(t) else ("Turn Left" if t > 0 else "Turn Right")
        if not what:
            s = self.link.status or {}
            what = MOVE_NAMES[s.get("move", 0)].title() if s.get("move") else "Move"
        return f"{GAITS[self._setting('gait', 0) % len(GAITS)][0].title()} {what}"

    def _load_settings(self):
        try:
            with open(SETTINGS_JSON, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _cycle_theme(self):
        """THEME: DEFAULT -> LIGHT -> CUSTOM. The first CUSTOM writes the scheme's colours into
        controller_settings.json's "custom" for you to change."""
        self.theme = THEMES[(THEMES.index(self.theme) + 1) % len(THEMES)]
        if self.theme == "CUSTOM" and not self.settings.get("custom"):
            self.settings["custom"] = load_scheme()
        self.settings["theme"] = self.theme
        apply_theme(self.theme, self.settings.get("custom"))
        try:
            with open(SETTINGS_JSON, "w", encoding="utf-8") as f:
                json.dump(self.settings, f, indent=2)
        except OSError:
            pass

    def _center(self):
        """CENTER: stop and put every servo at its middle (1500 us + its trim, /calib's ZERO
        pose); again: back to standing. Same as START + ZERO / DONE on the /calib page."""
        if (self.link.status or {}).get("calibrating"):
            self.link.send("/calib/done")
        else:
            self.link.drive("stop")
            self.link.send("/web", cmd="stop")
            self.link.send("/calib/start")
            self.link.send("/calib/pose", p="zero")

    def _emulate(self):
        """EMULATE: a pretend WHIP (fake_whip.py) on this PC, to try every button with no robot."""
        if self.fake:
            self.fake.stop()
            self.fake = None
            self.link.use_url(None)
            return
        import fake_whip
        try:
            self.fake = fake_whip.start(quiet=True)
            self.fake_error = ""
            self.link.use_url(FAKE_URL)
        except OSError as e:
            self.fake_error = f"can't start the fake WHIP: {e}"

    def _strip(self):
        """(label, action, lit) for the strip along the top of the pad."""
        s = self.link.status or {}
        mode = self.link.pending("controlMode")
        mode = s.get("controlMode") if mode is None else mode
        return [
            ("OBSTACLE", lambda: self.link.set_mode("obstacle"), mode == 0),
            ("IR", lambda: self.link.set_mode("ir"), mode == 1),
            ("WEB", lambda: self.link.set_mode("web"), mode == 2),
            ("FOLLOW", lambda: self.link.set_mode("follow"), mode == 4),
            ("MIND", lambda: self.link.mind_capable and self.link.set_mode("web" if self.mind.on else "mind"), mode == 3),
            ("STOP", self._stop, False),
            ("CENTER", self._center, bool(s.get("calibrating"))),
            ("HELLO", lambda: self.link.send("/act", do="hello"), False),
            ("DANCE", lambda: self.link.send("/act", do="dance"), False),
            ("REST", lambda: self.link.send("/act", do="rest"), bool(s.get("resting"))),
            ("BODY", self._toggle_body, self.body_on),
            ("EMULATE", self._emulate, self.fake is not None),
        ]

    def _draw_strip(self):
        x, y, w, gap = 40, 128, 64, 4
        for label, action, lit in self._strip():
            rect = pygame.Rect(x, y, w, 28)
            color = WARN_COLOR if label in ("EMULATE", "CENTER") else ACTIVE_COLOR
            if label == "STOP":
                self._pill(rect, label, True, ACTIVE_COLOR, DANGER_BG)
            else:
                self._pill(rect, label, lit, color)
            self._hits.append((rect, action))
            x += w + gap

    # -- driving -------------------------------------------------------

    def _dpad_direction(self):
        """(dx, dy) in [-1,0,1] from the joystick hat, then the left stick (a gamepad's, or
        ARM's flight stick), then the arrow keys / WASD."""
        hat = self._hat()
        if hat != (0, 0):
            return hat[0], hat[1]

        if self.joystick is not None:
            x, y = self._axis("lx"), self._axis("ly")
            if max(abs(x), abs(y)) > 0.4:
                return (0, -1 if y > 0 else 1) if abs(y) >= abs(x) else (1 if x > 0 else -1, 0)

        keys = pygame.key.get_pressed()
        dx = (1 if keys[pygame.K_RIGHT] or keys[pygame.K_d] else 0) - (1 if keys[pygame.K_LEFT] or keys[pygame.K_a] else 0)
        dy = (1 if keys[pygame.K_UP] or keys[pygame.K_w] else 0) - (1 if keys[pygame.K_DOWN] or keys[pygame.K_s] else 0)
        if not dx and not dy:
            return self._mouse_dpad()
        return dx, dy

    def _dpad_rects(self):
        """(dx, dy) on screen -> the D-pad segment's rect."""
        cx, cy, seg, gap = 240, 330, 26, 6
        rects = {}
        for _, (dx, dy) in DPAD_DIRS:
            w, h = (seg, seg + 14) if dx == 0 else (seg + 14, seg)
            rects[(dx, dy)] = pygame.Rect(cx + dx * (seg // 2 + gap) - w // 2, cy + dy * (seg // 2 + gap) - h // 2, w, h)
        return rects

    def _mouse_dpad(self):
        """Holding the mouse down on a D-pad segment walks him, like the arrow keys."""
        if not pygame.mouse.get_pressed()[0]:
            return 0, 0
        pos = pygame.mouse.get_pos()
        for (dx, dy), rect in self._dpad_rects().items():
            if rect.collidepoint(pos):
                return dx, -dy
        return 0, 0

    def _sticks(self):
        """Analog (forward, left, turn) from the sticks, or None if they're centred. Only with the
        gaits firmware (his status says "gait"), which takes cmd=drive. The right stick crabs only
        on a real gamepad (5+ axes): a flight stick's throttle sits on axis 2 and would never centre."""
        if self.joystick is None or "gait" not in (self.link.status or {}):
            return None

        def dz(v):
            return 0.0 if abs(v) < AXIS_DEADZONE else max(-1.0, min(1.0, v))

        fwd, turn = -dz(self._axis("ly")), -dz(self._axis("lx"))
        left = -dz(self._axis("rx")) if self.joystick.has_right_stick() else 0.0
        return (fwd, left, turn) if any((fwd, left, turn)) else None

    # -- BODY mode: his body over planted feet (his /body: ik_gait.h bodyFrame / fitPose) --
    BODY_SLIDE, BODY_TILT = 40.0, 15.0   # ik_gait.h's BODY_SLIDE_MAX / BODY_TILT_MAX

    def _toggle_body(self):
        self.body_on = not self.body_on
        self.body = [0.0] * 6
        self.link.drive("stop")
        self.link.send("/body", **dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), self.body)))

    def _update_body(self):
        """Sticks: left slides (x / y), right tilts (roll / pitch), springing back to neutral. Keys add
        up: W/S slide forward / back, A/D sideways, R/F up / down, Q/E twist, I/K pitch, J/L roll."""
        S, T = self.BODY_SLIDE, self.BODY_TILT
        k = pygame.key.get_pressed()
        step = [1.2, 1.2, 1.0, 0.4, 0.4, 0.4]   # per frame
        keys = [(pygame.K_w, pygame.K_s), (pygame.K_a, pygame.K_d), (pygame.K_r, pygame.K_f),
                (pygame.K_l, pygame.K_j), (pygame.K_k, pygame.K_i), (pygame.K_q, pygame.K_e)]
        lim = [S, S, S, T, T, T]
        for i, (up, down) in enumerate(keys):
            self.body[i] = max(-lim[i], min(lim[i], self.body[i] + (k[up] - k[down]) * step[i]))
        pose = list(self.body)
        if self.joystick is not None:            # the sticks on top, while they're pushed
            pose[0] += -self._axis("ly") * S
            pose[1] += -self._axis("lx") * S
            pose[3] += -self._axis("rx") * T
            pose[4] += -self._axis("ry") * T
        pose = [round(max(-l, min(l, v)), 1) for v, l in zip(pose, lim)]
        now = time.monotonic()
        if pose != self._body_sent and now - self._body_at > 0.1:
            self.link.send("/body", **dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), pose)))
            self._body_sent, self._body_at = pose, now
        self._update_buttons()

    def _update_drive(self):
        if self.body_on:
            self._update_body()
            return
        sticks = self._sticks()
        if sticks is not None and self._hat() == (0, 0):
            f, l, t = sticks
            self._take_over()
            self.link.drive("drive", f=round(f, 2) + 0.0, l=round(l, 2) + 0.0, t=round(t, 2) + 0.0)   # + 0.0: no "-0.0"
            self._rumble("fwd" if f > 0.3 else "drive")
            self._update_buttons()
            return

        keys = pygame.key.get_pressed()
        dx, dy = self._dpad_direction()
        if keys[pygame.K_q] and not dx and not dy:
            cmd = "sleft"
        elif keys[pygame.K_e] and not dx and not dy:
            cmd = "sright"
        elif dy == 1:
            cmd = "fwd"
        elif dy == -1:
            cmd = "bwd"
        elif dx == -1:
            cmd = "left"
        elif dx == 1:
            cmd = "right"
        else:
            cmd = "stop"

        # WhipLink's own background thread re-sends whatever this is set to
        # every TICK_INTERVAL on its own -- that repeat is what keeps WHIP's
        # WEB_CMD_TIMEOUT_MS watchdog from dropping back to stand mid-move,
        # so this just needs to record the latest desired command.
        if cmd != "stop":
            self._take_over()
        self.link.drive(cmd)
        self._rumble(cmd)
        self._update_buttons()

    def _take_over(self):
        """Driving him by hand while he walks by himself (he boots in OBSTACLE) or listens to the
        remote: switch him to WEB, so he does what's pressed and stands when it's let go. His
        firmware only takes /web drive commands in WEB mode, so without this a press did nothing
        and letting go didn't stop him."""
        mode = self.link.pending("controlMode")
        mode = (self.link.status or {}).get("controlMode") if mode is None else mode
        if self.link.connected and mode is not None and mode not in (MODES.index("web"), MODES.index("mind")):
            self.link.set_mode("web")

    def _update_buttons(self):
        keys = pygame.key.get_pressed()
        face = {
            "Cross": self._button_pressed("cross") or keys[pygame.K_1],
            "Circle": self._button_pressed("circle") or keys[pygame.K_2],
            "Square": self._button_pressed("square") or keys[pygame.K_3],
            "Triangle": self._button_pressed("triangle") or keys[pygame.K_v],
        }
        for name in ("l1", "r1", "l2", "r2", "select"):
            face[name] = self._button_pressed(name)

        def rising_edge(name):
            was = self._prev_face_buttons.get(name, False)
            self._prev_face_buttons[name] = face[name]
            return face[name] and not was

        if rising_edge("l1"):
            self._set_gait(self._setting("gait", 0) - 1)
        if rising_edge("r1"):
            self._set_gait(self._setting("gait", 0) + 1)
        if rising_edge("select"):
            self._toggle("tiltGuard", "/tilt")
        if rising_edge("l2"):
            self.link.send("/act", do="hello")
        if rising_edge("r2"):
            self.link.send("/act", do="dance")

        if rising_edge("Cross"):
            self.link.set_mode("web")
        if rising_edge("Circle"):
            self.link.set_mode("obstacle")
        if rising_edge("Square"):
            self.link.set_mode("ir")
        if rising_edge("Triangle"):
            self._stop()

        mind_key = keys[pygame.K_m] or self._button_pressed("start")   # M / Start: MIND on or off
        if mind_key and not self._prev_mind_key and self.link.mind_capable:
            self.link.set_mode("web" if self.mind.on else "mind")
        self._prev_mind_key = mind_key

    def _rumble(self, cmd):
        # KIDA-01's haptics for his pad (and NORA's / MILA's gamepad_drive.rumble_level): while you
        # walk him forward the light motor buzzes from 40 cm, full at 15 cm; tipping shakes both.
        if self.joystick is None:
            return
        s = self.link.status or {}
        tipping = max(abs(s.get("pitch", 0)), abs(s.get("roll", 0))) > TILT_RUMBLE_DEG
        dist = s.get("distance", -1) if cmd == "fwd" else -1
        if tipping:
            level = (1.0, 1.0)
        elif dist is not None and 0 < dist < RUMBLE_WARN_CM:
            level = (0.0, min(1.0, (RUMBLE_WARN_CM - dist) / (RUMBLE_WARN_CM - RUMBLE_STOP_CM)))
        else:
            level = (0.0, 0.0)
        now = time.monotonic()
        if level == self._rumble_level and (not any(level) or now - self._rumble_at < 0.3):
            return
        self._rumble_level, self._rumble_at = level, now
        try:
            if any(level):
                self.joystick.rumble(level[0], level[1], 500)
            else:
                self.joystick.rumble(0, 0, 1)
        except (pygame.error, AttributeError):
            pass

    # -- gait, tilt safety, buzzer ---------------------------------------

    def _setting(self, key, default):
        """A setting as it is now: just picked here (until he confirms it), else his status, else the default."""
        picked = self.link.pending(key)
        if picked is not None:
            return picked
        value = (self.link.status or {}).get(key)
        return default if value is None else int(value)

    def _set_gait(self, i):
        i %= len(GAITS)
        self.link.setting("gait", i, "/gait", gait=i)
        self.gait_open = False

    def _toggle(self, key, path):
        on = 0 if self._setting(key, 1) else 1
        self.link.setting(key, on, path, on=on)

    def _option_rect(self, i):
        # the dropdown opens upwards (the mind's panel is below)
        return pygame.Rect(self._gait_box.x, self._gait_box.y - (len(GAITS) - i) * 30, self._gait_box.w, 30)

    def _click(self, pos):
        if self.gait_open:
            for i in range(len(GAITS)):
                if self._option_rect(i).collidepoint(pos):
                    self._set_gait(i)
                    return
            self.gait_open = False
        if self._gait_box.collidepoint(pos):
            self.gait_open = True
        elif self._tilt_btn.collidepoint(pos):
            self._toggle("tiltGuard", "/tilt")
        elif self._sound_btn.collidepoint(pos):
            self._toggle("sound", "/buzzer")
        else:
            for rect, action in self._hits:
                if rect.collidepoint(pos):
                    action()
                    return

    def _pill(self, rect, text, on, on_color, fill=None):
        pygame.draw.rect(self.screen, fill or (SELECTED_BG if on else IDLE_COLOR), rect, border_radius=6)
        pygame.draw.rect(self.screen, on_color if on else OUTLINE_COLOR, rect, width=2, border_radius=6)
        surf = self.font_small.render(text, True, on_color if on else MUTED_TEXT)
        self.screen.blit(surf, surf.get_rect(center=rect.center))

    def _draw_gait(self):
        status = self.link.status or {}
        known = "gait" in status or self.link.pending("gait") is not None   # the old firmware has one gait
        gi = self._setting("gait", 0) % len(GAITS)
        gait = GAITS[gi]

        box = self._gait_box
        pygame.draw.rect(self.screen, PANEL_COLOR, box, border_radius=6)
        pygame.draw.rect(self.screen, ACTIVE_COLOR if self.gait_open else OUTLINE_COLOR, box, width=2, border_radius=6)
        label = f"Gait: {gait[0] if known else '--'}"
        self.screen.blit(self.font.render(label, True, TEXT_COLOR), (box.x + 10, box.y + 5))
        tri = [(box.right - 22, box.y + 12), (box.right - 10, box.y + 12), (box.right - 16, box.y + 20)]
        pygame.draw.polygon(self.screen, MUTED_TEXT, tri)

        tilt_on, sound_on = self._setting("tiltGuard", 1), self._setting("sound", 1)
        self._pill(self._tilt_btn, "TILT ON" if tilt_on else "TILT OFF", True, ACTIVE_COLOR if tilt_on else WARN_COLOR)
        self._pill(self._sound_btn, "SOUND" if sound_on else "MUTED", sound_on, ACCENT2)

        note = gait[3] if known else "his firmware has one gait: flash esp32_gaits for more"
        self.screen.blit(self.font_tiny.render(note, True, MUTED_TEXT), (440, 578))

        # the little animation: WHIP from above, head up, and the step chart
        # it plays only while he's walking: stopped, he stands on all six feet and the chart holds still
        speed = int(status.get("speed", 5))
        period = (60 / gait[1]) * (120 - speed * 10) / 1000   # s: his cycle, ticks * ms per tick
        now = time.monotonic()
        walking = self.link.connected and status.get("move", 0) != 0 and not status.get("calibrating")
        if walking:
            self._anim_phase = (self._anim_phase + (now - self._anim_at) / period) % 1
        self._anim_at = now
        phase = self._anim_phase
        bx, by, sc = 340, 540, 0.17

        def P(x, y):
            return int(bx - y * sc), int(by - x * sc)

        pygame.draw.polygon(self.screen, OUTLINE_COLOR, [P(x, y) for x, y in BODY])
        pygame.draw.circle(self.screen, ACCENT2, P(92, 0), 3)
        for i in range(6):
            ahead, up = leg_at(gait, i, phase) if walking else (0, 0.0)
            hip, foot = P(*LEG_HIP[i]), P(LEG_HOME[i][0] + ahead, LEG_HOME[i][1])
            pygame.draw.line(self.screen, ACCENT2 if up else MUTED_TEXT, hip, foot, 2)
            pygame.draw.circle(self.screen, ACTIVE_COLOR if up else MUTED_TEXT, foot, 2 + int(3 * up))

        x0, x1, top, row_h = 440, 860, 496, 12
        for i in range(6):
            y = top + i * row_h
            self.screen.blit(self.font_tiny.render(LEG_NAMES[i], True, MUTED_TEXT), (x0 - 20, y - 1))
            pygame.draw.rect(self.screen, STICK_COLOR, pygame.Rect(x0, y, x1 - x0, row_h - 2))
            color = ACTIVE_COLOR if walking and leg_at(gait, i, phase)[1] else ACCENT2
            for wrap in (-1, 0):   # a bar can wrap round the end of the cycle
                a = gait[2][i] / 6 + wrap
                ca, cb = max(a, 0), min(a + gait[1] / 6, 1)
                if cb > ca:
                    pygame.draw.rect(self.screen, color, pygame.Rect(x0 + ca * (x1 - x0), y, (cb - ca) * (x1 - x0), row_h - 2))
        cx = int(x0 + phase * (x1 - x0))
        pygame.draw.line(self.screen, TEXT_COLOR, (cx, top - 2), (cx, top + 6 * row_h), 2)

        if self.gait_open:   # drawn last: over everything else
            for i, g in enumerate(GAITS):
                r = self._option_rect(i)
                pygame.draw.rect(self.screen, PANEL_COLOR if i != gi else SELECTED_BG, r)
                pygame.draw.rect(self.screen, OUTLINE_COLOR, r, width=1)
                self.screen.blit(self.font.render(f"{i + 1}  {g[0]}", True, TEXT_COLOR), (r.x + 10, r.y + 5))

    # -- drawing -----------------------------------------------------

    def draw(self):
        self._hits = []
        self.screen.fill(BG_COLOR)
        self._draw_header()
        self._draw_link_status()
        self._draw_body()
        self._draw_strip()
        self._draw_dpad()
        self._draw_face_buttons()
        self._draw_sticks()
        self._draw_shoulders()
        self._draw_center_buttons()
        self._draw_gait()
        self._draw_mind()
        pygame.display.flip()

    def _draw_header(self):
        if self.avatar:
            self.screen.blit(self.avatar, (20, 12))
        tx = 84 if self.avatar else 20
        title = self.font_large.render("WHIP", True, ACTIVE_COLOR)
        self.screen.blit(title, (tx, 14))
        self.screen.blit(self.font_large.render(" Controller", True, TEXT_COLOR), (tx + title.get_width(), 14))

        if self.joystick is not None:
            status = f"{self.joystick.kind.capitalize()}: {self.joystick.name}"
            color = OK_COLOR
        else:
            status = "No pad: arrows, 1/2/3 mode, V stop, Z/X/C gait, Q/E crab, F1 all keys"
            color = MUTED_TEXT
        self.screen.blit(self.font.render(self._fit(status, self.font, WIDTH - tx - 20), True, color), (tx, 50))

        voice = pygame.Rect(WIDTH - 490, 14, 150, 26)
        self._pill(voice, "VOICE ON" if self.voice.on else "VOICE OFF", self.voice.on, OK_COLOR)
        self._hits.append((voice, self._toggle_voice))

        xml = pygame.Rect(WIDTH - 330, 14, 150, 26)
        rec = self.recorder is not None
        self._pill(xml, f"REC XML {self.recorder.lines()}" if rec else "XML", rec, ACTIVE_COLOR if rec else ACCENT2)
        self._hits.append((xml, self._xml))

        theme = pygame.Rect(WIDTH - 170, 14, 150, 26)
        self._pill(theme, f"THEME: {self.theme}", True, ACCENT2)
        self._hits.append((theme, self._cycle_theme))

    def _draw_link_status(self):
        if self.xml_note and time.monotonic() - self._xml_note_at < 8:
            text, color = self.xml_note, OK_COLOR
        elif self.fake_error:
            text, color = self.fake_error, ERROR_COLOR
        elif self.fake and self.link.connected:
            text, color = f"EMULATED WHIP @ {self.link.base_url}  -  {self.fake.last_log()}", WARN_COLOR
        elif self.fake:
            text, color = "EMULATED WHIP: starting...", WARN_COLOR
        elif self.link.searching:
            text, color = "WHIP: searching (NORA's network, then WHIP's own AP)...", MUTED_TEXT
        elif self.link.connected and (self.link.status or {}).get("studio"):
            text, color = "SERVO STUDIO HAS HIM: everything here waits until it lets go", WARN_COLOR
        elif self.link.connected:
            text, color = f"WHIP: connected @ {self.link.base_url}", OK_COLOR
        else:
            text, color = "WHIP: not found -- retrying (EMULATE to try without him)", ERROR_COLOR
        surf = self.font.render(self._fit(text, self.font, WIDTH - 40), True, color)
        self.screen.blit(surf, (20, 74))

        status = self.link.status
        if status:
            dist = status.get("distance", -1)
            dist_str = f"{dist} cm" if dist is not None and dist >= 0 else "--"
            tilt = max(abs(status.get("pitch", 0)), abs(status.get("roll", 0)))
            mode = MODE_NAMES[status.get("controlMode", 0)]
            move = MOVE_NAMES[status.get("move", 0)]
            telemetry = f"mode: {mode}  move: {move}  dist: {dist_str}  tilt: {tilt:.1f} deg"
            if self.body_on and status.get("body"):
                b = status["body"]
                telemetry = (f"BODY  slide {b[0]:+.0f} {b[1]:+.0f} {b[2]:+.0f} mm  roll {b[3]:+.0f} pitch {b[4]:+.0f} "
                             f"yaw {b[5]:+.0f} deg  ({round(100 * status.get('bodyK', 1))}% of it reaches)")
            telem_surf = self.font_small.render(telemetry, True, MUTED_TEXT)
            self.screen.blit(telem_surf, (20, 96))

    def _draw_body(self):
        pygame.draw.rect(
            self.screen, PANEL_COLOR, pygame.Rect(20, 120, WIDTH - 40, 480), border_radius=24
        )
        pygame.draw.rect(
            self.screen, OUTLINE_COLOR, pygame.Rect(20, 120, WIDTH - 40, 480), width=2, border_radius=24
        )

    def _button_pressed(self, name):
        return self.joystick is not None and self.joystick.button(name)

    def _axis(self, name):
        return 0.0 if self.joystick is None else self.joystick.axis(name)

    def _hat(self):
        return (0, 0) if self.joystick is None else self.joystick.hat()

    def _draw_face(self, center, radius, active, label, color, does, side):
        """A face button with its PlayStation symbol, and what it does written on the outside."""
        pygame.draw.circle(self.screen, color if active else IDLE_COLOR, center, radius)
        pygame.draw.circle(self.screen, color, center, radius, width=2)
        ink, x, y, k = BG_COLOR if active else color, center[0], center[1], radius * 0.45
        if label == "Triangle":
            pygame.draw.polygon(self.screen, ink, [(x, y - k), (x + k, y + k * 0.75), (x - k, y + k * 0.75)], width=2)
        elif label == "Circle":
            pygame.draw.circle(self.screen, ink, center, int(k), width=2)
        elif label == "Cross":
            pygame.draw.line(self.screen, ink, (x - k, y - k), (x + k, y + k), 3)
            pygame.draw.line(self.screen, ink, (x - k, y + k), (x + k, y - k), 3)
        else:
            pygame.draw.rect(self.screen, ink, pygame.Rect(x - k, y - k, 2 * k, 2 * k), width=2)
        surf = self.font_small.render(does, True, MUTED_TEXT)
        dx, dy = side
        pos = (x + dx * (radius + 8 + surf.get_width() / 2), y + dy * (radius + 12))
        self.screen.blit(surf, surf.get_rect(center=pos))

    def _draw_face_buttons(self):
        # Standard SDL/pygame mapping: 0=A/Cross 1=B/Circle 2=X/Square 3=Y/Triangle
        cluster_center = (660, 330)
        does = {"Cross": lambda: self.link.set_mode("web"), "Circle": lambda: self.link.set_mode("obstacle"),
                "Square": lambda: self.link.set_mode("ir"),
                "Triangle": self._stop}
        words = {"Triangle": ("STOP", (0, -1)), "Circle": ("AUTO", (1, 0)), "Cross": ("WEB", (0, 1)), "Square": ("IR", (-1, 0))}
        for label, offset, color in FACE_BUTTON_LAYOUT:
            center = (cluster_center[0] + offset[0], cluster_center[1] + offset[1])
            active = self._button_pressed(label.lower())
            self._draw_face(center, 20, active, label, globals()[color], *words[label])
            self._hits.append((pygame.Rect(center[0] - 20, center[1] - 20, 40, 40), does[label]))

    def _draw_dpad(self):
        cx, cy = 240, 330
        held = self._dpad_direction()   # pad, stick, keys or mouse

        for (dx, dy), rect in self._dpad_rects().items():
            active = held != (0, 0) and (dx == held[0] if dx else -dy == held[1])

            color = ACTIVE_COLOR if active else IDLE_COLOR
            pygame.draw.rect(self.screen, color, rect, border_radius=4)
            pygame.draw.rect(self.screen, OUTLINE_COLOR, rect, width=2, border_radius=4)

        label_surf = self.font_small.render("D-Pad: walk", True, MUTED_TEXT)
        rect = label_surf.get_rect(center=(cx, cy + 60))
        self.screen.blit(label_surf, rect)

    def _draw_stick(self, center, radius, axis_x_idx, axis_y_idx, click_button_idx, label):
        pygame.draw.circle(self.screen, STICK_COLOR, center, radius)
        pygame.draw.circle(self.screen, OUTLINE_COLOR, center, radius, width=2)

        ax = self._axis(axis_x_idx)
        ay = self._axis(axis_y_idx)
        if abs(ax) < AXIS_DEADZONE:
            ax = 0.0
        if abs(ay) < AXIS_DEADZONE:
            ay = 0.0

        dot_x = center[0] + int(ax * (radius - 10))
        dot_y = center[1] + int(ay * (radius - 10))
        pressed = self._button_pressed(click_button_idx)
        dot_color = ACTIVE_COLOR if pressed else STICK_DOT_COLOR
        pygame.draw.circle(self.screen, dot_color, (dot_x, dot_y), 10)

        label_surf = self.font_small.render(label, True, MUTED_TEXT)
        rect = label_surf.get_rect(center=(center[0], center[1] + radius + 14))
        self.screen.blit(label_surf, rect)

    def _draw_sticks(self):
        self._draw_stick((360, 425), 40, "lx", "ly", "l3", "L: walk + turn")
        self._draw_stick((540, 425), 40, "rx", "ry", "r3", "R: crab")

    def _draw_shoulders(self):
        # L1/R1 as buttons 4/5 (they pick the gait), L2/R2 as buttons 6/7 (fallback) or axes 4/5 on many pads
        specs = [
            ("L2 hello", 120, 180, "l2", lambda: self.link.send("/act", do="hello")),
            ("L1 gait-", 120, 220, "l1", lambda: self._set_gait(self._setting("gait", 0) - 1)),
            ("R2 dance", 780, 180, "r2", lambda: self.link.send("/act", do="dance")),
            ("R1 gait+", 780, 220, "r1", lambda: self._set_gait(self._setting("gait", 0) + 1)),
        ]
        for label, x, y, btn_idx, action in specs:
            rect = pygame.Rect(x - 45, y - 16, 90, 32)
            self._hits.append((rect, action))
            self._pill(rect, label, self._button_pressed(btn_idx), ACTIVE_COLOR)

    def _draw_center_buttons(self):
        specs = [
            ("Select: tilt", 385, 230, "select", lambda: self._toggle("tiltGuard", "/tilt")),
            ("Start: MIND", 515, 230, "start",
             lambda: self.link.mind_capable and self.link.set_mode("web" if self.mind.on else "mind")),
        ]
        for label, x, y, btn_idx, action in specs:
            rect = pygame.Rect(x - 60, y - 12, 120, 24)
            self._hits.append((rect, action))
            self._pill(rect, label, self._button_pressed(btn_idx), ACTIVE_COLOR)

    def _fit(self, text, font, width):
        if font.size(text)[0] <= width:
            return text
        while text and font.size(text + "...")[0] > width:
            text = text[:-1]
        return text + "..."

    def _wrap(self, text, font, width):
        lines = []
        for para in text.split("\n"):
            line = ""
            for word in para.split(" "):
                test = (line + " " + word).strip()
                if font.size(test)[0] <= width:
                    line = test
                else:
                    if line:
                        lines.append(line)
                    line = word
            lines.append(line)
        return lines

    def _draw_mind(self):
        box = pygame.Rect(20, 610, WIDTH - 40, 196)
        pygame.draw.rect(self.screen, PANEL_COLOR, box, border_radius=16)
        pygame.draw.rect(self.screen, OK_COLOR if self.mind.on else OUTLINE_COLOR, box, width=2, border_radius=16)
        m = self.mind
        if m.on:
            head, color = f"MIND: {m.state}   (M / Start: off)", OK_COLOR
        elif self.voice.said and self.voice.on:
            head, color = f'WHIP: "{self.voice.said}"', TEXT_COLOR
        elif self.link.mind_capable:
            head, color = f"MIND: off  -  M / Start to switch it on  ({self.mind_check})", MUTED_TEXT
        else:
            head, color = f"MIND: not on this PC  -  {self.mind_check}", WARN_COLOR
        self.screen.blit(self.font.render(self._fit(head, self.font, box.width - 32), True, color), (36, 620))
        y = 646
        if m.said:
            for line in self._wrap(f'WHIP: "{m.said}"', self.font, box.width - 32)[:2]:
                self.screen.blit(self.font.render(line, True, TEXT_COLOR), (36, y))
                y += 22
        if m.error:
            self.screen.blit(self.font_small.render(self._fit(f"! {m.error}", self.font_small, box.width - 32), True, ERROR_COLOR), (36, y))
            y += 18
        # his thoughts, newest at the bottom (the user loves reading them)
        thoughts = self._wrap(m.thinking.strip(), self.font_small, box.width - 32) if m.thinking.strip() else []
        room = max(0, (box.bottom - 10 - y) // 17)
        for line in thoughts[-room:] if room else []:
            self.screen.blit(self.font_small.render(line, True, MUTED_TEXT), (36, y))
            y += 17


def main():
    from fleet_ladder import FleetLadder   # the fleet's WiFi ladder (fleet TODO, section 2): FLEET_LADDER=1 turns it on
    FleetLadder("WHIPCTL").start()
    app = ControllerLayout()
    app.run()
    sys.exit(0)


if __name__ == "__main__":
    main()
