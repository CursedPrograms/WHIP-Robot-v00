"""fake_whip.py - a pretend WHIP on this PC, to try the controller with no robot.

Answers the same web calls as esp32_gaits.ino (/unlock, /web, /mode, /status, /gait, /tilt,
/buzzer, /act, /mind, /near) and acts them out: he "walks" in a pretend room 2 m long, his sonar
reading drops as he walks at the wall, he stops at 20 cm when driven by hand, and every command
he gets is printed here, so you can see what the controller sends.

    python scripts/fake_whip.py              # fake WHIP + the controller, pointed at him
    python scripts/fake_whip.py --no-controller
                                             # just the fake WHIP (try http://127.0.0.1:5005/status)

Or press EMULATE on the controller: it runs this inside itself (start() / stop()).

Standard library only. Nothing here talks to the real robot.
"""

import json
import math
import os
import random
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HOST, PORT = "127.0.0.1", 5005    # WHIP's port, on this PC only
CONTROL_PIN = "1234"              # as esp32_gaits.ino
WEB_TIMEOUT_S = 0.5               # no /web for this long: he stands (as the firmware)
MIND_HOST_TTL_S = 5.0
MIN_DISTANCE_CM = 35              # obstacle mode turns away here, MIND refuses to walk at it
HAND_WALL_CM = 20                 # driven by hand: no walking forward closer than this
ROOM_CM = 200                     # the pretend wall ahead, from where he starts
WALK_CM_S = 8                     # how fast he walks at speed 5
TURN_DEG_S = 30

MODES = {"obstacle": 0, "ir": 1, "web": 2, "mind": 3, "follow": 4}
MODE_NAMES = {v: k for k, v in MODES.items()}
STAND, WALK, BACK, LEFT, RIGHT = range(5)
WEB_MOVES = {"fwd": WALK, "bwd": BACK, "left": LEFT, "right": RIGHT, "sleft": WALK, "sright": WALK}
GAITS = ("tripod", "ripple", "wave", "quad", "amble", "metachronal")
LEG_NAMES = ("LF", "LM", "LB", "RF", "RM", "RB")
JOINT_NAMES = ("hip", "knee", "foot")
DEFAULT_CAL = [   # as ik_gait.h's DEFAULT_CAL: channel, rev, trim
    [(1, 0, 0), (2, 0, 0), (3, 0, 0)], [(4, 0, 0), (5, 0, 0), (6, 0, 0)], [(7, 0, 0), (8, 0, 0), (9, 0, 0)],
    [(24, 0, 0), (25, 1, 0), (26, 1, 0)], [(27, 0, 0), (28, 1, 0), (29, 1, 0)], [(30, 0, 0), (31, 1, 0), (32, 1, 0)],
]


class Whip:
    def __init__(self):
        self.lock = threading.Lock()
        self.unlocked = False
        self.mind_host_seen = 0.0
        self.mode = MODES["obstacle"]       # he boots in obstacle avoidance
        self.move = STAND
        self.web_move, self.web_at = STAND, 0.0
        self.drive = (0.0, 0.0, 0.0)        # cmd=drive: f / l / t
        self.mind_move, self.mind_until = STAND, 0.0
        self.wall_cm = float(ROOM_CM)       # how far the wall ahead is
        self.heading = 0.0
        self.blocked = False
        self.gait, self.height, self.speed = 0, 5, 5
        self.tilt_guard, self.sound, self.resting = True, True, False
        self.calibrating = False
        self.calibrated = False
        self.body, self.body_k = (0.0,) * 6, 1.0
        self.cal = [[list(c) for c in leg] for leg in DEFAULT_CAL]   # [ch, rev, trim] per leg / joint
        self.started = time.monotonic()

    # ---- the pretend world, stepped 20 times a second ------------------------
    def step(self, dt):
        with self.lock:
            now = time.monotonic()
            move, fwd = STAND, 0.0
            if self.mode == MODES["web"]:
                if now - self.web_at > WEB_TIMEOUT_S:
                    self.web_move, self.drive = STAND, (0.0, 0.0, 0.0)
                move = self.web_move
                f, _, t = self.drive
                if move == STAND and (abs(f) > 0.05 or abs(t) > 0.05):
                    move = (WALK if f > 0 else BACK) if abs(f) >= abs(t) else (LEFT if t > 0 else RIGHT)
                fwd = f if self.web_move == STAND else {WALK: 1, BACK: -1}.get(move, 0)
            elif self.mode == MODES["mind"]:
                if now < self.mind_until:
                    move = self.mind_move
                    fwd = {WALK: 1, BACK: -1}.get(move, 0)
                if now - self.mind_host_seen > MIND_HOST_TTL_S:
                    self.mode = MODES["web"]
                    log("mind host gone: back to WEB")
            elif self.mode == MODES["obstacle"] and now - self.started > 5:
                move, fwd = (LEFT, 0) if self.wall_cm < MIN_DISTANCE_CM else (WALK, 1)
            elif self.mode == MODES["follow"]:
                move = STAND

            self.blocked = False
            if fwd > 0 and self.mode in (MODES["web"], MODES["mind"]) and self.wall_cm < HAND_WALL_CM:
                fwd, move, self.blocked = 0, STAND, True
            if self.resting or self.calibrating:
                move, fwd = STAND, 0

            pace = WALK_CM_S * (0.4 + self.speed / 8)
            self.wall_cm = min(400.0, max(5.0, self.wall_cm - fwd * pace * dt))
            if move in (LEFT, RIGHT):
                self.heading += (TURN_DEG_S if move == LEFT else -TURN_DEG_S) * dt
                if abs(self.heading) > 90:   # turned away from the wall: a fresh one, further off
                    self.heading, self.wall_cm = 0.0, float(random.randint(80, ROOM_CM))
            if move != self.move:
                log(f"move: {('STAND', 'WALK', 'BACKWARD', 'TURN LEFT', 'TURN RIGHT')[move]}"
                    + (f"  (wall {self.wall_cm:.0f} cm)" if move == WALK else ""))
                self.move = move
            if self.mode == MODES["mind"] and now >= self.mind_until:
                self.mind_move = STAND

    def status(self):
        t = time.monotonic()
        walking = self.move != STAND
        wobble = 2.0 if walking else 0.3
        return {
            "distance": round(self.wall_cm), "sysState": 1, "controlMode": self.mode, "move": self.move,
            "pitch": round(wobble * math.sin(t * 3.1), 1), "roll": round(wobble * math.cos(t * 2.3), 1),
            "wifiMode": 1, "network": "FAKE", "mpuFound": 1, "body": [round(v, 1) for v in self.body], "bodyK": round(self.body_k, 2), "bodyMode": 0,
            "calibrated": int(self.calibrated), "calibrating": int(self.calibrating),
            "locked": 0, "mindHost": int(t - self.mind_host_seen < MIND_HOST_TTL_S),
            "mindSteps": int(self.mode == MODES["mind"] and t < self.mind_until),
            "blocked": int(self.blocked), "height": self.height, "speed": self.speed, "gait": self.gait,
            "tiltGuard": int(self.tilt_guard), "sound": int(self.sound), "resting": int(self.resting),
        }


whip = Whip()
LOG = deque(maxlen=50)   # what he did, newest last (the controller shows the last line)
_quiet = [False]


def log(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    if not LOG or LOG[-1].split(" ", 1)[1] != msg:
        LOG.append(line)
        if not _quiet[0]:
            print(line.replace(" ", " WHIP ", 1), flush=True)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):   # quiet: log() says what matters
        pass

    def reply(self, code, body, ctype=None):
        data = body if isinstance(body, bytes) else (json.dumps(body) if isinstance(body, dict) else body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype or ("application/json" if isinstance(body, dict) else "text/plain"))
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        with whip.lock:
            self.route(url.path, q)

    def route(self, path, q):
        w, now = whip, time.monotonic()
        if path == "/status":
            if q.get("ctl") == "1" and w.mind_host_seen:
                w.mind_host_seen = now
            return self.reply(200, w.status())
        if path == "/":            # his own web page, straight out of his firmware (esp32_gaits.ino)
            page = web_page()
            if q.get("unlocked") == "1":   # for screenshots: the page as it looks once the PIN's in
                page = page.replace('<div id="lockOverlay" style="', '<div id="lockOverlay" style="display:none !important;', 1)
            return self.reply(200, page, "text/html; charset=utf-8")
        if path in ("/avatar.jpg", "/favicon.ico"):
            with open(os.path.join(HERE, "..", "images", "whip_avatar.jpg"), "rb") as f:
                return self.reply(200, f.read(), "image/jpeg")
        if path == "/pose":        # his 18 joint angles and body pose (the page's 3D view), from his IK
            return self.reply(200, pose_json(w))
        if path == "/near":
            return self.reply(200, {"robots": []})
        if path == "/unlock":
            if q.get("pw") != CONTROL_PIN:
                log("unlock: wrong PIN")
                return self.reply(403, {"ok": False, "error": "bad pin"})
            w.unlocked = True
            if q.get("ctl") == "1":
                w.mind_host_seen = now
            log("unlocked by the controller" + (" (his mind host)" if q.get("ctl") == "1" else ""))
            return self.reply(200, {"ok": True, "controller": "127.0.0.1"})
        # /calib/state needs no PIN on the real one either (only changing things does)
        if path == "/calib/state":
            return self.reply(200, {
                "calibrating": int(w.calibrating), "calibrated": int(w.calibrated), "height": w.height,
                "speed": w.speed, "gait": w.gait, "pose": "h",
                "legs": [{"name": LEG_NAMES[l], "servos": [
                    {"joint": JOINT_NAMES[k], "ch": c[0], "rev": c[1], "trim": c[2]} for k, c in enumerate(leg)]}
                    for l, leg in enumerate(w.cal)]})
        if not w.unlocked:
            return self.reply(403, "LOCKED")

        if path == "/web":
            cmd = q.get("cmd", "")
            w.web_at = now
            if cmd == "drive":
                w.web_move = STAND
                w.drive = tuple(max(-1.0, min(1.0, float(q.get(k, 0) or 0))) for k in "flt")
            elif cmd in WEB_MOVES:
                w.web_move, w.drive = WEB_MOVES[cmd], (0.0, 0.0, 0.0)
                if cmd in ("sleft", "sright"):
                    log(f"crab {'left' if cmd == 'sleft' else 'right'}")
            elif cmd == "stop":
                w.web_move, w.drive = STAND, (0.0, 0.0, 0.0)
            else:
                return self.reply(400, "bad cmd")
            if w.mode != MODES["web"] and cmd != "stop":
                return self.reply(200, "NOT IN WEB MODE")
            if w.resting and cmd != "stop":
                w.resting = False
                log("gets up")
            return self.reply(200, "OK")
        if path == "/mode":
            m = q.get("m")
            if m not in MODES:
                return self.reply(400, "bad mode")
            if m == "mind" and now - w.mind_host_seen > MIND_HOST_TTL_S:
                return self.reply(409, "no mind controller running")
            w.mode = MODES[m]
            log(f"mode -> {m.upper()}")
            return self.reply(200, "OK")
        if path == "/mind":
            if w.mode != MODES["mind"]:
                return self.reply(409, {"ok": False, "error": "not in mind mode"})
            act = q.get("act")
            steps = max(1, min(4, int(q.get("steps", 1) or 1)))
            if act == "stop":
                w.mind_until = 0
                return self.reply(200, {"ok": True})
            moves = {"fwd": WALK, "bwd": BACK, "left": LEFT, "right": RIGHT}
            if act not in moves:
                return self.reply(400, {"ok": False, "error": "bad act"})
            if act == "fwd" and w.wall_cm < MIN_DISTANCE_CM:
                w.blocked = True
                log(f"mind: fwd refused, wall at {w.wall_cm:.0f} cm")
                return self.reply(200, {"ok": False, "blocked": True, "distance": round(w.wall_cm)})
            w.mind_move, w.mind_until = moves[act], now + 1.2 * steps
            log(f"mind: {act} x{steps}")
            return self.reply(200, {"ok": True})
        if path == "/gait":
            if "gait" in q:
                w.gait = max(0, min(len(GAITS) - 1, int(q["gait"])))
                log(f"gait -> {GAITS[w.gait]}")
            for k in ("height", "speed"):
                if k in q:
                    setattr(w, k, max(0, min(9, int(q[k]))))
                    log(f"{k} -> {getattr(w, k)}")
            return self.reply(200, w.status())
        if path == "/tilt":
            w.tilt_guard = q.get("on", "1") != "0"
            log(f"tilt safety {'ON' if w.tilt_guard else 'OFF'}")
            return self.reply(200, {"tiltGuard": int(w.tilt_guard)})
        if path == "/buzzer":
            w.sound = q.get("on", "1") != "0"
            log("sound ON (beep!)" if w.sound else "sound muted")
            return self.reply(200, {"sound": int(w.sound)})
        if path == "/act":
            a = q.get("do")
            if a == "rest":
                w.resting = not w.resting
                log("lies down" if w.resting else "gets up")
            elif a in ("hello", "dance", "wake"):
                if a == "wake":
                    w.resting = False
                log({"hello": "waves hello", "dance": "dances", "wake": "gets up"}[a])
            else:
                return self.reply(400, "bad act")
            return self.reply(200, "OK")
        if path == "/calib/set":
            if not w.calibrating:
                return self.reply(400, {"ok": False})
            c = w.cal[int(q["leg"])][int(q["joint"])]
            if "ch" in q:
                c[0] = max(0, min(32, int(q["ch"])))
            if "rev" in q:
                c[1] = int(q["rev"] == "1")
            if "trim" in q:
                c[2] = max(-400, min(400, int(q["trim"])))
            log(f"{LEG_NAMES[int(q['leg'])]} {JOINT_NAMES[int(q['joint'])]}: ch {c[0]} rev {c[1]} trim {c[2]:+d} us")
            return self.reply(200, {"ok": True})
        if path == "/calib/save":
            w.calibrated = True
            log("calibration saved in flash")
            return self.reply(200, {"ok": True})
        if path == "/body":   # his body over planted feet: fitted with his own maths (whip_body.fit_pose)
            import whip_body as wb
            want = tuple(max(-l, min(l, float(q.get(k, 0) or 0))) for k, l in
                         zip(("x", "y", "z", "roll", "pitch", "yaw"), (40, 40, 40, 15, 15, 15)))
            z = -90 + (-80) * w.height / 9
            w.body, w.body_k = wb.fit_pose([(x, y, z) for x, y in wb.HOME], want)
            w.mode, w.web_move = MODES["web"], STAND
            if any(want):
                log(f"body: slide {w.body[0]:+.0f} {w.body[1]:+.0f} {w.body[2]:+.0f}  roll {w.body[3]:+.0f} "
                    f"pitch {w.body[4]:+.0f} yaw {w.body[5]:+.0f}" + ("" if w.body_k >= 1 else f" ({w.body_k:.0%})"))
            else:
                log("body: back to neutral")
            return self.reply(200, {"ok": True, "k": round(w.body_k, 2)})
        if path == "/calib/start":
            w.calibrating, w.web_move = True, STAND
            log("calibrating: stops and holds still")
            return self.reply(200, {"calibrating": 1})
        if path == "/calib/pose":
            if not w.calibrating:
                return self.reply(409, {"ok": False, "error": "press START first"})
            pose = q.get("p", "home")
            log("ZERO: every servo at 1500 us + its trim" if pose == "zero" else f"pose: {pose}")
            return self.reply(200, {"calibrating": 1})
        if path == "/calib/done":
            w.calibrating = False
            log("calibration done: stands")
            return self.reply(200, {"calibrating": 0})
        if path in ("/motorlockOn", "/motorlockOff", "/shutdown"):
            log(path[1:])
            return self.reply(200, "OK")
        return self.reply(404, "not found")


HERE = os.path.dirname(os.path.abspath(__file__))


def web_page():
    """INDEX_HTML from esp32_gaits.ino: the page he serves, as it is in his firmware."""
    with open(os.path.join(HERE, "esp32_gaits", "esp32_gaits.ino"), encoding="utf-8") as f:
        src = f.read()
    key = 'INDEX_HTML[] PROGMEM = R"rawhtml('
    start = src.index(key) + len(key)
    return src[start:src.index(')rawhtml"', start)]


def pose_json(w):
    """/pose as he answers it: joint angles of his stance (with the body pose), walking or not."""
    import whip_body as wb
    z = -90 + (-80) * w.height / 9
    feet = [(x, y, z) for x, y in wb.HOME]
    if w.move != STAND:   # walking: one tripod in the air at a time, swinging (just for the drawing)
        t = time.monotonic() * 2.5
        for l in range(6):
            off = 0 if l in (0, 2, 4) else math.pi
            feet[l] = (feet[l][0] + 30 * math.sin(t + off), feet[l][1], z + max(0.0, 40 * math.sin(t + off + 1.57)))
    a = [wb.angles(l, f) or (0, 0, 0) for l, f in enumerate(wb.body_frame(feet, *w.body))]
    return {"a": [[round(v, 1) for v in leg] for leg in a], "b": [round(v, 1) for v in w.body],
            "walk": int(w.move != STAND), "bm": 0}


def world(w, stop):
    last = time.monotonic()
    while not stop.wait(0.05):
        now = time.monotonic()
        w.step(now - last)
        last = now


class Server(ThreadingHTTPServer):
    allow_reuse_address = False   # on Windows it would let a second fake WHIP share the port silently


class FakeWhip:
    """A running fake WHIP: stop() ends it, last_log() is what he did last."""
    def __init__(self, server, stop):
        self._server, self._stop = server, stop

    def stop(self):
        self._stop.set()
        self._server.shutdown()
        self._server.server_close()

    @staticmethod
    def last_log():
        return LOG[-1] if LOG else "booted, in OBSTACLE mode"


def start(quiet=False):
    """A fresh fake WHIP on HOST:PORT. OSError if the port's taken (another one running?)."""
    global whip
    _quiet[0] = quiet
    whip = Whip()
    LOG.clear()
    server = Server((HOST, PORT), Handler)
    stop = threading.Event()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=world, args=(whip, stop), daemon=True).start()
    return FakeWhip(server, stop)


def main():
    try:
        fake = start()
    except OSError as e:
        sys.exit(f"Can't open {HOST}:{PORT} ({e}). Is another fake WHIP running?")
    url = f"http://{HOST}:{PORT}"
    print(f"Fake WHIP at {url} (status: {url}/status). He boots in OBSTACLE mode; Ctrl+C stops him.", flush=True)

    controller = None
    if "--no-controller" not in sys.argv:
        here = os.path.dirname(os.path.abspath(__file__))
        controller = subprocess.Popen([sys.executable, os.path.join(here, "controller.py")],
                                      env={**os.environ, "WHIP_URL": url})
        print("Controller started, pointed at the fake WHIP.", flush=True)
    try:
        while controller is None or controller.poll() is None:
            time.sleep(0.5)
        print("Controller closed: fake WHIP stops.")
    except KeyboardInterrupt:
        if controller:
            controller.terminate()
    fake.stop()


if __name__ == "__main__":
    main()
