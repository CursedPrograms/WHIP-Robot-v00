"""macros.py - WHIP's user-made macros: what you did, with its timing, played back.

Saved as JSON in scripts/macros/, one file a macro, the steps his web page's macros use:
    {"name": "lap", "steps": [{"t": 0.0, "path": "/web?cmd=fwd"}, {"t": 1.4, "path": "/web?cmd=stop"}]}

A step is a call to WHIP, t seconds after the start. While a drive step is playing it's sent again and
again, because he stands if it goes quiet. Driving him yourself ends the macro.
"""
import json
import os
import re
import time
from urllib.parse import parse_qsl, urlencode, urlparse

FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "macros")
RECORDED = ("/web", "/act", "/mode", "/gait")
REPEAT_S = 0.2


def step_path(path, params):
    return path + ("?" + urlencode(params) if params else "")


def split_path(step):
    u = urlparse(step)
    return u.path, dict(parse_qsl(u.query))


def is_drive(step):
    path, params = split_path(step)
    return path == "/web" and params.get("cmd") != "stop"


class Recorder:
    def __init__(self):
        self.steps, self.start = [], 0.0
        self.on = False

    def begin(self, now):
        self.steps, self.start, self.on = [], now, True

    def note(self, path, params, now):
        """One call to him. A drive sent every frame is one step: only what changes counts."""
        if not self.on or path not in RECORDED:
            return
        step = step_path(path, params)
        if self.steps and self.steps[-1]["path"] == step:
            return
        self.steps.append({"t": round(now - self.start, 2), "path": step})

    def end(self):
        self.on = False
        return self.steps


class Player:
    """call(path, params) sends a step. tick() needs calling often, from the controller's loop."""

    def __init__(self, call):
        self.call = call
        self.steps, self.i, self.start, self.current, self.resent = [], 0, 0.0, None, 0.0
        self.on = False

    def begin(self, steps, now):
        self.steps, self.i, self.start, self.current, self.resent = steps, 0, now, None, now
        self.on = bool(steps)

    def tick(self, now):
        if not self.on:
            return
        while self.i < len(self.steps) and self.steps[self.i]["t"] <= now - self.start:
            self.current = self.steps[self.i]["path"]
            self.call(*split_path(self.current))
            self.resent = now
            self.i += 1
        if self.i >= len(self.steps):
            self.stop()
        elif self.current and is_drive(self.current) and now - self.resent >= REPEAT_S:
            self.call(*split_path(self.current))
            self.resent = now

    def stop(self):
        if self.on:
            self.call("/web", {"cmd": "stop"})
        self.on, self.current = False, None


def _file(name):
    return os.path.join(FOLDER, re.sub(r"[^A-Za-z0-9_-]+", "_", name) + ".json")


def names():
    if not os.path.isdir(FOLDER):
        return []
    return sorted(f[:-5] for f in os.listdir(FOLDER) if f.endswith(".json"))


def save(steps, name=None):
    os.makedirs(FOLDER, exist_ok=True)
    name = name or time.strftime("macro_%Y%m%d_%H%M%S")
    with open(_file(name), "w", encoding="utf-8") as f:
        json.dump({"name": name, "steps": steps}, f, indent=1)
    return name


def load(name):
    with open(_file(name), encoding="utf-8") as f:
        return json.load(f)["steps"]
