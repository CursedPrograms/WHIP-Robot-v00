"""serial_to_xml.py - WHIP's servo serial -> RTrobot XML macros, from the real robot or emulated.

His ESP32 drives the servo controller with text lines, "#<ch>P<us>...T<ms>", on the serial port that's
also wired to its USB plug. So whatever he does can be written down as RTrobot macros, the editor's
groups, one line per pose, exactly as he moved:

    python scripts/serial_to_xml.py --emulate          -> gait_emulated.xml: his IK and gaits run on this PC
                                                       (gait_test/serial_emu.cpp, his own gaits.h): Stand,
                                                       Zero, and every gait walking forward / back, turning
                                                       and crabbing, from standing to standing
    python scripts/serial_to_xml.py --port COM6        -> gait_real.xml: listen to the real WHIP on USB while he
                                                       moves; each burst of lines (a pause of 1.5 s ends it)
                                                       becomes a macro. Ctrl+C when done. --names "Fwd,Turn"
                                                       names them. --port auto: the one USB serial port

    --cal whip_calibration.json   his calibration (ik_values.py get), so the pulses are his real ones;
                                  default: that file if it's there, else his defaults (all trims 0)
    --out FILE                    another file instead (never gait.xml)

The controller does both with its XML button (EMULATE on: emulated; the real WHIP: records while you
drive, each macro named after what you drove, e.g. "Ripple Forward"). gait.xml, the hand-recorded
original, is never touched. Listening doesn't reset him: the port opens with DTR / RTS held off.
Leave the servo controller connected; the PC only listens.
"""

import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from xml.sax.saxutils import quoteattr

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
EMU_SRC = os.path.join(HERE, "gait_test", "serial_emu.cpp")
EMU_EXE = os.path.join(REPO, "build", "serial_emu.exe" if os.name == "nt" else "serial_emu")
DEFAULT_CAL = os.path.join(REPO, "whip_calibration.json")
GAIT_XML = os.path.join(REPO, "gait.xml")              # the hand-recorded original: never written
EMULATED_XML = os.path.join(REPO, "gait_emulated.xml")
REAL_XML = os.path.join(REPO, "gait_real.xml")
GROUPS = 255          # RTrobot's editor has 255 group slots
BAUD = 115200         # esp32_gaits.ino's CONTROLLER_BAUD
QUIET_S = 1.5         # this long without a line ends a recorded macro
SERVO_LINE = re.compile(r"^(?:#\d+P\d+)+T\d+$")


def arg(name, default=None):
    if name in sys.argv:
        i = sys.argv.index(name)
        value = sys.argv[i + 1]
        del sys.argv[i:i + 2]
        return value
    return default


def title(name):
    """'tripod turn left' -> 'Tripod Turn Left', like the hand-recorded macros' names."""
    return " ".join(w[:1].upper() + w[1:] for w in name.split(" "))


def to_xml(macros):
    """RTrobot's format, as the editor writes it: 255 <Group>s, one pose per line ending T<ms>D<ms>
    (move time, then wait as long before the next line, so the editor plays it at his pace)."""
    out = ['<?xml version="1.0" encoding="UTF-8"?>', "<RTrobot>"]
    for i in range(GROUPS):
        name, lines = macros[i] if i < len(macros) else ("", [])
        body = "\n".join(re.sub(r"T(\d+)$", r"T\1D\1", l) for l in lines)
        out.append(f"    <Group id=\"{i}\" alias={quoteattr(name)}>")
        out.append(f"        <value>{body}</value>")
        out.append("    </Group>")
    out.append("</RTrobot>")
    return "\n".join(out) + "\n"


def write(path, macros):
    if os.path.abspath(path) == os.path.abspath(GAIT_XML):
        raise ValueError("gait.xml is the hand-recorded original: write somewhere else")
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(to_xml(macros[:GROUPS]))


# ---- emulated: his own IK / gait code, run on this PC ----

def cal_text(state_or_path):
    """His /calib/state (a dict, or whip_calibration.json) -> serial_emu's cal.txt; None: his defaults."""
    state = state_or_path
    if isinstance(state_or_path, str) or state_or_path is None:
        if not state_or_path or not os.path.exists(state_or_path):
            return None
        with open(state_or_path, encoding="utf-8") as f:
            state = json.load(f)
    rows = [f"{state.get('height', 5)} {state.get('speed', 5)}"]
    rows += [f"{s['ch']} {int(bool(s['rev']))} {s['trim']}" for leg in state["legs"] for s in leg["servos"]]
    return "\n".join(rows) + "\n"


def compiler():
    found = shutil.which("g++")
    for p in (r"C:\msys64\ucrt64\bin\g++.exe", r"C:\msys64\mingw64\bin\g++.exe"):
        if not found and os.path.exists(p):
            found = p
    return found


def emulate(cal=DEFAULT_CAL):
    """[(name, [serial line, ...]), ...] for every movement, as his ESP32 would send them."""
    sources = [EMU_SRC] + [os.path.join(HERE, "esp32_gaits", f) for f in ("gaits.h", "ik_gait.h")]
    if not os.path.exists(EMU_EXE) or os.path.getmtime(EMU_EXE) < max(os.path.getmtime(s) for s in sources):
        gpp = compiler()
        if not gpp:
            raise RuntimeError("emulating needs g++ (MinGW / MSYS2 on Windows, build-essential on Linux)")
        os.makedirs(os.path.dirname(EMU_EXE), exist_ok=True)
        env = dict(os.environ, PATH=os.path.dirname(gpp) + os.pathsep + os.environ.get("PATH", ""))
        subprocess.run([gpp, "-std=gnu++17", "-O2", "-static", "-I", os.path.join(HERE, "gait_test", "stub"),
                        "-I", os.path.join(HERE, "esp32_gaits"), EMU_SRC, "-o", EMU_EXE],
                       check=True, env=env, capture_output=True)
    cmd = [EMU_EXE]
    text = cal_text(cal)
    if text:
        cal_file = os.path.join(REPO, "build", "serial_emu_cal.txt")
        os.makedirs(os.path.dirname(cal_file), exist_ok=True)
        with open(cal_file, "w") as f:
            f.write(text)
        cmd.append(cal_file)
    out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    macros = []
    for line in out.splitlines():
        if line.startswith("= "):
            macros.append((title(line[2:]), []))
        elif SERVO_LINE.match(line) and macros:
            macros[-1][1].append(line)
    return macros


# ---- real: his USB serial ----

def usb_port():
    """(the one USB serial port plugged in, "") or (None, why not)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None, "listening needs pyserial (pip install -r requirements.txt)"
    ports = [p.device for p in list_ports.comports() if p.vid]
    if len(ports) == 1:
        return ports[0], ""
    if not ports:
        return None, "no USB serial port: plug his ESP32 into this PC by USB"
    return None, f"more than one USB serial port: {', '.join(ports)}"


class Recorder:
    """Listens to his servo serial in the background. Each burst of lines becomes a macro named by
    namer() when it starts (the controller passes what's being driven). stop() -> the macros."""

    def __init__(self, port, namer=None):
        import serial
        self.macros, self._last, self._namer = [], 0.0, namer
        self._s = serial.Serial()
        self._s.port, self._s.baudrate, self._s.timeout = port, BAUD, 0.2
        self._s.dtr = self._s.rts = False   # held off before opening: toggling them resets the ESP32
        self._s.open()
        self._on = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def feed(self, raw):
        """One line from the port (also how a test plays lines in)."""
        if not SERVO_LINE.match(raw):
            return   # boot messages, half lines
        now = time.monotonic()
        if not self.macros or now - self._last > QUIET_S:
            name = (self._namer() if self._namer else "") or f"Recorded {len(self.macros) + 1}"
            same = sum(1 for n, _ in self.macros if n == name or n.startswith(name + " ("))
            self.macros.append((f"{name} ({same + 1})" if same else name, []))
        self.macros[-1][1].append(raw)
        self._last = now

    def _run(self):
        while self._on:
            try:
                self.feed(self._s.readline().decode("ascii", "replace").strip())
            except Exception:
                break   # unplugged

    def lines(self):
        return sum(len(l) for _, l in self.macros)

    def stop(self):
        self._on = False
        self._thread.join(1)
        self._s.close()
        return self.macros


def record(port, names):
    holder = []
    rec = Recorder(port, lambda: names[len(holder[0].macros)] if len(holder[0].macros) < len(names) else "")
    holder.append(rec)
    print(f"Listening to {port} at {BAUD}. Make him move; a {QUIET_S} s pause ends a macro. Ctrl+C when done.")
    try:
        while True:
            time.sleep(0.5)
            print(f"   {len(rec.macros)} macros, {rec.lines()} lines", end="\r")
    except KeyboardInterrupt:
        print()
    return rec.stop()


def main():
    port, out, cal = arg("--port"), arg("--out"), arg("--cal", DEFAULT_CAL)
    names = [n.strip() for n in (arg("--names", "") or "").split(",") if n.strip()]
    if "--emulate" in sys.argv:
        try:
            macros = emulate(cal)
        except RuntimeError as e:
            sys.exit(str(e))
        out = out or EMULATED_XML
        src = "emulated" + (f" with {os.path.basename(cal)}" if cal_text(cal) else " with his default calibration")
    elif port:
        if port == "auto":
            port, why = usb_port()
            if not port:
                sys.exit(why)
        try:
            macros = record(port, names)
        except ImportError:
            sys.exit("Listening needs pyserial: pip install -r requirements.txt")
        out = out or REAL_XML
        src = f"recorded on {port}"
    else:
        sys.exit(__doc__)
    if not macros:
        sys.exit("No servo lines: nothing written.")
    try:
        write(out, macros)
    except ValueError as e:
        sys.exit(str(e))
    print(f"Wrote {out} ({src}): {len(macros)} macros, {sum(len(l) for _, l in macros)} lines")
    for name, lines in macros:
        print(f"  {name:28} {len(lines):4} lines")


if __name__ == "__main__":
    main()
