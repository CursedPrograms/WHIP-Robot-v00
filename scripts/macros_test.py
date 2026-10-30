"""macros.py's tests: python scripts/macros_test.py"""
import macros


def run():
    r = macros.Recorder()
    r.note("/web", {"cmd": "fwd"}, 0.0)          # not recording yet
    assert r.steps == []
    r.begin(10.0)
    r.note("/web", {"cmd": "fwd"}, 10.0)
    r.note("/web", {"cmd": "fwd"}, 10.2)         # the same drive, every frame: one step
    r.note("/calib/pose", {"p": "zero"}, 10.3)   # not something a macro repeats
    r.note("/act", {"do": "hello"}, 11.0)
    r.note("/web", {"cmd": "stop"}, 12.5)
    steps = r.end()
    assert steps == [{"t": 0.0, "path": "/web?cmd=fwd"}, {"t": 1.0, "path": "/act?do=hello"}, {"t": 2.5, "path": "/web?cmd=stop"}]

    sent = []
    p = macros.Player(lambda path, params: sent.append((path, params)))
    p.begin([{"t": 0.0, "path": "/web?cmd=fwd"}, {"t": 1.0, "path": "/web?cmd=stop"}], 0.0)
    for i in range(0, 11):
        p.tick(i / 10)
    drives = [s for s in sent if s[1].get("cmd") == "fwd"]
    assert len(drives) >= 4, "a drive step keeps being sent"
    assert sent[-1] == ("/web", {"cmd": "stop"})
    assert not p.on

    sent.clear()
    p.begin([{"t": 0.0, "path": "/web?cmd=fwd"}, {"t": 9.0, "path": "/act?do=hello"}], 0.0)
    p.tick(0.0)
    p.stop()                                      # you took over
    assert sent[-1] == ("/web", {"cmd": "stop"}) and not p.on
    sent.clear()
    p.tick(9.5)
    assert sent == [], "a stopped macro stays stopped"

    drive = macros.step_path("/web", {"cmd": "drive", "f": 0.5, "l": 0.0, "t": -0.25})
    assert macros.split_path(drive) == ("/web", {"cmd": "drive", "f": "0.5", "l": "0.0", "t": "-0.25"})
    print("all passed")


run()
