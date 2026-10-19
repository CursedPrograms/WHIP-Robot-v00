#!/usr/bin/env python3
"""
WHIP Robot - Controller

Draws a PS2/DualShock-style controller layout and lights up buttons/sticks/
dpad/triggers live if a joystick is connected -- and drives WHIP over WiFi:

    D-pad / arrow keys : forward / backward / turn left / turn right
    Cross (A)          : switch WHIP into WEB control mode (required before
                         drive commands do anything -- he boots into
                         Obstacle Avoidance mode, same as arduino.ino)
    Circle (B)         : switch WHIP back to Obstacle Avoidance mode
    Square (X)         : switch WHIP to IR Remote mode
    Triangle (Y)       : emergency stop (drops back to standing immediately)

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

import os
import sys
import threading
import time

import pygame
import requests

import keymap   # scripts/keymap.py: every key and button, in its own window (F1)
import mind     # scripts/mind.py: his LLM mind (MIND mode), shared by every robot's controller

WIDTH, HEIGHT = 900, 820   # the bottom 200 px: the mind's panel
BG_COLOR = (18, 18, 22)
PANEL_COLOR = (32, 32, 38)
OUTLINE_COLOR = (70, 70, 80)
TEXT_COLOR = (220, 220, 225)
MUTED_TEXT = (130, 130, 140)
IDLE_COLOR = (55, 55, 65)
ACTIVE_COLOR = (90, 200, 120)
WARN_COLOR = (220, 160, 80)
STICK_COLOR = (45, 45, 55)
STICK_DOT_COLOR = (90, 200, 120)
AXIS_DEADZONE = 0.15

FACE_BUTTON_LAYOUT = [
    # label, offset (x, y) relative to face-button cluster center, color when active
    ("Triangle", (0, -42), (100, 200, 220)),
    ("Circle", (42, 0), (220, 100, 100)),
    ("Cross", (0, 42), (100, 140, 220)),
    ("Square", (-42, 0), (200, 100, 200)),
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

        self.base_url = None
        self.connected = False
        self.searching = True
        self.status = None
        self.mind_on = False        # MIND mode: mind.py drives through /mind, so no /web ticks
        self.mind_capable = False   # mind.can_think() said yes: unlock and check in as his mind host
        self._host_unlocked = False

        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def drive(self, cmd):
        with self._lock:
            self._desired_cmd = cmd

    def set_mode(self, mode):
        # One-shot, fire-and-forget -- doesn't need to share the tick loop's
        # cadence, and mode switches are rare compared to drive commands.
        if self.base_url:
            threading.Thread(target=self._send_mode, args=(self.base_url, mode), daemon=True).start()

    def _send_mode(self, base_url, mode):
        try:
            self._session.get(f"{base_url}/mode", params={"m": mode}, timeout=REQUEST_TIMEOUT)
        except requests.RequestException:
            pass

    def _run(self):
        while True:
            if not self.connected:
                self.searching = True
                self._discover()
                self.searching = False
                if not self.connected:
                    time.sleep(RECONNECT_DELAY)
                    continue
                self._unlock()
            self._tick()
            time.sleep(TICK_INTERVAL)

    def _discover(self):
        # 1. NORA's fleet registry -- only answers if we're on her network.
        nora_url = f"http://{AP_GATEWAY_IP}:{NORA_FLEET_PORT}"
        for attempt in range(WHIP_REGISTER_RETRIES):
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
            cmd = self._desired_cmd
        try:
            if self.mind_capable and not self._host_unlocked:   # can_think() finished after we connected
                self._unlock()
            if not self.mind_on:   # a "stop" here would cut the mind's moves short
                self._session.get(f"{self.base_url}/web", params={"cmd": cmd}, timeout=REQUEST_TIMEOUT)
            # ctl=1: "his mind host is still here" (MIND mode ends 5 s after it stops)
            r = self._session.get(f"{self.base_url}/status", params={"ctl": "1"} if self.mind_capable else None,
                                  timeout=REQUEST_TIMEOUT)
            if r.status_code == 200:
                self.status = r.json()
        except requests.RequestException:
            self.connected = False   # lost him -- next loop iteration re-discovers
            self.status = None


MODE_NAMES = ["OBSTACLE", "IR REMOTE", "WEB", "MIND"]
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
MIND_MOVE_TIMEOUT = 25.0   # s: the longest a /mind move (4 cycles) should take


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

        self._prev_face_buttons = {"Cross": False, "Circle": False, "Square": False, "Triangle": False}

    def _check_mind(self):
        ok, why = mind.can_think()
        self.mind_check = why
        self.link.mind_capable = ok

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
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = None

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_F1:
                    keymap.open_window()
                elif event.type in (
                    pygame.JOYDEVICEADDED,
                    pygame.JOYDEVICEREMOVED,
                ):
                    self._connect_joystick()

            self._update_drive()
            self._sync_mind()
            self.draw()
            self.clock.tick(60)

        pygame.quit()

    # -- driving -------------------------------------------------------

    def _dpad_direction(self):
        """(dx, dy) in [-1,0,1] from the joystick hat, then the left stick (a gamepad's, or
        ARM's flight stick), then the arrow keys / WASD."""
        hat = self._hat()
        if hat != (0, 0):
            return hat[0], hat[1]

        if self.joystick is not None and self.joystick.get_numaxes() >= 2:
            try:
                x, y = self.joystick.get_axis(0), self.joystick.get_axis(1)
            except pygame.error:
                x = y = 0.0
            if max(abs(x), abs(y)) > 0.4:
                return (0, -1 if y > 0 else 1) if abs(y) >= abs(x) else (1 if x > 0 else -1, 0)

        keys = pygame.key.get_pressed()
        dx = (1 if keys[pygame.K_RIGHT] or keys[pygame.K_d] else 0) - (1 if keys[pygame.K_LEFT] or keys[pygame.K_a] else 0)
        dy = (1 if keys[pygame.K_UP] or keys[pygame.K_w] else 0) - (1 if keys[pygame.K_DOWN] or keys[pygame.K_s] else 0)
        return dx, dy

    def _update_drive(self):
        dx, dy = self._dpad_direction()
        if dy == 1:
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
        self.link.drive(cmd)

        keys = pygame.key.get_pressed()
        face = {
            "Cross": self._button_pressed(0) or keys[pygame.K_z],
            "Circle": self._button_pressed(1) or keys[pygame.K_x],
            "Square": self._button_pressed(2) or keys[pygame.K_c],
            "Triangle": self._button_pressed(3) or keys[pygame.K_v],
        }

        def rising_edge(name):
            was = self._prev_face_buttons[name]
            self._prev_face_buttons[name] = face[name]
            return face[name] and not was

        if rising_edge("Cross"):
            self.link.set_mode("web")
        if rising_edge("Circle"):
            self.link.set_mode("obstacle")
        if rising_edge("Square"):
            self.link.set_mode("ir")
        if rising_edge("Triangle"):
            self.link.drive("stop")

        mind_key = keys[pygame.K_m] or self._button_pressed(11)   # M / Start: MIND on or off
        if mind_key and not self._prev_mind_key and self.link.mind_capable:
            self.link.set_mode("web" if self.mind.on else "mind")
        self._prev_mind_key = mind_key

    # -- drawing -----------------------------------------------------

    def draw(self):
        self.screen.fill(BG_COLOR)
        self._draw_header()
        self._draw_link_status()
        self._draw_body()
        self._draw_dpad()
        self._draw_face_buttons()
        self._draw_sticks()
        self._draw_shoulders()
        self._draw_center_buttons()
        self._draw_mind()
        pygame.display.flip()

    def _draw_header(self):
        if self.avatar:
            self.screen.blit(self.avatar, (20, 12))
        tx = 84 if self.avatar else 20
        title = self.font_large.render("WHIP Robot - Controller", True, TEXT_COLOR)
        self.screen.blit(title, (tx, 16))

        if self.joystick is not None:
            status = f"Connected: {self.joystick.get_name()}"
            color = ACTIVE_COLOR
        else:
            status = "No controller detected - use arrow keys + Z/X/C/V"
            color = MUTED_TEXT
        status_surf = self.font.render(status, True, color)
        self.screen.blit(status_surf, (tx, 50))

        hint = self.font_small.render("ESC or close window to quit", True, MUTED_TEXT)
        self.screen.blit(hint, (WIDTH - hint.get_width() - 20, 20))

    def _draw_link_status(self):
        if self.link.searching:
            text, color = "WHIP: searching (NORA's network, then WHIP's own AP)...", WARN_COLOR
        elif self.link.connected:
            text, color = f"WHIP: connected @ {self.link.base_url}", ACTIVE_COLOR
        else:
            text, color = "WHIP: not found -- retrying", (200, 90, 90)
        surf = self.font.render(text, True, color)
        self.screen.blit(surf, (20, 74))

        status = self.link.status
        if status:
            dist = status.get("distance", -1)
            dist_str = f"{dist} cm" if dist is not None and dist >= 0 else "--"
            tilt = max(abs(status.get("pitch", 0)), abs(status.get("roll", 0)))
            mode = MODE_NAMES[status.get("controlMode", 0)]
            move = MOVE_NAMES[status.get("move", 0)]
            telemetry = f"mode: {mode}  move: {move}  dist: {dist_str}  tilt: {tilt:.1f} deg"
            telem_surf = self.font_small.render(telemetry, True, MUTED_TEXT)
            self.screen.blit(telem_surf, (20, 96))

    def _draw_body(self):
        pygame.draw.rect(
            self.screen, PANEL_COLOR, pygame.Rect(20, 120, WIDTH - 40, 480), border_radius=24
        )
        pygame.draw.rect(
            self.screen, OUTLINE_COLOR, pygame.Rect(20, 120, WIDTH - 40, 480), width=2, border_radius=24
        )

    def _button_pressed(self, index):
        if self.joystick is None:
            return False
        try:
            return bool(self.joystick.get_button(index))
        except pygame.error:
            return False

    def _axis(self, index):
        if self.joystick is None:
            return 0.0
        try:
            return self.joystick.get_axis(index)
        except pygame.error:
            return 0.0

    def _hat(self):
        if self.joystick is None or self.joystick.get_numhats() == 0:
            return (0, 0)
        try:
            return self.joystick.get_hat(0)
        except pygame.error:
            return (0, 0)

    def _draw_circle_button(self, center, radius, active, label, active_color=ACTIVE_COLOR):
        color = active_color if active else IDLE_COLOR
        pygame.draw.circle(self.screen, color, center, radius)
        pygame.draw.circle(self.screen, OUTLINE_COLOR, center, radius, width=2)
        label_surf = self.font_small.render(label, True, TEXT_COLOR)
        rect = label_surf.get_rect(center=(center[0], center[1] + radius + 14))
        self.screen.blit(label_surf, rect)

    def _draw_face_buttons(self):
        # Standard SDL/pygame mapping: 0=A/Cross 1=B/Circle 2=X/Square 3=Y/Triangle
        button_index = {"Cross": 0, "Circle": 1, "Square": 2, "Triangle": 3}
        cluster_center = (660, 330)
        for label, offset, color in FACE_BUTTON_LAYOUT:
            center = (cluster_center[0] + offset[0], cluster_center[1] + offset[1])
            active = self._button_pressed(button_index[label])
            self._draw_circle_button(center, 20, active, label, color)

    def _draw_dpad(self):
        cx, cy = 240, 330
        seg = 26
        gap = 6
        hat = self._hat()

        for label, (dx, dy) in DPAD_DIRS:
            rect_w, rect_h = (seg, seg + 14) if dx == 0 else (seg + 14, seg)
            rx = cx + dx * (seg // 2 + gap) - rect_w // 2
            ry = cy + dy * (seg // 2 + gap) - rect_h // 2
            rect = pygame.Rect(rx, ry, rect_w, rect_h)

            active = (dx, dy) == (hat[0], hat[1]) if (dx, dy) != (0, 0) else False
            if dx != 0 and hat[0] == dx and dy == 0:
                active = True
            if dy != 0 and hat[1] == -dy and dx == 0:
                active = True

            color = ACTIVE_COLOR if active else IDLE_COLOR
            pygame.draw.rect(self.screen, color, rect, border_radius=4)
            pygame.draw.rect(self.screen, OUTLINE_COLOR, rect, width=2, border_radius=4)

        label_surf = self.font_small.render("D-Pad", True, TEXT_COLOR)
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

        label_surf = self.font_small.render(label, True, TEXT_COLOR)
        rect = label_surf.get_rect(center=(center[0], center[1] + radius + 14))
        self.screen.blit(label_surf, rect)

    def _draw_sticks(self):
        # Left stick: axes 0/1, click button 9 (common SDL mapping)
        # Right stick: axes 2/3, click button 10
        self._draw_stick((360, 450), 45, 0, 1, 9, "L-Stick")
        self._draw_stick((540, 450), 45, 2, 3, 10, "R-Stick")

    def _draw_shoulders(self):
        # L1/R1 as buttons 4/5, L2/R2 as buttons 6/7 (fallback) or axes 4/5 on many pads
        specs = [
            ("L2", 120, 180, 4),
            ("L1", 120, 220, 6),
            ("R2", 780, 180, 5),
            ("R1", 780, 220, 7),
        ]
        for label, x, y, btn_idx in specs:
            rect = pygame.Rect(x - 45, y - 16, 90, 32)
            active = self._button_pressed(btn_idx)
            color = ACTIVE_COLOR if active else IDLE_COLOR
            pygame.draw.rect(self.screen, color, rect, border_radius=6)
            pygame.draw.rect(self.screen, OUTLINE_COLOR, rect, width=2, border_radius=6)
            label_surf = self.font_small.render(label, True, TEXT_COLOR)
            lrect = label_surf.get_rect(center=rect.center)
            self.screen.blit(label_surf, lrect)

    def _draw_center_buttons(self):
        specs = [
            ("Select", 400, 230, 8),
            ("Start (MIND)", 500, 230, 11),
        ]
        for label, x, y, btn_idx in specs:
            rect = pygame.Rect(x - 50, y - 12, 100, 24)
            active = self._button_pressed(btn_idx)
            color = ACTIVE_COLOR if active else IDLE_COLOR
            pygame.draw.rect(self.screen, color, rect, border_radius=12)
            pygame.draw.rect(self.screen, OUTLINE_COLOR, rect, width=2, border_radius=12)
            label_surf = self.font_small.render(label, True, TEXT_COLOR)
            lrect = label_surf.get_rect(center=rect.center)
            self.screen.blit(label_surf, lrect)

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
        pygame.draw.rect(self.screen, ACTIVE_COLOR if self.mind.on else OUTLINE_COLOR, box, width=2, border_radius=16)
        m = self.mind
        if m.on:
            head, color = f"MIND: {m.state}   (M / Start: off)", ACTIVE_COLOR
        elif self.link.mind_capable:
            head, color = f"MIND: off  -  M / Start to switch it on  ({self.mind_check})", MUTED_TEXT
        else:
            head, color = f"MIND: not on this PC  -  {self.mind_check}", WARN_COLOR
        self.screen.blit(self.font.render(head, True, color), (36, 620))
        y = 646
        if m.said:
            for line in self._wrap(f'WHIP: "{m.said}"', self.font, box.width - 32)[:2]:
                self.screen.blit(self.font.render(line, True, TEXT_COLOR), (36, y))
                y += 22
        if m.error:
            self.screen.blit(self.font_small.render(f"! {m.error}", True, (200, 90, 90)), (36, y))
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
