#!/usr/bin/env python3
"""
WHIP Robot - Controller Layout Viewer

Pure UI: draws a PS2/DualShock-style controller layout and lights up
buttons/sticks/dpad/triggers live if a joystick is connected.

No robot control logic lives here - this is only for visualizing and
sanity-checking the physical controller input.
"""

import sys
import pygame

WIDTH, HEIGHT = 900, 560
BG_COLOR = (18, 18, 22)
PANEL_COLOR = (32, 32, 38)
OUTLINE_COLOR = (70, 70, 80)
TEXT_COLOR = (220, 220, 225)
MUTED_TEXT = (130, 130, 140)
IDLE_COLOR = (55, 55, 65)
ACTIVE_COLOR = (90, 200, 120)
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


class ControllerLayout:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("WHIP Robot - Controller Layout")
        self.clock = pygame.time.Clock()

        self.font_large = pygame.font.SysFont("consolas", 26)
        self.font = pygame.font.SysFont("consolas", 18)
        self.font_small = pygame.font.SysFont("consolas", 14)

        self.joystick = None
        self._connect_joystick()

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
                elif event.type in (
                    pygame.JOYDEVICEADDED,
                    pygame.JOYDEVICEREMOVED,
                ):
                    self._connect_joystick()

            self.draw()
            self.clock.tick(60)

        pygame.quit()

    # -- drawing -----------------------------------------------------

    def draw(self):
        self.screen.fill(BG_COLOR)
        self._draw_header()
        self._draw_body()
        self._draw_dpad()
        self._draw_face_buttons()
        self._draw_sticks()
        self._draw_shoulders()
        self._draw_center_buttons()
        pygame.display.flip()

    def _draw_header(self):
        title = self.font_large.render("WHIP Robot - Controller Layout", True, TEXT_COLOR)
        self.screen.blit(title, (20, 16))

        if self.joystick is not None:
            status = f"Connected: {self.joystick.get_name()}"
            color = ACTIVE_COLOR
        else:
            status = "No controller detected - showing static layout"
            color = MUTED_TEXT
        status_surf = self.font.render(status, True, color)
        self.screen.blit(status_surf, (20, 50))

        hint = self.font_small.render("ESC or close window to quit", True, MUTED_TEXT)
        self.screen.blit(hint, (WIDTH - hint.get_width() - 20, 20))

    def _draw_body(self):
        pygame.draw.rect(
            self.screen, PANEL_COLOR, pygame.Rect(20, 90, WIDTH - 40, HEIGHT - 110), border_radius=24
        )
        pygame.draw.rect(
            self.screen, OUTLINE_COLOR, pygame.Rect(20, 90, WIDTH - 40, HEIGHT - 110), width=2, border_radius=24
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
        cluster_center = (660, 300)
        for label, offset, color in FACE_BUTTON_LAYOUT:
            center = (cluster_center[0] + offset[0], cluster_center[1] + offset[1])
            active = self._button_pressed(button_index[label])
            self._draw_circle_button(center, 20, active, label, color)

    def _draw_dpad(self):
        cx, cy = 240, 300
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
        self._draw_stick((360, 420), 45, 0, 1, 9, "L-Stick")
        self._draw_stick((540, 420), 45, 2, 3, 10, "R-Stick")

    def _draw_shoulders(self):
        # L1/R1 as buttons 4/5, L2/R2 as buttons 6/7 (fallback) or axes 4/5 on many pads
        specs = [
            ("L2", 120, 150, 4),
            ("L1", 120, 190, 6),
            ("R2", 780, 150, 5),
            ("R1", 780, 190, 7),
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
            ("Select", 400, 200, 8),
            ("Start", 500, 200, 11),
        ]
        for label, x, y, btn_idx in specs:
            rect = pygame.Rect(x - 35, y - 12, 70, 24)
            active = self._button_pressed(btn_idx)
            color = ACTIVE_COLOR if active else IDLE_COLOR
            pygame.draw.rect(self.screen, color, rect, border_radius=12)
            pygame.draw.rect(self.screen, OUTLINE_COLOR, rect, width=2, border_radius=12)
            label_surf = self.font_small.render(label, True, TEXT_COLOR)
            lrect = label_surf.get_rect(center=rect.center)
            self.screen.blit(label_surf, lrect)


def main():
    app = ControllerLayout()
    app.run()
    sys.exit(0)


if __name__ == "__main__":
    main()
