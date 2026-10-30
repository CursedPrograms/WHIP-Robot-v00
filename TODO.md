# WHIP: TODO

## Walking by IK (2026-10-14)

WHIP now walks only by inverse kinematics: `scripts/esp32/ik_gait.h`, ported from Jakob Leander's hexapod
(https://github.com/JakobLeander/hexapod, MIT; same chassis). The recorded `gait.xml` gaits are no longer
in the firmware (the file stays in the repo). Everything online is unchanged: web page, fleet, IR,
obstacle avoidance, tilt safety, PIN.

- [ ] **Back legs slipping since the 2026-10-24 flash** (the user, 2026-10-25): the old map's reversed hips
      push the four corner feet ~90 deg off the body line (sideways, scrubbing), while the middle legs only
      run backwards. Flashed the fix 2026-10-26: check the corner feet push straight back now.
- [ ] **First flash of the U channel map (3a90afb): check forward.** If he walks backwards when you press
      forward, the hip reversal was wrong: flip all six hips' `rev` back (`ik_gait.h` DEFAULT_CAL,
      `whip_body.py`, `fake_whip.py`, `whip_calibration.json`, regenerate `gait_emulated.xml`), and bump
      `MAP_VERSION` to 3 with a `version < 3` hip flip in `remap()` for the saved calibration.
- [ ] Flash it with the big partition, like NORA: `esp32:esp32:esp32:PartitionScheme=huge_app`
      (scripts/flash.ps1 is set). It compiles (esp32 core 3.3.12): 1.9 MB = 60% of 3 MB; the default
      1.3 MB partition is too small, and was before the IK too (142%). Written and tested on a PC
      without the robot.
- [ ] Calibrate on a stand, feet off the ground: `http://<WHIP>:5005/calib`, unlock with the PIN.
      START, then ZERO. Every servo goes to its middle (90 degrees): each leg points straight out from
      the body, thigh level, foot straight down. That's how the user built WHIP, so trims should be
      near 0. WIGGLE each servo to check its channel, tick **rev** where a joint moves the wrong way,
      then HOME, SAVE, DONE.
- [x] First walks (2026-10-24, gaits sketch): walks better than the old firmware; servo offsets still to trim on /calib.
- [ ] **Fine-tune the height.** WHIP carries a 3S LiPo (a big RC-car pack, through a UBEC at 6 V)
      under the body, so he can't stand as low as Jakob's robot. Both sets of values are in
      `ik_gait.h`:
      - **Jakob's original**: feet 60-140 mm below the hips (`JAKOB_HOME_Z_MIN/MAX`); height 5 =
        104 mm, knees about 16 degrees up, the body hanging between them.
      - **WHIP's, in use**: 90-170 mm (`WHIP_HOME_Z_MIN/MAX`); height 5 = 134 mm, thighs about level.
        Every walking / turning pose is in reach down to 170 mm.
      - Pick the lowest height (slider on /calib) where the battery still clears the floor. Then
        adjust `WHIP_HOME_Z_MIN` so height 0 is just that, or swap `HOME_Z_MIN/MAX` to `JAKOB_*`
        if the battery moves.
- [x] **Smooth the swing with a Bezier curve** (2026-10-14). A stepping foot now arcs over on a
      quintic Bezier (`ik::swing`, points a, a, a+up, b+up, b, b): no corners, and it leaves and
      lands at zero speed. `ikStep` in esp32.ino plays each lift + land pair of key frames as one
      step of 2 x STEPS moves (same time as before); feet on the ground still move in straight
      lines. Checked on a PC: every swing, all gaits, heights 0-9, stays in reach and in servo range,
      peak still 30 mm (`LIFT`). Compiles at 1.89 MB. Still to watch on the robot: the feet now
      clear the ground higher near lift-off and touch-down than the old triangle did.
- [x] (crab, body IK: esp32_gaits) Later, easy with IK: walking sideways, "walk 10 cm", body tilt.

## Beyond most hexapods (2026-10-24)

What the serious hexapods (Phoenix, PhantomX, the ROS ones) do that WHIP doesn't yet:

- [x] **Body IK on the sticks** (the big one): while he stands, the left stick slides his body (x / y),
      the right stick tilts it (roll / pitch), L1 / R1 twist it (yaw); the feet stay planted. ik_gait.h
      already solves any foot position: move the feet the other way in the body frame. On the PS2 pad
      (ps2.h), the PC controller's sticks, and a mode in Servo Studio.
- [ ] **Terrain adaptation**: keep the body level on a slope with the MPU6050 (tilt the body against it).
- [ ] **Foot contact**: a switch or current sense per foot, so each foot stops when it touches down.
- [ ] **Physics simulation** (Gazebo / ROS, or a simple one in the emulator): the fake WHIP and Servo
      Studio show his geometry, not whether he'd fall over.
- [ ] First: flash esp32_gaits, calibrate, and try what's built (six gaits, PS2 pad, /servo, Servo Studio
      over WiFi): all compiled and PC-tested, none of it on the robot yet.
- [x] **Gaits sketch** (2026-10-24, `scripts/esp32_gaits`, flash_gaits.bat): tripod, ripple and wave from
      one phase engine (gaits.h), smooth lift-off / touch-down, stops by stepping; gait dropdown with a
      little animation on the web page and the controller (G / 1 2 3, L1 / R1); tilt safety switch
      (T, Select); 4x4 RGB grid on GPIO18 (leds.h); buzzer on GPIO19 (buzzer.h). Compiles, PC-tested,
      flashed to WHIP 2026-10-24.
  - [ ] On the robot: does each gait walk straight? Is the grid the right way round (GRID_ROTATE /
        GRID_MIRROR / GRID_SERPENTINE in leds.h)? Left and right turns swapped (the old firmware turned
        right on TurnLeft)?
  - [x] Gait on the IR remote ("3"; NORA's IR link / RIFT still to do).
  - [x] More (2026-10-24): analog sticks + crab, follow-me, wave hello, dance, crouch when locked,
        rest after a minute (gait.xml's fold, belly down first), picked-up stop, look-around at walls.
        PC-tested (reach at heights 0-9), compiles, flashed 2026-10-24.
  - [ ] On the robot: IR 3-7 do what they should; REST_Z (belly on the floor?); HELD_G_HIGH / LOW
        (does walking set off the picked-up stop?); does HELLO keep him balanced?
  - [ ] When it's good, move it into scripts/esp32.

## Network (field test 2026-10-14: "WHIP did not show in RIFT nor did he make his own WiFi")

**Done 2026-10-18: the fleet's ladder** (fleet TODO, section 2): he joins the highest network above him,
else hosts "WHIP" + the fleet list on :5000, looks again every 60 s and starts over when his network goes
quiet; the WiFi password is now the fleet's (secrets.h; it was 12345678). Compiles, not flashed. The notes
below are from before, kept for the history. What the code did (`setupWiFi()` in esp32.ino):
- At boot he scans for "NORA". If he sees her, he **joins her network** (15 s to connect) and makes no
  "WHIP" network. His site is then at `http://192.168.4.<n>:5005` (NORA's DHCP), and the only place
  that address shows up is NORA's fleet list (`http://192.168.4.1:5000/robots`), which he registers
  with (`registerWithFleet()`). RIFT only learns about him through that list.
- If NORA isn't there (as last time), he hosts **"WHIP"** (password 12345678) and his site is at
  `http://192.168.4.1:5005`.
- So "no WHIP network" most likely means he joined NORA. "Not in RIFT" then follows from RIFT losing NORA
  (her web server was blocked while her page was open, so RIFT also lost the roster that lists WHIP).
  Being fixed in RIFT: probe NORA on :5000.

To do:
- [ ] Check on the robot: Serial2 debug console (GPIO17, 115200) prints "Joined NORA. IP: ..." or
      "AP "WHIP" up". Or open `http://192.168.4.1:5000/robots` while on NORA's WiFi.
- [x] (the ladder, 2026-10-18: he checks every 60 s) Same problem as MILA had: he decides **once, at boot**. If NORA starts later he hosts his own AP
      forever, and if NORA reboots he never rejoins. Keep looking for NORA in the background and switch
      (see the MILA fix in MILA-Robot-v00/scripts/esp8266/esp8266.ino).
- [ ] Show his IP somewhere you can see it: his own AP name? A beep pattern? NORA's page lists the fleet
      roster, so check that shows his address with a link.
- [ ] Registration uses `http.setTimeout(2000)` against NORA's :5000. Fine once NORA's fleet server
      answers reliably; re-check after the RIFT / NORA fixes.

**Note for the next session (on the other PC):** WHIP's code was changed on the main PC
(D:\cc\DREAM Robotics Ecosystem). Check `git log` / `git status` before working. If this file or
`scripts/esp32/ik_gait.h` is missing, the work hasn't been pushed yet: ask the user, don't rewrite it.
Robot problems the user types to TINA are tests of TINA, not faults to fix.

## Later (the user, 2026-10-25)

- [x] (2026-10-30, /chirp, grid too) **Talk the fleet's Brainfuck on his buzzer**: `talk_bf.h` from RIFT/Fleet/brainfuck_talk.py, one tone per
      symbol (as IDA / ARM / DREAM), in a lower voice than the girls. **The LED grid lights along with each
      symbol.** Not now: after the build.

## Obstacle avoidance: his legs are wider than the sonar's cone (the user, 2026-10-26)

- [x] WEAVE (button on his page, `/gait?weave=1`, saved): in obstacle mode he steers a little left and right
      as he walks, so the beam sweeps, and stops at 48 cm instead of 35. Try it; tune WEAVE_TURN / WEAVE_CYCLES.
- [ ] Maybe: the sonar on a pan servo (SG90 / MG90S on a free board channel, 10-23), sweeping about +-45
      degrees while he walks. Two more sonars at the corners won't do: they'd see his own legs.
