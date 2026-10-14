# WHIP: TODO

## Walking by IK (2026-10-14)

WHIP now walks only by inverse kinematics: `scripts/esp32/ik_gait.h`, ported from Jakob Leander's hexapod
(https://github.com/JakobLeander/hexapod, MIT; same chassis). The recorded `gait.xml` gaits are no longer
in the firmware (the file stays in the repo). Everything online is unchanged: web page, fleet, IR,
obstacle avoidance, tilt safety, PIN.

- [ ] Flash it with the big partition, like NORA: `esp32:esp32:esp32:PartitionScheme=huge_app`
      (scripts/flash.ps1 is set). It compiles (esp32 core 3.3.12): 1.9 MB = 60% of 3 MB; the default
      1.3 MB partition is too small, and was before the IK too (142%). Written and tested on a PC
      without the robot.
- [ ] Calibrate on a stand, feet off the ground: `http://<WHIP>:5005/calib`, unlock with the PIN.
      START, then ZERO. Every servo goes to its middle (90 degrees): each leg points straight out from
      the body, thigh level, foot straight down. That's how the user built WHIP, so trims should be
      near 0. WIGGLE each servo to check its channel, tick **rev** where a joint moves the wrong way,
      then HOME, SAVE, DONE.
- [ ] First walks: slow (speed 2-3), still on the stand, then on the floor.
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
- [ ] **Smooth the swing with a Bezier curve** (the user wants this remembered). Right now a stepping
      foot moves between key frames in straight lines: up, across, down. The sharp corners make the
      servos reverse at once, which jerks them and wears them out. A Bezier swing path is C2-continuous,
      so the force on the leg changes gradually. Replace the linear interpolation of the swinging
      tripod (ikMoveTo / ik::lerp) with a Bezier path: lift-off, an arc over, touch-down. Feet on
      the ground keep moving in straight lines.
- [ ] Later, easy with IK: walking sideways, "walk 10 cm", body tilt, more gaits (wave, ripple).

**Note for the next session (on the other PC):** WHIP's code was changed on the main PC
(D:\cc\DREAM Robotics Ecosystem). Check `git log` / `git status` before working. If this file or
`scripts/esp32/ik_gait.h` is missing, the work hasn't been pushed yet: ask the user, don't rewrite it.
Robot problems the user types to TINA are tests of TINA, not faults to fix.
