"""whip_body.py - WHIP's legs in Python: his IK (ik_gait.h's solve) and its exact reverse.

    angles(leg, foot)        foot position (mm, body frame) -> (hip, knee, foot) degrees, or None if out of reach
    joints(leg, hip, knee, foot)  degrees -> the leg's points: hip anchor, knee, ankle, foot tip (x, y, z mm)
    to_us(deg, trim, rev, servo) / to_deg(us, trim, rev, servo)   a servo's pulse <-> its angle: servo is
                             that joint's type from servo_config.json (servo_config.model(leg, joint)); MG995 if none

The numbers are ik_gait.h's (Jakob Leander's chassis): change them there and here together. x is
forward, y left, z up; the hip joints sit at z = 0.
"""

import math

THIGH_LENGTH = 84.0        # knee servo horn to foot servo horn
FOOT_LENGTH = 127.0        # foot servo horn to the tip of the foot
HIP_TO_KNEE_LENGTH = 28.0  # hip servo horn to knee servo horn
FOOT_TIP_DEG = 14.0        # the foot's tip isn't straight below its servo
US_PER_DEG = 10.49
US_CENTER = 1500
LEGS = ("LF", "LM", "LB", "RF", "RM", "RB")
JOINTS = ("hip", "knee", "foot")
ANCHOR = ((74, 39), (0, 64), (-74, 39), (74, -39), (0, -64), (-74, -39))
MOUNT_DEG = (45, 90, 135, -45, -90, -135)   # each leg's direction at 1500 us (ik_gait.h MOUNT_DEG): corners diagonal
JAKOB_MOUNT_DEG = (27.8, 90, 152.2, -27.8, -90, -152.2)   # his JAKOB MOUNT setting: corners along the hip's position
mount = JAKOB_MOUNT_DEG    # the one in use (JAKOB MOUNT is his default): set_mount() follows his setting (/calib/state jakobMount)


def set_mount(jakob):
    """JAKOB MOUNT on or off, as he has it: angles() and joints() use it from now on."""
    global mount
    mount = JAKOB_MOUNT_DEG if jakob else MOUNT_DEG
BODY = ((74, 39), (0, 64), (-74, 39), (-74, -39), (0, -64), (74, -39))   # the outline, round the body
# His channels, directions and trims until he's calibrated (ik_gait.h's DEFAULT_CAL): (ch, rev, trim)
DEFAULT_CAL = (((24, True, 0), (25, True, 0), (26, True, 0)), ((27, True, 0), (28, True, 0), (29, True, 0)),
               ((30, True, 0), (31, True, 0), (32, True, 0)), ((7, True, 0), (8, False, 0), (9, False, 0)),
               ((4, True, 0), (5, False, 0), (6, False, 0)), ((1, True, 0), (2, False, 0), (3, False, 0)))
HOME_Z = -124.0            # height 5 of WHIP's stance (WHIP_HOME_Z_MIN / MAX)
HOME = ((160, 123), (0, 177), (-160, 123), (160, -123), (0, -177), (-160, -123))   # feet, mm (FB_X, WIDTH_*)


MG995 = {"us_per_deg": US_PER_DEG, "us_center": US_CENTER, "us_min": 500, "us_max": 2500, "deg_max": 90}


def to_us(deg, trim=0, rev=False, servo=None):
    """As ik_gait.h's toUs(): this servo type's middle + trim + degrees x its us per degree, kept in its limits."""
    s = servo or MG995
    deg = max(-s["deg_max"], min(s["deg_max"], deg))
    us = s["us_center"] + trim + round((-deg if rev else deg) * s["us_per_deg"])
    return max(s["us_min"], min(s["us_max"], us))


def to_deg(us, trim=0, rev=False, servo=None):
    s = servo or MG995
    d = (us - s["us_center"] - trim) / s["us_per_deg"]
    return -d if rev else d


def angles(leg, foot):
    """ik_gait.h's solve(): foot (x, y, z) -> (hip, knee, foot) degrees, or None if out of reach."""
    ax, ay = ANCHOR[leg]
    anchor_deg = mount[leg]
    x, y, z = foot[0] - ax, foot[1] - ay, foot[2]
    hip = math.degrees(math.atan2(y, x)) - anchor_deg
    while hip > 180: hip -= 360
    while hip < -180: hip += 360
    length = math.hypot(x, y)
    if length < 1:
        return None
    x = foot[0] - (ax + x * HIP_TO_KNEE_LENGTH / length)
    y = foot[1] - (ay + y * HIP_TO_KNEE_LENGTH / length)
    h = math.hypot(x, y)
    l = math.hypot(h, z)
    try:
        foot_deg = math.degrees(math.acos((THIGH_LENGTH ** 2 + FOOT_LENGTH ** 2 - l * l)
                                          / (2 * THIGH_LENGTH * FOOT_LENGTH))) - 90 - FOOT_TIP_DEG
        vb = math.acos((l * l + THIGH_LENGTH ** 2 - FOOT_LENGTH ** 2) / (2 * l * THIGH_LENGTH))
    except ValueError:
        return None
    knee = math.degrees(vb + math.atan2(z, h))
    return hip, knee, foot_deg


def joints(leg, hip, knee, foot):
    """The reverse of angles(): the leg's points for these joint angles, as (x, y, z) mm:
    [hip anchor, knee, ankle, foot tip]. In the leg's own upright plane the thigh rises at `knee`
    degrees from the knee joint, and the foot's angle at the ankle is foot + 90 + FOOT_TIP_DEG."""
    ax, ay = ANCHOR[leg]
    yaw = math.radians(mount[leg] + hip)
    c, s = math.cos(yaw), math.sin(yaw)
    kx, ky = ax + HIP_TO_KNEE_LENGTH * c, ay + HIP_TO_KNEE_LENGTH * s
    k = math.radians(knee)
    ah, az = THIGH_LENGTH * math.cos(k), THIGH_LENGTH * math.sin(k)       # ankle, in (out, up)
    f = k + math.pi + math.radians(foot + 90 + FOOT_TIP_DEG)               # the foot's direction from the ankle
    fh, fz = ah + FOOT_LENGTH * math.cos(f), az + FOOT_LENGTH * math.sin(f)
    return [(ax, ay, 0.0), (kx, ky, 0.0), (kx + ah * c, ky + ah * s, az), (kx + fh * c, ky + fh * s, fz)]


def body_frame(feet, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """Body IK (ik_gait.h's bodyFrame): the feet stay planted, the body slides (x forward, y left, z up,
    mm) and tilts (roll, pitch, yaw, degrees) over them. feet: six (x, y, z), body frame at rest ->
    where each foot is, seen from the moved body: R^-1 (f - t), R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    r = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
         (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
         (-sp, cp * sr, cp * cr))
    out = []
    for fx, fy, fz in feet:
        d = (fx - x, fy - y, fz - z)
        out.append(tuple(r[0][k] * d[0] + r[1][k] * d[1] + r[2][k] * d[2] for k in range(3)))
    return out


BODY_SLIDE_MAX, BODY_TILT_MAX = 40.0, 15.0   # ik_gait.h's: mm, degrees either way


def in_reach(feet_body, servos=None):
    """ik_gait.h's inReach(): every foot solvable and every joint inside its servo's reach."""
    for leg, f in enumerate(feet_body):
        a = angles(leg, f)
        if a is None:
            return False
        for j in range(3):
            lim = (servos[leg][j]["deg_max"] if servos else 90)
            if abs(a[j]) > lim:
                return False
    return True


def fit_pose(feet, pose, servos=None):
    """ik_gait.h's fitPose(): (the pose scaled back until every foot reaches, k). pose: (x, y, z, roll, pitch, yaw)."""
    if in_reach(body_frame(feet, *pose), servos):
        return tuple(pose), 1.0
    lo, hi = 0.0, 1.0
    for _ in range(10):
        mid = (lo + hi) / 2
        if in_reach(body_frame(feet, *(v * mid for v in pose)), servos):
            lo = mid
        else:
            hi = mid
    return tuple(v * lo for v in pose), lo


def body_to_world(p, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """A point on the body (body frame) -> where it is on the ground's frame: R p + t (to draw him)."""
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    r = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
         (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
         (-sp, cp * sr, cp * cr))
    return tuple(sum(r[i][k] * p[k] for k in range(3)) + (x, y, z)[i] for i in range(3))


if __name__ == "__main__":   # the reverse really is the reverse: random feet there and back
    import random
    worst = 0.0
    for _ in range(20000):
        leg = random.randrange(6)
        hx, hy = HOME[leg]
        p = (hx + random.uniform(-60, 60), hy + random.uniform(-50, 50), random.uniform(-180, -60))
        a = angles(leg, p)
        if a is None:
            continue
        q = joints(leg, *a)[-1]
        worst = max(worst, math.dist(p, q))
    print(f"worst round trip: {worst:.6f} mm")
