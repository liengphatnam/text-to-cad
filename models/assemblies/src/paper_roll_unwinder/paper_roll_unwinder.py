"""Paper roll unwinder (máy xả cuộn giấy) — a shaft-type mill roll stand.

Layout
------
A floor-standing unwind stand that feeds a 1600 mm web off a 1200 mm paper
roll, through a pivoting dancer loop, to a downstream converting machine
(corrugator, printer, sheeter).

* Units: millimeters.
* Origin: on the floor, under the roll axis, on the machine's centre plane.
* X: across the machine (the roll axis). Y: web feed direction (+Y is the
  exit). Z: up. The floor is Z=0 (the underside of the levelling feet).
* Fixed root: the welded base frame. The uprights, the roller stand and the
  control box are bolted to it.
* Roll assembly: air shaft + core + roll + chucks + brake disc, one group that
  turns about the roll axis (`roll_spin`). The dancer arm swings about its
  pivot (`dancer_swing`); each guide roller turns on its own journal.
* The web is modelled as a 1 mm sheet tangent to every roller it wraps. It is
  static geometry: posing the dancer does not re-thread it.

Assumptions (no drawing was supplied): 3" (76 mm) core, 1200 mm roll OD,
1600 mm web, roll axis 1000 mm above the floor, 120 mm dia guide rollers,
disc brake for tension on the operator side and a hollow-shaft gearmotor on
the drive side for assisted unwinding.
"""

from __future__ import annotations

from math import acos, atan2, cos, degrees, radians, sin, sqrt

import cadgen
from cadgen import build123d as bd
from cadgen import glb, step

from upright_plate import HEAD_R, plate_solid

DISPLAY_NAME = "Paper roll unwinder"

# ---------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------

# Roll and web
ROLL_OD = 1200.0
ROLL_WIDTH = 1600.0
CORE_ID = 78.0               # 3" core, 1 mm over the 76 mm air shaft
CORE_OD = 100.0
CORE_OVERHANG = 10.0         # core sticks out of each roll face
WEB_WIDTH = ROLL_WIDTH
WEB_T = 1.0                  # drawn web thickness (visual)
AXIS_Z = 1000.0              # roll axis height above the floor

# Shaft and chucks
SHAFT_D = 76.0
SHAFT_X0, SHAFT_X1 = -1110.0, 1110.0
CHUCK_OD = 180.0
CHUCK_T = 30.0

# Base frame
FOOT_D, FOOT_H = 90.0, 30.0
RAIL_X = 950.0               # rail / upright centre, each side
RAIL_W, RAIL_H = 140.0, 120.0
RAIL_Y0, RAIL_Y1 = -700.0, 1900.0
BEAM_YS = (-640.0, 600.0, 1840.0)
BEAM_SECTION = 120.0
FRAME_TOP_Z = FOOT_H + RAIL_H            # 150

# Uprights (the plate's own dimensions live in upright_plate.py)
UPRIGHT_T = 40.0
UPRIGHT_BASE_HALF = 450.0
FOOTPLATE_T = 20.0
HOUSING_R = HEAD_R          # the plate's head and the housing share one radius
HOUSING_L = 100.0
BORE_R = 40.0

# Brake (operator side, +X)
DISC_R, DISC_T = 175.0, 16.0
DISC_X0 = 1040.0
HUB_R, HUB_L = 60.0, 60.0

# Gearmotor (drive side, -X)
GEARBOX = (180.0, 200.0, 200.0)
GEARBOX_X = -1090.0
MOTOR_R, MOTOR_L = 85.0, 300.0
MOTOR_X0 = -1480.0

# Roller stand and web path
GUIDE_R = 60.0
GUIDE_L = 1800.0
JOURNAL_R = 20.0
DANCER_PIVOT_R = 25.0
ROLLER1_Y = 1100.0
ROLLER2_Y = 1700.0
ROLLER_Z = AXIS_Z + ROLL_OD / 2 - GUIDE_R   # 1540: roller tops level with the roll top
POST_SECTION = 80.0
POST_TOP_Z = 1740.0
DANCER_Y = 1400.0
DANCER_Z = 900.0
DANCER_PIVOT_Z = ROLLER_Z
DANCER_ARM_X = 870.0
DANCER_ROLLER_L = 1680.0
EXIT_Y = 2150.0

# Control box
CONTROL_BOX = (120.0, 240.0, 300.0)
CONTROL_BOX_POS = (RAIL_X + 80.0, 150.0, 550.0)

# Motion (animation clip): web speed 20 m/min
WEB_SPEED = 20000.0 / 60.0   # mm/s

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _label(shape, name: str, color: str, alpha: float = 1.0):
    shape.label = name
    shape.color = cadgen.srgb(color, alpha)
    return shape


def _box(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float):
    return bd.Pos(cx, cy, cz) * bd.Box(sx, sy, sz)


def _x_cylinder(cx: float, cy: float, cz: float, r: float, length: float):
    """A cylinder whose axis runs along X, centred on (cx, cy, cz)."""
    return bd.Pos(cx, cy, cz) * bd.Cylinder(r, length, rotation=(0, 90, 0))


def _x_span_cylinder(x0: float, x1: float, cy: float, cz: float, r: float):
    x0, x1 = sorted((x0, x1))
    return _x_cylinder((x0 + x1) / 2, cy, cz, r, x1 - x0)


def _yz_face(points):
    """A planar face in the YZ plane (X=0) from (y, z) polygon corners."""
    wire = bd.Polyline(*[(0.0, y, z) for y, z in points], close=True)
    return bd.make_face(wire)


def _extrude_x(face, x0: float, x1: float):
    solid = bd.extrude(face, amount=x1 - x0, dir=(1, 0, 0))
    return bd.Pos(x0, 0, 0) * solid


def _circle_point(center, r: float, angle_deg: float):
    cy, cz = center
    a = radians(angle_deg)
    return (cy + r * cos(a), cz + r * sin(a))


def _crossed_tangent_angle(c1, c2, r: float) -> float:
    """Angle (deg, on circle c1) of the crossed-belt tangent from c1 to c2.

    Equal radii: the tangent crosses the centre line at its midpoint. Of the
    two solutions the one on the +Y side of c1 is returned (the web leaves
    the top of c1 towards c2).
    """
    my, mz = (c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2
    dy, dz = my - c1[0], mz - c1[1]
    length = sqrt(dy * dy + dz * dz)
    beta = degrees(acos(r / length))
    base = degrees(atan2(dz, dy))
    a, b = base + beta, base - beta
    return a if cos(radians(a)) > cos(radians(b)) else b


# ---------------------------------------------------------------------------
# parts
# ---------------------------------------------------------------------------


def _base_frame():
    parts = []
    for sx in (-1, 1):
        parts.append(_box(sx * RAIL_X, (RAIL_Y0 + RAIL_Y1) / 2, FOOT_H + RAIL_H / 2,
                          RAIL_W, RAIL_Y1 - RAIL_Y0, RAIL_H))
    for y in BEAM_YS:
        parts.append(_box(0, y, FOOT_H + BEAM_SECTION / 2,
                          2 * RAIL_X + RAIL_W, BEAM_SECTION, BEAM_SECTION))
    frame = parts[0]
    for p in parts[1:]:
        frame = frame + p
    return _label(frame, "base_frame", "#2F4A6B")


def _feet():
    feet = []
    for sx in (-1, 1):
        for y in (RAIL_Y0 + 100.0, RAIL_Y1 - 100.0):
            foot = bd.Pos(sx * RAIL_X, y, FOOT_H / 2) * bd.Cylinder(FOOT_D / 2, FOOT_H)
            side = "left" if sx < 0 else "right"
            end = "rear" if y < 0 else "front"
            feet.append(_label(foot, f"levelling_foot:{side}_{end}", "#1B1B1B"))
    return feet


def _upright(sx: int):
    x = sx * RAIL_X
    plate_z0 = FRAME_TOP_Z + FOOTPLATE_T
    # The side plate is its own part (upright_plate.py, with its DXF and PDF
    # drawings); here it is turned into the machine frame and welded up.
    plate = bd.Pos(x, 0, plate_z0) * bd.Rotation(0, 0, 90) * plate_solid()
    footplate = _box(x, 0, FRAME_TOP_Z + FOOTPLATE_T / 2, 160.0, 2 * UPRIGHT_BASE_HALF + 100.0, FOOTPLATE_T)
    housing = _x_cylinder(x, 0, AXIS_Z, HOUSING_R, HOUSING_L)
    body = plate + footplate + housing
    if sx < 0:
        # torque-arm lug for the hollow-shaft gearmotor
        body = body + _box(x, -140.0, AXIS_Z, 40.0, 60.0, 60.0)
    body = body - _x_cylinder(x, 0, AXIS_Z, BORE_R, HOUSING_L + 2.0)
    return _label(body, "upright_left" if sx < 0 else "upright_right", "#2F4A6B")


def _roller_stand():
    parts = []
    post_z = (FRAME_TOP_Z + POST_TOP_Z) / 2
    post_h = POST_TOP_Z - FRAME_TOP_Z
    for sx in (-1, 1):
        for y in (ROLLER1_Y, ROLLER2_Y):
            parts.append(_box(sx * RAIL_X, y, post_z, POST_SECTION, POST_SECTION, post_h))
        # side tie carrying the dancer pivot
        parts.append(_box(sx * RAIL_X, DANCER_Y, DANCER_PIVOT_Z, POST_SECTION,
                          ROLLER2_Y - ROLLER1_Y - POST_SECTION, POST_SECTION))
    for y in (ROLLER1_Y, ROLLER2_Y):
        parts.append(_box(0, y, POST_TOP_Z - POST_SECTION / 2, 2 * RAIL_X + POST_SECTION,
                          POST_SECTION, POST_SECTION))
    stand = parts[0]
    for p in parts[1:]:
        stand = stand + p
    # journal bores for both guide rollers and the dancer pivot (1 mm clearance)
    bore_len = 2 * RAIL_X + 2 * POST_SECTION
    for y in (ROLLER1_Y, ROLLER2_Y):
        stand = stand - _x_cylinder(0, y, ROLLER_Z, JOURNAL_R + 1.0, bore_len)
    stand = stand - _x_cylinder(0, DANCER_Y, DANCER_PIVOT_Z, DANCER_PIVOT_R + 1.0, bore_len)
    return _label(stand, "roller_stand", "#2F4A6B")


def _guide_roller(name: str, y: float):
    roller = _x_cylinder(0, y, ROLLER_Z, GUIDE_R, GUIDE_L)
    journal = _x_cylinder(0, y, ROLLER_Z, JOURNAL_R, 2 * RAIL_X + 40.0)
    return _label(roller + journal, name, "#C8CCD0")


def _dancer_arm():
    pivot = _x_span_cylinder(-RAIL_X - 40.0, RAIL_X + 40.0, DANCER_Y, DANCER_PIVOT_Z, DANCER_PIVOT_R)
    arm = pivot
    arm_len = DANCER_PIVOT_Z - DANCER_Z
    for sx in (-1, 1):
        x = sx * DANCER_ARM_X
        arm = arm + _box(x, DANCER_Y, (DANCER_PIVOT_Z + DANCER_Z) / 2, 40.0, 60.0, arm_len)
        arm = arm + _x_cylinder(x, DANCER_Y, DANCER_PIVOT_Z, 45.0, 40.0)
        arm = arm + _x_cylinder(x, DANCER_Y, DANCER_Z, 45.0, 40.0)
    # the dancer roller's journal bores
    arm = arm - _x_cylinder(0, DANCER_Y, DANCER_Z, JOURNAL_R + 1.0, 2 * DANCER_ARM_X + 60.0)
    return _label(arm, "dancer_arm", "#D97A1F")


def _dancer_roller():
    roller = _x_cylinder(0, DANCER_Y, DANCER_Z, GUIDE_R, DANCER_ROLLER_L)
    journal = _x_cylinder(0, DANCER_Y, DANCER_Z, JOURNAL_R, 2 * DANCER_ARM_X + 40.0)
    return _label(roller + journal, "dancer_roller", "#C8CCD0")


def _shaft():
    shaft = _x_span_cylinder(SHAFT_X0, SHAFT_X1, 0, AXIS_Z, SHAFT_D / 2)
    return _label(shaft, "air_shaft", "#4A4F55")


def _core():
    length = ROLL_WIDTH + 2 * CORE_OVERHANG
    core = _x_cylinder(0, 0, AXIS_Z, CORE_OD / 2, length) - _x_cylinder(0, 0, AXIS_Z, CORE_ID / 2, length + 2.0)
    return _label(core, "paper_core", "#6B4A2B")


def _paper_roll():
    roll = _x_cylinder(0, 0, AXIS_Z, ROLL_OD / 2, ROLL_WIDTH) - _x_cylinder(0, 0, AXIS_Z, CORE_OD / 2, ROLL_WIDTH + 2.0)
    return _label(roll, "paper_roll", "#C9A063")


def _chucks():
    chucks = []
    for sx in (-1, 1):
        x0 = ROLL_WIDTH / 2 + CORE_OVERHANG
        chuck = _x_span_cylinder(sx * x0, sx * (x0 + CHUCK_T), 0, AXIS_Z, CHUCK_OD / 2)
        chuck = chuck - _x_span_cylinder(sx * (x0 - 1.0), sx * (x0 + CHUCK_T + 1.0), 0, AXIS_Z, SHAFT_D / 2)
        chucks.append(_label(chuck, "chuck_left" if sx < 0 else "chuck_right", "#8A8F96"))
    return chucks


def _brake_disc():
    disc = _x_span_cylinder(DISC_X0, DISC_X0 + DISC_T, 0, AXIS_Z, DISC_R)
    hub = _x_cylinder(DISC_X0 + DISC_T / 2, 0, AXIS_Z, HUB_R, HUB_L)
    bore = _x_cylinder(DISC_X0 + DISC_T / 2, 0, AXIS_Z, SHAFT_D / 2, HUB_L + 2.0)
    return _label(disc + hub - bore, "brake_disc", "#5C6066")


def _brake_caliper():
    disc_mid = DISC_X0 + DISC_T / 2
    caliper = _box(disc_mid, 0, AXIS_Z + DISC_R, 40.0, 130.0, 90.0)
    # the pad slot the disc rim runs through (2 mm pad clearance each side)
    slot = _box(disc_mid, 0, AXIS_Z + DISC_R - 10.0, DISC_T + 4.0, 140.0, 70.0)
    bracket = _box((RAIL_X + HOUSING_L / 2 + disc_mid - 20.0) / 2, 0, AXIS_Z + DISC_R,
                   disc_mid - 20.0 - (RAIL_X + HOUSING_L / 2), 50.0, 90.0)
    return _label(caliper + bracket - slot, "brake_caliper", "#B3261E")


def _gearmotor():
    gearbox = _box(GEARBOX_X, 0, AXIS_Z, *GEARBOX) - _x_cylinder(GEARBOX_X, 0, AXIS_Z, BORE_R, GEARBOX[0] + 2.0)
    motor = _x_span_cylinder(MOTOR_X0, MOTOR_X0 + MOTOR_L, 0, AXIS_Z, MOTOR_R)
    fan_cowl = _x_span_cylinder(MOTOR_X0 - 40.0, MOTOR_X0, 0, AXIS_Z, MOTOR_R + 5.0)
    terminal_box = _box(MOTOR_X0 + MOTOR_L / 2, 0, AXIS_Z + MOTOR_R + 30.0, 120.0, 100.0, 70.0)
    # torque arm: from a lug on the gearbox to the lug on the left upright
    gearbox_lug = _box(GEARBOX_X, -140.0, AXIS_Z, 120.0, 60.0, 60.0)
    lug_face_x = -RAIL_X - UPRIGHT_T / 2            # the upright lug's outer face
    torque_arm = _box((GEARBOX_X + lug_face_x) / 2, -140.0, AXIS_Z, lug_face_x - GEARBOX_X, 40.0, 60.0)
    body = gearbox + motor + fan_cowl + terminal_box + gearbox_lug + torque_arm
    return _label(body, "gearmotor", "#5C6066")


def _control_box():
    box = _box(*CONTROL_BOX_POS, *CONTROL_BOX)
    return _label(box, "control_box", "#D9DBDD")


def _web():
    r = GUIDE_R
    c1 = (ROLLER1_Y, ROLLER_Z)
    c2 = (DANCER_Y, DANCER_Z)
    c3 = (ROLLER2_Y, ROLLER_Z)
    roll_top = (0.0, AXIS_Z + ROLL_OD / 2)

    rm = r + WEB_T / 2                           # the web's mid-surface radius on a roller
    a1 = _crossed_tangent_angle(c1, c2, rm)      # leave roller 1 here
    a2_in = a1 + 180.0                           # arrive on the dancer
    a2_out = 360.0 - a2_in + 180.0               # leave the dancer (mirror about Y=DANCER_Y)
    a3 = 180.0 - a1                              # arrive on roller 2

    def strip(p, q):
        """A straight web strip whose mid-surface runs from p to q (WEB_T thick)."""
        dy, dz = q[0] - p[0], q[1] - p[1]
        length = sqrt(dy * dy + dz * dz)
        ny, nz = -dz / length * WEB_T / 2, dy / length * WEB_T / 2
        return _yz_face([(p[0] - ny, p[1] - nz), (q[0] - ny, q[1] - nz), (q[0] + ny, q[1] + nz), (p[0] + ny, p[1] + nz)])

    def wrap(center, start, end):
        """A web arc hugging a roller from `start` to `end` degrees."""
        mid = (start + end) / 2
        pin = [_circle_point(center, r, a) for a in (start, mid, end)]
        pout = [_circle_point(center, r + WEB_T, a) for a in (end, mid, start)]
        to3 = lambda p: (0.0, p[0], p[1])
        edges = [
            bd.ThreePointArc(*(to3(p) for p in pin)),
            bd.Line(to3(pin[2]), to3(pout[0])),
            bd.ThreePointArc(*(to3(p) for p in pout)),
            bd.Line(to3(pout[2]), to3(pin[0])),
        ]
        return bd.make_face(bd.Wire(edges))

    faces = [
        strip((roll_top[0], roll_top[1] + WEB_T / 2), _circle_point(c1, rm, 90.0)),
        wrap(c1, 90.0, a1),
        strip(_circle_point(c1, rm, a1), _circle_point(c2, rm, a2_in)),
        wrap(c2, a2_in, a2_out),
        strip(_circle_point(c2, rm, a2_out), _circle_point(c3, rm, a3)),
        wrap(c3, a3, 90.0),
        strip(_circle_point(c3, rm, 90.0), (EXIT_Y, ROLLER_Z + rm)),
    ]
    web = None
    for face in faces:
        solid = _extrude_x(face, -WEB_WIDTH / 2, WEB_WIDTH / 2)
        web = solid if web is None else web + solid
    return _label(web, "paper_web", "#E8D7B3")


# ---------------------------------------------------------------------------
# kinematics and animation
# ---------------------------------------------------------------------------

KINEMATICS = {
    "mates": [
        cadgen.revolute("roll_spin", parent="#upright_right", child="#roll_assembly",
                        origin=(0, 0, AXIS_Z), direction=(1, 0, 0), limits=(-180, 180)),
        cadgen.revolute("dancer_swing", parent="#roller_stand", child="#dancer_assembly",
                        origin=(0, DANCER_Y, DANCER_PIVOT_Z), direction=(1, 0, 0), limits=(-25, 25)),
        cadgen.revolute("guide_roller_1_spin", parent="#roller_stand", child="#guide_roller_1",
                        origin=(0, ROLLER1_Y, ROLLER_Z), direction=(1, 0, 0), limits=(-180, 180)),
        cadgen.revolute("guide_roller_2_spin", parent="#roller_stand", child="#guide_roller_2",
                        origin=(0, ROLLER2_Y, ROLLER_Z), direction=(1, 0, 0), limits=(-180, 180)),
        cadgen.revolute("dancer_roller_spin", parent="#dancer_arm", child="#dancer_roller",
                        origin=(0, DANCER_Y, DANCER_Z), direction=(1, 0, 0), limits=(-180, 180)),
    ],
    "poses": {
        "rest": {"dancer_swing": 0},
        "slack_web": {"dancer_swing": 18},
        "tight_web": {"dancer_swing": -18},
    },
}

_ROLL_DEG_PER_S = degrees(WEB_SPEED / (ROLL_OD / 2))
_GUIDE_DEG_PER_S = degrees(WEB_SPEED / GUIDE_R)

ANIMATION_JS = f"""
const AXIS_Z = {AXIS_Z};
const ROLL_RATE = {_ROLL_DEG_PER_S:.4f};     // deg/s at {WEB_SPEED / 1000 * 60:.0f} m/min web speed
const GUIDE_RATE = {_GUIDE_DEG_PER_S:.4f};
const ROLL_PARTS = ["air_shaft", "paper_core", "paper_roll", "chuck_left", "chuck_right", "brake_disc"];

export const clips = {{
  unwind: {{
    label: "Unwind at {WEB_SPEED / 1000 * 60:.0f} m/min",
    duration: 12,
    loop: true,
    update(t, m) {{
      // The web leaves the top of the roll towards +Y: a negative turn about +X.
      for (const name of ROLL_PARTS) m.get(name).rotate([1, 0, 0], -ROLL_RATE * t, [0, 0, AXIS_Z]);
      m.get("guide_roller_1").rotate([1, 0, 0], -GUIDE_RATE * t, [0, {ROLLER1_Y}, {ROLLER_Z}]);
      m.get("guide_roller_2").rotate([1, 0, 0], -GUIDE_RATE * t, [0, {ROLLER2_Y}, {ROLLER_Z}]);
      // The web runs under the dancer, so it turns the other way.
      m.get("dancer_roller").rotate([1, 0, 0], GUIDE_RATE * t, [0, {DANCER_Y}, {DANCER_Z}]);
      // The dancer breathes with the tension loop.
      const swing = 6 * Math.sin(2 * Math.PI * t / 6);
      m.get("dancer_arm").rotate([1, 0, 0], swing, [0, {DANCER_Y}, {DANCER_PIVOT_Z}]);
      m.get("dancer_roller").rotate([1, 0, 0], swing, [0, {DANCER_Y}, {DANCER_PIVOT_Z}]);
    }},
  }},
}};
"""

MATERIALS = {
    "definitions": {
        "painted_steel": {"name": "Painted steel", "roughness": 0.55, "metalness": 0.25},
        "steel": {"name": "Machined steel", "roughness": 0.35, "metalness": 0.9},
        "aluminum": {"name": "Anodised aluminium roller", "roughness": 0.3, "metalness": 0.85},
        "paper": {"name": "Kraft paper", "roughness": 0.95, "metalness": 0.0},
        "rubber": {"name": "Rubber", "roughness": 0.9, "metalness": 0.0},
    },
    "assignments": [
        {"targets": ["#base_frame", "#upright_left", "#upright_right", "#roller_stand",
                     "#dancer_arm", "#gearmotor", "#control_box", "#brake_caliper"], "material": "painted_steel"},
        {"targets": ["#air_shaft", "#chuck_left", "#chuck_right", "#brake_disc"], "material": "steel"},
        {"targets": ["#guide_roller_1", "#guide_roller_2", "#dancer_roller"], "material": "aluminum"},
        {"targets": ["#paper_roll", "#paper_core", "#paper_web"], "material": "paper"},
        {"targets": ["#levelling_foot:left_rear", "#levelling_foot:left_front",
                     "#levelling_foot:right_rear", "#levelling_foot:right_front"], "material": "rubber"},
    ],
}


# ---------------------------------------------------------------------------
# model
# ---------------------------------------------------------------------------


@step(out="../../STEP/paper_roll_unwinder/paper_roll_unwinder.step",
      kinematics=KINEMATICS, materials=MATERIALS, animation=ANIMATION_JS)
@glb(out="../../GLB/paper_roll_unwinder/paper_roll_unwinder.glb")
def paper_roll_unwinder():
    roll_assembly = bd.Compound(
        children=[_shaft(), _core(), _paper_roll(), *_chucks(), _brake_disc()],
        label="roll_assembly",
    )
    dancer_assembly = bd.Compound(children=[_dancer_arm(), _dancer_roller()], label="dancer_assembly")
    return bd.Compound(
        children=[
            _base_frame(),
            *_feet(),
            _upright(-1),
            _upright(1),
            roll_assembly,
            _brake_caliper(),
            _gearmotor(),
            _control_box(),
            _roller_stand(),
            _guide_roller("guide_roller_1", ROLLER1_Y),
            _guide_roller("guide_roller_2", ROLLER2_Y),
            dancer_assembly,
            _web(),
        ],
        label="paper_roll_unwinder",
    )


if __name__ == "__main__":
    paper_roll_unwinder()
