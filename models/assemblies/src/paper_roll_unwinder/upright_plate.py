"""Upright plate of the paper roll unwinder: the 40 mm steel side plate.

One of the two laser- or plasma-cut plates each upright is welded from (the
footplate and the bearing housing are welded onto it). The assembly in
`paper_roll_unwinder.py` fuses this plate into each upright; this model is the
same plate alone, in its own fabrication frame, for its drawings.

* Units: millimeters.
* X: plate width; Z: plate height, bottom edge on Z=0 (the welding datum, where
  the plate sits on its footplate); Y: thickness, centred.
* The bore is on the plate's centre line, at the roll axis height above the
  footplate, inside a semicircular head of the bearing housing's radius.
"""

from __future__ import annotations

from cadgen import build123d as bd
from cadgen import step

THICKNESS = 40.0
BOTTOM_WIDTH = 900.0
TOP_WIDTH = 220.0
BORE_Z = 830.0              # footplate top to the roll axis
HEAD_R = 110.0              # semicircular head around the bore (the housing's radius)
HEIGHT = BORE_Z + HEAD_R    # overall plate height
BORE_D = 80.0               # shaft passage, bored after welding to the housing


def plate_solid() -> bd.Shape:
    """The plate at the origin: profile in XZ, thickness along Y."""
    body = bd.make_face(bd.Polyline(
        (-BOTTOM_WIDTH / 2, 0.0, 0.0),
        (BOTTOM_WIDTH / 2, 0.0, 0.0),
        (TOP_WIDTH / 2, 0.0, BORE_Z),
        (-TOP_WIDTH / 2, 0.0, BORE_Z),
        close=True,
    ))
    head = bd.Pos(0, 0, BORE_Z) * (bd.Plane.XZ * bd.Circle(HEAD_R)).face()
    profile = body + head
    plate = bd.extrude(profile, amount=THICKNESS / 2, both=True)
    bore = bd.Pos(0, 0, BORE_Z) * bd.Cylinder(BORE_D / 2, THICKNESS + 2.0, rotation=(90, 0, 0))
    return plate - bore


@step(out="../../STEP/paper_roll_unwinder/upright_plate.step")
def upright_plate():
    plate = plate_solid()
    plate.label = "upright_plate"
    return plate


if __name__ == "__main__":
    upright_plate()
