"""Cut profile DXF of the upright plate; geometry reused from upright_plate.py.

One closed outline and the bore circle on the CUT layer, 1:1, for laser or
plasma cutting of the 40 mm plate. No kerf compensation: the cutting service
applies its own.
"""

from __future__ import annotations

from cadgen import build123d as bd
from cadgen import dxf, flatten

from upright_plate import THICKNESS, upright_plate   # a child: tracked by its result

KERF = 0.0


@dxf(out="../../DXF/paper_roll_unwinder/upright_plate_flat.dxf")
def upright_plate_flat():
    profile = flatten.flat_pattern(
        upright_plate(),
        normal_axis="y", normal_sign=-1.0,
        coordinate_axis="y", coordinate=-THICKNESS / 2,
        kerf=KERF,
    )
    # Stand the profile up (height along +Y) with the bottom edge on Y=0, centred.
    upright = bd.Rotation(0, 0, -90) * profile
    bb = upright.bounding_box()
    return bd.Pos(-(bb.min.X + bb.max.X) / 2, -bb.min.Y, 0) * upright


if __name__ == "__main__":
    upright_plate_flat()
