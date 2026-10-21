"""Engineering drawing (PDF) of the upright plate.

Documents the STEP that `upright_plate.py` writes: run that model first, then
this script. Every dimension references the model's constants, so a change to
the plate moves the dimension with it.
"""

from pathlib import Path

import cadgen
from cadgen.eng_drawing import Sheet, eng_drawing
from upright_plate import BORE_D, BORE_Z, BOTTOM_WIDTH, HEIGHT, THICKNESS, TOP_WIDTH

STEP = Path(__file__).parent / "../../STEP/paper_roll_unwinder/upright_plate.step"


@eng_drawing(out="../../PDF/paper_roll_unwinder/upright_plate_drawing.pdf")
def upright_plate_drawing():
    part = cadgen.read_step(STEP)
    sheet = Sheet(
        "A2", scale=0.2, title="UPRIGHT PLATE - PAPER ROLL UNWINDER",
        part_number="PRU-UP-001", material="STEEL PLATE S355JR, 40 THK",
        revision="A", author="text-to-cad", general_tolerance="ISO 2768-m",
        notes=[
            "2 PCS PER MACHINE, LEFT AND RIGHT HAND IDENTICAL.",
            "PROFILE LASER OR PLASMA CUT 1:1 FROM upright_plate_flat.dxf.",
            "FINISH BORE AFTER WELDING TO BEARING HOUSING.",
            "DRESS CUT EDGES, REMOVE DROSS.",
        ],
    )
    top, front, right = sheet.three_views(part)
    front.overall()
    front.dim((-TOP_WIDTH / 2, -THICKNESS / 2, HEIGHT), (TOP_WIDTH / 2, -THICKNESS / 2, HEIGHT))
    front.dim((BOTTOM_WIDTH / 2, -THICKNESS / 2, 0), (BOTTOM_WIDTH / 2, -THICKNESS / 2, BORE_Z))
    front.hole((0, -THICKNESS / 2, BORE_Z), BORE_D, thru=True, angle=30)
    right.dim((0, -THICKNESS / 2, 0), (0, THICKNESS / 2, 0))
    return sheet


if __name__ == "__main__":
    upright_plate_drawing()
