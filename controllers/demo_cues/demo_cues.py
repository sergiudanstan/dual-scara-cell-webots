"""Cues for the HTML5 demo, added by tools/build_site.sh.

At SHOT seconds, save a thumbnail of the current view.
At WALK seconds, send the operator through the scanner fields (PLC key P).
controllerArgs: <shot.jpg> <shot_t> <walk_t>
"""
import sys

from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
jpg, shot_t, walk_t = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
shot = walk = False
plc = None
kids = sup.getRoot().getField("children")
for i in range(kids.getCount()):
    n = kids.getMFNode(i)
    if n.getTypeName() == "Robot" and n.getField("name").getSFString() == "cell PLC":
        plc = n
        break

while sup.step(dt) != -1:
    t = sup.getTime()
    if not shot and t >= shot_t:
        shot = True
        sup.step(dt)
        sup.exportImage(jpg, 90)
        print("thumbnail", jpg)
    if not walk and t >= walk_t:
        walk = True
        if plc:
            plc.getField("customData").setSFString("P")
            print("operator walk")
