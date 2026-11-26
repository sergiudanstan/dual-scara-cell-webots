#!/usr/bin/env python3
"""Generate worlds/dual_scara_cell.wbt from lib/layout.py.

Two Epson SCARA T6-602S share one infeed. Each fills its own tray. A full tray leaves north.
Static geometry is plain shapes. Only the robots have physics. Parts and trays are kinematic.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(ROOT, "lib"))
import layout as L  # noqa: E402
from geom import ik_world  # noqa: E402

OUT = os.path.join(ROOT, "worlds", "dual_scara_cell.wbt")
R = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/"
PROTOS = [
    R + "objects/backgrounds/protos/TexturedBackground.proto",
    R + "objects/backgrounds/protos/TexturedBackgroundLight.proto",
    R + "humans/pedestrian/protos/Pedestrian.proto",
    "../protos/ScaraT6Cell.proto",
]

out = []


def w(s):
    out.append(s)


def f(*v):
    return " ".join("%.5g" % x for x in v)


def app(color, rough=0.5, metal=0.0, emissive=None, transp=0.0):
    e = " emissiveColor %s" % emissive if emissive else ""
    t = " transparency %g" % transp if transp else ""
    return "PBRAppearance { baseColor %s roughness %g metalness %g%s%s }" % (color, rough, metal, e, t)


ALU = app("0.78 0.8 0.82", 0.35, 0.8)
DARK = app("0.12 0.12 0.13", 0.7, 0.2)
STEEL = app("0.6 0.62 0.64", 0.3, 0.9)
YELLOW = app("0.98 0.76 0.05", 0.5)
BLACK = app("0.05 0.05 0.05", 0.6)
RUBBER = app("0.08 0.08 0.09", 0.9)
MESH = app("0.15 0.16 0.17", 0.6, 0.3, transp=0.55)
GREY = app("0.55 0.57 0.6", 0.6)
BLUE = app("0.15 0.35 0.7", 0.45, 0.2)
WHITE = app("0.9 0.9 0.88", 0.4, 0.1)
POLY = app("0.75 0.85 0.95", 0.1, transp=0.55)


def box(size, a, pos=(0, 0, 0), rot=None):
    r = " rotation %s" % f(*rot) if rot else ""
    return "Pose { translation %s%s children [ Shape { appearance %s geometry Box { size %s } } ] }" % (
        f(*pos), r, a, f(*size))


def cyl(r_, h, a, pos=(0, 0, 0), rot=None, sub=24):
    r = " rotation %s" % f(*rot) if rot else ""
    return ("Pose { translation %s%s children [ Shape { appearance %s geometry Cylinder { radius %g height %g "
            "subdivision %d } } ] }" % (f(*pos), r, a, r_, h, sub))


def group(items):
    return "Group { children [\n  %s\n] }" % "\n  ".join(items)


def bar(p0, p1, s, a):
    c = [(p0[i] + p1[i]) / 2 for i in range(3)]
    size = [max(abs(p1[i] - p0[i]), s) for i in range(3)]
    return box(size, a, c)


def home_joints(side):
    q = ik_world(L.BASE[side], side, L.home_world(side), [0.0, 1.2, 0.0, 0.0])
    if q is None:
        raise SystemExit("home pose of SCARA %s is not reachable" % side)
    return q


def generate():
    del out[:]
    w("#VRML_SIM R2025a utf8")
    for p in PROTOS:
        w('EXTERNPROTO "%s"' % p)
    w("""WorldInfo {
  info [ "Two Epson SCARA T6-602S sharing one infeed" ]
  title "Doua SCARA Epson T6"
  basicTimeStep 16
  contactProperties [ ContactProperties { softCFM 0.001 } ]
}
DEF VIEW Viewpoint {
  orientation -0.18 0.05 0.98 2.45
  position 4.2 -4.4 2.8
  exposure 1.2
  follow "none"
}
TexturedBackground { texture "factory" luminosity 0.9 }
TexturedBackgroundLight { texture "factory" luminosity 1.1 }""")

    floor()
    table()
    conveyor()
    fence()
    console()
    robots()
    products()
    plc()
    operator()
    text = "\n".join(out) + "\n"
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as fh:
        fh.write(text)
    return text


def floor():
    fl = [box((16, 14, 0.02), app("0.46 0.47 0.48", 0.9), (0, 0.2, -0.01))]
    x0, x1, y0, y1 = L.FENCE
    for (a, b) in [((x0 - 0.08, y0 - 0.08), (x1 + 0.08, y0 - 0.08)),
                   ((x0 - 0.08, y1 + 0.08), (x1 + 0.08, y1 + 0.08)),
                   ((x0 - 0.08, y0 - 0.08), (x0 - 0.08, y1 + 0.08)),
                   ((x1 + 0.08, y0 - 0.08), (x1 + 0.08, y1 + 0.08))]:
        n = max(1, int(math.dist(a, b) / 0.2))
        for i in range(n):
            t = (i + 0.5) / n
            cx, cy = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            horiz = abs(b[0] - a[0]) > abs(b[1] - a[1])
            size = (math.dist(a, b) / n * 0.92, 0.08, 0.002) if horiz else (0.08, math.dist(a, b) / n * 0.92, 0.002)
            fl.append(box(size, YELLOW if i % 2 == 0 else BLACK, (cx, cy, 0.001)))
    for side in ("A", "B"):
        fl.append(box((0.28, 1.3, 0.002), app("0.15 0.35 0.75", 0.8), (L.OUT[side][0], 1.15, 0.0015)))
    fl.append(box((0.7, 3.2, 0.002), app("0.15 0.55 0.25", 0.8), (2.15, 0.6, 0.0015)))
    sx, sy = L.SCANNER
    fl.append(sector(sx, sy, L.WARN_R, app("1 0.8 0", 0.9, transp=0.62), 0.003))
    fl.append(sector(sx, sy, L.STOP_R, app("0.9 0.1 0.05", 0.9, transp=0.5), 0.004))
    w("Solid { name \"floor\" children [ %s ] boundingObject Plane { size 16 14 } locked TRUE }" % group(fl))


def sector(sx, sy, r, a, z, n=36):
    a0, a1 = 0.0, math.pi          # north half-disk
    pts = [(0, 0)] + [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    coords = ", ".join("%.4g %.4g %.4g" % (sx + x, sy + y, z) for x, y in pts)
    idx = " ".join("0 %d %d -1" % (i, i + 1) for i in range(1, n + 1))
    return ("Shape { appearance %s geometry IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ %s ] } }"
            % (a, coords, idx))


def table():
    tb = []
    x0, x1, y0, y1 = L.TABLE
    tz = L.TABLE_Z
    tb.append(box((x1 - x0, y1 - y0, 0.02), app("0.35 0.37 0.4", 0.4, 0.5),
                  ((x0 + x1) / 2, (y0 + y1) / 2, tz - 0.01)))
    for x in (x0 + 0.05, x1 - 0.05):
        for y in (y0 + 0.05, y1 - 0.05):
            tb.append(bar((x, y, 0.02), (x, y, tz - 0.02), 0.05, ALU))
            tb.append(cyl(0.03, 0.02, RUBBER, (x, y, 0.01)))
    for side, col in (("A", BLUE), ("B", app("0.2 0.45 0.35", 0.45))):
        bx, by, _, _ = L.BASE[side]
        tb.append(box((0.28, 0.24, L.SCARA_RISER), GREY, (bx, by, tz + L.SCARA_RISER / 2)))
        sx, sy = L.STATION[side]
        tb.append(box((L.TRAY_L + 0.02, L.TRAY_W + 0.02, 0.004), col, (sx, sy, tz + 0.002)))
        mx, my = L.MAG[side]
        for dx in (-1, 1):
            for dy in (-1, 1):
                px, py = mx + dx * (L.TRAY_L / 2 + 0.01), my + dy * (L.TRAY_W / 2 - 0.01)
                tb.append(bar((px, py, tz), (px, py, tz + L.MAG_COUNT * L.TRAY_H + 0.04), 0.012, ALU))
    w('Solid { name "work table" children [ %s ] }' % group(tb))


def conveyor():
    cv = []
    x, y0, y1, z, w_ = L.CONV_X, L.CONV_Y0, L.CONV_Y1, L.CONV_Z, L.CONV_W
    cv.append(box((w_, y1 - y0, 0.012), RUBBER, (x, (y0 + y1) / 2, z - 0.006)))
    for s in (-1, 1):
        cv.append(box((0.02, y1 - y0 + 0.04, 0.06), ALU, (x + s * (w_ / 2 + 0.01), (y0 + y1) / 2, z - 0.03)))
        gx = x + s * (L.PART_R + 0.012)
        cv.append(box((0.004, y1 - y0 - 0.08, 0.025), STEEL, (gx, (y0 + y1) / 2 + 0.02, z + 0.02)))
    cv.append(box((w_ - 0.01, 0.012, 0.04), DARK, (x, L.PICK[1] + L.PART_R + 0.004, z + 0.02)))
    for y in (y0 + 0.05, (y0 + y1) / 2, y1 - 0.08):
        for s in (-1, 1):
            cv.append(bar((x + s * 0.08, y, 0.02), (x + s * 0.08, y, z - 0.04), 0.035, ALU))
    cv.append(box((0.55, 0.45, 1.15), app("0.85 0.86 0.88", 0.5), (x, y0 - 0.15, 0.58)))
    cv.append(box((0.55, 0.45, 0.04), app("0.2 0.35 0.6", 0.4, 0.4), (x, y0 - 0.15, 1.17)))
    for y, tag in ((L.PICK[1], "B1"), (L.ACCU_Y, "B5")):
        cv.append(box((0.03, 0.025, 0.035), app("0.95 0.5 0.05", 0.5), (x - 0.09, y, z + 0.02)))
        cv.append(box((0.012, 0.02, 0.025), app("0.8 0.1 0.1", 0.3), (x + 0.07, y, z + 0.02)))
    for side in ("A", "B"):
        sx, sy = L.STATION[side]
        ox, oy = L.OUT[side]
        cv.append(box((0.16, oy - sy, 0.012), app("0.25 0.27 0.3", 0.6, 0.4), (sx, (sy + oy) / 2, L.TABLE_Z + 0.02)))
        for s in (-1, 1):
            cv.append(box((0.015, oy - sy, 0.03), ALU, (sx + s * 0.09, (sy + oy) / 2, L.TABLE_Z + 0.04)))
    w('Solid { name "conveyor and outfeed" children [ %s ] }' % group(cv))


def fence():
    x0, x1, y0, y1 = L.FENCE
    fz, fh = 0.0, 1.9
    fe = []

    def run(a, b, skips):
        horiz = abs(b[1] - a[1]) < 1e-9
        lo, hi = (a[0], b[0]) if horiz else (a[1], b[1])
        if lo > hi:
            lo, hi = hi, lo
        n = max(1, round((hi - lo) / 1.0))
        cuts = [lo + (hi - lo) * i / n for i in range(n + 1)]
        for lo_s, hi_s in skips:
            cuts = [c for c in cuts if not lo_s < c < hi_s] + [lo_s, hi_s]
        cuts = sorted(set(cuts))
        for c in cuts:
            x, y = (c, a[1]) if horiz else (a[0], c)
            fe.append(bar((x, y, 0), (x, y, fh), 0.045, YELLOW))
            fe.append(box((0.1, 0.1, 0.006), YELLOW, (x, y, 0.003)))
        for c0, c1 in zip(cuts, cuts[1:]):
            if any(c0 >= lo_s - 1e-6 and c1 <= hi_s + 1e-6 for lo_s, hi_s in skips):
                continue
            m, span = (c0 + c1) / 2, c1 - c0 - 0.05
            if span <= 0.05:
                continue
            if horiz:
                fe.append(box((span, 0.008, fh - 0.15), MESH, (m, a[1], 0.08 + (fh - 0.15) / 2)))
                fe.append(box((span, 0.03, 0.03), YELLOW, (m, a[1], fh - 0.04)))
            else:
                fe.append(box((0.008, span, fh - 0.15), MESH, (a[0], m, 0.08 + (fh - 0.15) / 2)))
                fe.append(box((0.03, span, 0.03), YELLOW, (a[0], m, fh - 0.04)))

    run((x0, y0), (x1, y0), [L.CONV_OPEN])
    run((x0, y1), (x1, y1), [L.OUT_OPEN["A"], L.OUT_OPEN["B"]])
    run((x0, y0), (x0, y1), [])
    dx, dy, dw = L.DOOR
    run((x1, y0), (x1, y1), [(dy - dw / 2, dy + dw / 2)])
    # tunnel over the conveyor mouth
    z = L.CONV_Z
    fe.append(box((0.30, 0.006, 0.22), POLY, (0, y0 - 0.16, z + 0.12)))
    fe.append(box((0.006, 0.32, 0.22), POLY, (-0.15, y0 - 0.16, z + 0.12)))
    fe.append(box((0.006, 0.32, 0.22), POLY, (0.15, y0 - 0.16, z + 0.12)))
    fe.append(box((0.30, 0.32, 0.006), POLY, (0, y0 - 0.16, z + 0.23)))
    w('Solid { name "safety fence" children [ %s ] }' % group(fe))
    hinge_y = dy - dw / 2
    w('DEF DOOR Pose { translation %s children [ %s ] }' % (
        f(dx, hinge_y, 0), group([
            box((0.012, dw - 0.04, fh - 0.12), MESH, (0, dw / 2, 0.06 + (fh - 0.12) / 2)),
            box((0.03, dw - 0.04, 0.03), YELLOW, (0, dw / 2, fh - 0.04)),
            box((0.04, 0.04, 0.12), BLACK, (0.03, dw - 0.1, 1.05)),
        ])))


def console():
    hx, hy = 1.70, 0.15
    sc = [box((0.08, 0.16, 0.10), app("0.95 0.8 0.1", 0.4), (*L.SCANNER, 0.22)),
          cyl(0.035, 0.03, BLACK, (L.SCANNER[0], L.SCANNER[1] + 0.02, 0.28)),
          bar((L.SCANNER[0], L.SCANNER[1], 0), (L.SCANNER[0], L.SCANNER[1], 0.16), 0.04, DARK),
          box((0.36, 0.28, 1.05), app("0.86 0.87 0.88", 0.5), (hx, hy, 0.52)),
          box((0.02, 0.22, 0.16), app("0.05 0.08 0.12", 0.3, emissive="0.05 0.25 0.4"), (hx - 0.18, hy, 0.85)),
          cyl(0.028, 0.03, app("0.9 0.85 0.1", 0.4), (hx - 0.19, hy - 0.06, 0.62), (0, 1, 0, math.pi / 2)),
          cyl(0.02, 0.04, app("0.85 0.05 0.05", 0.4), (hx - 0.2, hy + 0.05, 0.62), (0, 1, 0, math.pi / 2))]
    w('Solid { name "console" children [ %s ] }' % group(sc))
    lx, ly = 1.70, 0.55
    lamps = [bar((lx, ly, 0), (lx, ly, 1.7), 0.03, DARK)]
    for i, (name, col) in enumerate([("RED", "0.9 0.1 0.08"), ("AMBER", "1 0.6 0.05"), ("GREEN", "0.1 0.85 0.2")]):
        lamps.append("Pose { translation %s children [ Shape { appearance DEF LAMP_%s PBRAppearance { baseColor %s "
                     "roughness 0.3 transparency 0.15 emissiveColor 0 0 0 } geometry Cylinder { radius 0.04 height 0.07 } } ] }"
                     % (f(lx, ly, 1.95 - i * 0.08), name, col))
    w("Pose { children [ %s ] }" % " ".join(lamps))


def cup():
    return ", ".join([
        cyl(0.012, 0.03, STEEL, (0, 0, -0.015)),
        box((0.028, 0.028, 0.01), DARK, (0, 0, -0.03)),
        cyl(0.015, 0.014, app("0.1 0.3 0.85", 0.6), (0, 0, -0.042)),
        cyl(0.018, 0.004, app("0.1 0.3 0.85", 0.6), (0, 0, -0.056)),
    ])


def robots():
    for side, channel in (("A", 1), ("B", 3)):
        q = home_joints(side)
        bx, by, bz, yaw = L.BASE[side]
        slot = ("handSlot [ Receiver { name \"scara_rx\" channel %d } Emitter { name \"scara_tx\" channel %d } "
                "Pose { children [ %s ] } DEF SCARA_%s_TCP Pose { translation 0 0 %g } ]"
                % (channel, channel + 1, cup(), side, -L.SCARA_TOOL))
        w("ScaraT6Cell { translation %s rotation 0 0 1 %g name \"SCARA %s\" controller \"scara_robot\" "
          "supervisor TRUE q1 %g q2 %g q4 %g shaft %g %s }"
          % (f(bx, by, bz), yaw, side, q[0], q[1], q[3], q[2], slot))


def products():
    tray_children = [box((L.TRAY_L, L.TRAY_W, L.TRAY_FLOOR), WHITE, (0, 0, L.TRAY_FLOOR / 2))]
    t = 0.004
    for s in (-1, 1):
        tray_children.append(box((t, L.TRAY_W, L.TRAY_H), WHITE, (s * (L.TRAY_L / 2 - t / 2), 0, L.TRAY_H / 2)))
        tray_children.append(box((L.TRAY_L, t, L.TRAY_H), WHITE, (0, s * (L.TRAY_W / 2 - t / 2), L.TRAY_H / 2)))
    shapes = group(tray_children)
    first = True
    for side in ("A", "B"):
        for i in range(L.MAG_COUNT + 1):
            if i < L.MAG_COUNT:
                pos = (L.MAG[side][0], L.MAG[side][1], L.TABLE_Z + i * L.TRAY_H)
            else:
                pos = (L.STATION[side][0], L.STATION[side][1], L.TABLE_Z)
            use = shapes if first else "USE TRAY_GEOM"
            line = ('DEF %s Solid { translation %s name "%s" children [ %s ] boundingObject Pose { translation 0 0 %g '
                    'children [ Box { size %s } ] } }' % (
                        L.tray_def(side, i), f(*pos), L.tray_def(side, i), use, L.TRAY_H / 2,
                        f(L.TRAY_L, L.TRAY_W, L.TRAY_H)))
            if first:
                line = line.replace("children [ Group", "children [ DEF TRAY_GEOM Group", 1)
                first = False
            w(line)
    part = group([cyl(L.PART_R, L.PART_H, app("0.85 0.55 0.2", 0.35), sub=20),
                  cyl(L.PART_R * 0.45, 0.003, DARK, (0, 0, L.PART_H / 2), sub=16)])
    for i in range(L.N_PARTS):
        pos = (-3.0 + 0.05 * (i % 12), -3.6 - 0.05 * (i // 12), -0.4)
        use = part if i == 0 else "USE PART_GEOM"
        line = ('DEF PART_%d Solid { translation %s name "part %d" children [ %s ] '
                'boundingObject Cylinder { radius %g height %g } }' % (i, f(*pos), i, use, L.PART_R, L.PART_H))
        if i == 0:
            line = line.replace("children [ Group", "children [ DEF PART_GEOM Group", 1)
        w(line)


def plc():
    def dsensor(name, pos, yaw):
        return ('DistanceSensor { translation %s rotation 0 0 1 %g name "%s" lookupTable [ 0 0 0, 0.5 0.5 0 ] '
                'numberOfRays 1 aperture 0.05 }' % (f(*pos), yaw, name))

    z = L.CONV_Z + L.PART_H / 2
    dev = [
        dsensor("B1", (L.PICK[0] - 0.07, L.PICK[1], z), 0.0),
        dsensor("B5", (L.CONV_X - 0.07, L.ACCU_Y, z), 0.0),
        dsensor("B2A", (L.STATION["A"][0], L.STATION["A"][1] - 0.12, L.TABLE_Z + 0.02), math.pi / 2),
        dsensor("B2B", (L.STATION["B"][0], L.STATION["B"][1] - 0.12, L.TABLE_Z + 0.02), math.pi / 2),
        'Emitter { name "to_a" channel 1 }', 'Receiver { name "from_a" channel 2 }',
        'Emitter { name "to_b" channel 3 }', 'Receiver { name "from_b" channel 4 }',
        'Emitter { name "to_operator" channel 5 }',
    ]
    w('Robot { name "cell PLC" controller "cell_plc" supervisor TRUE children [ %s ] }' % " ".join(dev))


def operator():
    ox, oy = L.OPERATOR_HOME
    w('DEF OPERATOR Pedestrian { translation %s rotation 0 0 1 %g name "operator" controller "operator" '
      'shirtColor 0.95 0.45 0.05 pantsColor 0.15 0.18 0.25 '
      'bodySlot [ Receiver { name "op_rx" channel 5 } ] }' % (f(ox, oy, 1.27), math.pi))


if __name__ == "__main__":
    text = generate()
    print("wrote", os.path.abspath(OUT), len(text) // 1024, "kB")
