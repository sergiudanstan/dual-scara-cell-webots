"""Frames and the elbow choice shared by the controllers, the world generator and the tests.

A target is accepted only when one IK solution keeps the elbow on that robot's own side of the cell.
The cup is round, so joint 4 is held at 0.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "scara_robot"))
import scara_kin  # noqa: E402
import layout as L  # noqa: E402


def world_to_base(base, p):
    """World (x, y, z) or (x, y, z, yaw) -> the same point in the robot base frame."""
    import math
    bx, by, bz, yaw = base
    c, s = math.cos(-yaw), math.sin(-yaw)
    u, v = p[0] - bx, p[1] - by
    z = p[2] - bz
    extra = (p[3] - yaw) if len(p) > 3 else 0.0
    return (c * u - s * v, s * u + c * v, z, extra)


def base_to_world(base, p):
    import math
    bx, by, bz, yaw = base
    c, s = math.cos(yaw), math.sin(yaw)
    return (bx + c * p[0] - s * p[1], by + s * p[0] + c * p[1], bz + p[2])


def elbow_world(base, q):
    x, y = scara_kin.elbow(q)
    return base_to_world(base, (x, y, 0.0))[:2]


def tcp_world(base, q, tool=L.SCARA_TOOL):
    return base_to_world(base, scara_kin.fk(q, tool)[:3])


def on_own_side(side, x):
    return x <= L.HALF_X if side == "A" else x >= -L.HALF_X


def ik_base(base, side, target_base, q_ref, tool=L.SCARA_TOOL):
    """Base-frame TCP target -> [q1, q2, d, 0], or None.

    Both elbow solutions are computed. A solution that puts the elbow across the centre line is rejected.
    Of the remaining ones, the elbow closest to q_ref is kept.
    """
    x, y, z, yaw = target_base
    q_ref = q_ref or [0.0, 1.0, 0.0, 0.0]
    cands = []
    for seed in (2.0, -2.0):
        q = scara_kin.ik(x, y, z, yaw, tool, [q_ref[0], seed, q_ref[2], 0.0])
        if q is None:
            continue
        q = [q[0], q[1], q[2], 0.0]
        if any(abs(q[1] - c[1]) < 1e-4 and abs(q[0] - c[0]) < 1e-4 for c in cands):
            continue
        cands.append(q)
    good = [q for q in cands if on_own_side(side, elbow_world(base, q)[0])]
    if not good:
        return None
    good.sort(key=lambda q: abs(q[1] - q_ref[1]) + 0.25 * abs(q[0] - q_ref[0]))
    return good[0]


def ik_world(base, side, world_xyz, q_ref, tool=L.SCARA_TOOL):
    return ik_base(base, side, world_to_base(base, world_xyz), q_ref, tool)
