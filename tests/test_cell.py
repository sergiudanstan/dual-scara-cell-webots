"""Offline checks for the two-SCARA cell. No Webots required.

  python3 tests/test_cell.py
"""
import math
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "lib"))
sys.path.insert(0, os.path.join(ROOT, "controllers", "scara_robot"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import layout as L  # noqa: E402
import scara_kin  # noqa: E402
from dispatch import choose  # noqa: E402
from geom import base_to_world, elbow_world, ik_base, ik_world, on_own_side, tcp_world, world_to_base  # noqa: E402
from motion import Executor, quintic  # noqa: E402


def _arm(placed=0, **kw):
    a = {"ready": True, "busy": False, "holding": False, "tray": True,
         "pockets_left": L.PARTS_PER_TRAY, "placed": placed}
    a.update(kw)
    return a


def test_roundtrip_and_reject():
    for x, y, z in [(0.40, 0.15, 0.02), (0.30, -0.20, -0.04), (0.50, 0.05, 0.0)]:
        q = scara_kin.ik(x, y, z, 0.0, L.SCARA_TOOL)
        assert q is not None
        p = scara_kin.fk(q, L.SCARA_TOOL)
        assert max(abs(a - b) for a, b in zip(p[:3], (x, y, z))) < 1e-9
    assert scara_kin.ik(1.2, 0.0, 0.0, 0.0, L.SCARA_TOOL) is None
    assert scara_kin.ik(0.4, 0.0, 0.4, 0.0, L.SCARA_TOOL) is None


def test_both_robots_reach_the_cell():
    seed = [0.0, 1.0, 0.0, 0.0]
    for side in ("A", "B"):
        base = L.BASE[side]
        points = [L.PICK, L.home_world(side), L.via_world(side)]
        points += [L.pocket_world(side, i) for i in range(L.PARTS_PER_TRAY)]
        for p in points:
            for z in (p[2], L.CLEAR_Z):
                q = ik_world(base, side, (p[0], p[1], z), seed)
                assert q is not None, (side, p, z)
                assert on_own_side(side, elbow_world(base, q)[0]), (side, p, elbow_world(base, q))
                seed = q


def test_pick_shift_of_two_metres_is_unreachable():
    """A dead IK path would still return a pose. Moving the pick outside the arm must fail."""
    base = L.BASE["A"]
    near = (L.PICK[0] + 0.005, L.PICK[1], L.PICK[2])
    far = (L.PICK[0] + 2.0, L.PICK[1], L.PICK[2])
    assert ik_world(base, "A", near, [0, 1, 0, 0]) is not None
    assert ik_world(base, "A", far, [0, 1, 0, 0]) is None


def _track(side):
    """Replay one pick-place and return TCP and elbow traces in the world, plus the clear index."""
    base = L.BASE[side]
    q = ik_world(base, side, L.home_world(side), [0.0, 1.2, 0.0, 0.0])

    def ik(t, q_ref):
        return ik_base(base, side, t, q_ref)

    home = world_to_base(base, L.home_world(side))
    pick = world_to_base(base, L.PICK)
    via = world_to_base(base, L.via_world(side))
    place = world_to_base(base, L.pocket_world(side, 0))
    up = lambda p: (p[0], p[1], L.CLEAR_Z - base[2], 0.0)
    clear_at = []
    steps = [
        ("j", up(pick)), ("l", pick, 0.45), ("a", lambda: None, 0.12),
        ("l", up(pick), 0.45), ("j", up(via)), ("a", lambda: clear_at.append(0), 0.0),
        ("j", up(place)), ("l", place, 0.45), ("l", up(place), 0.45), ("j", up(home)),
    ]
    ex = Executor(ik, q, 5.0, min_t=0.35)
    ex.tcp = up(home)
    ex.load(steps, ex.tcp)
    tcp, elb = [], []
    prev = list(ex.q)
    marked = None
    while ex.busy():
        ex.update(0.016)
        assert ex.error is None, (side, ex.error)
        dq = max(abs(a - b) for a, b in zip(prev[:2], ex.q[:2])) / 0.016
        dd = abs(ex.q[2] - prev[2]) / 0.016
        assert dq < 10.0, (side, dq)
        assert dd < 0.8, (side, dd)
        prev = list(ex.q)
        tcp.append(tcp_world(base, ex.q))
        elb.append(elbow_world(base, ex.q))
        if clear_at and marked is None:
            marked = len(tcp)
    assert marked is not None
    return tcp, elb, marked


def test_paths_stay_on_their_own_side_and_share_the_pick_in_turn():
    for side, sign in (("A", 1), ("B", -1)):
        tcp, elb, clear_i = _track(side)
        # before the column is released the arm may sit on the pick (x ~ 0) but not on the other half
        for i, (p, e) in enumerate(zip(tcp, elb)):
            limit = L.HALF_X if side == "A" else -L.HALF_X
            if side == "A":
                assert p[0] <= limit + 0.03, (i, p)
                assert e[0] <= limit + 0.03, (i, e)
            else:
                assert p[0] >= limit - 0.03, (i, p)
                assert e[0] >= limit - 0.03, (i, e)
        # after "clear", the TCP is outside the shared column
        assert abs(tcp[clear_i - 1][0]) >= L.COLUMN_X, tcp[clear_i - 1]
        # home is over the tray, not over the pick
        assert abs(tcp[-1][0] - L.STATION[side][0]) < 0.03
        assert sign * tcp[-1][0] < 0


def test_dispatch_alternates_and_holds_the_column():
    arms = {"A": _arm(0), "B": _arm(0)}
    assert choose(arms, True, None) == "A"
    assert choose(arms, True, "A") is None
    arms["A"].update(busy=True, holding=True, placed=0)
    assert choose(arms, True, None) == "B"
    arms["B"].update(busy=True)
    assert choose(arms, True, "B") is None
    arms["A"].update(busy=False, holding=False, placed=1, pockets_left=5)
    arms["B"].update(busy=False, holding=False, placed=1, pockets_left=5)
    assert choose(arms, True, None) == "A"
    arms["B"]["placed"] = 0
    assert choose(arms, True, None) == "B"
    assert choose(arms, False, None) is None
    arms["B"]["tray"] = False
    arms["A"]["placed"] = 3
    assert choose(arms, True, None) == "A"
    arms["A"]["pockets_left"] = 0
    assert choose(arms, True, None) is None


def test_world_mentions_two_robots():
    import gen_world
    text = gen_world.generate()
    assert text.count("ScaraT6Cell {") == 2
    assert 'name "SCARA A"' in text and 'name "SCARA B"' in text
    assert "channel 1" in text and "channel 3" in text
    assert 'controller "scara_robot"' in text
    assert 'controller "cell_plc"' in text
    # the folded start is not the stretched q = 0 pose
    assert text.count("q1 0 q2 0") == 0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
