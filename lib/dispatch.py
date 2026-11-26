"""Who takes the next part. Pure function: no Webots, no clocks.

A robot may pick only when a part is waiting, the pick column is free, and that robot
is idle over its own tray with an empty pocket. The robot that has placed fewer parts goes first.
A tie goes to A, so the two sides alternate.
"""

SIDES = ("A", "B")


def eligible(arm, part_ready, zone_owner):
    if zone_owner is not None or not part_ready:
        return False
    return (arm["ready"] and not arm["busy"] and not arm["holding"] and arm["tray"]
            and arm["pockets_left"] > 0)


def choose(arms, part_ready, zone_owner):
    """Return 'A', 'B', or None."""
    opts = [name for name in SIDES if eligible(arms[name], part_ready, zone_owner)]
    if not opts:
        return None
    opts.sort(key=lambda name: (arms[name]["placed"], name))
    return opts[0]
