"""Pure completion predicate: all configured cows in one valid snapshot."""

import math


def all_cows_at_goal(names, poses, expected_names, goal, radius):
    if not expected_names or len(names) != len(poses):
        return False
    expected = set(expected_names)
    if len(expected) != len(expected_names):
        return False
    found = set()
    for name, pose in zip(names, poses):
        if name not in expected:
            continue
        if name in found:
            return False
        found.add(name)
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        if not all(math.isfinite(v) for v in (x, y, z)):
            return False
        if math.hypot(x - goal[0], y - goal[1]) > radius:
            return False
    return found == expected
