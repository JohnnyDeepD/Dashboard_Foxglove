"""Throwaway check: does each FTG failure mode produce its advice? (no ROS graph needed)"""
import sys

import numpy as np

sys.path.insert(0, "src/f1tenth_visual_common")
from f1tenth_visual_common.controller_debug import FtgDebugPublisher  # noqa: E402


class FakePub:
    def __init__(self, sink, topic):
        self.sink, self.topic = sink, topic

    def publish(self, msg):
        self.sink[self.topic] = msg


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


class FakeNode:
    def __init__(self):
        self.sink = {}

    def create_publisher(self, _type, topic, _qos):
        return FakePub(self.sink, topic)

    def get_clock(self):
        return FakeClock()


def run(name, kwargs, frames=4):
    node = FakeNode()
    dbg = FtgDebugPublisher(node)
    for _ in range(frames):
        dbg.publish(**kwargs)
    advice = node.sink["/debug/ftg/advice"].data
    print(f"--- {name}")
    print("   advice:", advice.replace("\n", "\n           "))
    return node.sink, advice


sink, a = run("1. no gap",
              dict(nearest_dist=1.0, steer=0.0, speed=1.0, gap=(None, None)))
assert "[No gap]" in a and "SAFE_THRESHOLD" in a

sink, a = run("1b. no gap because bubble ate the scan",
              dict(nearest_dist=1.0, steer=0.0, speed=1.0, gap=(None, None),
                   bubble_start=0, bubble_end=160))
assert "[Bubble too large]" in a
assert "[No gap]" not in a

sink, a = run("2. bubble too small (scraping on a straight)",
              dict(nearest_dist=0.15, steer=-0.05, speed=2.0, gap=(0, 199),
                   bubble_start=0, bubble_end=4))
assert "[Bubble too small]" in a
assert sink["/debug/ftg/bubble_beams"].data == 4.0

# Aim at the right wall of a 100-beam gap starting at 0 → best_offset ≈ +0.91
sink, a = run("3. corner, aiming at the wall",
              dict(nearest_dist=1.2, steer=0.30, speed=2.0, gap=(0, 99),
                   best_point=95, bubble_start=0, bubble_end=40))
assert "[Corner]" in a and "yellow AIM" in a
assert sink["/debug/ftg/best_offset"].data > 0.8

sink, a = run("3c. huge bubble pins AIM on the wall",
              dict(nearest_dist=1.2, steer=0.30, speed=1.0, gap=(0, 99),
                   best_point=95, bubble_start=0, bubble_end=160))
assert "[Bubble too large]" in a

sink, a = run("3b. too close while turning, AIM centered (must stay OK)",
              dict(nearest_dist=0.15, steer=0.30, speed=2.0, gap=(0, 199),
                   bubble_start=0, bubble_end=40))
assert a == "OK", f"expected OK, got: {a}"
assert "[Bubble too small]" not in a

node = FakeNode()
dbg = FtgDebugPublisher(node)
for _ in range(4):
    dbg.publish(nearest_dist=0.15, steer=0.30, speed=2.0, gap=(0, 199),
                bubble_start=0, bubble_end=4)
for _ in range(20):
    dbg.publish(nearest_dist=0.15, steer=-0.05, speed=2.0, gap=(0, 199),
                bubble_start=0, bubble_end=4)
a = node.sink["/debug/ftg/advice"].data
print("--- 3d. still close after a turn (must not be bubble_small)")
print("   advice:", a.replace("\n", "\n           "))
assert "[Bubble too small]" not in a

sink, a = run("4. straight wobble (AIM off-center)",
              dict(nearest_dist=1.8, steer=0.05, speed=2.0, gap=(0, 399),
                   best_point=20, bubble_start=0, bubble_end=20))
assert "[Straight wobble]" in a
assert abs(sink["/debug/ftg/best_offset"].data) > 0.4

sink, a = run("5. too fast in the turn (folded into Corner)",
              dict(nearest_dist=1.2, steer=0.35, speed=5.0, gap=(0, 399),
                   bubble_start=0, bubble_end=40))
assert "[Corner]" in a

sink, a = run("6. healthy driving (must stay OK)",
              dict(nearest_dist=1.8, steer=0.05, speed=3.0, gap=(100, 499),
                   best_point=300, bubble_start=0, bubble_end=20))
assert a == "OK", f"expected OK, got: {a}"
assert abs(sink["/debug/ftg/best_offset"].data) < 0.1

# AIM right of center (expected > 0) but steer is negative.
sink, a = run("6b. steer sign flipped",
              dict(nearest_dist=1.8, steer=-0.20, speed=1.0, gap=(0, 399),
                   best_point=250, ranges=np.full(400, 3.0),
                   angle_increment=0.004, bubble_start=0, bubble_end=20))
assert "[Steer sign]" in a

sink, a = run("6c. steer sign matches AIM (must stay OK)",
              dict(nearest_dist=1.8, steer=0.20, speed=1.0, gap=(0, 399),
                   best_point=250, ranges=np.full(400, 3.0),
                   angle_increment=0.004, bubble_start=0, bubble_end=20))
assert a == "OK", f"expected OK, got: {a}"
assert "[Steer sign]" not in a

sink, a = run("7. only 2 frames of no_gap (must not fire yet)",
              dict(nearest_dist=1.0, steer=0.0, speed=1.0, gap=(None, None)), frames=2)
assert a == "OK", f"expected OK, got: {a}"

# Markers: a forward corridor should draw gap + AIM + BUBBLE.
ranges = np.full(80, 3.0)
ranges[0] = 0.8
sink, a = run(
    "8. 3D markers (gap / AIM / bubble)",
    dict(
        steer=0.0,
        speed=1.0,
        gap=(20, 59),
        best_point=40,
        ranges=ranges,
        angle_increment=0.004,
        angle_min=-2.35,
        window_start=120,
        nearest_index=0,
        bubble_start=0,
        bubble_end=8,
    ),
)
markers = sink["/debug/ftg/markers"].markers
kinds = {m.id: (m.action, m.text if m.text else m.type) for m in markers}
print("   markers:", kinds)
assert all(m.action == 0 for m in markers), "expected all markers ADD"
assert any(m.text == "AIM" for m in markers)
assert any(m.text == "BUBBLE" for m in markers)
assert any(m.id == 0 and len(m.points) >= 3 for m in markers)

print("\nALL FTG ADVICE CHECKS PASSED")
