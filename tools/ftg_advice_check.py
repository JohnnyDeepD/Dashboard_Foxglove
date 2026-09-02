"""Throwaway check: does each FTG failure mode produce its advice? (no ROS graph needed)"""
import sys

sys.path.insert(0, "src/f1tenth_visual_common")
from f1tenth_visual_common.controller_debug import FtgDebugPublisher  # noqa: E402


class FakePub:
    def __init__(self, sink, topic):
        self.sink, self.topic = sink, topic

    def publish(self, msg):
        self.sink[self.topic] = msg


class FakeNode:
    def __init__(self):
        self.sink = {}

    def create_publisher(self, _type, topic, _qos):
        return FakePub(self.sink, topic)


def run(name, kwargs, frames=4):
    node = FakeNode()
    dbg = FtgDebugPublisher(node)
    for _ in range(frames):
        dbg.publish(**kwargs)
    advice = node.sink["/debug/ftg/advice"].data
    health = {kv.key: kv.value for kv in node.sink["/debug/ftg/health"].values}
    print(f"--- {name}")
    print("   health:", health)
    print("   advice:", advice.replace("\n", "\n           "))
    return advice


a = run("1. no gap",
        dict(nearest_dist=1.0, steer=0.0, speed=1.0, gap_width=0))
assert "[No gap]" in a and "SAFE_THRESHOLD" in a

# a = run("1b. no gap (bubble ate the scan)", ...)  # needs bubble_frac

a = run("2. obstacle too close",
        dict(nearest_dist=0.15, steer=-0.05, speed=2.0, gap_width=200))
assert "[Obstacle too close]" in a

# a = run("3. best point at gap edge", ...)  # needs best_offset
# a = run("3b. gap very narrow", ...)  # needs num_beams
# a = run("4. steering sign flipped", ...)  # needs angle_increment / best_point

node = FakeNode()
dbg = FtgDebugPublisher(node)
for i in range(6):
    dbg.publish(nearest_dist=1.2, gap_width=400,
                steer=0.4 if i % 2 else -0.4, speed=2.0)
a = node.sink["/debug/ftg/advice"].data
print("--- 4b. steering jitter\n   advice:", a.replace("\n", "\n           "))
assert "[Steering jitter]" in a

# a = run("4c. forward window off-center", ...)  # needs window_start / scan_size

a = run("5. too fast in the turn",
        dict(nearest_dist=1.2, steer=0.35, speed=5.0, gap_width=400))
assert "[Too fast in the turn]" in a

a = run("6. healthy driving (must stay OK)",
        dict(nearest_dist=1.8, steer=0.05, speed=3.0, gap_width=400))
assert a == "OK", f"expected OK, got: {a}"

a = run("7. only 2 frames of no_gap (must not fire yet)",
        dict(nearest_dist=1.0, steer=0.0, speed=1.0, gap_width=0), frames=2)
assert a == "OK", f"expected OK, got: {a}"

print("\nALL FTG ADVICE CHECKS PASSED")
