# Follow The Gap Debug Dashboard

Video: *(coming soon)*

Use this package to see your Follow The Gap code in Foxglove: the gap you chose, the beam you aim at, and the safety bubble.

## 1. Add this package to your workspace

Clone this repository into the `src` folder of your ROS 2 workspace, then build:

```bash
cd <your_ws>
colcon build --packages-select f1tenth_visual_common
source install/setup.bash
```

## 2. Import the Foxglove layout

File: `src/f1tenth_visual_common/foxglove/layout_ftg_debug.json`

In Foxglove:

1. Open **Layouts** (left sidebar)
2. Click **+ Add** → **Import personal layout**
3. Select `layout_ftg_debug.json`
4. Click **Open**

## 3. Add three lines to your gap_follow node

Rename the algorithm fields to match your code. Leave `scan=data` as the `LaserScan` callback argument.

```python
from f1tenth_visual_common.controller_debug import FtgDebugPublisher

# __init__:
self._debug = FtgDebugPublisher(self)

# after you publish /drive:
self._debug.publish(
    scan=data,
    ranges=forward_lidar,
    #ranges=proc_ranges, 
    window_start=120,          # first index of your lidar slice
    steer=steering_command,
    speed=velocity_command,
    gap=gap,                   # (start, end) or (None, None)
    best_point=best_point,     # None if no gap
    nearest_index=nearest_obstacle_index,
    bubble_start=bubble_start,
    bubble_end=bubble_end,
)
```

## 4. Build and run

```bash
cd <your_ws>
colcon build
source install/setup.bash
```

1. Start the simulator:
   ```bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
2. Start the Foxglove bridge:
   ```bash
   ros2 run foxglove_bridge foxglove_bridge
   ```
3. Run your gap_follow node.
4. Open Foxglove and connect to the bridge.

## 5. What you should see in 3D

- **Green wedge** — the free gap you chose
- **Yellow AIM ball** — the beam you are steering toward
- **Red BUBBLE** — safety bubble around the closest obstacle

Do not use chunk averaging with this dashboard. Smooth with a moving average instead.

| What you see in 3D | Advice | What to change |
|---|---|---|
| Yellow AIM jumps left/right on a straight | `[Straight wobble]` | Aim at the gap midpoint |
| Yellow AIM sits off-center in a gap (not jumping) | `[Far AIM]` | Aim at the gap midpoint, not the farthest beam |
| Yellow AIM sits on the edge of the green gap | `[Corner AIM]` | Use the gap midpoint |
| Steering is large, speed is still high, and you are about to hit a wall | `[Corner speed]` | Scale speed down when steering is large |
| Yellow AIM goes one way, the car steers the other | `[Steer sign]` | Left is positive. Flip the sign of the steering angle |
| Red BUBBLE is tiny and you get close to a wall on a straight | `[Bubble too small]` | Increase the safety bubble |
| Red BUBBLE ate the gap / AIM on a wall | `[Bubble too large]` | Shrink the safety bubble |
| Green gap / yellow AIM sit at the wrong angle | `[Chunking]` | Turn off chunk averaging; use a moving average |
| The lidar window includes beams behind the car | `[Rear scan]` | Use only the forward slice |
| Steering is huge (degrees or a beam index) | `[Steer units]` | Use a radian steering angle |
| Yellow AIM is off to the side, steering is ~0 | `[Steer unused]` | Convert the AIM beam to a steering angle in radians |

---

## Coming later (WIP)

Pure Pursuit, MPC, and MPPI dashboards are still in progress. Do not use them yet.
