
## Quick Start (English)

Use this when running the dashboard for the first time.

### Foxglove Layout Import (Very Short)

After you clone this repo, use this file:
`src/f1tenth_visual_common/foxglove/layout_f1tenth_gym.json`

In Foxglove:
1. Open **Layouts** (left sidebar)
2. Click **+ Add** **Import Personal layout** 
3. Select `layout_f1tenth_gym.json`
(it is here `src/f1tenth_visual_common/foxglove/layout_f1tenth_gym.json`)
4. Click **Open**
5. Confirm topics are visible:
   - `/visual/racing_line`
   - `/visual/actual_path`
   - `/hud/kpi`


### Run Flow (Choose One)

#### A) Simulator (F1TENTH Gym)
1. **Run the simulator bridge**
   ```bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
2. **Run Foxglove bridge**
   ```bash
   ros2 run foxglove_bridge foxglove_bridge
   ```
3. **Run dashboard node(s)**
   ```bash
   cd <your_repo_root>
   source install/setup.bash
   ros2 launch f1tenth_visual_common dashboard.launch.py
   ```

#### B) Real Car
1. **Match `topics.yaml` to your real-car topics first.**
   - Edit `src/f1tenth_visual_common/config/topics.yaml`.
   - At minimum, check odometry/pose and drive topic names.
2. **Run your real-car stack** (it must publish odometry/pose and drive topics).
3. **Run Foxglove bridge**
   ```bash
   ros2 run foxglove_bridge foxglove_bridge
   ```
4. **Run dashboard node(s)**
   ```bash
   cd <your_repo_root>
   source install/setup.bash
   ros2 launch f1tenth_visual_common dashboard.launch.py
   ```



---

## MPPI Debug Dashboard

Import `foxglove/layout_mppi_debug.json` in Foxglove for an MPPI-specific debug view:
- **Advice panel**: real-time diagnosis (e.g. ESS low → raise temperature, steer saturated → check waypoints)
- **Health table**: ESS ratio, cost min/mean/max, saturation flags
- **3D**: sample rollout trajectories + chosen trajectory
- **Plots**: ESS ratio and mean cost over time

Add 3 lines to your MPPI node (f1tenth_planning `Dynamic_MPPI_Planner` based):
```python
from f1tenth_visual_common.controller_debug import MppiDebugPublisher, extract_from_f1tenth_planning
# __init__:      self._debug = MppiDebugPublisher(self, frame_id="map")
# after plan():  self._debug.publish(**extract_from_f1tenth_planning(self._planner))
```

For other MPPI implementations, pass arrays directly:
```python
self._debug.publish(sampled_xy=..., rewards=..., chosen_xy=..., temperature=...)
```

See `controller_debug.py` → `MppiDebugPublisher` for full parameter list.

---

## MPC Debug Dashboard (Simple MPC)

Import `foxglove/layout_mpc_debug.json` for an MPC-specific debug view:
- **Advice panel**: real-time diagnosis (off-track, steer saturated, cost spike)
- **Health table**: steer_ratio, waypoint_dist, cost, saturation flag
- **Plots**: steer ratio + waypoint distance over time

`simple_mpc_node` publishes these automatically. For your own MPC node, add 2 lines:
```python
from f1tenth_visual_common.controller_debug import MpcDebugPublisher
# __init__:   self._debug = MpcDebugPublisher(self)
# each step:  self._debug.publish(steer_ratio=..., waypoint_dist=..., cost=..., reacquire_dist=...)
```

**Advice guide:**

| Message | Cause | Fix |
|---|---|---|
| Off-track | car lost nearest waypoint | add waypoints at corners |
| Steer saturated | max steer reached every step | reduce `w_cte` or `speed_mps` |
| Steer near limit | consistently >85% of max | reduce `w_cte` or `speed_mps` |
| Cost spike | sudden large cost jump | check corner waypoints |

---

## Follow The Gap Debug Dashboard

Import `foxglove/layout_ftg_debug.json`. The 3D view is the lesson:

- **Green wedge** — the free gap you chose
- **Yellow AIM ball** — the beam you are steering toward
- **Red BUBBLE** — safety bubble around the closest obstacle

```python
# __init__:
from f1tenth_visual_common.controller_debug import FtgDebugPublisher
self._debug = FtgDebugPublisher(self)

# after publishing /drive. Rename the algorithm fields to match your code.
# scan=data is the LaserScan callback argument — leave it as-is.
self._debug.publish(
    scan=data,
    ranges=proc_ranges,        # ranges after pre processed, the one actually used in FTG
    steer=steering_command,
    speed=velocity_command,
    gap=gap,                   # (start, end) or (None, None)
    best_point=best_point,     # None if no gap
    #nearest_index=nearest_obstacle_index,
    #bubble_start=bubble_start,
    #bubble_end=bubble_end,
)

#self._debug.publish could be much simpler with ranges=proc_ranges, the actual value that students are using
#for lidar_callback, deleted ones could be inferred in controller_debug.py

#Former version with ranges=forward_lidar,
        #foxglove dashboard
        self._debug.publish(
            scan=data,
            ranges=forward_lidar,
            #ranges=proc_ranges,
            window_start=120,               #lidar window start index
            steer=steering_command,
            speed=velocity_command,
            gap=gap,                        # (start, end) or (None, None)
            best_point=best_point,          # None if no gap
            nearest_index=nearest_obstacle_index,
            bubble_start=bubble_start,
            bubble_end=bubble_end,
            #chunk_size=CHUNK_SIZE,  # added for chunking
        )
```

After `colcon build`, source `install/setup.bash` so the node picks up this publisher.

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

`[Straight wobble]` fires on a straight when the AIM offset jumps by more
than 0.12 from the last scan (noise / chasing a far beam). `[Far AIM]`
fires on a straight when AIM stays off-center (`abs(best_offset) > 0.4`)
and more than 25 beams from the gap midpoint, and is not jumping (stuck
farthest beam). The lab search of midpoint ±25 does not fire. They are
exclusive. `[Corner AIM]` still needs turning, so it does not overlap.
Race midpoint bias looks like `[Far AIM]`; we do not special-case it.
Both are off while `[Chunking]` is on.
Artificial ±1 m range noise can flip jump vs stuck, so after 20 clear
frames (`rearm`) a new wobble/Far AIM line can appear. That demo is
extreme; leave it. Do not raise rearm. If student logs actually stack
those two, stop counting one as “off” while the other is on.

`[Chunking]` fires if the lab-formula length ratio is about 2 or more. It
suppresses `[Steer sign]`, `[Straight wobble]`, `[Far AIM]`, and the corner tips. The
line is repeated only after a different tip is logged, not every scan.

`[Rear scan]` fires if either end of the passed lidar window is more than
2.0 rad (~115°) from forward (`window_start` is treated as the first beam
of that slice). On a typical 270° / ~1080-beam lidar that means:

- `ranges[120:960]` with `window_start=120` — no warning (~±105°)
- `ranges[100:980]` with `window_start=100` — no warning (about the limit)
- `ranges[40:1040]` with `window_start=40` — `[Rear scan]` (~±125°)
- `ranges[0:1080]` with `window_start=0` — `[Rear scan]` (~±135°)

A safe forward slice is about `[100:980]` to `[120:960]`. It does not fire
while `[Chunking]` is on.

`[Steer units]` fires if `|steer| > 1.5` (a car is ~0.42 rad max), unless
that value already matches the AIM angle. That also catches a beam index
used as steering (`* angle_increment` missing) once AIM is 2+ beams off
center. It does not fire while `[Chunking]` is on. A degrees-sized
`steer` is not treated as a corner.

`[Steer unused]` fires if the AIM angle is more than 0.15 rad off center
and `|steer| < 0.06`. It does not fire while `[Chunking]` or
`[Steer units]` is on.

`[Bubble too large]` fires when the bubble is ≥ 80 beams and there is no gap, or AIM is on the wall. In that case `[No gap]` is not printed (the bubble ate the space, it is not a threshold bug).

`[Bubble too small]` fires when the bubble is < 16 beams, you scrape on a
straight, and it is not leftover from a turn. Lab radius 10 (~20 beams)
sits in the quiet band. Students pick many sizes; we only warn when the
car actually breaks, not “use 10”. Do not retune 16 / 80 until we see
false positives.

We do not warn for “cutting the inside” or “not turning enough” — those are too vague to tell the student what to change.

**Not added (on purpose).** Add one tip at a time; the stacked chunk/AIM/wobble
change broke corners.

- `window_start` vs slice equality — `[Rear scan]` is enough for an uncropped
  scan; comparing to `scan.ranges[...]` false-fires on processed ranges
- `[AIM behind]` (mixed indices / AIM drawn behind the car) — not the same
  as `[Rear scan]`; wait until that shows up in 3D
- Chunking from “steer much smaller than AIM” — `steer=0` looked like chunking
- `turning` from “steer vs recent” — a held corner looks straight
- `turning` threshold `|steer| > 0.15` instead of `0.5 * 0.4189` — retunes
  wobble and corner together; wait for a small-max-steer student
- Do not make every new tip suppress sign / wobble / corner (only `[Chunking]`
  does that)

---

Recommended topic setup in Foxglove:
- 3D panel:
  - `/map`
  - `/visual/racing_line`
  - `/visual/actual_path`
- Time Series panel:
  - `/stats/lap_time`
  - `/stats/cross_track_error`
  - `/stats/compute_ms`
- Table panel:
  - `/hud/kpi.values[:]`
- Gauge panel:
  - `/hud/challenge_score`
  - `/hud/gpu_mem_usage_percent`


# Problem Solving
**Advice panel:** the log keeps different warnings as they appear. The same
warning is not printed again until it has been *off* for 20 lidar frames
(so hitting a wall does not flood the panel). Advice text does not include
live numbers such as `nearest_dist`, because a changing number was treated
as a new message every scan.

**Chunk Size in FTG**: do not chunk if you use this dashboard. `[Chunking]`
asks to turn chunk averaging off and smooth with a moving average instead.

**ranges**: Students might use processed lidar ranges or the original ranges, consider both cases -> infer it by 
(len(scan.ranges) - len(ranges)) // 2