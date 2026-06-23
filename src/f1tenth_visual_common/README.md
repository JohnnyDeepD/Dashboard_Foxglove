
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

