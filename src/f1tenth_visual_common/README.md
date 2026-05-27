
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

