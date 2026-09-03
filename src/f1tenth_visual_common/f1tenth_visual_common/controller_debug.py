"""Foxglove debug publishers for the three student controllers.

Each class is used the same way: construct it on the ROS node, then call
``publish(...)`` every control step. Foxglove layouts subscribe to:

  /debug/mppi/*   MppiDebugPublisher    layout_mppi_debug.json
  /debug/mpc/*    MpcDebugPublisher     layout_mpc_debug.json
  /debug/ftg/*    FtgDebugPublisher     layout_ftg_debug.json

Advice strings go to ``.../advice``. Health tables go to ``.../health``.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# MPPI helper: sampling-health metric used by MppiDebugPublisher
# ---------------------------------------------------------------------------
def ess_ratio_from_rewards(
    total_reward: np.ndarray,
    temperature: float,
    damping: float,
    range_normalize: bool,
) -> float:
    """Effective sample size ratio (ESS / n_samples) of MPPI-style softmax weights.

    Replicates the solver weighting: range-normalize returns before softmax so the
    metric stays scale-invariant. Returns -1.0 when undefined.
    """
    n = int(total_reward.shape[0])
    if n == 0:
        return -1.0
    if temperature <= 0.0:
        temperature = 1.0
    r_max = float(np.max(total_reward))
    r_min = float(np.min(total_reward))
    denom = (r_max - r_min) + damping if range_normalize else 1.0
    if denom <= 0.0:
        denom = 1.0
    w = np.exp((total_reward - r_max) / denom / temperature)
    w_sum = float(np.sum(w))
    if w_sum <= 0.0:
        return -1.0
    w = w / w_sum
    ess = 1.0 / float(np.sum(w * w))
    return ess / float(n)


# ---------------------------------------------------------------------------
# MPPI — /debug/mppi/*  (Foxglove: layout_mppi_debug.json)
# Student node: construct once, then publish() after each plan().
# ---------------------------------------------------------------------------
class MppiDebugPublisher:
    """Algorithm-agnostic publisher for MPPI-style debug visualization.

    Feed it generic numpy arrays from any sampling-based controller; it publishes a
    standard ``/debug/mppi/*`` topic contract that the shared Foxglove layout consumes.
    Nothing here depends on a specific MPPI implementation.
    """

    def __init__(
        self,
        node,
        *,
        frame_id: str = "map",
        topic_prefix: str = "/debug/mppi",
        max_samples: int = 40,
        saturation_margin: float = 0.98,
        qos: int = 5,
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._max_samples = max(1, int(max_samples))
        self._saturation_margin = float(saturation_margin)

        # Topics the MPPI Foxglove layout reads.
        self._samples_pub = node.create_publisher(MarkerArray, f"{topic_prefix}/samples", qos)
        self._chosen_pub = node.create_publisher(Path, f"{topic_prefix}/chosen", qos)
        self._health_pub = node.create_publisher(DiagnosticStatus, f"{topic_prefix}/health", qos)
        self._ess_pub = node.create_publisher(Float32, f"{topic_prefix}/ess_ratio", qos)
        self._cost_pub = node.create_publisher(Float32, f"{topic_prefix}/cost_mean", qos)
        self._steer_sat_pub = node.create_publisher(Float32, f"{topic_prefix}/steer_sat", qos)
        self._accel_sat_pub = node.create_publisher(Float32, f"{topic_prefix}/accel_sat", qos)
        self._advice_pub = node.create_publisher(String, f"{topic_prefix}/advice", qos)
        self._prev_cost_mean: float = 0.0
        self._warning_log: list = []
        self._last_tip_set: set = set()

    def publish(
        self,
        *,
        sampled_xy: Optional[np.ndarray] = None,
        rewards: Optional[np.ndarray] = None,
        chosen_xy: Optional[np.ndarray] = None,
        temperature: float = 1.0,
        damping: float = 0.0,
        range_normalize: bool = True,
        steer: Optional[float] = None,
        steer_limit: Optional[float] = None,
        accel: Optional[float] = None,
        accel_limit: Optional[float] = None,
        speed: Optional[float] = None,
        speed_limit: Optional[float] = None,
    ) -> None:
        """Publish the standard debug topics.

        Args:
            sampled_xy: sampled rollout positions, shape [n_samples, horizon, >=2].
            rewards: per-sample reward, shape [n_samples, horizon] or [n_samples].
            chosen_xy: selected/optimal trajectory positions, shape [K, >=2].
            temperature, damping: MPPI weighting params (for the ESS metric).
            steer/accel/speed (+ *_limit): optional, used for saturation flags.
        """
        stamp = self._node.get_clock().now().to_msg()

        # 3D: grey sample rollouts + blue chosen path
        if sampled_xy is not None:
            arr = np.asarray(sampled_xy)
            if arr.ndim == 3 and arr.shape[0] > 0 and arr.shape[2] >= 2:
                self._samples_pub.publish(self._build_samples(arr, stamp))

        if chosen_xy is not None:
            ch = np.asarray(chosen_xy)
            if ch.ndim == 2 and ch.shape[0] > 0 and ch.shape[1] >= 2:
                self._chosen_pub.publish(self._build_chosen(ch, stamp))

        ess_ratio = -1.0
        cost_min = cost_mean = cost_max = 0.0
        if rewards is not None:
            r = np.asarray(rewards)
            total_reward = r.sum(axis=1) if r.ndim == 2 else (r if r.ndim == 1 else None)
            if total_reward is not None and total_reward.shape[0] > 0:
                cost = -total_reward
                cost_min = float(np.min(cost))
                cost_mean = float(np.mean(cost))
                cost_max = float(np.max(cost))
                ess_ratio = ess_ratio_from_rewards(
                    total_reward, float(temperature), float(damping), bool(range_normalize)
                )

        # Saturation flags for the health table / advice.
        steer_sat = self._saturated(steer, steer_limit)
        accel_sat = self._saturated(accel, accel_limit)
        speed_sat = self._saturated(speed, speed_limit, signed=False)

        status = DiagnosticStatus()
        status.name = "f1tenth_visual_common/mppi_health"
        status.hardware_id = "mppi"
        status.level = (
            DiagnosticStatus.WARN if (0.0 <= ess_ratio < 0.1) else DiagnosticStatus.OK
        )
        status.message = "mppi health"
        status.values = [
            KeyValue(key="ess_ratio", value=f"{ess_ratio:.4f}"),
            KeyValue(key="cost_min", value=f"{cost_min:.3f}"),
            KeyValue(key="cost_mean", value=f"{cost_mean:.3f}"),
            KeyValue(key="cost_max", value=f"{cost_max:.3f}"),
            KeyValue(key="steer_saturated", value=str(bool(steer_sat))),
            KeyValue(key="accel_saturated", value=str(bool(accel_sat))),
            KeyValue(key="speed_saturated", value=str(bool(speed_sat))),
        ]
        self._health_pub.publish(status)
        self._ess_pub.publish(Float32(data=float(ess_ratio)))
        self._cost_pub.publish(Float32(data=float(cost_mean)))
        self._steer_sat_pub.publish(Float32(data=1.0 if steer_sat else 0.0))
        self._accel_sat_pub.publish(Float32(data=1.0 if accel_sat else 0.0))
        self._advice_pub.publish(String(data=self._get_advice(
            ess_ratio, steer_sat, accel_sat, speed_sat, cost_mean
        )))

    # Advice rules (ESS collapse, saturation, cost spike). Appends new tips only.
    def _get_advice(
        self,
        ess_ratio: float,
        steer_sat: bool,
        accel_sat: bool,
        speed_sat: bool,
        cost_mean: float,
    ) -> str:
        import time as _time
        tips = []
        ess_low = 0.0 <= ess_ratio < 0.1

        if ess_low:
            tips.append("[ESS low] Raise the temperature parameter")
        if steer_sat and ess_low:
            tips.append("[Steer saturated] Fix ESS first (raise temperature)")
        if steer_sat and not ess_low:
            tips.append("[Steer saturated] Check yaw cost or corner waypoints; last resort: reduce the target/reference speed")
        if accel_sat or speed_sat:
            tips.append("[Speed saturated] Reduce the target/reference speed")
        if cost_mean > 0.0 and self._prev_cost_mean > 0.0:
            if cost_mean > 5.0 * self._prev_cost_mean:
                tips.append("[Cost spike] Car off line — check corner waypoints")
        self._prev_cost_mean = cost_mean if cost_mean > 0.0 else self._prev_cost_mean

        current_tips = set(tips)
        new_tips = current_tips - self._last_tip_set
        self._last_tip_set = current_tips
        for tip in tips:
            if tip in new_tips:
                ts = _time.strftime("%H:%M:%S")
                self._warning_log.append(f"[{ts}] {tip}")

        return "\n".join(self._warning_log) if self._warning_log else "OK"

    def _saturated(
        self, value: Optional[float], limit: Optional[float], signed: bool = True
    ) -> bool:
        if value is None or limit is None:
            return False
        limit = abs(float(limit))
        if limit <= 0.0:
            return False
        v = abs(float(value)) if signed else float(value)
        return v >= self._saturation_margin * limit

    def _build_samples(self, sampled_xy: np.ndarray, stamp) -> MarkerArray:
        n = sampled_xy.shape[0]
        step = max(1, n // self._max_samples)

        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = stamp
        marker.ns = "mppi_samples"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.35)
        marker.pose.orientation.w = 1.0

        for i in range(0, n, step):
            traj = sampled_xy[i]
            for t in range(traj.shape[0] - 1):
                marker.points.append(Point(x=float(traj[t, 0]), y=float(traj[t, 1]), z=0.0))
                marker.points.append(
                    Point(x=float(traj[t + 1, 0]), y=float(traj[t + 1, 1]), z=0.0)
                )

        return MarkerArray(markers=[marker])

    def _build_chosen(self, chosen_xy: np.ndarray, stamp) -> Path:
        path = Path()
        path.header.frame_id = self._frame_id
        path.header.stamp = stamp
        for k in range(chosen_xy.shape[0]):
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = float(chosen_xy[k, 0])
            pose.pose.position.y = float(chosen_xy[k, 1])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path


# ---------------------------------------------------------------------------
# MPC — /debug/mpc/*  (Foxglove: layout_mpc_debug.json)
# Student node: construct once, then publish() after each control step.
# ---------------------------------------------------------------------------
class MpcDebugPublisher:
    """Generic debug publisher for any MPC-style controller.

    Publishes standard /debug/mpc/* topics consumed by layout_mpc_debug.json.
    The student passes steer_ratio, waypoint_dist, cost computed in their node.
    """

    def __init__(
        self,
        node,
        *,
        topic_prefix: str = "/debug/mpc",
        saturation_margin: float = 0.98,
        qos: int = 5,
    ) -> None:
        self._node = node
        self._saturation_margin = float(saturation_margin)
        self._health_pub = node.create_publisher(DiagnosticStatus, f"{topic_prefix}/health", qos)
        self._steer_ratio_pub = node.create_publisher(Float32, f"{topic_prefix}/steer_ratio", qos)
        self._dist_pub = node.create_publisher(Float32, f"{topic_prefix}/waypoint_dist", qos)
        self._cost_pub = node.create_publisher(Float32, f"{topic_prefix}/cost", qos)
        self._advice_pub = node.create_publisher(String, f"{topic_prefix}/advice", qos)
        self._prev_cost: float = 0.0
        self._prev_waypoint_dist: float = 0.0
        self._warning_log: list = []
        self._last_tip_set: set = set()

    def publish(
        self,
        *,
        steer_ratio: float = 0.0,
        waypoint_dist: float = 0.0,
        cost: float = 0.0,
        reacquire_dist: float = 1.0,
        steer_saturated: bool = False,
    ) -> None:
        self._steer_ratio_pub.publish(Float32(data=float(steer_ratio)))
        self._dist_pub.publish(Float32(data=float(waypoint_dist)))
        self._cost_pub.publish(Float32(data=float(cost)))

        off_track = waypoint_dist > reacquire_dist

        status = DiagnosticStatus()
        status.name = "f1tenth_visual_common/mpc_health"
        status.hardware_id = "simple_mpc"
        status.level = DiagnosticStatus.WARN if (off_track or steer_saturated) else DiagnosticStatus.OK
        status.message = "mpc health"
        status.values = [
            KeyValue(key="steer_ratio", value=f"{steer_ratio:.3f}"),
            KeyValue(key="waypoint_dist_m", value=f"{waypoint_dist:.3f}"),
            KeyValue(key="cost", value=f"{cost:.3f}"),
            KeyValue(key="steer_saturated", value=str(bool(steer_saturated))),
            KeyValue(key="off_track", value=str(bool(off_track))),
        ]
        self._health_pub.publish(status)
        self._advice_pub.publish(String(data=self._get_advice(
            steer_ratio, waypoint_dist, cost, reacquire_dist, steer_saturated
        )))

    # Advice rules (off-track, corner rollout, steer limit, cost spike).
    def _get_advice(
        self,
        steer_ratio: float,
        waypoint_dist: float,
        cost: float,
        reacquire_dist: float,
        steer_saturated: bool,
    ) -> str:
        import time as _time
        tips = []

        if waypoint_dist > reacquire_dist:
            tips.append("[Off-track] Car lost path — check corner waypoints or reduce target speed")
        dist_spike = waypoint_dist > 1.5 * max(self._prev_waypoint_dist, 0.1)
        self._prev_waypoint_dist = waypoint_dist if waypoint_dist > 0 else self._prev_waypoint_dist
        #if steer_saturated and dist_spike:
        if steer_saturated and waypoint_dist > 0.3:
            tips.append("[Corner rollout issue] Steer maxed when waypoint_dist jumped — "
                        "rollout is not anticipating the corner. "
                        "Pre-compute reference so it advances proportionally to speed each rollout step (no fixed index).")
        elif steer_saturated and cost > 80.0:
            tips.append("[High cost + steer maxed] Rollout may not see track correctly. "
                        "Try changing the waypoint spacing or interpolate the waypoints")
        elif steer_saturated:
            tips.append("[Steer saturated] Reduce cross-track-error cost weight or target speed in controller params")
        if steer_ratio > 0.85 and not steer_saturated:
            tips.append("[Steer near limit] Consider reducing cross-track-error cost weight or increasing max steering angle limit")
        if cost > 0.0 and self._prev_cost > 0.0 and cost > 5.0 * self._prev_cost:
            tips.append("[Cost spike] Rollout cost jumped — check waypoints near corners")
        self._prev_cost = cost if cost > 0.0 else self._prev_cost

        current_tips = set(tips)
        new_tips = current_tips - self._last_tip_set
        self._last_tip_set = current_tips
        for tip in tips:
            if tip in new_tips:
                ts = _time.strftime("%H:%M:%S")
                self._warning_log.append(f"[{ts}] {tip}")

        return "\n".join(self._warning_log) if self._warning_log else "OK"


# ---------------------------------------------------------------------------
# Follow The Gap — /debug/ftg/*  (Foxglove: layout_ftg_debug.json)
# Student node: construct once, then publish() after sending /drive.
# Required args: nearest_dist [m], steer [rad], speed [m/s], gap_width [beams].
# ---------------------------------------------------------------------------
class FtgDebugPublisher:
    """Generic debug publisher for any Follow-The-Gap controller.

    Publishes standard /debug/ftg/* topics consumed by layout_ftg_debug.json.
    Students pass four values from their lidar callback: nearest obstacle
    distance, steering, speed, and gap width (0 if no gap was found).
    """

    def __init__(
        self,
        node,
        *,
        topic_prefix: str = "/debug/ftg",
        collision_dist: float = 0.35,
        steer_jump_rad: float = 0.25,
        persist_frames: int = 3,
        rearm_frames: int = 10,
        qos: int = 5,
    ) -> None:
        self._node = node
        self._collision_dist = float(collision_dist)
        self._steer_jump_rad = float(steer_jump_rad)
        self._persist_frames = max(1, int(persist_frames))
        self._rearm_frames = max(1, int(rearm_frames))
        self._steer_limit = 0.4189

        # Topics the FTG Foxglove layout reads.
        self._health_pub = node.create_publisher(DiagnosticStatus, f"{topic_prefix}/health", qos)
        self._nearest_pub = node.create_publisher(Float32, f"{topic_prefix}/nearest_dist", qos)
        self._gap_width_pub = node.create_publisher(Float32, f"{topic_prefix}/gap_width", qos)
        self._best_offset_pub = node.create_publisher(Float32, f"{topic_prefix}/best_offset", qos)
        self._steer_ratio_pub = node.create_publisher(Float32, f"{topic_prefix}/steer_ratio", qos)
        self._advice_pub = node.create_publisher(String, f"{topic_prefix}/advice", qos)

        self._prev_steer: Optional[float] = None
        self._counters: dict = {}
        self._warning_log: list = []
        self._latched_tags: set = set()
        self._clear_counts: dict = {}

    def publish(
        self,
        *,
        nearest_dist: float = -1.0,
        steer: float = 0.0,
        speed: float = 0.0,
        gap_width: float = 0.0,
        # Extra fields kept for a richer dashboard later (commented out, not deleted):
        # gap_start: int = -1,
        # gap_end: int = -1,
        # best_point: int = -1,
        # num_beams: int = 0,
        # bubble_start: Optional[int] = None,
        # bubble_end: Optional[int] = None,
        # steer_limit: float = 0.4189,
        # angle_increment: Optional[float] = None,
        # window_start: Optional[int] = None,
        # scan_size: Optional[int] = None,
    ) -> None:
        """Publish the standard FTG debug topics.

        Args:
            nearest_dist: closest obstacle [m].
            steer: steering sent to /drive [rad].
            speed: speed sent to /drive [m/s].
            gap_width: chosen gap size in beams; 0 if none.
        """
        # --- derived metrics from the four student values ---
        # no_gap = gap_start < 0 or gap_end < 0 or best_point < 0
        # num_beams = max(0, int(num_beams))
        # if no_gap:
        #     gap_width = 0.0
        #     best_offset = 0.0
        # else:
        #     gap_width = float(gap_end - gap_start + 1)
        #     gap_center = 0.5 * (gap_start + gap_end)
        #     half = max(1e-6, 0.5 * gap_width)
        #     best_offset = float(np.clip((best_point - gap_center) / half, -1.0, 1.0))
        no_gap = float(gap_width) <= 0.0
        # best_offset = 0.0

        # bubble_width = 0.0
        # if bubble_start is not None and bubble_end is not None:
        #     bubble_width = float(max(0, int(bubble_end) - int(bubble_start)))
        # bubble_frac = bubble_width / num_beams if num_beams > 0 else 0.0

        # steer_limit = abs(float(steer_limit)) or 1e-6
        steer_ratio = abs(float(steer)) / self._steer_limit
        steer_jump = 0.0 if self._prev_steer is None else abs(float(steer) - self._prev_steer)
        self._prev_steer = float(steer)

        # gap_width_deg = (
        #     math.degrees(gap_width * float(angle_increment)) if angle_increment else -1.0
        # )

        # Index-to-angle mapping check: the best point sits left of the window
        # center, so the steering command should point left as well.
        # Only a clear disagreement counts: both the best point and the steering
        # command must be well away from center, otherwise smoothing/noise trips it.
        # sign_mismatch = False
        # if angle_increment and not no_gap and num_beams > 0:
        #     beam_offset = best_point - 0.5 * num_beams
        #     if abs(beam_offset * float(angle_increment)) > 0.15 and abs(steer) > 0.05:
        #         sign_mismatch = (beam_offset > 0) != (float(steer) > 0)

        # Off-center forward window: the "straight ahead" index of the slice must
        # be the middle of the slice, otherwise steering has a constant bias.
        # window_offset = 0.0
        # if window_start is not None and scan_size is not None and num_beams > 0:
        #     window_center = float(window_start) + 0.5 * num_beams
        #     window_offset = window_center - 0.5 * float(scan_size)

        # --- Foxglove plots + health table ---
        self._nearest_pub.publish(Float32(data=float(nearest_dist)))
        self._gap_width_pub.publish(Float32(data=float(gap_width)))
        # self._best_offset_pub.publish(Float32(data=best_offset))
        self._steer_ratio_pub.publish(Float32(data=float(steer_ratio)))

        too_close = 0.0 <= nearest_dist < self._collision_dist
        status = DiagnosticStatus()
        status.name = "f1tenth_visual_common/ftg_health"
        status.hardware_id = "follow_the_gap"
        status.level = DiagnosticStatus.WARN if (no_gap or too_close) else DiagnosticStatus.OK
        status.message = "ftg health"
        status.values = [
            KeyValue(key="no_gap", value=str(bool(no_gap))),
            KeyValue(key="nearest_dist_m", value=f"{nearest_dist:.3f}"),
            KeyValue(key="gap_width_beams", value=f"{float(gap_width):.0f}"),
            # KeyValue(key="gap_width_deg", value=f"{gap_width_deg:.1f}"),
            # KeyValue(key="best_offset", value=f"{best_offset:+.2f}"),
            # KeyValue(key="bubble_beams", value=f"{bubble_width:.0f}"),
            # KeyValue(key="bubble_frac", value=f"{bubble_frac:.2f}"),
            KeyValue(key="steer_ratio", value=f"{steer_ratio:.3f}"),
            # KeyValue(key="steer_deg", value=f"{math.degrees(float(steer)):+.1f}"),
            # KeyValue(key="steer_jump_deg", value=f"{math.degrees(steer_jump):.1f}"),
            KeyValue(key="speed_mps", value=f"{float(speed):.2f}"),
            # KeyValue(key="window_offset_beams", value=f"{window_offset:+.0f}"),
        ]
        self._health_pub.publish(status)

        # --- advice panel ---
        self._advice_pub.publish(String(data=self._get_advice(
            no_gap=no_gap,
            nearest_dist=float(nearest_dist),
            gap_width=float(gap_width),
            # num_beams=num_beams,
            # best_offset=best_offset,
            # bubble_frac=bubble_frac,
            steer_ratio=steer_ratio,
            steer_jump=steer_jump,
            speed=float(speed),
            # sign_mismatch=sign_mismatch,
            # window_offset=window_offset,
        )))

    # Ignore one-frame noise: condition must hold persist_frames in a row.
    def _persisted(self, key: str, condition: bool) -> bool:
        """True once ``condition`` held for ``persist_frames`` consecutive calls."""
        count = self._counters.get(key, 0) + 1 if condition else 0
        self._counters[key] = count
        return count >= self._persist_frames

    def _get_advice(
        self,
        *,
        no_gap: bool,
        nearest_dist: float,
        gap_width: float,
        # num_beams: int,
        # best_offset: float,
        # bubble_frac: float,
        steer_ratio: float,
        steer_jump: float,
        speed: float,
        # sign_mismatch: bool,
        # window_offset: float,
    ) -> str:
        import time as _time
        tips = []
        # gap_frac = gap_width / num_beams if num_beams > 0 else 0.0

        # --- advice rules (what the student should change) ---
        # 1. Driving straight into a wall: no gap survives the threshold.
        if self._persisted("no_gap", no_gap):
            # if bubble_frac > 0.5:
            #     tips.append(
            #         "[No gap] The safety bubble erased most of the scan "
            #         f"({bubble_frac:.0%} of the beams). Shrink the bubble radius."
            #     )
            # else:
            tips.append(
                "[No gap] No beam passes the free-space threshold. "
                "Lower SAFE_THRESHOLD, or raise the range clip (RANGE_LIMIT) "
                "so distant free space is not clipped away."
            )

        # 2. Driving through obstacles: the bubble is not protecting the car.
        if self._persisted("too_close", 0.0 <= nearest_dist < self._collision_dist):
            tips.append(
                f"[Obstacle too close] nearest_dist {nearest_dist:.2f} m. "
                #f"[Obstacle too close]. "
                "Widen the safety bubble around the closest beam, and make sure "
                "the disparity extender runs so obstacle edges are not seen as free."
            )

        # 3. Clipping the wall in corners: best point sits at the gap edge.
        # if self._persisted("best_at_edge", not no_gap and abs(best_offset) > 0.8):
        #     side = "left" if best_offset > 0 else "right"
        #     tips.append(
        #         f"[Best point at gap edge] best_offset {best_offset:+.2f} ({side} edge). "
        #         "find_best_point is aiming at the gap boundary, i.e. the wall. "
        #         "Aim at the gap midpoint or at the farthest beam inside the gap."
        #     )
        # if self._persisted("narrow_gap", not no_gap and 0.0 < gap_frac < 0.05):
        #     tips.append(
        #         f"[Gap very narrow] gap_width {gap_width:.0f} beams ({gap_frac:.0%} of the window). "
        #         "Lower SAFE_THRESHOLD or shrink the bubble; a gap this thin makes steering twitchy."
        #     )

        # 4. Steering jitter: index-to-angle mapping is wrong.
        # if self._persisted("sign_mismatch", sign_mismatch):
        #     tips.append(
        #         "[Steering sign flipped] The best point is on one side but the car steers "
        #         "to the other. Check the index-to-angle mapping: "
        #         "steer = (best_point - num_beams/2) * angle_increment."
        #     )
        if self._persisted("steer_jump", steer_jump > self._steer_jump_rad):
            tips.append(
                f"[Steering jitter] Steering jumped {math.degrees(steer_jump):.0f}deg in one scan. "
                "Check that the best point index is measured on the sliced forward window "
                "(not the full scan), and increase the smoothing window."
            )
        # if self._persisted("window_off_center", abs(window_offset) > 5.0):
        #     tips.append(
        #         f"[Forward window off-center] The slice center is {window_offset:+.0f} beams "
        #         "away from the scan center, so 'straight ahead' is biased. "
        #         "Slice symmetrically, e.g. ranges[120:960] of 1080 beams."
        #     )

        # 5. Too fast to make the turn.
        if self._persisted("fast_in_turn", steer_ratio > 0.6 and speed > 3.0):
            tips.append(
                f"[Too fast in the turn] steer_ratio {steer_ratio:.2f} at {speed:.1f} m/s. "
                "Scale the speed down when the steering angle is large."
            )
        if self._persisted("steer_saturated", steer_ratio >= 0.98):
            tips.append(
                #"[Steer saturated] Steering is pinned at its limit. Check find_best_point "
                #"and the bubble width before raising the steering limit."
                #"[Steer maxed] Full turn — the free gap might not be in front of the car. "
                #"Check bubble size, then find_best_point. Do not raise max steer first."
                "[Steer saturated] Steering is at the limit. It is OK in a tight corner. "
                "If this happens on a straight, check bubble size and find_best_point. "
                "Do not raise max steer first."
            )

        # --- history log: same [Tag] does not re-append until it has been
        # off for rearm_frames (default 10). Different tags still stack. ---
        current_tags = {self._advice_tag(tip) for tip in tips}
        for tag in list(self._latched_tags):
            if tag in current_tags:
                self._clear_counts[tag] = 0
            else:
                self._clear_counts[tag] = self._clear_counts.get(tag, 0) + 1
                if self._clear_counts[tag] >= self._rearm_frames:
                    self._latched_tags.discard(tag)
                    self._clear_counts.pop(tag, None)

        for tip in tips:
            tag = self._advice_tag(tip)
            if tag not in self._latched_tags:
                self._latched_tags.add(tag)
                self._clear_counts[tag] = 0
                ts = _time.strftime("%H:%M:%S")
                self._warning_log.append(f"[{ts}] {tip}")
        del self._warning_log[:-100]

        return "\n".join(self._warning_log) if self._warning_log else "OK"

    @staticmethod
    def _advice_tag(tip: str) -> str:
        if tip.startswith("[") and "]" in tip:
            return tip[1:tip.index("]")]
        return tip


# ---------------------------------------------------------------------------
# Adapter: f1tenth_planning Dynamic_MPPI_Planner -> MppiDebugPublisher.publish()
# ---------------------------------------------------------------------------
def extract_from_f1tenth_planning(planner) -> dict:
    """Adapter: pull generic debug arrays out of a f1tenth_planning MPPI planner.

    Returns a dict ready to splat into ``MppiDebugPublisher.publish(**...)``. Reads
    attributes defensively so a planner-version change never breaks the control path.
    Works for any planner exposing ``solver.samples`` / ``x_pred`` in the same layout.
    """
    out = {
        "sampled_xy": None,
        "rewards": None,
        "chosen_xy": None,
        "temperature": 1.0,
        "damping": 0.0,
    }

    solver = getattr(planner, "solver", None)
    samples = getattr(solver, "samples", None) if solver is not None else None
    if samples is not None and len(samples) >= 3:
        s_sampled = np.asarray(samples[1])  # [n_samples, N, nx]
        if s_sampled.ndim == 3 and s_sampled.shape[2] >= 2:
            out["sampled_xy"] = s_sampled[:, :, :2]
        out["rewards"] = np.asarray(samples[2])  # [n_samples, N]

    x_pred = getattr(planner, "x_pred", None)
    if x_pred is not None:
        x_arr = np.asarray(x_pred)  # [nx, N+1]
        if x_arr.ndim == 2 and x_arr.shape[0] >= 2:
            out["chosen_xy"] = x_arr[:2, :].T  # [N+1, 2]

    config = getattr(solver, "config", None)
    out["temperature"] = float(getattr(config, "temperature", 1.0))
    out["damping"] = float(getattr(config, "damping", 0.0))
    return out
