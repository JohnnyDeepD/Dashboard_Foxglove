from __future__ import annotations

from typing import Optional

import numpy as np
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


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
