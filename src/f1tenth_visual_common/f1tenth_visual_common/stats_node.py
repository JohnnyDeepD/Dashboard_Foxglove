from __future__ import annotations

import subprocess
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


Point2 = Tuple[float, float]


def _to_point(values: Sequence[float], default: Point2) -> Point2:
    if len(values) != 2:
        return default
    return float(values[0]), float(values[1])

class StatsNode(Node):
    def __init__(self) -> None:
        super().__init__("stats_node")

        self.declare_parameter("use_odom", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/pose")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("cross_track_error_topic", "/stats/cross_track_error")
        self.declare_parameter("lap_time_topic", "/stats/lap_time")
        self.declare_parameter("compute_ms_topic", "/stats/compute_ms")
        self.declare_parameter("challenge_score_topic", "/stats/challenge_score")
        self.declare_parameter("hud_kpi_topic", "/hud/kpi")
        self.declare_parameter("hud_challenge_score_topic", "/hud/challenge_score")
        self.declare_parameter("hud_gpu_mem_usage_topic", "/hud/gpu_mem_usage_percent")
        self.declare_parameter("summary_topic", "/stats/summary")
        self.declare_parameter("kpi_text_topic", "/stats/kpi_text")
        self.declare_parameter("highlights_topic", "/stats/highlights")
        self.declare_parameter("frame_id", "map")

        self.declare_parameter("gate_a_xy", [0.0, -0.5])
        self.declare_parameter("gate_b_xy", [0.0, 0.5])
        self.declare_parameter("crossing_positive_direction", True)
        self.declare_parameter("min_lap_time_sec", 5.0)
        self.declare_parameter("crossing_cooldown_sec", 2.0)
        self.declare_parameter("use_start_point_lap", False)
        self.declare_parameter("start_revisit_radius_m", 0.6)
        self.declare_parameter("start_arm_distance_m", 1.5)

        self._use_odom = bool(self.get_parameter("use_odom").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._drive_topic = str(self.get_parameter("drive_topic").value)
        self._cte_topic = str(self.get_parameter("cross_track_error_topic").value)
        self._crossing_positive = bool(
            self.get_parameter("crossing_positive_direction").value
        )
        self._min_lap_sec = float(self.get_parameter("min_lap_time_sec").value)
        self._cooldown_sec = float(self.get_parameter("crossing_cooldown_sec").value)
        self._use_start_point_lap = bool(self.get_parameter("use_start_point_lap").value)
        self._start_revisit_radius_m = max(
            0.1, float(self.get_parameter("start_revisit_radius_m").value)
        )
        self._start_arm_distance_m = max(
            self._start_revisit_radius_m + 0.1,
            float(self.get_parameter("start_arm_distance_m").value),
        )

        self._gate_a = _to_point(self.get_parameter("gate_a_xy").value, (0.0, -0.5))
        self._gate_b = _to_point(self.get_parameter("gate_b_xy").value, (0.0, 0.5))

        self._lap_pub = self.create_publisher(
            Float32, str(self.get_parameter("lap_time_topic").value), 10
        )
        self._compute_pub = self.create_publisher(
            Float32, str(self.get_parameter("compute_ms_topic").value), 20
        )
        self._challenge_score_pub = self.create_publisher(
            Float32, str(self.get_parameter("challenge_score_topic").value), 20
        )
        self._hud_kpi_pub = self.create_publisher(
            DiagnosticStatus, str(self.get_parameter("hud_kpi_topic").value), 10
        )
        self._hud_challenge_score_pub = self.create_publisher(
            Float32, str(self.get_parameter("hud_challenge_score_topic").value), 20
        )
        self._hud_gpu_mem_pub = self.create_publisher(
            Float32, str(self.get_parameter("hud_gpu_mem_usage_topic").value), 20
        )
        self._summary_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("summary_topic").value), 10
        )
        self._kpi_text_pub = self.create_publisher(
            String, str(self.get_parameter("kpi_text_topic").value), 10
        )
        self._highlights_pub = self.create_publisher(
            String, str(self.get_parameter("highlights_topic").value), 10
        )

        self._last_gate_side: Optional[float] = None
        self._last_crossing_wall: Optional[float] = None
        self._lap_start_wall: Optional[float] = None
        self._lap_count: int = 0
        self._last_lap: float = 0.0
        self._fastest_lap: Optional[float] = None
        self._last_cte_m: float = 0.0
        self._last_compute_ms: float = 0.0
        self._start_point_xy: Optional[Point2] = None
        self._start_gate_armed: bool = False

        if self._use_odom:
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 50)
        else:
            self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, 50)
        self.create_subscription(AckermannDriveStamped, self._drive_topic, self._on_drive, 50)
        self.create_subscription(Float32, self._cte_topic, self._on_cte, 50)

        self.create_timer(0.5, self._publish_summary)
        self.get_logger().info(
            f"stats_node ready (source={'odom' if self._use_odom else 'pose'})"
        )

    def _on_cte(self, msg: Float32) -> None:
        self._last_cte_m = float(msg.data)

    def _signed_gate_side(self, x: float, y: float) -> float:
        ax, ay = self._gate_a
        bx, by = self._gate_b
        vx = bx - ax
        vy = by - ay
        wx = x - ax
        wy = y - ay
        return vx * wy - vy * wx

    def _on_odom(self, msg: Odometry) -> None:
        self._check_lap(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_pose(self, msg: PoseStamped) -> None:
        self._check_lap(msg.pose.position.x, msg.pose.position.y)

    def _check_lap(self, x: float, y: float) -> None:
        if self._use_start_point_lap:
            self._check_lap_from_start_point(x, y)
            return

        now = time.monotonic()
        side = self._signed_gate_side(x, y)
        if self._last_gate_side is None:
            self._last_gate_side = side
            return

        crossed = False
        if self._crossing_positive:
            crossed = self._last_gate_side < 0.0 <= side
        else:
            crossed = self._last_gate_side > 0.0 >= side
        self._last_gate_side = side

        if not crossed:
            return
        if self._last_crossing_wall is not None and (
            now - self._last_crossing_wall < self._cooldown_sec
        ):
            return
        self._last_crossing_wall = now

        if self._lap_start_wall is None:
            self._lap_start_wall = now
            return

        lap_time = now - self._lap_start_wall
        if lap_time < self._min_lap_sec:
            return

        self._lap_start_wall = now
        self._lap_count += 1
        self._last_lap = lap_time
        if self._fastest_lap is None or lap_time < self._fastest_lap:
            self._fastest_lap = lap_time
        out = Float32()
        out.data = float(lap_time)
        self._lap_pub.publish(out)

    def _check_lap_from_start_point(self, x: float, y: float) -> None:
        now = time.monotonic()
        if self._start_point_xy is None:
            self._start_point_xy = (float(x), float(y))
            self._lap_start_wall = now
            self.get_logger().info(
                f"Start-point lap anchor set at ({x:.3f}, {y:.3f}), "
                f"radius={self._start_revisit_radius_m:.2f} m"
            )
            return

        sx, sy = self._start_point_xy
        dx = float(x) - sx
        dy = float(y) - sy
        dist = (dx * dx + dy * dy) ** 0.5

        if not self._start_gate_armed:
            if dist >= self._start_arm_distance_m:
                self._start_gate_armed = True
            return

        if dist > self._start_revisit_radius_m:
            return
        if self._last_crossing_wall is not None and (
            now - self._last_crossing_wall < self._cooldown_sec
        ):
            return
        self._last_crossing_wall = now

        if self._lap_start_wall is None:
            self._lap_start_wall = now
            self._start_gate_armed = False
            return

        lap_time = now - self._lap_start_wall
        if lap_time < self._min_lap_sec:
            return

        self._lap_start_wall = now
        self._lap_count += 1
        self._last_lap = lap_time
        self._start_gate_armed = False
        if self._fastest_lap is None or lap_time < self._fastest_lap:
            self._fastest_lap = lap_time
        out = Float32()
        out.data = float(lap_time)
        self._lap_pub.publish(out)

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        t0 = time.perf_counter()
        msg_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age_ms = 0.0
        if msg_ns > 0:
            now_ros = self.get_clock().now()
            age_ms = max(0.0, (now_ros.nanoseconds - msg_ns) / 1_000_000.0)

        # Keep callback timing on the same callback path used in deployment.
        duration_ms = (time.perf_counter() - t0) * 1000.0
        compute_metric = age_ms if msg_ns > 0 else duration_ms
        self._last_compute_ms = compute_metric
        out = Float32()
        out.data = float(compute_metric)
        self._compute_pub.publish(out)

    def _publish_summary(self) -> None:
        _, gpu_mem_used, gpu_mem_total = self._read_gpu_stats()
        gpu_mem_usage_percent = (
            (gpu_mem_used / gpu_mem_total * 100.0)
            if gpu_mem_total > 0.0 and gpu_mem_used >= 0.0
            else -1.0
        )
        fastest_lap = self._fastest_lap if self._fastest_lap is not None else 0.0
        lap_delta_sec = (
            (self._last_lap - fastest_lap)
            if self._last_lap > 0.0 and fastest_lap > 0.0
            else 0.0
        )
        cte_grade = self._cte_grade(self._last_cte_m)
        challenge_score = self._challenge_score(
            cte_now=self._last_cte_m,
            lap_delta_sec=lap_delta_sec,
            compute_ms=self._last_compute_ms,
            lap_count=self._lap_count,
        )

        status = DiagnosticStatus()
        status.name = "f1tenth_visual_common/stats"
        status.hardware_id = "f1tenth_gym"
        status.level = DiagnosticStatus.OK
        status.message = "kpi"
        status.values = [
            KeyValue(key="lap_time_sec", value=f"{self._last_lap:.3f}"),
            KeyValue(key="lap_time_fastest_sec", value=f"{fastest_lap:.3f}"),
            KeyValue(key="lap_count", value=str(self._lap_count)),
            KeyValue(key="cross_track_error_now_m", value=f"{self._last_cte_m:.3f}"),
            KeyValue(key="compute_ms", value=f"{self._last_compute_ms:.3f}"),
            KeyValue(key="gpu_mem_usage_percent", value=f"{gpu_mem_usage_percent:.1f}"),
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self._summary_pub.publish(msg)

        self._hud_kpi_pub.publish(status)

        challenge_msg = Float32()
        challenge_msg.data = float(challenge_score)
        self._challenge_score_pub.publish(challenge_msg)
        self._hud_challenge_score_pub.publish(challenge_msg)

        hud_gpu_mem = Float32()
        hud_gpu_mem.data = float(gpu_mem_usage_percent if gpu_mem_usage_percent >= 0.0 else 0.0)
        self._hud_gpu_mem_pub.publish(hud_gpu_mem)

        self._publish_kpi_text(
            lap_time=self._last_lap,
            lap_time_fastest=fastest_lap,
            lap_count=self._lap_count,
            cte_now=self._last_cte_m,
            compute_ms=self._last_compute_ms,
            gpu_mem_used=gpu_mem_used,
            gpu_mem_total=gpu_mem_total,
        )
        self._publish_highlights(
            lap_delta_sec=lap_delta_sec,
            cte_grade=cte_grade,
            cte_now=self._last_cte_m,
            lap_count=self._lap_count,
        )

    def _publish_kpi_text(
        self,
        lap_time: float,
        lap_time_fastest: float,
        lap_count: int,
        cte_now: float,
        compute_ms: float,
        gpu_mem_used: float,
        gpu_mem_total: float,
    ) -> None:
        if gpu_mem_total > 0.0 and gpu_mem_used >= 0.0:
            gpu_mem_pct = (gpu_mem_used / gpu_mem_total) * 100.0
            gpu_display = f"{gpu_mem_used:.1f}/{gpu_mem_total:.1f} MB ({gpu_mem_pct:.1f}%)"
        else:
            gpu_display = "N/A"

        lines = [
            f"| lap time (last)      | {lap_time:>8.3f} s                  |",
            f"| lap time (fastest)   | {lap_time_fastest:>8.3f} s                  |",
            f"| lap count            | {lap_count:>8d}                     |",
            f"| cross track error    | {cte_now:>8.3f} m                  |",
            f"| compute ms           | {compute_ms:>8.3f} ms                 |",
            f"| gpu memory usage     | {gpu_display:<28}|",
        ]
        out = String()
        out.data = "\n".join(lines)
        self._kpi_text_pub.publish(out)

    def _cte_grade(self, cte_now: float) -> str:
        if cte_now <= 0.15:
            return "A+"
        if cte_now <= 0.35:
            return "A"
        if cte_now <= 0.60:
            return "B"
        if cte_now <= 1.00:
            return "C"
        return "D"

    def _challenge_score(
        self, cte_now: float, lap_delta_sec: float, compute_ms: float, lap_count: int
    ) -> float:
        # Student-facing score: rewards stable line tracking and consistent laps.
        score = 50.0 if lap_count <= 0 else 100.0
        score -= min(70.0, max(0.0, cte_now) * 60.0)
        score -= min(20.0, max(0.0, lap_delta_sec) * 10.0)
        score -= min(10.0, max(0.0, compute_ms) * 0.2)
        return max(0.0, min(100.0, score))

    def _publish_highlights(
        self, lap_delta_sec: float, cte_grade: str, cte_now: float, lap_count: int
    ) -> None:
        if lap_count <= 0 or self._last_lap <= 0.0:
            lap_delta_text = "Lap Delta: waiting for first valid lap"
        elif abs(lap_delta_sec) < 1e-6:
            lap_delta_text = "Lap Delta: +0.000 s (best lap matched)"
        elif lap_delta_sec < 0.0:
            lap_delta_text = f"Lap Delta: {lap_delta_sec:.3f} s (faster than best)"
        else:
            lap_delta_text = f"Lap Delta: +{lap_delta_sec:.3f} s (behind best)"

        lines = [
            "Student Challenge",
            "-----------------",
            lap_delta_text,
            f"CTE Grade: {cte_grade} (now {cte_now:.3f} m)",
            "Goal: Keep CTE <= 0.35 m and improve lap delta",
        ]
        out = String()
        out.data = "\n".join(lines)
        self._highlights_pub.publish(out)

    def _read_gpu_stats(self) -> Tuple[float, float, float]:
        """Return (util_percent, mem_used_mb, mem_total_mb), -1.0 when unavailable."""
        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used,memory.total",
                    "--format=csv,noheader,nounits",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=0.2,
            )
            first = result.stdout.strip().splitlines()[0]
            util_s, used_s, total_s = [v.strip() for v in first.split(",")[:3]]
            return float(util_s), float(used_s), float(total_s)
        except Exception:
            return -1.0, -1.0, -1.0


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StatsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
