from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32

from f1tenth_visual_common.waypoint_io import load_waypoints_csv


Point2 = Tuple[float, float]


def _flattened_to_points(values: Sequence[float]) -> List[Point2]:
    if len(values) < 4 or len(values) % 2 != 0:
        return []
    points: List[Point2] = []
    for i in range(0, len(values), 2):
        points.append((float(values[i]), float(values[i + 1])))
    return points


def _distance_point_to_segment(px: float, py: float, a: Point2, b: Point2) -> float:
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = max(0.0, min(1.0, t))
    cx = ax + t * dx
    cy = ay + t * dy
    return math.hypot(px - cx, py - cy)


class TrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_node")

        self.declare_parameter("use_odom", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/pose")
        self.declare_parameter("racing_line_topic", "/visual/racing_line")
        self.declare_parameter("use_external_racing_line", False)
        self.declare_parameter("external_racing_line_topic", "/planning/racing_line")
        self.declare_parameter("actual_path_topic", "/visual/actual_path")
        self.declare_parameter("cross_track_error_topic", "/stats/cross_track_error")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("racing_line_xy", [0.0, 0.0, 1.0, 0.0])
        self.declare_parameter("waypoint_csv", "")
        self.declare_parameter("max_actual_points", 3000)

        self._use_odom = bool(self.get_parameter("use_odom").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._max_actual_points = int(self.get_parameter("max_actual_points").value)
        self._use_external_racing_line = bool(
            self.get_parameter("use_external_racing_line").value
        )

        waypoint_csv = str(self.get_parameter("waypoint_csv").value)
        self._racing_points = load_waypoints_csv(waypoint_csv)
        if self._racing_points:
            self.get_logger().info(
                f"Loaded {len(self._racing_points)} racing waypoints from {waypoint_csv}"
            )
        else:
            raw_line = list(self.get_parameter("racing_line_xy").value)
            self._racing_points = _flattened_to_points(raw_line)
        if len(self._racing_points) < 2:
            self.get_logger().warn(
                "racing_line_xy is invalid (need >=2 points); CTE will stay at 0.0."
            )
            self._racing_points = [(0.0, 0.0), (1.0, 0.0)]

        racing_line_topic = str(self.get_parameter("racing_line_topic").value)
        actual_path_topic = str(self.get_parameter("actual_path_topic").value)
        cte_topic = str(self.get_parameter("cross_track_error_topic").value)

        self._racing_pub = self.create_publisher(Path, racing_line_topic, 10)
        self._actual_pub = self.create_publisher(Path, actual_path_topic, 10)
        self._cte_pub = self.create_publisher(Float32, cte_topic, 10)

        self._actual_path = Path()
        self._actual_path.header.frame_id = self._frame_id
        self._racing_path = self._build_racing_path()
        if self._use_external_racing_line:
            self.create_subscription(
                Path,
                str(self.get_parameter("external_racing_line_topic").value),
                self._on_external_racing_line,
                10,
            )

        if self._use_odom:
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 50)
        else:
            self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, 50)

        self.create_timer(1.0, self._publish_racing_line)
        self.get_logger().info(
            f"trajectory_node ready (source={'odom' if self._use_odom else 'pose'})"
        )

    def _build_racing_path(self) -> Path:
        path = Path()
        path.header.frame_id = self._frame_id
        for x, y in self._racing_points:
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _publish_racing_line(self) -> None:
        self._racing_path.header.stamp = self.get_clock().now().to_msg()
        for pose in self._racing_path.poses:
            pose.header.stamp = self._racing_path.header.stamp
        self._racing_pub.publish(self._racing_path)

    def _on_external_racing_line(self, msg: Path) -> None:
        if not msg.poses:
            return
        self._racing_path = msg
        points: List[Point2] = []
        for pose in msg.poses:
            points.append((pose.pose.position.x, pose.pose.position.y))
        if len(points) >= 2:
            self._racing_points = points

    def _on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._on_pose(pose)

    def _on_pose(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            msg.header.frame_id = self._frame_id

        self._actual_path.header.stamp = msg.header.stamp
        self._actual_path.header.frame_id = msg.header.frame_id

        self._actual_path.poses.append(msg)
        if len(self._actual_path.poses) > self._max_actual_points:
            self._actual_path.poses.pop(0)

        self._actual_pub.publish(self._actual_path)
        self._publish_cte(msg.pose.position.x, msg.pose.position.y)

    def _publish_cte(self, px: float, py: float) -> None:
        best = float("inf")
        for i in range(len(self._racing_points) - 1):
            dist = _distance_point_to_segment(
                px, py, self._racing_points[i], self._racing_points[i + 1]
            )
            if dist < best:
                best = dist
        if not math.isfinite(best):
            best = 0.0

        cte = Float32()
        cte.data = float(best)
        self._cte_pub.publish(cte)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
