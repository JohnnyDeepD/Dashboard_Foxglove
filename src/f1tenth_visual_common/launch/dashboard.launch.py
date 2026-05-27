from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    package_name = "f1tenth_visual_common"
    share_dir = get_package_share_directory(package_name)
    default_topics = os.path.join(share_dir, "config", "topics.yaml")
    default_track = os.path.join(share_dir, "config", "track.yaml")
    default_waypoints = os.path.join(share_dir, "config", "waypoints_map.csv")

    use_sim_time = LaunchConfiguration("use_sim_time")
    topics_file = LaunchConfiguration("topics_file")
    track_file = LaunchConfiguration("track_file")
    waypoint_file = LaunchConfiguration("waypoint_file")

    trajectory_node = Node(
        package=package_name,
        executable="trajectory_node",
        name="trajectory_node",
        output="screen",
        parameters=[
            topics_file,
            track_file,
            {"use_sim_time": use_sim_time, "waypoint_csv": waypoint_file},
        ],
    )

    stats_node = Node(
        package=package_name,
        executable="stats_node",
        name="stats_node",
        output="screen",
        parameters=[topics_file, track_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("topics_file", default_value=default_topics),
            DeclareLaunchArgument("track_file", default_value=default_track),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            trajectory_node,
            stats_node,
        ]
    )
