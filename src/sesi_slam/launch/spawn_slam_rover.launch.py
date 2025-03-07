import os
import yaml
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_sesi_sim_path = get_package_share_directory("sesi_sim")
    pkg_slam_toolbox_path = get_package_share_directory("slam_toolbox")

    # This is set due to not being able to get slam_toolbox working with namespace
    mapper_ns = ""
    mapper_spawn_loc = "0.0 0.0 0.0"

    mapper_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sesi_sim_path, "launch", "spawn_rover.launch.py")
        ),
        launch_arguments={
            "namespace": mapper_ns,
            "spawn_loc": mapper_spawn_loc,
            "rviz_launch": "true",
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_path, "launch", "online_async_launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription(
        [
            mapper_spawn,
            slam_launch,
        ]
    )
