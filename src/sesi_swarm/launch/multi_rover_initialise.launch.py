import os
import yaml
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_sesi_sim_path = get_package_share_directory("sesi_sim")
    pkg_sesi_nav_path = get_package_share_directory("sesi_nav")
    pkg_sesi_slam_path = get_package_share_directory("sesi_slam")
    
    
    rover_agent_1_config = os.path.join(pkg_sesi_nav_path, "config_files", "rover_agent_1_amcl.yaml")
    rover_agent_2_config = os.path.join(pkg_sesi_nav_path, "config_files", "rover_agent_2_amcl.yaml")
    map_file = os.path.join(pkg_sesi_slam_path, "maps", "square_world_save.yaml")

    rover_1_ns = "agent_1"
    rover_2_ns = "agent_2"

    # IMPORTANT: Make sure config file initial pose matches these values
    rover_1_spawn = "2.5 -1.0 0"
    rover_2_spawn = "-2.5 -1.0 0"

    rover_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sesi_sim_path, "launch", "spawn_rover.launch.py")
        ),
        launch_arguments={
            "namespace": rover_1_ns,
            "spawn_loc": rover_1_spawn,
            "rviz_launch": "false",
        }.items(),
    )

    rover_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sesi_sim_path, "launch", "spawn_rover.launch.py")
        ),
        launch_arguments={
            "namespace": rover_2_ns,
            "spawn_loc": rover_2_spawn,
            "rviz_launch": "false",
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"topic_name": "map"},
            {"frame_id": "map"},
            {"yaml_filename": map_file},
        ],
    )

    rover_1_amcl = Node(
        namespace="agent_1",
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[rover_agent_1_config],
    )

    rover_2_amcl = Node(
        namespace="agent_2",
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[rover_agent_2_config],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"bond_timeout": 0.0},
            {"node_names": ["map_server", f"{rover_1_ns}/amcl", f"{rover_2_ns}/amcl"]},
        ],
    )

    return LaunchDescription(
        [
            rover_1,
            rover_2,
            map_server,
            rover_1_amcl,
            rover_2_amcl,
            lifecycle_manager,
        ]
    )
