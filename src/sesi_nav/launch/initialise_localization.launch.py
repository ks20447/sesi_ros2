import os
import yaml
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)


def localization(
    context: LaunchContext,
    namespace: LaunchConfiguration,
    map_file: LaunchConfiguration,
    amcl_file: LaunchConfiguration,
):

    namespace = context.perform_substitution(namespace)
    map_file = context.perform_substitution(map_file)
    amcl_file = context.perform_substitution(amcl_file)

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

    rover_amcl = Node(
        namespace=namespace,
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_file],
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
            {"node_names": ["map_server", f"{namespace}/amcl"]},
        ],
    )

    return [
        map_server,
        rover_amcl,
        lifecycle_manager,
    ]


def generate_launch_description():

    pkg_sesi_slam_path = get_package_share_directory("sesi_slam")
    pkg_sesi_nav_path = get_package_share_directory("sesi_nav")
    
    rover_agent_config = os.path.join(
        pkg_sesi_nav_path, "config_files", f"rover_agent_amcl.yaml"
    )

    name_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )

    map_argument = DeclareLaunchArgument(
        "map_file",
        default_value=os.path.join(
            pkg_sesi_slam_path, "maps", "square_world_save.yaml"
        ),
        description="Map file to pass to map server. ",
    )
    
    amcl_file_argument = DeclareLaunchArgument(
        "amcl_file",
        default_value=rover_agent_config,
        description="AMCL file to pass to AMCL node.",
    )

    namespace = LaunchConfiguration("namespace")
    map_file = LaunchConfiguration("map_file")
    amcl_file = LaunchConfiguration("amcl_file")

    return LaunchDescription(
        [
            name_argument,
            map_argument,
            amcl_file_argument,
            OpaqueFunction(function=localization, args=[namespace, map_file, amcl_file]),
        ]
    )
