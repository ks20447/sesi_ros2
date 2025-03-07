import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    controller_yaml_agent_1 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_1_controller.yaml",
    )
    planner_yaml_agent_1 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_1_planner_server.yaml",
    )
    recovery_yaml_agent_1 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_1_recovery.yaml",
    )
    bt_navigator_yaml_agent_1 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_1_bt_navigator.yaml",
    )

    controller_yaml_agent_2 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_2_controller.yaml",
    )
    planner_yaml_agent_2 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_2_planner_server.yaml",
    )
    recovery_yaml_agent_2 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_2_recovery.yaml",
    )
    bt_navigator_yaml_agent_2 = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        "rover_agent_2_bt_navigator.yaml",
    )

    return LaunchDescription(
        [
            # agent_1
            Node(
                namespace="agent_1",
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[controller_yaml_agent_1],
            ),
            Node(
                namespace="agent_1",
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[planner_yaml_agent_1],
            ),
            Node(
                namespace="agent_1",
                package="nav2_behaviors",
                executable="behavior_server",
                name="recoveries_server",
                parameters=[recovery_yaml_agent_1],
                output="screen",
            ),
            Node(
                namespace="agent_1",
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[bt_navigator_yaml_agent_1],
            ),
            # agent_2
            Node(
                namespace="agent_2",
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[controller_yaml_agent_2],
            ),
            Node(
                namespace="agent_2",
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[planner_yaml_agent_2],
            ),
            Node(
                namespace="agent_2",
                package="nav2_behaviors",
                executable="behavior_server",
                name="recoveries_server",
                parameters=[recovery_yaml_agent_2],
                output="screen",
            ),
            Node(
                namespace="agent_2",
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[bt_navigator_yaml_agent_2],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                output="screen",
                parameters=[
                    {"autostart": True},
                    {"bond_timeout": 0.0},
                    {
                        "node_names": [
                            "agent_1/controller_server",
                            "agent_1/planner_server",
                            "agent_1/recoveries_server",
                            "agent_1/bt_navigator",
                            "agent_2/controller_server",
                            "agent_2/planner_server",
                            "agent_2/recoveries_server",
                            "agent_2/bt_navigator",
                        ]
                    },
                ],
            ),
        ]
    )
