import os
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchContext
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)


def navigation(context: LaunchContext, namespace: LaunchConfiguration):
    
    namespace = context.perform_substitution(namespace)

    controller_yaml_agent = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        f"rover_{namespace}_controller.yaml",
    )
    planner_yaml_agent = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        f"rover_{namespace}_planner_server.yaml",
    )
    recovery_yaml_agent = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        f"rover_{namespace}_recovery.yaml",
    )
    bt_navigator_yaml_agent = os.path.join(
        get_package_share_directory("sesi_nav"),
        "config_files",
        f"rover_{namespace}_bt_navigator.yaml",
    )

    return [
        Node(
            namespace=namespace,
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[controller_yaml_agent],
        ),
        Node(
            namespace=namespace,
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[planner_yaml_agent],
        ),
        Node(
            namespace=namespace,
            package="nav2_behaviors",
            executable="behavior_server",
            name="recoveries_server",
            parameters=[recovery_yaml_agent],
            output="screen",
        ),
        Node(
            namespace=namespace,
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[bt_navigator_yaml_agent],
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
                        f"{namespace}/controller_server",
                        f"{namespace}/planner_server",
                        f"{namespace}/recoveries_server",
                        f"{namespace}/bt_navigator",
                    ]
                },
            ],
        ),
    ]


def generate_launch_description():

    name_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )

    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            name_argument,
            OpaqueFunction(function=navigation, args=[namespace]),
        ]
    )
