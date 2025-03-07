import os
import xacro
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)


def rviz_launch(
    context: LaunchContext,
    namespace: LaunchConfiguration,
):

    pkg_sesi_viz_path = get_package_share_directory("sesi_viz")

    namespace = context.perform_substitution(namespace)

    if namespace == "":
        rviz_config_file = os.path.join(pkg_sesi_viz_path, "config", f"mapper_agent.rviz")
    else:
        rviz_config_file = os.path.join(pkg_sesi_viz_path, "config", f"rover_{namespace}.rviz")

    rviz_node = Node(
        namespace=namespace,
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[
            ("/odom", "odom"),
            ("/initialpose", "initialpose"),
            ("/goal_pose", "goal_pose"),
        ],
    )

    return [rviz_node]


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
            OpaqueFunction(function=rviz_launch, args=[namespace]),
        ]
    )
