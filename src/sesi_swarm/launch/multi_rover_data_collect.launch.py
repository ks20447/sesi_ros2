import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def data_collector_nodes(context: LaunchContext, filename: LaunchConfiguration):

    filename = str(context.perform_substitution(filename))

    rover_1_patrol = Node(
        package="sesi_tasks",
        executable="data_collector_exe",
        parameters=[
            {"agent_ns": "agent_1"},
            {"filename": filename},
            {"duration": 30.0},
        ],
        output="screen",
    )

    rover_2_patrol = Node(
        package="sesi_tasks",
        executable="data_collector_exe",
        parameters=[
            {"agent_ns": "agent_2"},
            {"filename": filename},
            {"duration": 30.0},
        ],
        output="screen",
    )

    return [rover_1_patrol, rover_2_patrol]


def generate_launch_description():

    filename_argument = DeclareLaunchArgument(
        "filename",
        default_value="",
        description="Filename prefix for data collection .csv file.",
    )

    filename = LaunchConfiguration("filename")

    return LaunchDescription(
        [
            filename_argument,
            OpaqueFunction(function=data_collector_nodes, args=[filename]),
        ]
    )
