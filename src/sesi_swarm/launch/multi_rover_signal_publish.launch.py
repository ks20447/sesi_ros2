from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    emitter_server = Node(
        package="sesi_emitter",
        executable="signal_emitter_server",
        output="screen",
        parameters=[
            {"x": 0.0, "y": 0.0, "s": 1.0, "max_dist": 5.0, "type": "directional"}
        ],
    )

    emitter_client_1 = Node(
        package="sesi_emitter",
        executable="signal_emitter_client",
        namespace="agent_1",
        output="screen",
        parameters=[{"agent_ns": "agent_1"}],
    )

    emitter_client_2 = Node(
        package="sesi_emitter",
        executable="signal_emitter_client",
        namespace="agent_2",
        output="screen",
        parameters=[{"agent_ns": "agent_2"}],
    )

    return LaunchDescription([emitter_server, emitter_client_1, emitter_client_2])
