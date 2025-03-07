import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_sesi_sim_prefix = get_package_prefix("sesi_sim")
    pkg_sesi_worlds_path = get_package_share_directory("sesi_worlds")
    pkg_sesi_agents_path = get_package_share_directory("sesi_agents")
    pkg_ros_gz_sim_path = get_package_share_directory("ros_gz_sim")

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_sesi_worlds_path, "worlds", "square_world.sdf"),
        description="Path to the Gazebo world file",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim_path, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
    )

    # No longer needed due to use of hooks
    # gz_resource_env = SetEnvironmentVariable(
    #     name="GZ_SIM_RESOURCE_PATH",
    #     value=":".join(
    #         [
    #             pkg_sesi_agents_path,
    #             # os.path.join(os.getenv("COLCON_PREFIX_PATH", ""), "..", "src"),
    #         ]
    #     ),
    # )

    gz_plugin_env = SetEnvironmentVariable(
        name="GZ_GUI_PLUGIN_PATH",
        value=":".join(
            [
                os.path.join(get_package_share_directory("leo_gz_plugins"), "lib"),
            ]
        ),
    )

    return LaunchDescription(
        [gz_plugin_env, sim_world, topic_bridge, gz_sim]
    )
