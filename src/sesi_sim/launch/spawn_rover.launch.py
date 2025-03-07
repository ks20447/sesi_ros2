import os
import xacro
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)


def spawn_rover(
    context: LaunchContext,
    namespace: LaunchConfiguration,
    spawn_loc: LaunchConfiguration,
    rviz_launch: LaunchConfiguration,
):

    pkg_sesi_sim_path = get_package_share_directory("sesi_sim")
    pkg_sesi_agents_path = get_package_share_directory("sesi_agents")
    pkg_sesi_viz_path = get_package_share_directory("sesi_viz")

    namespace = context.perform_substitution(namespace)
    namespace_remap = namespace if namespace == "" else "/" + namespace
    gazebo_name = "rover" + f"_{namespace}"

    spawn_arg = context.perform_substitution(spawn_loc).split()
    spawn_location = {"x": spawn_arg[0], "y": spawn_arg[1], "z": spawn_arg[2]}

    rover_description = xacro.process(
        os.path.join(pkg_sesi_agents_path, "urdf", "leo_sim.urdf.xacro"),
        mappings={"robot_ns": namespace, "mecanum_wheels": "false"},
    )

    robot_state_publisher = Node(
        namespace=namespace,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, {"robot_description": rover_description}],
    )

    rover_spawn = Node(
        namespace=namespace,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            gazebo_name,
            "-x",
            spawn_location["x"],
            "-y",
            spawn_location["y"],
            "-z",
            spawn_location["z"],
        ],
    )

    topic_bridge = Node(
        namespace=namespace,
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        arguments=[
            namespace_remap + "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            namespace_remap + "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            namespace_remap + "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            namespace_remap
            + "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            namespace_remap + "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            namespace_remap + "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    image_bridge = Node(
        namespace=namespace,
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge",
        arguments=["camera/image_raw"],
        output="screen",
    )

    rviz_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sesi_viz_path, "launch", "agent_rviz.launch.py")
        ),
        launch_arguments={"namespace": namespace}.items(),
        condition=IfCondition(rviz_launch),
    )

    return [
        robot_state_publisher,
        rover_spawn,
        topic_bridge,
        image_bridge,
        rviz_launch_file,
    ]


def generate_launch_description():

    name_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )

    spawn_loc = DeclareLaunchArgument(
        "spawn_loc", default_value="0 0 0", description="Spawn location ('X Y Z')"
    )

    rviz_launch = DeclareLaunchArgument(
        "rviz_launch",
        default_value="true",
        description="Whether to launch Rviz application on spawn",
    )

    namespace = LaunchConfiguration("namespace")
    spawn = LaunchConfiguration("spawn_loc")
    rviz = LaunchConfiguration("rviz_launch")

    return LaunchDescription(
        [
            name_argument,
            spawn_loc,
            rviz_launch,
            OpaqueFunction(function=spawn_rover, args=[namespace, spawn, rviz]),
        ]
    )
