import os
import time
import math
import rclpy
import numpy as np
import pandas as pd
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol_node")

        self.declare_parameter("namespace", "")
        namespace = self.get_parameter("namespace").get_parameter_value().string_value

        self.client = ActionClient(
            self, NavigateToPose, f"{namespace}/navigate_to_pose"
        )

        self.declare_parameter("patrol_shape", "square")
        self.declare_parameter("sensing_range", 5.0)

        self.patrol_shape = (
            self.get_parameter("patrol_shape")
            .get_parameter_value()
            .string_value.lower()
        )

        self.sensing_range = (
            self.get_parameter("sensing_range").get_parameter_value().double_value
        )

        self.patrol_csv = f"{self.patrol_shape}.csv"
        self.patrol_perimeter = 2 * self.sensing_range * math.pi

        self.patrol_side_length = self.calculate_side_length()

        pkg_sesi_patrol_path = get_package_share_directory("sesi_patrol")
        self.waypoints = pd.read_csv(
            os.path.join(pkg_sesi_patrol_path, "patrol_routes", self.patrol_csv)
        )

        self.get_logger().info(
            f"Patrol Node Created. Perimeter: {self.patrol_perimeter:.2f}, Side Length: {self.patrol_side_length}"
        )

        self.current_waypoint = 0
        self.client.wait_for_server()
        self.send_next_goal()

    def calculate_side_length(self):

        formulas = {
            "triangle": self.patrol_perimeter / 3,
            "square": self.patrol_perimeter / 4,
            "hexagon": self.patrol_perimeter / 6,
        }

        return formulas.get(self.patrol_shape, "Invalid formula")

    def send_next_goal(self):

        self.x = self.waypoints["x"][self.current_waypoint] * self.patrol_side_length
        self.y = self.waypoints["y"][self.current_waypoint] * self.patrol_side_length

        self.next_waypoint = (self.current_waypoint + 1) % len(self.waypoints)

        self.x_next = self.waypoints["x"][self.next_waypoint] * self.patrol_side_length
        self.y_next = self.waypoints["y"][self.next_waypoint] * self.patrol_side_length

        self.yaw = math.atan2(self.y_next - self.y, self.x_next - self.x)
        self.yaw_quat = quaternion_from_euler(0, 0, self.yaw)

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.x = self.yaw_quat[0]
        pose.pose.orientation.y = self.yaw_quat[1]
        pose.pose.orientation.z = self.yaw_quat[2]
        pose.pose.orientation.w = self.yaw_quat[3]
        goal_msg.pose = pose

        self.get_logger().info(
            f"Navigating to waypoint {self.current_waypoint + 1} ({self.x}, {self.y}, {self.yaw})"
        )
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info("Goal reached!")
        else:
            self.get_logger().info("Goal failed!")

        self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
        # time.sleep(2)  # Pause before moving to next waypoint
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
