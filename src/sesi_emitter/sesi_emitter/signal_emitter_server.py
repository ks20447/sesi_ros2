import math
import rclpy
import random
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter
from sesi_interfaces.srv import Emitter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sesi_interfaces.msg import SignalStrength, Antenna
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


class SignalStrengthEmitter(Node):

    def __init__(self):

        super().__init__("signal_emitter_service")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("s", 1.0)
        self.declare_parameter("max_dist", 5.0)
        self.declare_parameter("type", "RSS")

        self.x = self.get_parameter("x").get_parameter_value().double_value
        self.y = self.get_parameter("y").get_parameter_value().double_value
        self.signal_strength = (
            self.get_parameter("s").get_parameter_value().double_value
        )
        self.signal_max_distance = (
            self.get_parameter("max_dist").get_parameter_value().double_value
        )

        self.emitter_type = (
            self.get_parameter("type").get_parameter_value().string_value.upper()
        )
        if self.emitter_type == "RSS":
            self.calculate_signal_strength_func = self.r_squared_strength
        elif self.emitter_type == "DIRECTIONAL":
            self.calculate_signal_strength_func = self.sinc_strength
        elif self.emitter_type == "DIRECTIONAL_RSS":
            self.calculate_signal_strength_func = self.directional_rss_strength

        self.agent_publishers = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.publish_signal_strength)

        self.srv = self.create_service(
            Emitter, "/add_agent_to_emitter", self.add_agent_callback
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Emitter Server Started")
        self.get_logger().info(
            f"{self.emitter_type} emitter placed at ({self.x}, {self.y}) with strength {self.signal_strength} and max distance {self.signal_max_distance}"
        )
        self.get_logger().info(f"Number of active agents: {len(self.agent_publishers)}")

    def add_agent_callback(self, request, response):

        namespace = request.agent_ns

        try:
            signal_strength_publisher = self.create_publisher(
                SignalStrength, f"{namespace}/signal_strength", 10
            )

            self.agent_publishers[namespace] = signal_strength_publisher

            response.success = True
            self.get_logger().info(f'Agent "{namespace}" added to emitter')
            self.get_logger().info(
                f"Number of active agents: {len(self.agent_publishers)}"
            )

        except:
            response.success = False
            self.get_logger().warn(f'Failed to add agent "{namespace}" to emitter')

        response.num_agents = len(self.agent_publishers)

        return response

    def parameters_callback(self, params):

        for param in params:
            if param.name == "x" and param.type_ == Parameter.Type.DOUBLE:
                self.x = param.value
                self.get_logger().info(f"Updated parameter '{param.name}' to {self.x}")
            elif param.name == "y" and param.type_ == Parameter.Type.DOUBLE:
                self.y = param.value
                self.get_logger().info(f"Updated parameter '{param.name}' to {self.y}")
            elif param.name == "s" and param.type_ == Parameter.Type.DOUBLE:
                self.signal_strength = param.value
                self.get_logger().info(
                    f"Updated parameter '{param.name}' to {self.signal_strength}"
                )
            elif param.name == "max_dist" and param.type_ == Parameter.Type.DOUBLE:
                self.signal_max_distance = param.value
                self.get_logger().info(
                    f"Updated parameter '{param.name}' to {self.signal_max_distance}"
                )
            elif param.name == "type" and param.type_ == Parameter.Type.STRING:

                if param.value.upper() == "RSS":
                    self.emitter_type = "RSS"
                    self.calculate_signal_strength_func = self.r_squared_strength
                elif param.value.upper() == "DIRECTIONAL":
                    self.emitter_type = "DIRECTIONAL"
                    self.calculate_signal_strength_func = self.sinc_strength
                elif param.value.upper() == "DIRECTIONAL_RSS":
                    self.emitter_type = "DIRECTIONAL_RSS"
                    self.calculate_signal_strength_func = self.directional_rss_strength
                self.get_logger().info(
                    f"Updated parameter '{param.name}' to {param.value.upper()}"
                )
            else:
                self.get_logger().warn(
                    f"Parameter {param.name} not handled or invalid type."
                )
                return SetParametersResult(successful=False)

        self.get_logger().info(
            f"Emitter updated: type: {self.emitter_type}, x: {self.x}, y: {self.y}), source strength: {self.signal_strength}, max distance: {self.signal_max_distance}"
        )

        return SetParametersResult(successful=True)

    def get_antenna_positions_and_yaw(self, namespace):

        try:

            antenna_names = ["left", "right", "middle", "left_back", "right_back"]
            antenna_positions = []
            antenna_yaws = []

            for antenna in antenna_names:

                transform = self.tf_buffer.lookup_transform(
                    f"map", f"{namespace}/antenna_link_{antenna}", rclpy.time.Time()
                )

                x = transform.transform.translation.x
                y = transform.transform.translation.y
                q = transform.transform.rotation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                antenna_positions.extend([x, y])
                antenna_yaws.append(yaw)

            return antenna_positions, antenna_yaws

        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ) as e:
            self.get_logger().warn(f"Transform error: {e}")
            return None, None

    def r_squared_strength(self, positions, yaws):

        strengths = []

        for i in self.num_antenna:

            dist = math.sqrt(
                (self.x - positions[2 * i]) ** 2 + (self.y - positions[2 * i + 1]) ** 2
            )

            if dist > self.signal_max_distance:
                strength = 0
            else:
                strength = self.signal_strength / (dist**2 + 1e-6)
                strength += random.gauss(
                    self.signal_strength / 10, self.signal_strength / 100
                )
                strength = min(max(strength, 0), 1000)

            strengths.append(float(strength))

        return strengths

    def sinc_strength(self, positions, yaws):

        strengths = []

        for i in self.num_antenna:

            dist = math.sqrt(
                (self.x - positions[2 * i]) ** 2 + (self.y - positions[2 * i + 1]) ** 2
            )

            if dist > self.signal_max_distance:
                strength = 0
            else:
                yaw = yaws[i]

                dx = self.x - positions[2 * i]
                dy = self.y - positions[2 * i + 1]
                target_yaw = math.atan2(dy, dx)
                angle_diff = target_yaw - yaw
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
                strength = (np.sinc(4 * angle_diff)) ** 2
                strength += random.gauss(0, 0.03)
                strength = min(max(strength, 0), 1)

            strengths.append(float(strength))

        return strengths

    def directional_rss_strength(self, positions, yaws):

        strengths = []

        for i in self.num_antenna:

            dist = math.sqrt(
                (self.x - positions[2 * i]) ** 2 + (self.y - positions[2 * i + 1]) ** 2
            )

            if dist > self.signal_max_distance:
                strength = 0
            else:
                yaw = yaws[i]

                dx = self.x - positions[2 * i]
                dy = self.y - positions[2 * i + 1]
                target_yaw = math.atan2(dy, dx)
                angle_diff = target_yaw - yaw
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

                sinc_strength = (np.sinc(angle_diff)) ** 2
                rss_strength = self.signal_strength / (dist**2 + 1e-6)

                strength = sinc_strength * rss_strength
                strength += random.gauss(
                    self.signal_strength / 10, self.signal_strength / 100
                )
                strength = min(max(strength, 0), 1000)

            strengths.append(float(strength))

        return strengths

    def publish_signal_strength(self):

        for namespace, publisher in self.agent_publishers.items():

            positions, yaws = self.get_antenna_positions_and_yaw(namespace)

            if positions is None or yaws is None:
                return

            self.num_antenna = range(len(yaws))
            antenna_strengths = self.calculate_signal_strength_func(positions, yaws)
            timestamp = self.get_clock().now().to_msg()

            msg = SignalStrength()
            antenna_data = []

            for i in self.num_antenna:

                x = positions[i * 2]
                y = positions[i * 2 + 1]
                yaw = yaws[i]
                s = antenna_strengths[i]

                antenna = Antenna(x=x, y=y, yaw=yaw, s=s)
                antenna_data.append(antenna)

            msg.agent_ns = namespace
            msg.timestamp.sec = timestamp.sec
            msg.timestamp.nanosec = timestamp.nanosec
            msg.antenna_array = antenna_data

            publisher.publish(msg)


def main():

    rclpy.init()

    node = SignalStrengthEmitter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
