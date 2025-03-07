import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CirclePatrol(Node):

    def __init__(self):

        super().__init__("circle_patrol")

        self.declare_parameter("x_velocity", 0.425)
        self.declare_parameter("z_angular", 1.0)
        self.declare_parameter("namespace", "")

        self.x_velocity = (
            self.get_parameter("x_velocity").get_parameter_value().double_value
        )
        self.z_angular = (
            self.get_parameter("z_angular").get_parameter_value().double_value
        )

        namespace = self.get_parameter("namespace").get_parameter_value().string_value

        if namespace != "":
            self.namespace = "/" + namespace
        else:
            self.namespace = namespace

        self.vel_publisher = self.create_publisher(
            Twist, f"{self.namespace}/cmd_vel", 10
        )

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        action = Twist()

        action.linear.x = self.x_velocity
        action.angular.z = self.z_angular

        self.vel_publisher.publish(action)


def main(args=None):

    rclpy.init(args=args)

    node = CirclePatrol()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
