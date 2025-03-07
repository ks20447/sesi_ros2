import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sesi_interfaces.srv import GridPub
from sesi_interfaces.msg import SignalStrength
from std_msgs.msg import Header, Float32
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


class GridPublisherServer(Node):
    def __init__(self):
        super().__init__("signal_strength_grid_service")

        self.srv = self.create_service(
            GridPub, "/publish_signal_strength_grid", self.add_publisher_callback
        )

        self.publisher_list = []
        self.grids = []

        self.subscriber = None

        self.get_logger().info("Grid Publisher Server Started")

    def add_publisher_callback(self, request, response):

        agent_ns = request.agent_ns

        self.grid_resolution = request.resolution
        self.grid_width = int(16 / self.grid_resolution)
        self.grid_height = int(16 / self.grid_resolution)
        self.grid_origin = (-10.0, -10.0)  # Bottom-left corner in world coordinates

        antenna_names = ["left", "right", "middle", "right_back", "left_back"]
        publisher_names = []

        for i, antenna in enumerate(antenna_names):
            grid = OccupancyGrid()
            grid.header = Header(frame_id="map")
            grid.info.resolution = self.grid_resolution
            grid.info.width = self.grid_width
            grid.info.height = self.grid_height
            grid.info.origin.position.x = self.grid_origin[0]
            grid.info.origin.position.y = self.grid_origin[1]
            grid.info.origin.position.z = 0.0
            grid.data = [-1] * (self.grid_width * self.grid_height)

            self.grids.append(grid)

            publisher = f"{agent_ns}/signal_grid_map_antenna_{antenna}"
            publisher_names.append(publisher)

            self.publisher_list.append(
                self.create_publisher(OccupancyGrid, publisher, 10)
            )

        self.subscriber = self.create_subscription(
            SignalStrength,
            f"{agent_ns}/signal_strength",
            self.signal_strength_callback,
            10,
        )

        response.success = True
        response.publisher_names = publisher_names

        return response

    def signal_strength_callback(self, msg):

        for i, antenna in enumerate(msg.antenna_array):

            x = antenna.x
            y = antenna.y
            signal_strength = antenna.s

            grid_x = int(((x) - self.grid_origin[0]) / self.grid_resolution)
            grid_y = int(((y) - self.grid_origin[1]) / self.grid_resolution)
            grid_index = int(grid_y * self.grid_width + grid_x)

            self.grids[i].data[grid_index] = int(min(signal_strength, 127))

            self.publisher_list[i].publish(self.grids[i])


def main():

    rclpy.init()

    node = GridPublisherServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
