import os
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sesi_interfaces.msg import SignalStrength


class DataCollect(Node):

    def __init__(self):

        super().__init__("data_collect")

        self.declare_parameter("agent_ns", "")

        self.declare_parameter("filename", "")

        self.declare_parameter("duration", 10.0)

        namespace = self.get_parameter("agent_ns").get_parameter_value().string_value
        filename = self.get_parameter("filename").get_parameter_value().string_value
        duration = self.get_parameter("duration").get_parameter_value().double_value

        self.duration = duration

        os.makedirs("data", exist_ok=True)
        self.filename = f"data/{filename}_{namespace}.csv"

        self.data_subscriber = self.create_subscription(
            SignalStrength, f"{namespace}/signal_strength", self.data_callback, 10
        )

        self.data = [
            [
                "timestamp",
                "left_x",
                "left_y",
                "left_yaw",
                "left_s",
                "right_x",
                "right_y",
                "right_yaw",
                "right_s",
                "middle_x",
                "middle_y",
                "middle_yaw",
                "middle_s",
                "right_back_x",
                "right_back_y",
                "right_back_yaw",
                "right_back_s",
                "left_back_x",
                "left_back_y",
                "left_back_yaw",
                "left_back_s",
            ]
        ]

        self.timer = self.create_timer(self.duration, self.stop_data_subscription)
        self.time_start = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"Starting data collection for {self.duration:.2f} seconds. ({namespace})"
        )

    def data_callback(self, msg):

        sec = msg.timestamp.sec - self.time_start.sec
        milisec = int(msg.timestamp.nanosec / 1e6)
        if milisec < 100:
            timestamp = float(f"{sec}.0{milisec}")
            if milisec < 10:
                timestamp = float(f"{sec}.00{milisec}")
        else:
            timestamp = float(f"{sec}.{milisec}")

        antenna_data = [timestamp]
        for antenna in msg.antenna_array:

            x = round(antenna.x, 3)
            y = round(antenna.y, 3)
            yaw = round(antenna.yaw, 3)
            s = round(antenna.s, 3)

            antenna_data.extend([x, y, yaw, s])

        self.data.append(antenna_data)

    def save_data(self):

        with open(self.filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerows(self.data)

    def stop_data_subscription(self):

        self.save_data()
        self.get_logger().info(f"Data saved to {self.filename}.")
        self.get_logger().info(
            f"Data collection successfully finished after {self.duration:.2f} seconds."
        )
        self.data_subscriber  # Keep the object in scope (ROS will handle shutdown properly)
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = DataCollect()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
