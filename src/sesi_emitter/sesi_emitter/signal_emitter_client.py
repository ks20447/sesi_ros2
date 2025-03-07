import rclpy
from rclpy.node import Node
from sesi_interfaces.srv import Emitter


class AddEmitterAgent(Node):

    def __init__(self):
        super().__init__("add_emitter_agent")

        name_service = "/add_agent_to_emitter"
        self.client = self.create_client(Emitter, name_service)

        self.declare_parameter("agent_ns", "")

        self.agent_ns = (
            self.get_parameter("agent_ns").get_parameter_value().string_value
        )

        # Wait for the service to be available (checks every second)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service " + name_service + " not available, waiting again..."
            )

        self.send_request()

    def send_request(self):

        self.req = Emitter.Request()
        # Set the request label
        self.req.agent_ns = self.agent_ns

        # Send the request asynchronously
        self.future = self.client.call_async(self.req)


def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)

    # Declare the node constructor
    client = AddEmitterAgent()

    while rclpy.ok():
        # Spin once to check for a service response
        rclpy.spin_once(client)

        if client.future.done():
            try:
                # Check if a response from the service was received
                response = client.future.result()
            except Exception as e:
                # Log any exceptions
                client.get_logger().info(f"Service call failed: {e}")
            else:
                # Log the service response
                client.get_logger().info(f"Success: {response.success}")
            break

    # Destroy the client node
    client.destroy_node()

    # Shutdown the ROS communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()
