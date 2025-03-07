import os
import sys
import rclpy
from rclpy.node import Node
from jinja2 import Template
from ament_index_python.packages import get_package_share_directory


class ConfigGenerateNode(Node):

    def __init__(self):
        super().__init__("ConfigGenerateNode")

        if len(sys.argv) != 5:
            self.get_logger().warn(f"Invalid number of arguments. (namespace x y yaw)")
        else:
            self.namespace = str(sys.argv[1])
            self.x = float(sys.argv[2])
            self.y = float(sys.argv[3])
            self.yaw = float(sys.argv[4])

            self.generate_files()

    def generate_files(self):

        pkg_sesi_nav_config_path = get_package_share_directory("sesi_nav")

        TEMPLATE_FOLDER = os.path.join(pkg_sesi_nav_config_path, "config_templates")
        OUTPUT_FOLDER = "config_files"

        # Ensure output folder exists
        os.makedirs(OUTPUT_FOLDER, exist_ok=True)

        # Get all template files
        template_files = [f for f in os.listdir(TEMPLATE_FOLDER) if f.endswith(".yaml")]

        # Process each template
        for template_file in template_files:
            template_path = os.path.join(TEMPLATE_FOLDER, template_file)

            # Load the template content
            with open(template_path, "r") as file:
                template_str = file.read()

            # Generate YAML
            template = Template(template_str)
            yaml_content = template.render(
                namespace=self.namespace, x=self.x, y=self.y, yaw=self.yaw
            )

            # Create output filename
            output_filename = f"rover_{self.namespace}_{template_file}"
            output_path = os.path.join(OUTPUT_FOLDER, output_filename)

            # Save the generated YAML
            with open(output_path, "w") as f:
                f.write(yaml_content)
            self.get_logger().info(f"Generated {output_path}")



def main(args=None):
    rclpy.init(args=args)
    node = ConfigGenerateNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
