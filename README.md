# SWARM ENABLED SPATIAL INTELLIGENCE

## Overview
A brief description of your ROS 2 project, its purpose, and functionality.

## Installation
Instructions on how to install the necessary dependencies and set up the environment.

```bash
# Clone the repository
git clone https://github.com/ks20447/sesi_ros2.git
cd sesi_ros2

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
```

## Usage
How to run and use the project.

```bash
# Source the setup script
source install/setup.bash

# Run the nodes
ros2 launch your_package your_launch_file.launch.py
```

## Nodes
Description of the main nodes in the project.

### Node 1
- **Name:** `node_1`
- **Subscribed Topics:** `/topic_1`
- **Published Topics:** `/topic_2`
- **Parameters:**
    - `param_1`: Description of parameter 1

### Node 2
- **Name:** `node_2`
- **Subscribed Topics:** `/topic_3`
- **Published Topics:** `/topic_4`
- **Parameters:**
    - `param_2`: Description of parameter 2

## Launch Files
Description of the launch files and their purpose.

### `your_launch_file.launch.py`
- Launches `node_1` and `node_2`
- Parameters:
    - `param_1`: Description of parameter 1
    - `param_2`: Description of parameter 2

