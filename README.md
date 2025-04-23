# QRB ROS Motion Manager Service

## Overview
The `qrb_ros_motion_manager_service` is a ROS-based service designed for managing motion in robotic systems. It interfaces with the motion control components, handling commands and responses efficiently. The service is part of a broader ecosystem developed for robotics motion management, and it leverages ROS messages and actions for seamless integration and communication.

## How to Build
1. **Clone the Repository:**
   ```bash
   git clone https://github.com/qualcomm-qrb-ros/qrb_ros_motion_manager_service.git
   ```
2. **Navigate to the workspace:**
   ```bash
   cd /path/to/your/catkin_ws
   ```
3. **Add the package:**
   Place the cloned `qrb_ros_motion_manager_service` in the `src` directory of your workspace.
4. **Install Dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
5. **Build the Workspace:**
   ```bash
   catkin_make
   ```
6. **Source the Setup File:**
   ```bash
   source devel/setup.bash
   ```

## How to Run
1. **Launch the ROS Core:**
   ```bash
   roscore
   ```
2. **Run the Motion Manager Service:**
   ```bash
   rosrun qrb_ros_motion_manager_service motion_manager_service_node
   ```
3. **Use ROS tools (like `rosservice`, `rostopic`, etc.) to interact with the service.**

## Package Content
- **include/qrb_ros_motion_service**: Header files defining service and utility interfaces.
- **src/**: Contains the implementation of the motion manager service.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: Package metadata and dependencies.

## API (ROS Messages)
The `qrb_ros_motion_manager_service` uses ROS messages defined in the `qrb_ros_motion_msgs` package. For detailed message formats, visit [QRB ROS Motion Messages](https://github.com/qualcomm-qrb-ros/qrb_ros_interfaces/tree/main/qrb_ros_motion_msgs).

**Commonly Used Messages:**
- `MotionCommand`: Used to send commands to the motion manager.
- `MotionStatus`: Used to receive status updates from the motion system.
- `MotionError`: Used to report any errors encountered.

For additional details on message definitions and examples, refer to the `qrb_ros_motion_msgs` documentation.
