# NAO6 ROS2 Setup Guide for HRIStudio

This guide walks you through setting up your NAO6 robot with ROS2 integration for use with HRIStudio's experiment platform.

## Prerequisites

- NAO6 robot with NAOqi OS 2.8.7+
- Ubuntu 22.04.5 LTS computer (x86_64)
- Network connectivity between computer and NAO6
- Administrative access to both systems

## Overview

The integration uses the `naoqi_driver2` package to bridge NAOqi with ROS2, exposing all robot capabilities through standard ROS2 topics and services. HRIStudio connects via WebSocket using `rosbridge_server`.

## Step 1: NAO6 Network Configuration

1. **Power on your NAO6** and wait for boot completion
2. **Connect NAO6 to your network**:
   - Press chest button to get IP address
   - Or use Choregraphe to configure WiFi
3. **Verify connectivity**:
   ```bash
   ping nao.local  # or robot IP address
   ```

## Step 2: ROS2 Humble Installation

Install ROS2 Humble on your Ubuntu 22.04 system:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 3: Install NAO ROS2 Packages

Install the required ROS2 packages for NAO6 integration:

```bash
# Install naoqi_driver2 and dependencies
sudo apt install ros-humble-naoqi-driver2
sudo apt install ros-humble-naoqi-bridge-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-nav-msgs
sudo apt install ros-humble-std-msgs

# Install rosbridge for HRIStudio communication
sudo apt install ros-humble-rosbridge-suite

# Install additional useful packages
sudo apt install ros-humble-rqt
sudo apt install ros-humble-rqt-common-plugins
```

## Step 4: Configure NAO Connection

Create a launch file for easy NAO6 connection:

```bash
# Create workspace
mkdir -p ~/nao_ws/src
cd ~/nao_ws

# Create launch file directory
mkdir -p src/nao_launch/launch

# Create the launch file
cat > src/nao_launch/launch/nao6_hristudio.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # NAO IP configuration
        DeclareLaunchArgument('nao_ip', default_value='nao.local'),
        DeclareLaunchArgument('nao_port', default_value='9559'),
        DeclareLaunchArgument('bridge_port', default_value='9090'),
        
        # NAOqi Driver
        Node(
            package='naoqi_driver2',
            executable='naoqi_driver',
            name='naoqi_driver',
            parameters=[{
                'nao_ip': LaunchConfiguration('nao_ip'),
                'nao_port': LaunchConfiguration('nao_port'),
                'publish_joint_states': True,
                'publish_odometry': True,
                'publish_camera': True,
                'publish_sensors': True,
                'joint_states_frequency': 30.0,
                'odom_frequency': 30.0,
                'camera_frequency': 15.0,
                'sensor_frequency': 10.0
            }],
            output='screen'
        ),
        
        # Rosbridge WebSocket Server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': LaunchConfiguration('bridge_port'),
                'address': '0.0.0.0',
                'authenticate': False,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': 10000000
            }],
            output='screen'
        )
    ])
EOF

# Create package.xml
cat > src/nao_launch/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypeid="pf3"?>
<package format="3">
  <name>nao_launch</name>
  <version>1.0.0</version>
  <description>Launch files for NAO6 HRIStudio integration</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>naoqi_driver2</exec_depend>
  <exec_depend>rosbridge_server</exec_depend>
</package>
EOF

# Create CMakeLists.txt
cat > src/nao_launch/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(nao_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()
EOF

# Build the workspace
colcon build
source install/setup.bash
```

## Step 5: Test NAO Connection

Start the NAO6 ROS2 integration:

```bash
cd ~/nao_ws
source install/setup.bash

# Launch with your NAO's IP address
ros2 launch nao_launch nao6_hristudio.launch.py nao_ip:=YOUR_NAO_IP
```

Replace `YOUR_NAO_IP` with your NAO's actual IP address (e.g., `192.168.1.100`).

## Step 6: Verify ROS2 Topics

In a new terminal, verify that NAO topics are publishing:

```bash
source /opt/ros/humble/setup.bash

# List all topics
ros2 topic list

# You should see these NAO6 topics:
# /cmd_vel - Robot velocity commands
# /odom - Odometry data
# /joint_states - Joint positions and velocities
# /joint_angles - NAO-specific joint control
# /camera/front/image_raw - Front camera
# /camera/bottom/image_raw - Bottom camera
# /imu/torso - Inertial measurement unit
# /bumper - Foot bumper sensors
# /hand_touch - Hand tactile sensors
# /head_touch - Head tactile sensors
# /sonar/left - Left ultrasonic sensor
# /sonar/right - Right ultrasonic sensor
# /info - Robot information

# Test specific topics
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
ros2 topic echo /info --once
```

## Step 7: Test Robot Control

Test basic robot control:

```bash
# Make NAO say something
ros2 topic pub /speech std_msgs/msg/String "data: 'Hello from ROS2!'" --once

# Move head (be careful with joint limits)
ros2 topic pub /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed \
  "joint_names: ['HeadYaw']
   joint_angles: [0.5]
   speed: 0.3" --once

# Basic walking command (very small movement)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.1, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}" --once

# Stop movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}" --once
```

## Step 8: Configure HRIStudio

1. **Start HRIStudio** with your development setup
2. **Add NAO6 Plugin Repository**:
   - Go to Admin → Plugin Repositories
   - Add the HRIStudio official repository if not already present
   - Sync to get the latest plugins including `nao6-ros2`

3. **Install NAO6 Plugin**:
   - In your study, go to Plugins
   - Install the "NAO6 Robot (ROS2 Integration)" plugin
   - Configure the ROS bridge URL: `ws://YOUR_COMPUTER_IP:9090`

4. **Create Experiment**:
   - Use the experiment designer
   - Add NAO6 actions from the robot blocks section
   - Configure parameters for each action

5. **Run Trial**:
   - Ensure your NAO6 ROS2 system is running
   - Start a trial in HRIStudio
   - Control the robot through the wizard interface

## Available Robot Actions

Your NAO6 plugin provides these actions for experiments:

### Movement Actions
- **Walk with Velocity**: Control linear/angular velocity
- **Stop Walking**: Emergency stop
- **Set Joint Angle**: Control individual joints
- **Turn Head**: Head orientation control

### Interaction Actions
- **Say Text**: Text-to-speech via ROS2

### Sensor Actions
- **Get Camera Image**: Capture from front/bottom cameras
- **Get Joint States**: Read all joint positions
- **Get IMU Data**: Inertial measurement data
- **Get Bumper Status**: Foot contact sensors
- **Get Touch Sensors**: Hand/head touch detection
- **Get Sonar Range**: Ultrasonic distance sensors
- **Get Robot Info**: General robot status

## Troubleshooting

### NAO Connection Issues
```bash
# Check NAO network connectivity
ping nao.local

# Check NAOqi service
telnet nao.local 9559

# Restart NAOqi on NAO
# (Use robot's web interface or Choregraphe)
```

### ROS2 Issues
```bash
# Check if naoqi_driver2 is running
ros2 node list | grep naoqi

# Check topic publication rates
ros2 topic hz /joint_states

# Restart the launch file
ros2 launch nao_launch nao6_hristudio.launch.py nao_ip:=YOUR_NAO_IP
```

### HRIStudio Connection Issues
```bash
# Verify rosbridge is running
netstat -an | grep 9090

# Check WebSocket connection
curl -i -N -H "Connection: Upgrade" \
     -H "Upgrade: websocket" \
     -H "Sec-WebSocket-Key: test" \
     -H "Sec-WebSocket-Version: 13" \
     http://localhost:9090
```

### Robot Safety
- Always keep emergency stop accessible
- Start with small movements and low speeds
- Monitor robot battery level
- Ensure clear space around robot
- Never leave robot unattended during operation

## Performance Optimization

### Network Optimization
```bash
# Increase network buffer sizes for camera data
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
```

### ROS2 Optimization
```bash
# Adjust QoS settings for better performance
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI=file:///path/to/cyclonedx.xml
```

## Next Steps

1. **Experiment Design**: Create experiments using NAO6 actions
2. **Data Collection**: Use sensor actions for research data
3. **Custom Actions**: Extend the plugin with custom behaviors
4. **Multi-Robot**: Scale to multiple NAO6 robots
5. **Advanced Features**: Implement navigation, manipulation, etc.

## Support Resources

- **NAO Documentation**: https://developer.softbankrobotics.com/nao6
- **naoqi_driver2**: https://github.com/ros-naoqi/naoqi_driver2
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **HRIStudio Docs**: See `docs/` folder
- **Community**: HRIStudio Discord/Forum

---

**Success!** Your NAO6 is now ready for use with HRIStudio experiments. The robot's capabilities are fully accessible through the visual experiment designer and real-time wizard interface.