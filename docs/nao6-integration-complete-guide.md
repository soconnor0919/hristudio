# NAO6 HRIStudio Integration: Complete Setup and Troubleshooting Guide

This comprehensive guide documents the complete process of integrating a NAO6 robot with HRIStudio, including all troubleshooting steps and solutions discovered during implementation.

## Overview

NAO6 integration with HRIStudio provides full robot control through a web-based interface, enabling researchers to conduct Human-Robot Interaction experiments with real-time robot control, sensor monitoring, and data collection.

**Integration Architecture:**
```
HRIStudio Web Interface → WebSocket → ROS Bridge → NAOqi Driver → NAO6 Robot
```

## Prerequisites

### Hardware Requirements
- NAO6 robot (NAOqi OS 2.8.7+)
- Ubuntu 22.04 LTS computer
- Network connectivity between computer and NAO6
- Administrative access to both systems

### Software Requirements
- ROS2 Humble
- NAOqi Driver2 for ROS2
- rosbridge-suite
- HRIStudio platform
- SSH access to NAO robot

## Part 1: ROS2 and NAO Driver Setup

### 1.1 Install ROS2 Humble

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

### 1.2 Install Required ROS2 Packages

```bash
# Install rosbridge for HRIStudio communication
sudo apt install ros-humble-rosbridge-suite

# Install additional useful packages
sudo apt install ros-humble-rqt
sudo apt install ros-humble-rqt-common-plugins
```

### 1.3 Set Up NAO Workspace

**Note:** We assume you already have a NAO workspace at `~/naoqi_ros2_ws` with the NAOqi driver installed.

```bash
# Verify workspace exists
ls ~/naoqi_ros2_ws/src/naoqi_driver2
```

If you need to set up the workspace from scratch, refer to the NAOqi ROS2 documentation.

### 1.4 Create Integrated Launch Package

Create a launch package that combines NAOqi driver with rosbridge:

```bash
cd ~/naoqi_ros2_ws
mkdir -p src/nao_launch/launch
```

**Create launch file** (`src/nao_launch/launch/nao6_hristudio.launch.py`):

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # NAO IP configuration
        DeclareLaunchArgument("nao_ip", default_value="nao.local"),
        DeclareLaunchArgument("nao_port", default_value="9559"),
        DeclareLaunchArgument("username", default_value="nao"),
        DeclareLaunchArgument("password", default_value="nao"),
        DeclareLaunchArgument("network_interface", default_value="eth0"),
        DeclareLaunchArgument("qi_listen_url", default_value="tcp://0.0.0.0:0"),
        DeclareLaunchArgument("namespace", default_value="naoqi_driver"),
        DeclareLaunchArgument("bridge_port", default_value="9090"),
        
        # NAOqi Driver
        Node(
            package="naoqi_driver",
            executable="naoqi_driver_node",
            name="naoqi_driver",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                "nao_ip": LaunchConfiguration("nao_ip"),
                "nao_port": LaunchConfiguration("nao_port"),
                "username": LaunchConfiguration("username"),
                "password": LaunchConfiguration("password"),
                "network_interface": LaunchConfiguration("network_interface"),
                "qi_listen_url": LaunchConfiguration("qi_listen_url"),
                "publish_joint_states": True,
                "publish_odometry": True,
                "publish_camera": True,
                "publish_sensors": True,
                "joint_states_frequency": 30.0,
                "odom_frequency": 30.0,
                "camera_frequency": 15.0,
                "sensor_frequency": 10.0,
            }],
            output="screen",
        ),
        
        # Rosbridge WebSocket Server for HRIStudio
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            parameters=[{
                "port": LaunchConfiguration("bridge_port"),
                "address": "0.0.0.0",
                "authenticate": False,
                "fragment_timeout": 600,
                "delay_between_messages": 0,
                "max_message_size": 10000000,
            }],
            output="screen",
        ),
        
        # ROS API Server (required for rosbridge functionality)
        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
        ),
    ])
```

**Create package.xml**:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypeid="pf3"?>
<package format="3">
  <name>nao_launch</name>
  <version>1.0.0</version>
  <description>Launch files for NAO6 HRIStudio integration</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>naoqi_driver</exec_depend>
  <exec_depend>rosbridge_server</exec_depend>
  <exec_depend>rosapi</exec_depend>
</package>
```

**Create CMakeLists.txt**:

```cmake
cmake_minimum_required(VERSION 3.8)
project(nao_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### 1.5 Build the Workspace

```bash
cd ~/naoqi_ros2_ws
I_AGREE_TO_NAO_MESHES_LICENSE=1 I_AGREE_TO_PEPPER_MESHES_LICENSE=1 colcon build --symlink-install
source install/setup.bash
```

## Part 2: NAO Network Configuration and Connection

### 2.1 Verify NAO Network Connectivity

```bash
# Test basic connectivity
ping -c 4 nao.local

# Test NAOqi service port
timeout 5 bash -c 'echo "test" | nc nao.local 9559' && echo "NAOqi port is open!" || echo "NAOqi port might be closed"

# Alternative test
telnet nao.local 9559
# Press Ctrl+C to exit if connection succeeds
```

### 2.2 Find NAO Credentials

The default NAO credentials are typically:
- Username: `nao`  
- Password: Usually `nao`, but can be custom

**Common passwords to try:**
- `nao` (default)
- Institution name (e.g., `bucknell`)
- Custom password set by administrator

## Part 3: HRIStudio Database Integration

### 3.1 Update Database Schema

The HRIStudio database needs to include NAO6 robot definitions and plugins.

**Update robots in seed script** (`scripts/seed-dev.ts`):

```typescript
const robots = [
  {
    name: "TurtleBot3 Burger",
    manufacturer: "ROBOTIS",
    model: "TurtleBot3 Burger",
    description: "A compact, affordable, programmable, ROS2-based mobile robot for education and research",
    capabilities: ["differential_drive", "lidar", "imu", "odometry"],
    communicationProtocol: "ros2" as const,
  },
  {
    name: "NAO Humanoid Robot",
    manufacturer: "SoftBank Robotics",
    model: "NAO V6",
    description: "Humanoid robot designed for education, research, and social interaction with ROS2 integration",
    capabilities: [
      "speech",
      "vision", 
      "walking",
      "gestures",
      "joint_control",
      "touch_sensors",
      "sonar_sensors",
      "camera_feed",
      "imu",
      "odometry",
    ],
    communicationProtocol: "ros2" as const,
  },
];
```

### 3.2 Create NAO6 Plugin Repository

Create local plugin repository at `public/nao6-plugins/`:

**Repository metadata** (`public/nao6-plugins/repository.json`):

```json
{
  "name": "NAO6 ROS2 Integration Repository",
  "description": "Official NAO6 robot plugins for ROS2-based Human-Robot Interaction experiments",
  "version": "1.0.0",
  "author": {
    "name": "HRIStudio Team",
    "email": "support@hristudio.com"
  },
  "trust": "official",
  "license": "MIT",
  "robots": [
    {
      "name": "NAO6",
      "manufacturer": "SoftBank Robotics", 
      "model": "NAO V6",
      "communicationProtocol": "ros2"
    }
  ],
  "ros2": {
    "distro": "humble",
    "packages": ["naoqi_driver2", "naoqi_bridge_msgs", "rosbridge_suite"],
    "bridge": {
      "protocol": "websocket",
      "defaultPort": 9090
    }
  }
}
```

### 3.3 Seed Database

```bash
# Start database
sudo docker compose up -d

# Push schema changes  
bun db:push

# Seed with NAO6 data
bun db:seed
```

## Part 4: Web Interface Integration

### 4.1 Create NAO Test Page

Create `src/app/(dashboard)/nao-test/page.tsx` with the robot control interface.

**Key points:**
- Use `~` import alias (not `@`)
- Connect to WebSocket at `ws://YOUR_IP:9090`
- Use correct ROS topic names (without `/naoqi_driver` prefix for control topics)

**Important Topic Mapping:**
- Speech: `/speech` (not `/naoqi_driver/speech`)
- Movement: `/cmd_vel` (not `/naoqi_driver/cmd_vel`)  
- Joint control: `/joint_angles` (not `/naoqi_driver/joint_angles`)
- Sensor data: `/naoqi_driver/joint_states`, `/naoqi_driver/bumper`, etc.

## Part 5: Critical Troubleshooting

### 5.1 Robot Not Responding to Commands

**Symptom:** ROS topics receive commands but robot doesn't move.

**Root Cause:** NAO robots start in "safe mode" with loose joints and need to be "awakened."

**Solution - SSH Wake-Up Method:**

```bash
# Install sshpass for automated SSH
sudo apt install sshpass -y

# Wake up robot via SSH
sshpass -p "YOUR_NAO_PASSWORD" ssh nao@nao.local "python2 -c \"
import sys
sys.path.append('/opt/aldebaran/lib/python2.7/site-packages')
import naoqi

try:
    motion = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559)
    print 'Connected to ALMotion'
    print 'Current stiffness:', motion.getStiffnesses('Body')[0] if motion.getStiffnesses('Body') else 'No stiffness data'
    
    print 'Waking up robot...'
    motion.wakeUp()
    
    print 'Robot should now be awake!'
    
except Exception as e:
    print 'Error:', str(e)
\""
```

**Alternative Physical Method:**
1. Press and hold the chest button for 3 seconds
2. Wait for the robot to stiffen and stand up
3. Robot should now respond to movement commands

### 5.2 Connection Issues

**Port Already in Use:**
```bash
# Kill existing processes
sudo fuser -k 9090/tcp
pkill -f "rosbridge\|naoqi\|ros2"
```

**Database Connection Issues:**
```bash
# Check Docker containers
sudo docker ps

# Restart database
sudo docker compose down
sudo docker compose up -d
```

### 5.3 Import Alias Issues

**Error:** Module import failures in React components.

**Solution:** Use `~` import alias consistently:
```typescript
import { Button } from "~/components/ui/button";
// NOT: import { Button } from "@/components/ui/button";
```

## Part 6: Verification and Testing

### 6.1 System Verification Script

Create verification script to test all components:

```bash
#!/bin/bash
echo "=== NAO6 HRIStudio Integration Verification ==="

# Test 1: ROS2 Setup
echo "✓ ROS2 Humble: $ROS_DISTRO"

# Test 2: NAO Connectivity  
ping -c 1 nao.local && echo "✓ NAO reachable" || echo "✗ NAO not reachable"

# Test 3: Workspace Build
[ -f ~/naoqi_ros2_ws/install/setup.bash ] && echo "✓ Workspace built" || echo "✗ Workspace not built"

# Test 4: Database Running
sudo docker ps | grep -q postgres && echo "✓ Database running" || echo "✗ Database not running"

echo "=== Verification Complete ==="
```

### 6.2 End-to-End Test Procedure

**Terminal 1: Start ROS Integration**
```bash
cd ~/naoqi_ros2_ws
source install/setup.bash
ros2 launch install/nao_launch/share/nao_launch/launch/nao6_hristudio.launch.py nao_ip:=nao.local password:=YOUR_PASSWORD
```

**Terminal 2: Wake Up Robot**
```bash
# Use SSH method from Section 5.1
sshpass -p "YOUR_PASSWORD" ssh nao@nao.local "python2 -c \"...\""
```

**Terminal 3: Start HRIStudio**
```bash
cd /path/to/hristudio
bun dev
```

**Web Interface Test:**
1. Go to `http://localhost:3000/nao-test`
2. Click "Connect" - should show "Connected"
3. Test speech: Enter text and click "Say Text"
4. Test movement: Use arrow buttons to make robot walk
5. Test head control: Move sliders to control head position
6. Monitor sensor data in tabs

### 6.3 Command-Line Testing

**Test Speech:**
```bash
ros2 topic pub --once /speech std_msgs/String "data: 'Hello from ROS2'"
```

**Test Movement:**
```bash
ros2 topic pub --times 3 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

**Test Head Movement:**
```bash
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed '{joint_names: ["HeadYaw"], joint_angles: [0.5], speed: 0.3}'
```

## Part 7: Production Deployment

### 7.1 Launch Script Creation

Create production-ready launch script (`scripts/launch_nao6.sh`):

```bash
#!/bin/bash
# NAO6 HRIStudio Integration Launch Script

set -e

# Configuration
NAO_IP="${NAO_IP:-nao.local}"
NAO_PASSWORD="${NAO_PASSWORD:-nao}"
BRIDGE_PORT="${BRIDGE_PORT:-9090}"

# Function to wake up robot
wake_up_robot() {
    echo "Waking up NAO robot..."
    sshpass -p "$NAO_PASSWORD" ssh nao@$NAO_IP "python2 -c \"
import sys
sys.path.append('/opt/aldebaran/lib/python2.7/site-packages')
import naoqi
motion = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559)
motion.wakeUp()
print 'Robot awakened'
\""
}

# Main execution
echo "Starting NAO6 HRIStudio Integration"
echo "NAO IP: $NAO_IP"
echo "Bridge Port: $BRIDGE_PORT"

# Check connections
ping -c 1 $NAO_IP || { echo "Cannot reach NAO"; exit 1; }

# Start ROS integration
cd ~/naoqi_ros2_ws
source install/setup.bash

# Wake up robot in background
wake_up_robot &

# Launch ROS system
exec ros2 launch install/nao_launch/share/nao_launch/launch/nao6_hristudio.launch.py \
    nao_ip:="$NAO_IP" \
    password:="$NAO_PASSWORD" \
    bridge_port:="$BRIDGE_PORT"
```

### 7.2 Service Integration (Optional)

Create systemd service for automatic startup:

```ini
[Unit]
Description=NAO6 HRIStudio Integration
After=network.target

[Service]
Type=simple
User=your_user
Environment=NAO_IP=nao.local
Environment=NAO_PASSWORD=your_password
ExecStart=/path/to/launch_nao6.sh
Restart=always

[Install]
WantedBy=multi-user.target
```

## Part 8: Safety and Best Practices

### 8.1 Safety Guidelines

- **Always keep emergency stop accessible** in the web interface
- **Start with small movements and low speeds** when testing
- **Monitor robot battery level** during long sessions
- **Ensure clear space around robot** before movement commands
- **Never leave robot unattended** during operation

### 8.2 Performance Optimization

**Network Optimization:**
```bash
# Increase network buffer sizes for camera data
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
```

**ROS2 Optimization:**
```bash
# Use optimized RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

### 8.3 Troubleshooting Checklist

**Before Starting:**
- [ ] NAO robot powered on and connected to network
- [ ] ROS2 Humble installed and sourced
- [ ] NAO workspace built successfully
- [ ] Database running (Docker container)
- [ ] Correct NAO password known

**During Operation:**
- [ ] rosbridge WebSocket server running on port 9090
- [ ] NAO robot in standing position (not crouching)
- [ ] Robot joints stiffened (not loose)
- [ ] HRIStudio web interface connected to ROS bridge

**If Commands Not Working:**
1. Check robot is awake and standing
2. Verify topic names in web interface match ROS topics
3. Test commands from command line first
4. Check rosbridge logs for errors

## Part 9: Future Enhancements

### 9.1 Advanced Features

- **Multi-camera streaming** for experiment recording
- **Advanced gesture recognition** through touch sensors
- **Autonomous behavior integration** with navigation
- **Multi-robot coordination** for group interaction studies

### 9.2 Plugin Development

The NAO6 integration supports the HRIStudio plugin system for adding custom behaviors and extending robot capabilities.

## Conclusion

This guide provides a complete integration of NAO6 robots with HRIStudio, enabling researchers to conduct sophisticated Human-Robot Interaction experiments with full robot control, real-time data collection, and web-based interfaces.

The key insight discovered during implementation is that NAO robots require explicit "wake-up" commands to enable motor control, which must be performed before any movement commands will be executed.

**Support Resources:**
- NAO Documentation: https://developer.softbankrobotics.com/nao6
- naoqi_driver2: https://github.com/ros-naoqi/naoqi_driver2  
- ROS2 Humble: https://docs.ros.org/en/humble/
- HRIStudio Documentation: See `docs/` folder

---

**Integration Status: Production Ready ✅**

*Last Updated: January 2025*
*Tested With: NAO V6.0 / NAOqi 2.8.7.4 / ROS2 Humble / HRIStudio v1.0*