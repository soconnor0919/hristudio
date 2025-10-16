# NAO6 HRIStudio Quick Reference

**Essential commands for NAO6 robot integration with HRIStudio**

## 🚀 Quick Start (5 Steps)

### 1. Start ROS Integration
```bash
cd ~/naoqi_ros2_ws
source install/setup.bash
ros2 launch install/nao_launch/share/nao_launch/launch/nao6_hristudio.launch.py nao_ip:=nao.local password:=robolab
```

### 2. Wake Up Robot (CRITICAL!)
```bash
sshpass -p "robolab" ssh nao@nao.local "python2 -c \"
import sys
sys.path.append('/opt/aldebaran/lib/python2.7/site-packages')
import naoqi
motion = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559)
motion.wakeUp()
print 'Robot awakened'
\""
```

### 3. Start HRIStudio
```bash
cd /home/robolab/Documents/Projects/hristudio
bun dev
```

### 4. Access Test Interface
- URL: `http://localhost:3000/nao-test`
- Login: `sean@soconnor.dev` / `password123`

### 5. Test Robot
- Click "Connect" to WebSocket
- Try speech: "Hello from HRIStudio!"
- Use movement buttons to control robot

## 🛠️ Essential Commands

### Connection Testing
```bash
# Test NAO connectivity
ping nao.local

# Test NAOqi service
telnet nao.local 9559

# Check ROS topics
ros2 topic list | grep naoqi
```

### Manual Robot Control
```bash
# Speech test
ros2 topic pub --once /speech std_msgs/String "data: 'Hello world'"

# Movement test (robot must be awake!)
ros2 topic pub --times 3 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Head movement test
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed '{joint_names: ["HeadYaw"], joint_angles: [0.5], speed: 0.3}'

# Stop all movement
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Status Checks
```bash
# Check robot info
ros2 service call /naoqi_driver/get_robot_config naoqi_bridge_msgs/srv/GetRobotInfo

# Monitor joint states
ros2 topic echo /naoqi_driver/joint_states --once

# Check ROS nodes
ros2 node list

# Check WebSocket connection
ss -an | grep 9090
```

## 🔧 Troubleshooting

### Robot Not Moving
**Problem:** Commands sent but robot doesn't move
**Solution:** Robot needs to be awakened first
```bash
# Wake up via SSH (see step 2 above)
# OR press chest button for 3 seconds
```

### Connection Issues
```bash
# Kill existing processes
sudo fuser -k 9090/tcp
pkill -f "rosbridge\|naoqi\|ros2"

# Restart database
sudo docker compose down && sudo docker compose up -d
```

### Import Errors in Web Interface
**Problem:** React component import failures
**Solution:** Use `~` import alias consistently:
```typescript
import { Button } from "~/components/ui/button";
// NOT: import { Button } from "@/components/ui/button";
```

## 📊 Key Topics

### Input Topics (Robot Control)
- `/speech` - Text-to-speech
- `/cmd_vel` - Movement commands
- `/joint_angles` - Joint position control

### Output Topics (Sensor Data)
- `/naoqi_driver/joint_states` - Joint positions/velocities
- `/naoqi_driver/bumper` - Foot sensors
- `/naoqi_driver/hand_touch` - Hand touch sensors
- `/naoqi_driver/head_touch` - Head touch sensors
- `/naoqi_driver/sonar/left` - Left ultrasonic sensor
- `/naoqi_driver/sonar/right` - Right ultrasonic sensor
- `/naoqi_driver/camera/front/image_raw` - Front camera
- `/naoqi_driver/camera/bottom/image_raw` - Bottom camera

## 🔗 WebSocket Integration

**ROS Bridge URL:** `ws://134.82.159.25:9090`

**Message Format:**
```javascript
// Publish command
{
  "op": "publish",
  "topic": "/speech",
  "type": "std_msgs/String",
  "msg": {"data": "Hello world"}
}

// Subscribe to topic
{
  "op": "subscribe",
  "topic": "/naoqi_driver/joint_states",
  "type": "sensor_msgs/JointState"
}
```

## 🎯 Common Use Cases

### Make Robot Speak
```bash
ros2 topic pub --once /speech std_msgs/String "data: 'Welcome to the experiment'"
```

### Walk Forward 3 Steps
```bash
ros2 topic pub --times 3 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Turn Head Left
```bash
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed '{joint_names: ["HeadYaw"], joint_angles: [0.8], speed: 0.2}'
```

### Emergency Stop
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 🚨 Safety Notes

- **Always wake up robot before movement commands**
- **Keep emergency stop accessible**
- **Start with small movements (0.05 m/s)**
- **Monitor battery level during experiments**
- **Ensure clear space around robot**

## 📝 Credentials

**Default NAO Login:**
- Username: `nao`
- Password: `robolab` (institution-specific)

**HRIStudio Login:**
- Email: `sean@soconnor.dev`
- Password: `password123`

## 🔄 Complete Restart Procedure

```bash
# 1. Kill all processes
sudo fuser -k 9090/tcp
pkill -f "rosbridge\|naoqi\|ros2"

# 2. Restart database
sudo docker compose down && sudo docker compose up -d

# 3. Start ROS integration
cd ~/naoqi_ros2_ws && source install/setup.bash
ros2 launch install/nao_launch/share/nao_launch/launch/nao6_hristudio.launch.py nao_ip:=nao.local password:=robolab

# 4. Wake up robot (in another terminal)
sshpass -p "robolab" ssh nao@nao.local "python2 -c \"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; naoqi.ALProxy('ALMotion', '127.0.0.1', 9559).wakeUp()\""

# 5. Start HRIStudio (in another terminal)
cd /home/robolab/Documents/Projects/hristudio && bun dev
```

---

**📖 For detailed setup instructions, see:** [NAO6 Complete Integration Guide](./nao6-integration-complete-guide.md)

**✅ Integration Status:** Production Ready  
**🤖 Tested With:** NAO V6.0 / NAOqi 2.8.7.4 / ROS2 Humble