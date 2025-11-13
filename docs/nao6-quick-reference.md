# NAO6 Quick Reference

Essential commands for using NAO6 robots with HRIStudio.

## Quick Start

### 1. Start NAO Integration
```bash
cd ~/naoqi_ros2_ws
source install/setup.bash
ros2 launch nao_launch nao6_hristudio.launch.py nao_ip:=nao.local password:=robolab
```

### 2. Wake Robot
Press chest button for 3 seconds, or use:
```bash
# Via SSH (institution-specific password)
ssh nao@nao.local
# Then run wake-up command (see integration repo docs)
```

### 3. Start HRIStudio
```bash
cd ~/Documents/Projects/hristudio
bun dev
```

### 4. Test Connection
- Open: `http://localhost:3000/nao-test`
- Click "Connect" 
- Test robot commands

## Essential Commands

### Test Connectivity
```bash
ping nao.local                    # Test network
ros2 topic list | grep naoqi      # Check ROS topics
```

### Manual Control
```bash
# Speech
ros2 topic pub --once /speech std_msgs/String "data: 'Hello world'"

# Movement (robot must be awake!)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}}'
```

### Monitor Status
```bash
ros2 topic echo /naoqi_driver/battery        # Battery level
ros2 topic echo /naoqi_driver/joint_states   # Joint positions
```

## Troubleshooting

**Robot not moving:** Press chest button for 3 seconds to wake up

**WebSocket fails:** Check rosbridge is running on port 9090
```bash
ss -an | grep 9090
```

**Connection lost:** Restart rosbridge
```bash
pkill -f rosbridge
ros2 run rosbridge_server rosbridge_websocket
```

## ROS Topics

**Commands (Input):**
- `/speech` - Text-to-speech
- `/cmd_vel` - Movement
- `/joint_angles` - Joint control

**Sensors (Output):**
- `/naoqi_driver/joint_states` - Joint data
- `/naoqi_driver/battery` - Battery level
- `/naoqi_driver/bumper` - Foot sensors
- `/naoqi_driver/sonar/*` - Distance sensors
- `/naoqi_driver/camera/*` - Camera feeds

## WebSocket

**URL:** `ws://localhost:9090`

**Example message:**
```javascript
{
  "op": "publish",
  "topic": "/speech",
  "type": "std_msgs/String",
  "msg": {"data": "Hello world"}
}
```

## More Information

See **[nao6-hristudio-integration](../../nao6-hristudio-integration/)** repository for:
- Complete installation guide
- Detailed usage instructions
- Full troubleshooting guide
- Plugin definitions
- Launch file configurations

## Common Use Cases

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