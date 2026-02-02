# NAO6 HRIStudio Plugin Repository

**Official NAO6 robot integration plugins for the HRIStudio platform**

## Overview

This repository contains production-ready plugins for integrating NAO6 robots with HRIStudio experiments. The plugins provide comprehensive robot control capabilities including movement, speech synthesis, sensor monitoring, and safety features optimized for human-robot interaction research.

## Available Plugins

### 🤖 NAO6 Enhanced ROS2 Integration (`nao6-ros2-enhanced.json`)

**Complete NAO6 robot control for HRIStudio experiments**

**Features:**
- ✅ **Speech Synthesis** - Text-to-speech with volume and speed control
- ✅ **Movement Control** - Walking, turning, and precise positioning
- ✅ **Posture Management** - Stand, sit, crouch, and custom poses
- ✅ **Head Movement** - Gaze control and attention direction
- ✅ **Gesture Library** - Wave, point, applause, and custom animations
- ✅ **LED Control** - Visual feedback with colors and patterns
- ✅ **Sensor Monitoring** - Touch, bumper, sonar, and camera sensors
- ✅ **Safety Features** - Emergency stop and velocity limits
- ✅ **System Control** - Wake/rest and status monitoring

**Requirements:**
- NAO6 robot with NAOqi 2.8.7.4+
- ROS2 Humble or compatible
- Network connectivity to robot
- `nao_launch` package for ROS integration

**Installation:**
1. Install in HRIStudio study via Plugin Management
2. Configure robot IP and WebSocket URL
3. Launch ROS integration: `ros2 launch nao_launch nao6_production.launch.py`
4. Test connection in HRIStudio experiment designer

## Plugin Actions Reference

### Speech & Communication
| Action | Description | Parameters |
|--------|-------------|------------|
| **Speak Text** | Text-to-speech synthesis | text, volume, speed, wait |
| **LED Control** | Visual feedback with colors | ledGroup, color, intensity, pattern |

### Movement & Posture
| Action | Description | Parameters |
|--------|-------------|------------|
| **Move Robot** | Linear and angular movement | direction, distance, speed, duration |
| **Set Posture** | Predefined poses | posture, speed, waitForCompletion |
| **Move Head** | Gaze and attention control | yaw, pitch, speed, presetDirection |
| **Perform Gesture** | Animations and gestures | gesture, intensity, speed, repeatCount |

### Sensors & Monitoring
| Action | Description | Parameters |
|--------|-------------|------------|
| **Monitor Sensors** | Touch, bumper, sonar detection | sensorType, duration, sensitivity |
| **Check Robot Status** | Battery, joints, system health | statusType, logToExperiment |

### Safety & System
| Action | Description | Parameters |
|--------|-------------|------------|
| **Emergency Stop** | Immediate motion termination | stopType, safePosture |
| **Wake Up / Rest** | Power management | action, waitForCompletion |

## Quick Start Examples

### 1. Basic Greeting
```json
{
  "sequence": [
    {"action": "nao_wake_rest", "parameters": {"action": "wake"}},
    {"action": "nao_speak", "parameters": {"text": "Hello! Welcome to our experiment."}},
    {"action": "nao_gesture", "parameters": {"gesture": "wave"}}
  ]
}
```

### 2. Interactive Task
```json
{
  "sequence": [
    {"action": "nao_speak", "parameters": {"text": "Please touch my head when ready."}},
    {"action": "nao_sensor_monitor", "parameters": {"sensorType": "touch", "duration": 30}},
    {"action": "nao_speak", "parameters": {"text": "Thank you! Let's begin."}}
  ]
}
```

### 3. Attention Direction
```json
{
  "sequence": [
    {"action": "nao_head_movement", "parameters": {"presetDirection": "left"}},
    {"action": "nao_speak", "parameters": {"text": "Look over there please."}},
    {"action": "nao_gesture", "parameters": {"gesture": "point_left"}}
  ]
}
```

## Installation & Setup

### Prerequisites
- **HRIStudio Platform** - Web-based WoZ research platform
- **NAO6 Robot** - With NAOqi 2.8.7.4 or compatible
- **ROS2 Humble** - Robot Operating System 2
- **Network Setup** - Robot and computer on same network

### Step 1: Install NAO ROS2 Packages
```bash
# Clone and build NAO ROS2 workspace
cd ~/naoqi_ros2_ws
colcon build --packages-select nao_launch
source install/setup.bash
```

### Step 2: Start Robot Integration
```bash
# Launch comprehensive NAO integration
ros2 launch nao_launch nao6_production.launch.py \
  nao_ip:=nao.local \
  password:=robolab \
  bridge_port:=9090
```

### Step 3: Install Plugin in HRIStudio
1. **Access HRIStudio** - Open your study in HRIStudio
2. **Plugin Management** - Go to Study → Plugins
3. **Browse Store** - Find "NAO6 Robot (Enhanced ROS2 Integration)"
4. **Install Plugin** - Click install and configure settings
5. **Configure WebSocket** - Set URL to `ws://localhost:9090`

### Step 4: Test Integration
1. **Open Experiment Designer** - Create or edit an experiment
2. **Add Robot Action** - Drag NAO6 action from plugin section
3. **Configure Parameters** - Set speech text, movement, etc.
4. **Test Connection** - Use "Check Robot Status" action
5. **Run Trial** - Execute experiment and verify robot responds

## Configuration Options

### Robot Connection
- **Robot IP** - IP address or hostname (default: `nao.local`)
- **Password** - Robot authentication password
- **WebSocket URL** - ROS bridge connection (default: `ws://localhost:9090`)

### Safety Settings
- **Max Linear Velocity** - Maximum movement speed (default: 0.2 m/s)
- **Max Angular Velocity** - Maximum rotation speed (default: 0.8 rad/s)
- **Safety Monitoring** - Enable automatic safety checks
- **Auto Wake-up** - Automatically wake robot when experiment starts

### Performance Tuning
- **Speech Volume** - Default volume level (default: 0.7)
- **Movement Speed** - Default movement speed factor (default: 0.5)
- **Battery Monitoring** - Track battery level during experiments

## Troubleshooting

### ❌ Robot Not Responding
**Problem:** Commands sent but robot doesn't react  
**Solution:** 
- Check robot is awake: Press chest button for 3 seconds
- Verify network connectivity: `ping nao.local`
- Use "Wake Up / Rest Robot" action in experiment

### ❌ WebSocket Connection Failed
**Problem:** HRIStudio cannot connect to robot  
**Solution:**
- Verify rosbridge is running: `ros2 node list | grep rosbridge`
- Check port availability: `ss -an | grep 9090`
- Restart integration: Kill processes and relaunch

### ❌ Movements Too Fast/Unsafe
**Problem:** Robot moves too quickly or unpredictably  
**Solution:**
- Reduce max velocities in plugin configuration
- Lower movement speed parameters in actions
- Use "Emergency Stop" action if needed

### ❌ Speech Not Working
**Problem:** Robot doesn't speak or audio issues  
**Solution:**
- Check robot volume settings
- Verify text-to-speech service: `ros2 topic echo /speech`
- Ensure speakers are functioning

## Safety Guidelines

### ⚠️ Important Safety Notes
- **Clear Space** - Ensure 2m clearance around robot during movement
- **Emergency Stop** - Keep emergency stop action easily accessible
- **Supervision** - Never leave robot unattended during experiments
- **Battery Monitoring** - Check battery level for long sessions
- **Stable Surface** - Keep robot on level, stable flooring

### Emergency Procedures
```bash
# Immediate stop via CLI
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Or use HRIStudio emergency stop action
# Add "Emergency Stop" action to experiment for quick access
```

## Technical Details

### ROS2 Topics Used
- **Input Topics** (Robot Control):
  - `/speech` - Text-to-speech commands
  - `/cmd_vel` - Movement commands  
  - `/joint_angles` - Joint position control
  - `/led_control` - LED color control

- **Output Topics** (Sensor Data):
  - `/naoqi_driver/joint_states` - Joint positions
  - `/naoqi_driver/bumper` - Foot sensors
  - `/naoqi_driver/hand_touch` - Hand sensors
  - `/naoqi_driver/head_touch` - Head sensors
  - `/naoqi_driver/sonar/*` - Ultrasonic sensors

### WebSocket Communication
- **Protocol** - rosbridge v2.0 WebSocket
- **Default Port** - 9090
- **Message Format** - JSON-based ROS message serialization
- **Authentication** - None (local network)

## Development & Contributing

### Plugin Development
1. **Follow Schema** - Use provided JSON schema for action definitions
2. **Test Thoroughly** - Verify with real NAO6 hardware
3. **Document Actions** - Provide clear parameter descriptions
4. **Safety First** - Include appropriate safety measures

### Testing Checklist
- [ ] Robot connectivity and wake-up
- [ ] All movement actions with safety limits
- [ ] Speech synthesis with various texts
- [ ] Sensor monitoring and event detection
- [ ] Emergency stop functionality
- [ ] WebSocket communication stability

## Support & Resources

### Documentation
- **HRIStudio Docs** - [Platform documentation](../../docs/)
- **NAO6 Integration Guide** - [Complete setup guide](../../docs/nao6-integration-complete-guide.md)
- **Quick Reference** - [Essential commands](../../docs/nao6-quick-reference.md)

### Community & Support
- **GitHub Repository** - [hristudio/nao6-ros2-plugins](https://github.com/hristudio/nao6-ros2-plugins)
- **Issue Tracker** - Report bugs and request features
- **Email Support** - robolab@hristudio.com

### Version Information
- **Plugin Version** - 2.0.0 (Enhanced Integration)
- **HRIStudio Compatibility** - v1.0+
- **ROS2 Distro** - Humble (recommended)
- **NAO6 Compatibility** - NAOqi 2.8.7.4+
- **Last Updated** - December 2024

---

## License

**MIT License** - See [LICENSE](LICENSE) file for details

## Citation

If you use these plugins in your research, please cite:

```bibtex
@software{nao6_hristudio_plugins,
  title={NAO6 HRIStudio Integration Plugins},
  author={HRIStudio RoboLab Team},
  year={2024},
  url={https://github.com/hristudio/nao6-ros2-plugins},
  version={2.0.0}
}
```

---

**Maintained by:** HRIStudio RoboLab Team  
**Contact:** robolab@hristudio.com  
**Repository:** [hristudio/nao6-ros2-plugins](https://github.com/hristudio/nao6-ros2-plugins)

*Part of the HRIStudio platform for advancing Human-Robot Interaction research*