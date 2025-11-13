# NAO6 HRIStudio Integration

**Complete integration package for NAO6 humanoid robots with the HRIStudio research platform**

## 🎯 Overview

This repository contains all components needed to integrate NAO6 robots with HRIStudio for Human-Robot Interaction research. It provides production-ready ROS2 packages, web interface plugins, control scripts, and comprehensive documentation for seamless robot operation through the HRIStudio platform.

## 📦 Repository Structure

```
nao6-hristudio-integration/
├── README.md                     # This file
├── launch/                       # ROS2 launch configurations
│   ├── nao6_production.launch.py      # Production-optimized launch
│   └── nao6_hristudio_enhanced.launch.py # Enhanced with monitoring
├── scripts/                      # Utilities and automation
│   ├── test_nao_topics.py            # ROS topics simulator
│   ├── test_websocket.py             # WebSocket bridge tester
│   ├── verify_nao6_bridge.sh         # Integration verification
│   └── seed-nao6-plugin.ts           # Database seeding for HRIStudio
├── plugins/                      # HRIStudio plugin definitions
│   ├── repository.json               # Plugin repository metadata
│   ├── nao6-ros2-enhanced.json      # Complete NAO6 plugin
│   └── README.md                     # Plugin documentation
├── examples/                     # Usage examples and tools
│   ├── nao_control.py                # Command-line robot control
│   └── start_nao6_hristudio.sh       # Automated startup script
└── docs/                         # Documentation
    ├── NAO6_INTEGRATION_COMPLETE.md  # Complete integration guide
    ├── INSTALLATION.md               # Installation instructions
    ├── USAGE.md                      # Usage examples
    └── TROUBLESHOOTING.md            # Common issues and solutions
```

## 🚀 Quick Start

### Prerequisites
- **NAO6 Robot** with NAOqi 2.8.7.4+
- **Ubuntu 22.04** with ROS2 Humble
- **HRIStudio Platform** (web interface)
- **Network connectivity** between computer and robot

### 1. Clone Repository
```bash
git clone <repository-url> ~/nao6-hristudio-integration
cd ~/nao6-hristudio-integration
```

### 2. Install Dependencies
```bash
# Install ROS2 packages
sudo apt update
sudo apt install ros-humble-rosbridge-suite ros-humble-naoqi-driver

# Install Python dependencies
pip install websocket-client
```

### 3. Setup NAOqi ROS2 Workspace
```bash
# Build the enhanced nao_launch package
cd ~/naoqi_ros2_ws
colcon build --packages-select nao_launch
source install/setup.bash
```

### 4. Start Integration
```bash
# Option A: Use automated startup script
./examples/start_nao6_hristudio.sh --nao-ip nao.local --password robolab

# Option B: Manual launch
ros2 launch nao_launch nao6_production.launch.py nao_ip:=nao.local password:=robolab
```

### 5. Configure HRIStudio
```bash
# Seed NAO6 plugin into HRIStudio database
cd ~/Documents/Projects/hristudio
bun run ../nao6-hristudio-integration/scripts/seed-nao6-plugin.ts

# Start HRIStudio web interface
bun dev
```

### 6. Test Integration
- Open: `http://localhost:3000/nao-test`
- Login: `sean@soconnor.dev` / `password123`
- Click "Connect" to establish WebSocket connection
- Try robot commands and verify responses

## 🎮 Available Robot Actions

The NAO6 plugin provides 9 comprehensive actions for HRIStudio experiments:

### 🗣️ Communication
- **Speak Text** - Text-to-speech with volume/speed control
- **LED Control** - Visual feedback with colors and patterns

### 🚶 Movement & Posture
- **Move Robot** - Walking, turning with safety limits
- **Set Posture** - Stand, sit, crouch positions
- **Move Head** - Gaze control and attention direction
- **Perform Gesture** - Wave, point, applause, custom animations

### 📡 Sensors & Monitoring
- **Monitor Sensors** - Touch, bumper, sonar detection
- **Check Robot Status** - Battery, joints, system health

### 🛡️ Safety & System
- **Emergency Stop** - Immediate motion termination
- **Wake Up / Rest** - Power management

## 🔧 Command-Line Tools

### Robot Control
```bash
# Direct robot control
python3 examples/nao_control.py --ip nao.local wake
python3 examples/nao_control.py --ip nao.local speak "Hello world"
python3 examples/nao_control.py --ip nao.local move 0.1 0 0
python3 examples/nao_control.py --ip nao.local pose Stand
python3 examples/nao_control.py --ip nao.local emergency
```

### Integration Testing
```bash
# Verify all components
./scripts/verify_nao6_bridge.sh

# Test WebSocket connectivity
python3 scripts/test_websocket.py

# Simulate robot topics (without hardware)
python3 scripts/test_nao_topics.py
```

## 🌐 WebSocket Communication

### Connection Details
- **URL**: `ws://localhost:9090`
- **Protocol**: rosbridge v2.0
- **Format**: JSON messages

### Sample Messages
```javascript
// Speech command
{
  "op": "publish",
  "topic": "/speech",
  "type": "std_msgs/String",
  "msg": {"data": "Hello from HRIStudio!"}
}

// Movement command
{
  "op": "publish",
  "topic": "/cmd_vel",
  "type": "geometry_msgs/Twist",
  "msg": {
    "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  }
}

// Subscribe to sensors
{
  "op": "subscribe",
  "topic": "/naoqi_driver/joint_states",
  "type": "sensor_msgs/JointState"
}
```

## 📋 Key Topics

### Input Topics (Robot Control)
- `/speech` - Text-to-speech commands
- `/cmd_vel` - Movement control
- `/joint_angles` - Joint positioning
- `/led_control` - Visual feedback

### Output Topics (Sensor Data)
- `/naoqi_driver/joint_states` - Joint positions/velocities
- `/naoqi_driver/bumper` - Foot sensors
- `/naoqi_driver/hand_touch` - Hand touch sensors
- `/naoqi_driver/head_touch` - Head touch sensors
- `/naoqi_driver/sonar/left` - Left ultrasonic sensor
- `/naoqi_driver/sonar/right` - Right ultrasonic sensor
- `/naoqi_driver/battery` - Battery level

## 🛡️ Safety Features

### Automated Safety
- **Velocity Limits** - Maximum speed constraints (0.2 m/s linear, 0.8 rad/s angular)
- **Emergency Stop** - Immediate motion termination
- **Battery Monitoring** - Low battery warnings
- **Fall Detection** - Automatic safety responses
- **Wake-up Management** - Proper robot state handling

### Manual Safety Controls
```bash
# Emergency stop via CLI
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Emergency stop via script
python3 examples/nao_control.py --ip nao.local emergency

# Or use HRIStudio emergency stop action
```

## 🔍 Troubleshooting

### Common Issues

**Robot not responding to commands**
```bash
# Check robot is awake
python3 examples/nao_control.py --ip nao.local status

# Wake up robot
python3 examples/nao_control.py --ip nao.local wake
# OR press chest button for 3 seconds
```

**WebSocket connection failed**
```bash
# Check rosbridge is running
ros2 node list | grep rosbridge

# Restart integration
pkill -f rosbridge && pkill -f rosapi
ros2 launch nao_launch nao6_production.launch.py nao_ip:=nao.local
```

**Network connectivity issues**
```bash
# Test basic connectivity
ping nao.local
telnet nao.local 9559

# Check robot credentials
ssh nao@nao.local  # Password: robolab (institution-specific)
```

For detailed troubleshooting, see [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

## 📖 Documentation

### Complete Guides
- **[Installation Guide](docs/INSTALLATION.md)** - Detailed setup instructions
- **[Usage Guide](docs/USAGE.md)** - Examples and best practices
- **[Integration Complete](docs/NAO6_INTEGRATION_COMPLETE.md)** - Comprehensive overview
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Problem resolution

### Quick References
- **Launch Files** - See `launch/` directory
- **Plugin Definitions** - See `plugins/` directory
- **Example Scripts** - See `examples/` directory

## 🎯 Research Applications

### Experiment Types
- **Social Interaction** - Gestures, speech, gaze studies
- **Human-Robot Collaboration** - Shared task experiments
- **Behavior Analysis** - Touch, proximity, response studies
- **Navigation Studies** - Movement and spatial interaction
- **Multimodal Interaction** - Combined speech, gesture, movement

### Data Capture
- **Synchronized Timestamps** - All robot actions and sensor events
- **Sensor Fusion** - Touch, vision, audio, movement data
- **Real-time Logging** - Comprehensive event capture
- **Export Capabilities** - Data analysis and visualization

## 🏆 Features & Benefits

### ✅ Production Ready
- **Tested Integration** - Verified with NAO V6.0 / NAOqi 2.8.7.4
- **Safety First** - Comprehensive safety monitoring
- **Performance Optimized** - Tuned for stable experiments
- **Error Handling** - Robust failure management

### ✅ Researcher Friendly
- **Web Interface** - No programming required for experiments
- **Visual Designer** - Drag-and-drop experiment creation
- **Real-time Control** - Live robot operation during trials
- **Multiple Roles** - Researcher, wizard, observer access

### ✅ Developer Friendly
- **Open Source** - MIT licensed components
- **Modular Design** - Extensible architecture
- **Comprehensive APIs** - ROS2 and WebSocket interfaces
- **Documentation** - Complete setup and usage guides

## 🚀 Getting Started Examples

### Basic Experiment Workflow
1. **Design** - Create experiment in HRIStudio visual designer
2. **Configure** - Set robot parameters and safety limits  
3. **Execute** - Run trial with real-time robot control
4. **Analyze** - Review captured data and events
5. **Iterate** - Refine experiment based on results

### Sample Experiment: Greeting Interaction
```javascript
// HRIStudio experiment sequence
[
  {"action": "nao_wake_rest", "parameters": {"action": "wake"}},
  {"action": "nao_pose", "parameters": {"posture": "Stand"}},
  {"action": "nao_speak", "parameters": {"text": "Hello! Welcome to our study."}},
  {"action": "nao_gesture", "parameters": {"gesture": "wave"}},
  {"action": "nao_sensor_monitor", "parameters": {"sensorType": "touch", "duration": 30}}
]
```

## 🤝 Contributing

### Development Setup
1. Fork this repository
2. Create feature branch: `git checkout -b feature-name`
3. Test with real NAO6 hardware
4. Submit pull request with documentation updates

### Guidelines
- Follow ROS2 conventions for launch files
- Test all changes with physical robot
- Update documentation for new features
- Ensure backward compatibility

## 📞 Support

### Resources
- **GitHub Issues** - Report bugs and request features
- **Documentation** - Complete guides in `docs/` folder
- **HRIStudio Platform** - Web interface documentation

### Requirements
- **NAO6 Robot** - NAO V6.0 with NAOqi 2.8.7.4+
- **ROS2 Humble** - Ubuntu 22.04 recommended
- **Network Setup** - Robot and computer on same network
- **HRIStudio** - Web platform for experiment design

## 📄 License

MIT License - See LICENSE file for details

## 🏅 Citation

If you use this integration in your research, please cite:

```bibtex
@software{nao6_hristudio_integration,
  title={NAO6 HRIStudio Integration},
  author={HRIStudio RoboLab Team},
  year={2024},
  url={https://github.com/hristudio/nao6-integration},
  version={2.0.0}
}
```

---

**Status**: Production Ready ✅  
**Tested With**: NAO V6.0 / NAOqi 2.8.7.4 / ROS2 Humble / HRIStudio v1.0  
**Last Updated**: December 2024  

*Advancing Human-Robot Interaction research through standardized, accessible, and reliable tools.*