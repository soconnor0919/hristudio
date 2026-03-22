# NAO6 Quick Reference

Essential commands for using NAO6 robots with HRIStudio.

## Quick Start (Docker)

### 1. Start Docker Integration
```bash
cd ~/Documents/Projects/nao6-hristudio-integration
docker compose up -d
```

The robot will automatically wake up and autonomous life will be disabled on startup.

### 2. Start HRIStudio
```bash
cd ~/Documents/Projects/hristudio
bun dev
```

### 3. Verify Connection
- Open: `http://localhost:3000`
- Navigate to trial wizard
- WebSocket should connect automatically

## Docker Services

| Service | Port | Description |
|---------|------|-------------|
| nao_driver | - | NAOqi driver node |
| ros_bridge | 9090 | WebSocket bridge |
| ros_api | - | ROS API services |

## ROS Topics

**Commands (Publish to these):**
```
/speech          - Text-to-speech
/cmd_vel         - Velocity commands (movement)
/joint_angles    - Joint position commands
```

**Sensors (Subscribe to these):**
```
/camera/front/image_raw    - Front camera
/camera/bottom/image_raw  - Bottom camera
/joint_states             - Joint positions
/imu/torso               - IMU data
/bumper                   - Foot bumpers
/{hand,head}_touch        - Touch sensors
/sonar/{left,right}       - Ultrasonic sensors
/info                     - Robot info
```

## Manual Control

### Test Connectivity
```bash
# Network
ping 10.0.0.42

# ROS topics (inside Docker)
docker exec -it nao6-hristudio-integration-nao_driver-1 ros2 topic list
```

### Direct Commands (inside Docker)
```bash
# Speech
docker exec -it nao6-hristudio-integration-nao_driver-1 \
  ros2 topic pub --once /speech std_msgs/String "{data: 'Hello'}"

# Movement (robot must be awake!)
docker exec -it nao6-hristudio-integration-nao_driver-1 \
  ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}}"
```

### Robot Control via SSH
```bash
# SSH to robot
sshpass -p "nao" ssh nao@10.0.0.42

# Wake up
qicli call ALMotion.wakeUp

# Disable autonomous life
qicli call ALAutonomousLife.setState disabled

# Go to stand
qicli call ALRobotPosture.goToPosture Stand 0.5
```

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

## Troubleshooting

**Robot not moving:**
- Check robot is awake: `qicli call ALMotion.isWakeUp` → returns `true`
- If not: `qicli call ALMotion.wakeUp`

**WebSocket fails:**
```bash
# Check rosbridge is running
docker compose ps

# View logs
docker compose logs ros_bridge
```

**Connection issues:**
```bash
# Restart Docker
docker compose down && docker compose up -d

# Check robot IP in .env
cat nao6-hristudio-integration/.env
```

## Environment Variables

Create `nao6-hristudio-integration/.env`:
```
NAO_IP=10.0.0.42
NAO_USERNAME=nao
NAO_PASSWORD=nao
BRIDGE_PORT=9090
```

## 🚨 Safety Notes

- **Always verify robot is awake before movement commands**
- **Keep emergency stop accessible** (`qicli call ALMotion.rest()`)
- **Start with small movements (0.05 m/s)**
- **Monitor battery level**
- **Ensure clear space around robot**

## Credentials

**NAO Robot:**
- IP: `10.0.0.42` (configurable)
- Username: `nao`
- Password: `nao`

**HRIStudio:**
- Email: `sean@soconnor.dev`
- Password: `password123`

## Complete Restart

```bash
# 1. Restart Docker integration
cd ~/Documents/Projects/nao6-hristudio-integration
docker compose down
docker compose up -d

# 2. Verify robot is awake (check logs)
docker compose logs nao_driver | grep -i "wake\|autonomous"

# 3. Start HRIStudio
cd ~/Documents/Projects/hristudio
bun dev
```

---

**✅ Integration Status:** Production Ready  
**🤖 Tested With:** NAO V6 / ROS2 Humble / Docker
