# Tutorial 6: Robot Integration

Learn how to connect and configure robots for your HRI studies.

## Objectives

- Connect NAO6 robot to HRIStudio
- Configure robot plugins
- Test robot connection
- Troubleshoot common issues

## Supported Robots

HRIStudio supports multiple robot platforms:

| Robot | Protocol | Actions |
|-------|----------|---------|
| **NAO6** | ROS2 | Speech, movement, gestures, sensors |
| **TurtleBot3** | ROS2 | Navigation, sensors |
| **Mock Robot** | WebSocket | All actions (simulation) |

## Understanding the Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      HRIStudio Platform                       │
│                                                              │
│   ┌──────────────┐              ┌──────────────────────┐     │
│   │ Wizard       │◄────────────►│ Robot Communication  │     │
│   │ Interface    │   WebSocket │ Service              │     │
│   └──────────────┘              └──────────┬───────────┘     │
│                                            │                 │
│                                            │ ROS Bridge      │
│                                      ┌─────▼─────┐           │
│                                      │ rosbridge │           │
│                                      │ :9090     │           │
│                                      └─────┬─────┘           │
└────────────────────────────────────────────┼─────────────────┘
                                             │
                           ┌─────────────────┼─────────────────┐
                           │                 │                 │
                     ┌─────▼─────┐     ┌─────▼─────┐           │
                     │  NAO      │     │  NAO      │           │
                     │  Driver   │     │  Robot    │           │
                     │  (ROS2)   │◄───►│  (naoqi)  │           │
                     └───────────┘     └───────────┘           │
                           Network                   Robot     │
```

## Step 1: Set Up NAO6 Robot

### Network Configuration

1. Connect NAO6 to your network:
   ```
   # On the robot, say "Connect to Wi-Fi"
   # Or use the Choregraphe interface
   ```

2. Note the robot's IP address:
   ```
   # On the robot, say "What is my IP address?"
   # Or check robot's network settings
   ```

3. Verify network access:
   ```bash
   ping nao.local
   # Or ping the IP directly:
   ping 192.168.1.100
   ```

### Robot Credentials

Default credentials:
```
Username: nao
Password: robolab
```

### Wake Up Robot

Before connecting, wake up the robot:

```bash
ssh nao@192.168.1.100
# Enter password when prompted

# Wake up the robot
python -c "from naoqi import ALProxy; proxy = ALProxy('ALMotion', '192.168.1.100', 9559); proxy.wakeUp()"
```

## Step 2: Start Docker Services

### Using Docker Compose

```bash
cd ~/nao6-hristudio-integration

# Set robot IP
export NAO_IP=192.168.1.100

# Start services
docker compose up -d
```

### Services Overview

| Service | Port | Purpose |
|---------|------|---------|
| `nao_driver` | - | ROS2 driver for NAO |
| `ros_bridge` | 9090 | WebSocket bridge |
| `ros_api` | - | Topic introspection |

### Verify Services

```bash
# Check running containers
docker ps

# View logs
docker compose logs -f

# Test WebSocket connection
ws://localhost:9090
```

## Step 3: Configure HRIStudio

### Install Robot Plugin

1. Go to **Plugins** in sidebar
2. Select your study
3. Click **Browse Plugins**
4. Find **NAO6 Robot (ROS2 Integration)**
5. Click **Install**

### Configure Plugin

Set robot connection:

```
┌─────────────────────────────────────────────────────────────┐
│ NAO6 Robot Configuration                                   │
├─────────────────────────────────────────────────────────────┤
│ Robot Name: NAO6-Lab                                      │
│ Robot IP: 192.168.1.100                                  │
│ WebSocket URL: ws://localhost:9090                        │
│                                                             │
│ Advanced Settings:                                         │
│ □ Use Simulation Mode                                     │
│ Connection Timeout: 30 seconds                            │
└─────────────────────────────────────────────────────────────┘
```

### Environment Variables

Create `hristudio/.env.local`:

```bash
# Robot connection
NAO_ROBOT_IP=192.168.1.100
NAO_PASSWORD=robolab
NAO_USERNAME=nao

# WebSocket bridge
NEXT_PUBLIC_ROS_BRIDGE_URL=ws://localhost:9090
```

## Step 4: Test Connection

### Using the NAO Test Page

1. Navigate to: `http://localhost:3000/nao-test`
2. Click **Connect**
3. Verify connection status

### Connection Status Indicators

| Status | Meaning |
|--------|---------|
| **Connected** | Robot responding normally |
| **Connecting** | Attempting connection |
| **Error** | Connection failed |
| **Timeout** | Robot not responding |

### Test Actions

Test basic robot actions:

| Action | Expected Behavior |
|--------|-------------------|
| Say Text | Robot speaks |
| Wave | Robot waves arm |
| Walk Forward | Robot walks |
| Turn Left | Robot turns |

## Step 5: Robot Actions Reference

### Speech Actions

| Action | Parameters | Description |
|--------|------------|-------------|
| `say_text` | `text` | Speak text |
| `say_with_emotion` | `text`, `emotion` | Emotional speech |
| `set_volume` | `level` | Set speech volume |
| `set_language` | `language` | Set speech language |

### Movement Actions

| Action | Parameters | Description |
|--------|------------|-------------|
| `walk_forward` | `speed`, `duration` | Walk forward |
| `walk_backward` | `speed` | Walk backward |
| `turn_left` | `speed` | Turn left |
| `turn_right` | `speed` | Turn right |
| `stop` | - | Stop all movement |

### Head Actions

| Action | Parameters | Description |
|--------|------------|-------------|
| `move_head` | `yaw`, `pitch`, `speed` | Move head to position |
| `turn_head` | `yaw`, `pitch` | Turn head (relative) |

### Arm Actions

| Action | Parameters | Description |
|--------|------------|-------------|
| `move_arm` | `arm`, joint angles | Move arm to position |
| `wave` | `arm` | Wave gesture |

### Autonomous Life

| Action | Parameters | Description |
|--------|------------|-------------|
| `wake_up` | - | Wake robot from rest |
| `rest` | - | Put robot to rest |
| `set_autonomous_life` | `enabled` | Toggle autonomous behavior |

## Step 6: Troubleshooting

### Common Issues

#### Robot Not Found

```
Error: Cannot connect to robot at 192.168.1.100
```

**Solutions:**
1. Verify IP address: `ping 192.168.1.100`
2. Check robot is powered on
3. Verify network connectivity
4. Try `nao.local` hostname

#### WebSocket Connection Failed

```
Error: WebSocket connection to ws://localhost:9090 failed
```

**Solutions:**
1. Check Docker is running
2. Verify ros_bridge container: `docker ps`
3. Check port 9090 is not blocked
4. Restart services: `docker compose restart`

#### Robot Not Responding

Robot connected but actions don't execute.

**Solutions:**
1. Wake up robot: `ssh nao@IP python -c "from naoqi import ALProxy; p=ALProxy('ALMotion','IP',9559);p.wakeUp()"`
2. Check robot is not in rest mode
3. Verify no blocking software on robot

#### Action Timeout

```
Error: Action timed out after 30 seconds
```

**Solutions:**
1. Robot may be busy with previous action
2. Check network latency
3. Increase timeout in settings

### Diagnostic Commands

```bash
# Check Docker containers
docker ps

# View all logs
docker compose logs

# View specific service
docker compose logs ros_bridge

# Restart services
docker compose restart

# Stop and start fresh
docker compose down
docker compose up -d
```

### Network Troubleshooting

```bash
# Check robot IP
ssh nao@IP "ifconfig"

# Test from robot
ssh nao@IP "curl localhost:9090"

# Check firewall
sudo iptables -L
```

## Step 7: Robot Maintenance

### Battery Management

- Check battery before each session
- Aim for >50% battery
- Charge during breaks
- Replace battery if <20% capacity

### Calibration

Periodically calibrate:
- Joint positions
- Camera alignment
- Touch sensors
- Sound localization

### Software Updates

Keep robot software updated:
- NAOqi version
- ROS2 packages
- HRIStudio plugin

## Security Considerations

### Network Security

- Use encrypted network (WPA2/WPA3)
- Firewall robot from internet
- Use strong passwords

### SSH Access

- Change default passwords
- Use SSH keys when possible
- Limit SSH access

### Data Security

- Robot camera data may be sensitive
- Store data securely
- Follow IRB guidelines

## Simulation Mode

For testing without a robot:

1. Enable simulation mode in settings
2. Or set `NEXT_PUBLIC_SIMULATION_MODE=true`
3. All actions are simulated locally

See [Simulation Mode Tutorial](09-simulation-mode.md) for details.

## Next Steps

Now that your robot is connected:

1. **[Running Trials](04-running-trials.md)** - Execute trials with robot
2. **[Wizard Interface](05-wizard-interface.md)** - Control the robot
3. **[Data & Analysis](08-data-and-analysis.md)** - Collect interaction data

---

**Previous**: [Wizard Interface](05-wizard-interface.md) | **Next**: [Forms & Surveys](07-forms-and-surveys.md)
