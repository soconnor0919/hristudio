# HRIStudio Mock Robot Simulation

This directory contains a mock robot server for simulating NAO6 robot connections without a physical robot.

## Quick Start

### Option 1: Standalone Mock Server (Recommended for testing)

```bash
cd scripts/mock-robot
bun install
bun dev
```

This starts the mock robot WebSocket server on `ws://localhost:9090`.

### Option 2: Docker Compose Mock Mode

```bash
cd nao6-hristudio-integration
docker compose -f docker-compose.yml -f docker-compose.mock.yml --profile mock up -d
```

### Option 3: Client-Side Simulation (No server needed)

Enable simulation mode in the wizard interface:
- Set `NEXT_PUBLIC_SIMULATION_MODE=true` in your `.env` file
- Or use the simulation toggle in the UI

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    HRIStudio Platform                       │
├─────────────────────────────────────────────────────────────┤
│  Wizard Interface (Browser)                                 │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ wizard-ros-service.ts                               │   │
│  │  ├── simulationMode: true → Simulates locally      │   │
│  │  └── simulationMode: false → Connects to server    │   │
│  └─────────────────────────────────────────────────────┘   │
│                           │                                 │
│              ┌────────────┴────────────┐                   │
│              │                         │                   │
│         Real Mode                 Simulation Mode           │
│              │                         │                   │
│              ▼                         ▼                   │
│    ┌─────────────────┐         ┌─────────────────┐         │
│    │ Mock Robot     │         │ Local JS        │         │
│    │ WebSocket      │         │ Simulation      │         │
│    │ Server         │         │ (No server)     │         │
│    └─────────────────┘         └─────────────────┘         │
└─────────────────────────────────────────────────────────────┘
```

## Mock Robot Server Protocol

The mock server implements the rosbridge WebSocket protocol:

### Supported Operations

| Operation | Description |
|-----------|-------------|
| `subscribe` | Subscribe to a topic |
| `unsubscribe` | Unsubscribe from a topic |
| `publish` | Publish a message to a topic |
| `call_service` | Call a ROS service |
| `advertise` | Advertise a topic |
| `unadvertise` | Stop advertising a topic |

### Simulated Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions (26 joints) |
| `/naoqi_driver/battery` | `naoqi_bridge_msgs/Battery` | Battery status (85%) |
| `/bumper` | `naoqi_bridge_msgs/Bumper` | Bumper contact sensors |
| `/hand_touch` | `naoqi_bridge_msgs/HandTouch` | Hand touch sensors |
| `/head_touch` | `naoqi_bridge_msgs/HeadTouch` | Head touch sensors |
| `/sonar/left` | `sensor_msgs/Range` | Left sonar distance |
| `/sonar/right` | `sensor_msgs/Range` | Right sonar distance |

### Simulated Services

| Service | Response |
|---------|----------|
| `/naoqi_driver/get_robot_info` | `{ robotName: "MOCK-NAO6", robotVersion: "6.0" }` |
| `/naoqi_driver/get_joint_names` | List of 26 joint names |
| `/naoqi_driver/get_position` | Current position `{ x, y, theta }` |
| `/naoqi_driver/is_waking_up` | `{ is_waking_up: false }` |

### Supported Actions

| Action | Parameters | Description |
|--------|------------|-------------|
| `say_text` | `text` | Speak text |
| `walk_forward` | `speed` | Walk forward |
| `walk_backward` | `speed` | Walk backward |
| `turn_left` | `speed` | Turn left |
| `turn_right` | `speed` | Turn right |
| `stop` | - | Stop all movement |
| `move_head` | `yaw`, `pitch`, `speed` | Move head |

## Configuration

### Environment Variables

```bash
# Mock Robot Server (scripts/mock-robot)
MOCK_ROBOT_PORT=9090              # WebSocket port
MOCK_PUBLISH_INTERVAL=100         # Sensor update interval (ms)

# HRIStudio Client
NEXT_PUBLIC_SIMULATION_MODE=true   # Enable client-side simulation
NEXT_PUBLIC_ROS_BRIDGE_URL=ws://localhost:9090
```

## Testing

### 1. Start Mock Server
```bash
cd scripts/mock-robot
bun dev
```

### 2. Start HRIStudio
```bash
cd hristudio
bun dev
```

### 3. Test Connection
Visit `http://localhost:3000/nao-test` and click "Connect". You should see:
- Connection status: `connected`
- Battery: ~85%
- Joint states updating
- Log messages showing subscriptions

### 4. Test Actions
Use the wizard interface to test:
- Speech actions
- Movement actions
- Head control

## Troubleshooting

### "Connection timeout" error
- Ensure mock server is running: `curl ws://localhost:9090`
- Check port is correct (default 9090)

### "Not connected to ROS bridge" error
- Enable simulation mode: `NEXT_PUBLIC_SIMULATION_MODE=true`
- Or connect to mock server first

### Actions not executing
- Check connection status in wizard interface
- Enable simulation mode if using client-side simulation

## Files

| File | Description |
|------|-------------|
| `scripts/mock-robot/src/server.ts` | TypeScript mock server |
| `scripts/mock-robot/server.js` | JavaScript mock server (for Docker) |
| `src/lib/ros/wizard-ros-service.ts` | Client with simulation mode |
| `src/hooks/useWizardRos.ts` | React hook with simulation support |
| `docker-compose.mock.yml` | Docker mock service |
| `robot-plugins/plugins/nao6-mock.json` | Mock NAO6 plugin |

## Development

### Adding New Simulated Actions

1. Edit `scripts/mock-robot/src/server.ts`
2. Add handler in `handlePublish()` or `handleServiceCall()`
3. Update `nao6-mock.json` plugin with new action definition

### Adding New Simulated Sensors

1. Edit `scripts/mock-robot/src/server.ts`
2. Add topic publishing in `publishRobotState()`
3. Update subscriber topics in `WizardRosService.subscribeToRobotTopics()`
