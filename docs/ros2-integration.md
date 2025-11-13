# ROS2 Integration Guide for HRIStudio

## Overview

HRIStudio integrates with ROS2-based robots through the rosbridge protocol, enabling web-based control and monitoring of robots without requiring ROS2 installation on the server. This approach provides flexibility and simplifies deployment, especially on platforms like Vercel.

## Architecture

### Communication Flow

```
HRIStudio Web App → WebSocket → rosbridge_server → ROS2 Robot
                                      ↓
                              ROS2 Topics/Services
```

### Key Components

1. **rosbridge_suite**: Provides WebSocket interface to ROS2
2. **roslib.js**: JavaScript library for ROS communication
3. **HRIStudio Plugin System**: Abstracts robot-specific implementations
4. **Message Type Definitions**: TypeScript interfaces for ROS2 messages

## ROS2 Bridge Setup

### Robot-Side Configuration

The robot or a companion computer must run rosbridge:

```bash
# Install rosbridge suite
sudo apt update
sudo apt install ros-humble-rosbridge-suite

# Launch rosbridge with WebSocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Custom Launch File

Create `hristudio_bridge.launch.xml`:

```xml
<launch>
  <arg name="port" default="9090"/>
  <arg name="address" default="0.0.0.0"/>
  <arg name="ssl" default="false"/>
  <arg name="certfile" default=""/>
  <arg name="keyfile" default=""/>
  
  <node pkg="rosbridge_server" exec="rosbridge_websocket" name="rosbridge_websocket">
    <param name="port" value="$(var port)"/>
    <param name="address" value="$(var address)"/>
    <param name="ssl" value="$(var ssl)"/>
    <param name="certfile" value="$(var certfile)"/>
    <param name="keyfile" value="$(var keyfile)"/>
    
    <!-- Limit message sizes for security -->
    <param name="max_message_size" value="10000000"/>
    <param name="unregister_timeout" value="10.0"/>
    
    <!-- Enable specific services only -->
    <param name="services_glob" value="['/hristudio/*']"/>
    <param name="topics_glob" value="['/hristudio/*', '/tf', '/tf_static']"/>
  </node>
</launch>
```

## Client-Side Implementation

### ROS Connection Manager

`src/lib/ros/connection.ts`:

```typescript
import ROSLIB from 'roslib';

export class RosConnection {
  private ros: ROSLIB.Ros | null = null;
  private url: string;
  private reconnectInterval: number = 5000;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private listeners: Map<string, Set<(data: any) => void>> = new Map();

  constructor(url: string = process.env.NEXT_PUBLIC_ROSBRIDGE_URL || 'ws://localhost:9090') {
    this.url = url;
  }

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      if (this.ros?.isConnected) {
        resolve();
        return;
      }

      this.ros = new ROSLIB.Ros({
        url: this.url,
        options: {
          // Enable compression for better performance
          compression: 'png',
          // Throttle rate for topic subscriptions
          throttle_rate: 100,
        }
      });

      this.ros.on('connection', () => {
        console.log('Connected to ROS bridge');
        this.clearReconnectTimer();
        resolve();
      });

      this.ros.on('error', (error) => {
        console.error('ROS connection error:', error);
        reject(error);
      });

      this.ros.on('close', () => {
        console.log('ROS connection closed');
        this.scheduleReconnect();
      });
    });
  }

  private scheduleReconnect() {
    if (this.reconnectTimer) return;
    
    this.reconnectTimer = setTimeout(() => {
      console.log('Attempting to reconnect to ROS...');
      this.connect().catch(console.error);
    }, this.reconnectInterval);
  }

  private clearReconnectTimer() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  disconnect() {
    this.clearReconnectTimer();
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
  }

  isConnected(): boolean {
    return this.ros?.isConnected || false;
  }

  getRos(): ROSLIB.Ros | null {
    return this.ros;
  }

  // Topic subscription helper
  subscribe<T>(topicName: string, messageType: string, callback: (message: T) => void): () => void {
    if (!this.ros) throw new Error('Not connected to ROS');

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
      compression: 'png',
      throttle_rate: 100
    });

    topic.subscribe(callback);

    // Return unsubscribe function
    return () => {
      topic.unsubscribe(callback);
    };
  }

  // Service call helper
  async callService<TRequest, TResponse>(
    serviceName: string,
    serviceType: string,
    request: TRequest
  ): Promise<TResponse> {
    if (!this.ros) throw new Error('Not connected to ROS');

    const service = new ROSLIB.Service({
      ros: this.ros,
      name: serviceName,
      serviceType: serviceType
    });

    return new Promise((resolve, reject) => {
      service.callService(
        new ROSLIB.ServiceRequest(request),
        (response: TResponse) => resolve(response),
        (error: string) => reject(new Error(error))
      );
    });
  }

  // Action client helper
  createActionClient(actionName: string, actionType: string): ROSLIB.ActionClient {
    if (!this.ros) throw new Error('Not connected to ROS');

    return new ROSLIB.ActionClient({
      ros: this.ros,
      serverName: actionName,
      actionName: actionType
    });
  }
}

// Singleton instance
export const rosConnection = new RosConnection();
```

### ROS2 Message Types

`src/lib/ros/types.ts`:

```typescript
// Common ROS2 message types
export interface Header {
  stamp: {
    sec: number;
    nanosec: number;
  };
  frame_id: string;
}

export interface Twist {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

export interface Pose {
  position: {
    x: number;
    y: number;
    z: number;
  };
  orientation: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
}

export interface JointState {
  header: Header;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

export interface BatteryState {
  header: Header;
  voltage: number;
  temperature: number;
  current: number;
  charge: number;
  capacity: number;
  percentage: number;
  power_supply_status: number;
  power_supply_health: number;
  power_supply_technology: number;
  present: boolean;
}

// HRIStudio specific messages
export interface HRICommand {
  action_id: string;
  action_type: string;
  parameters: Record<string, any>;
  timeout: number;
}

export interface HRIResponse {
  action_id: string;
  success: boolean;
  message: string;
  data: Record<string, any>;
  duration_ms: number;
}

export interface HRIState {
  robot_id: string;
  connected: boolean;
  battery: BatteryState;
  pose: Pose;
  joint_states: JointState;
  custom_data: Record<string, any>;
}
```

## ROS2 Robot Plugin Implementation

### Base ROS2 Plugin

`src/lib/plugins/ros2/base-plugin.ts`:

```typescript
import { RobotPlugin, ActionDefinition, ActionResult, RobotState } from '@/lib/plugins/types';
import { rosConnection } from '@/lib/ros/connection';
import { HRICommand, HRIResponse, HRIState, Twist } from '@/lib/ros/types';
import ROSLIB from 'roslib';
import { z } from 'zod';

export abstract class BaseROS2Plugin implements RobotPlugin {
  abstract id: string;
  abstract name: string;
  abstract version: string;
  abstract robotId: string;
  
  protected namespace: string = '/hristudio';
  protected commandTopic: ROSLIB.Topic | null = null;
  protected stateTopic: ROSLIB.Topic | null = null;
  protected currentState: HRIState | null = null;
  protected pendingCommands: Map<string, (response: HRIResponse) => void> = new Map();

  abstract configSchema: z.ZodSchema;
  abstract defaultConfig: Record<string, any>;
  abstract actions: ActionDefinition[];

  async initialize(config: any): Promise<void> {
    // Validate config
    this.configSchema.parse(config);
    
    // Set namespace if provided
    if (config.namespace) {
      this.namespace = config.namespace;
    }
  }

  async connect(): Promise<boolean> {
    try {
      await rosConnection.connect();
      
      const ros = rosConnection.getRos();
      if (!ros) return false;

      // Subscribe to robot state
      this.stateTopic = new ROSLIB.Topic({
        ros,
        name: `${this.namespace}/robot_state`,
        messageType: 'hristudio_msgs/HRIState'
      });

      this.stateTopic.subscribe((state: HRIState) => {
        this.currentState = state;
      });

      // Create command publisher
      this.commandTopic = new ROSLIB.Topic({
        ros,
        name: `${this.namespace}/commands`,
        messageType: 'hristudio_msgs/HRICommand'
      });

      // Subscribe to responses
      const responseTopic = new ROSLIB.Topic({
        ros,
        name: `${this.namespace}/responses`,
        messageType: 'hristudio_msgs/HRIResponse'
      });

      responseTopic.subscribe((response: HRIResponse) => {
        const handler = this.pendingCommands.get(response.action_id);
        if (handler) {
          handler(response);
          this.pendingCommands.delete(response.action_id);
        }
      });

      // Wait for initial state
      await this.waitForState(5000);
      
      return true;
    } catch (error) {
      console.error('Failed to connect to ROS2:', error);
      return false;
    }
  }

  async disconnect(): Promise<void> {
    if (this.stateTopic) {
      this.stateTopic.unsubscribe();
      this.stateTopic = null;
    }
    
    if (this.commandTopic) {
      this.commandTopic = null;
    }
    
    this.currentState = null;
    this.pendingCommands.clear();
  }

  async executeAction(action: ActionDefinition, params: any): Promise<ActionResult> {
    if (!this.commandTopic) {
      return {
        success: false,
        error: 'Not connected to robot',
        duration: 0
      };
    }

    const startTime = Date.now();
    const actionId = `${action.id}_${Date.now()}`;

    try {
      // Validate parameters
      const validatedParams = action.parameterSchema.parse(params);

      // Create command
      const command: HRICommand = {
        action_id: actionId,
        action_type: action.id,
        parameters: validatedParams,
        timeout: action.timeout || 30000
      };

      // Send command and wait for response
      const response = await this.sendCommand(command);

      return {
        success: response.success,
        data: response.data,
        error: response.success ? undefined : response.message,
        duration: Date.now() - startTime
      };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        duration: Date.now() - startTime
      };
    }
  }

  async getState(): Promise<RobotState> {
    if (!this.currentState) {
      return {
        connected: false
      };
    }

    return {
      connected: this.currentState.connected,
      battery: this.currentState.battery.percentage,
      position: {
        x: this.currentState.pose.position.x,
        y: this.currentState.pose.position.y,
        z: this.currentState.pose.position.z
      },
      sensors: {
        jointStates: this.currentState.joint_states,
        ...this.currentState.custom_data
      }
    };
  }

  protected sendCommand(command: HRICommand): Promise<HRIResponse> {
    return new Promise((resolve, reject) => {
      if (!this.commandTopic) {
        reject(new Error('Command topic not initialized'));
        return;
      }

      // Set timeout
      const timeout = setTimeout(() => {
        this.pendingCommands.delete(command.action_id);
        reject(new Error('Command timeout'));
      }, command.timeout);

      // Store handler
      this.pendingCommands.set(command.action_id, (response) => {
        clearTimeout(timeout);
        resolve(response);
      });

      // Publish command
      this.commandTopic.publish(command);
    });
  }

  protected async waitForState(timeoutMs: number): Promise<void> {
    const startTime = Date.now();
    while (!this.currentState && Date.now() - startTime < timeoutMs) {
      await new Promise(resolve => setTimeout(resolve, 100));
    }
    if (!this.currentState) {
      throw new Error('Timeout waiting for robot state');
    }
  }

  // Helper methods for common robot controls
  protected async moveBase(linear: number, angular: number): Promise<ActionResult> {
    const ros = rosConnection.getRos();
    if (!ros) {
      return { success: false, error: 'Not connected', duration: 0 };
    }

    const cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    const twist: Twist = {
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    };

    cmdVelTopic.publish(twist);
    
    return { success: true, duration: 0 };
  }
}
```

### TurtleBot3 Plugin Example

`src/lib/plugins/robots/turtlebot3.ts`:

```typescript
import { BaseROS2Plugin } from '../ros2/base-plugin';
import { ActionDefinition } from '@/lib/plugins/types';
import { z } from 'zod';

export class TurtleBot3Plugin extends BaseROS2Plugin {
  id = 'turtlebot3-burger';
  name = 'TurtleBot3 Burger';
  version = '1.0.0';
  robotId = 'turtlebot3';

  configSchema = z.object({
    namespace: z.string().default('/tb3'),
    maxLinearVelocity: z.number().default(0.22),
    maxAngularVelocity: z.number().default(2.84),
    rosbridge_url: z.string().url().optional(),
  });

  defaultConfig = {
    namespace: '/tb3',
    maxLinearVelocity: 0.22,
    maxAngularVelocity: 2.84,
  };

  actions: ActionDefinition[] = [
    {
      id: 'move_forward',
      name: 'Move Forward',
      description: 'Move the robot forward',
      category: 'movement',
      icon: 'arrow-up',
      parameterSchema: z.object({
        distance: z.number().min(0).max(5).describe('Distance in meters'),
        speed: z.number().min(0).max(0.22).default(0.1).describe('Speed in m/s'),
      }),
      timeout: 30000,
      retryable: true,
    },
    {
      id: 'turn',
      name: 'Turn',
      description: 'Turn the robot',
      category: 'movement',
      icon: 'rotate-cw',
      parameterSchema: z.object({
        angle: z.number().min(-180).max(180).describe('Angle in degrees'),
        speed: z.number().min(0).max(2.84).default(0.5).describe('Angular speed in rad/s'),
      }),
      timeout: 20000,
      retryable: true,
    },
    {
      id: 'speak',
      name: 'Speak',
      description: 'Make the robot speak using TTS',
      category: 'interaction',
      icon: 'volume-2',
      parameterSchema: z.object({
        text: z.string().max(500).describe('Text to speak'),
        voice: z.enum(['male', 'female']).default('female'),
        speed: z.number().min(0.5).max(2).default(1),
      }),
      timeout: 60000,
      retryable: false,
    },
    {
      id: 'led_color',
      name: 'Set LED Color',
      description: 'Change the robot LED color',
      category: 'feedback',
      icon: 'lightbulb',
      parameterSchema: z.object({
        color: z.enum(['red', 'green', 'blue', 'yellow', 'white', 'off']),
        duration: z.number().min(0).max(60).default(0).describe('Duration in seconds (0 = permanent)'),
      }),
      timeout: 5000,
      retryable: true,
    },
  ];

  async initialize(config: any): Promise<void> {
    await super.initialize(config);
    
    // TurtleBot3 specific initialization
    if (config.rosbridge_url) {
      // Override default rosbridge URL if provided
      process.env.NEXT_PUBLIC_ROSBRIDGE_URL = config.rosbridge_url;
    }
  }

  // Override executeAction for robot-specific implementations
  async executeAction(action: ActionDefinition, params: any): Promise<ActionResult> {
    // For movement actions, we can use direct topic publishing
    // for better real-time control
    if (action.id === 'move_forward') {
      return this.moveForward(params.distance, params.speed);
    } else if (action.id === 'turn') {
      return this.turn(params.angle, params.speed);
    }
    
    // For other actions, use the base implementation
    return super.executeAction(action, params);
  }

  private async moveForward(distance: number, speed: number): Promise<ActionResult> {
    const startTime = Date.now();
    const duration = (distance / speed) * 1000; // Convert to milliseconds
    
    // Start moving
    await this.moveBase(speed, 0);
    
    // Wait for movement to complete
    await new Promise(resolve => setTimeout(resolve, duration));
    
    // Stop
    await this.moveBase(0, 0);
    
    return {
      success: true,
      data: { distance, speed },
      duration: Date.now() - startTime
    };
  }

  private async turn(angleDegrees: number, speed: number): Promise<ActionResult> {
    const startTime = Date.now();
    const angleRad = (angleDegrees * Math.PI) / 180;
    const duration = Math.abs(angleRad / speed) * 1000;
    
    // Start turning (negative for clockwise)
    const angularVel = angleDegrees > 0 ? speed : -speed;
    await this.moveBase(0, angularVel);
    
    // Wait for turn to complete
    await new Promise(resolve => setTimeout(resolve, duration));
    
    // Stop
    await this.moveBase(0, 0);
    
    return {
      success: true,
      data: { angle: angleDegrees, speed },
      duration: Date.now() - startTime
    };
  }
}
```

## ROS2 Node for HRIStudio

For robots to work with HRIStudio, they need a ROS2 node that implements the HRIStudio protocol:

`hristudio_robot_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, BatteryState
from hristudio_msgs.msg import HRICommand, HRIResponse, HRIState
import json
import time
from threading import Thread

class HRIStudioRobotNode(Node):
    def __init__(self):
        super().__init__('hristudio_robot_node')
        
        # Publishers
        self.state_pub = self.create_publisher(HRIState, '/hristudio/robot_state', 10)
        self.response_pub = self.create_publisher(HRIResponse, '/hristudio/responses', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            HRICommand,
            '/hristudio/commands',
            self.command_callback,
            10
        )
        
        # Robot control publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State update timer
        self.state_timer = self.create_timer(0.1, self.publish_state)
        
        # Action handlers
        self.action_handlers = {
            'move_forward': self.handle_move_forward,
            'turn': self.handle_turn,
            'speak': self.handle_speak,
            'led_color': self.handle_led_color,
        }
        
        self.get_logger().info('HRIStudio Robot Node started')
    
    def command_callback(self, msg):
        """Handle incoming commands from HRIStudio"""
        self.get_logger().info(f'Received command: {msg.action_type}')
        
        # Execute action in separate thread to avoid blocking
        thread = Thread(target=self.execute_command, args=(msg,))
        thread.start()
    
    def execute_command(self, command):
        """Execute a command and send response"""
        start_time = time.time()
        response = HRIResponse()
        response.action_id = command.action_id
        
        try:
            # Parse parameters
            params = json.loads(command.parameters) if isinstance(command.parameters, str) else command.parameters
            
            # Execute action
            if command.action_type in self.action_handlers:
                result = self.action_handlers[command.action_type](params)
                response.success = result['success']
                response.message = result.get('message', '')
                response.data = json.dumps(result.get('data', {}))
            else:
                response.success = False
                response.message = f'Unknown action type: {command.action_type}'
        
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Error executing command: {e}')
        
        response.duration_ms = int((time.time() - start_time) * 1000)
        self.response_pub.publish(response)
    
    def publish_state(self):
        """Publish current robot state"""
        state = HRIState()
        state.robot_id = 'turtlebot3'
        state.connected = True
        
        # Add current sensor data
        # This would come from actual robot sensors
        state.battery.percentage = 85.0
        state.pose.position.x = 0.0
        state.pose.position.y = 0.0
        state.pose.orientation.w = 1.0
        
        self.state_pub.publish(state)
    
    def handle_move_forward(self, params):
        """Move robot forward"""
        distance = params.get('distance', 0)
        speed = params.get('speed', 0.1)
        
        # Calculate duration
        duration = distance / speed
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        
        # Wait for movement to complete
        time.sleep(duration)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        return {
            'success': True,
            'data': {'distance': distance, 'speed': speed}
        }
    
    def handle_turn(self, params):
        """Turn robot"""
        angle = params.get('angle', 0)
        speed = params.get('speed', 0.5)
        
        # Convert to radians
        angle_rad = angle * 3.14159 / 180
        duration = abs(angle_rad) / speed
        
        # Publish velocity command
        twist = Twist()
        twist.angular.z = speed if angle > 0 else -speed
        self.cmd_vel_pub.publish(twist)
        
        # Wait for turn to complete
        time.sleep(duration)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        return {
            'success': True,
            'data': {'angle': angle, 'speed': speed}
        }
    
    def handle_speak(self, params):
        """Text to speech"""
        text = params.get('text', '')
        # Implement TTS here
        self.get_logger().info(f'Speaking: {text}')
        return {'success': True, 'data': {'text': text}}
    
    def handle_led_color(self, params):
        """Set LED color"""
        color = params.get('color', 'off')
        # Implement LED control here
        self.get_logger().info(f'Setting LED to: {color}')
        return {'success': True, 'data': {'color': color}}

def main(args=None):
    rclpy.init(args=args)
    node = HRIStudioRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Security Considerations

### 1. Authentication
- Use SSL/TLS for rosbridge connections in production
- Implement token-based authentication for rosbridge
- Restrict topic/service access patterns

### 2. Network Security
- Use VPN or SSH tunnels for remote robot connections
- Implement firewall rules to restrict rosbridge access
- Use separate network segments for robot communication

### 3. Message Validation
- Validate all incoming messages on the robot side
- Implement rate limiting to prevent DoS attacks
- Sanitize string inputs to prevent injection attacks

## Deployment Considerations

### Local Development
- Robot and development machine on same network
- Direct WebSocket connection to rosbridge
- No SSL required

### Production (Vercel)
- Robot behind NAT/firewall
- Use reverse proxy or tunnel (e.g., ngrok, Cloudflare Tunnel)
- SSL/TLS required for secure communication
- Consider latency for real-time control

### Hybrid Approach
- Local "robot companion" server near robot
- Companion server connects to both robot and Vercel app
- Reduces latency for critical operations
- Maintains security boundaries

## Testing ROS2 Integration

### Unit Tests

`src/lib/ros/__tests__/connection.test.ts`:

```typescript
import { RosConnection } from '../connection';
import { vi, describe, it, expect, beforeEach } from 'vitest';

vi.mock('roslib', () => ({
  default: {
    Ros: vi.fn().mockImplementation(() => ({
      on: vi.fn(),
      connect: vi.fn(),
      close: vi.fn(),
      isConnected: true,
    })),
    Topic: vi.fn(),
    Service: vi.fn(),
  },
}));

describe('RosConnection', () => {
  let connection: RosConnection;

  beforeEach(() => {
    connection = new RosConnection('ws://test:9090');
  });

  it('should connect successfully', async () => {
    await expect(connection.connect()).resolves.toBeUndefined();
    expect(connection.isConnected()).toBe(true);
  });

  // Add more tests...
});
```

### Integration Tests
- Use rosbridge_server in test mode
- Mock robot responses
- Test error scenarios
- Verify message formats

## Performance Optimization

1. **Message Throttling**: Limit frequency of state updates
2. **Compression**: Enable PNG compression for image topics
3. **Selective Subscriptions**: Only subscribe to needed topics
4. **Connection Pooling**: Reuse WebSocket connections
5. **Client-Side Caching**: Cache robot capabilities

## Troubleshooting

### Common Issues

1. **Connection Refused**
   - Check rosbridge is running
   - Verify firewall rules
   - Check WebSocket URL

2. **Message Type Errors**
   - Ensure message types match between client and robot
   - Verify ROS2 workspace is sourced

3. **High Latency**
   - Check network conditions
   - Consider local rosbridge proxy
   - Optimize message sizes

4. **Authentication Failures**
   - Verify SSL certificates
   - Check authentication tokens
   - Review rosbridge configuration