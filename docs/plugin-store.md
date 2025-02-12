# Robot Plugin Store Architecture

The Robot Plugin Store is a central system for managing robot definitions and their associated actions. It enables contributors to add new robotics platforms and actions, which can then be used in the experiment designer and ultimately bridge to ROS2 or other robotics middleware.

## Overview

The plugin store consists of:
- A JSON-based plugin format for defining robots and their actions
- A loader system for managing and serving these plugins
- An admin interface for managing plugins
- Integration with the experiment designer
- A bridge to ROS2 for executing actions

## Plugin Schema

### Robot Plugin Definition

```typescript
interface RobotPlugin {
  // Core metadata
  robotId: string;          // Unique identifier for this robot
  name: string;            // Display name
  description?: string;    // Optional description
  platform: string;        // e.g., "ROS2", "custom"
  version: string;         // Semver version number
  
  // Manufacturer information
  manufacturer: {
    name: string;         // Manufacturer name
    website: string;      // Manufacturer website
    support?: string;     // Support URL
  };

  // Documentation
  documentation: {
    mainUrl: string;      // Main documentation URL
    apiReference?: string; // API/ROS2 interface documentation
    wikiUrl?: string;     // Wiki or community documentation
    videoUrl?: string;    // Video tutorial or overview
  };
  
  // Visual assets
  assets: {
    thumbnailUrl: string;  // Small preview image
    images: {             // Various robot images
      main: string;       // Main robot image
      angles?: {          // Optional different view angles
        front?: string;
        side?: string;
        top?: string;
      };
      dimensions?: string; // Technical drawing with dimensions
    };
    model?: {            // 3D model information
      format: "URDF" | "glTF" | "other";
      url: string;
    };
  };

  // Technical specifications
  specs: {
    dimensions: {
      length: number;     // in meters
      width: number;
      height: number;
      weight: number;     // in kg
    };
    capabilities: string[]; // e.g., ["differential_drive", "lidar", "camera"]
    maxSpeed: number;      // in m/s
    batteryLife: number;   // in hours
    payload?: number;      // max payload in kg
  };

  // Available actions for this robot
  actions: ActionDefinition[];
  
  // Platform-specific configuration
  ros2Config: {
    namespace: string;
    nodePrefix: string;
    defaultTopics: {
      cmd_vel: string;
      odom: string;
      scan: string;
      [key: string]: string;
    };
  };
}

interface ActionDefinition {
  actionId: string;       // Unique identifier for this action
  type: ActionType;       // Type of action (move, speak, etc.)
  title: string;         // Display name
  description: string;   // Description of what the action does
  icon?: string;        // Icon identifier for the UI
  
  // Parameter definition (using JSON Schema)
  parameters: {
    type: "object";
    properties: Record<string, {
      type: string;
      title: string;
      description?: string;
      default?: any;
      minimum?: number;
      maximum?: number;
      enum?: string[];
      unit?: string;     // e.g., "m/s", "rad", "m"
    }>;
    required: string[];
  };

  // ROS2 Integration details
  ros2: {
    messageType: string;    // ROS message type
    topic?: string;        // ROS topic to publish to
    service?: string;      // ROS service to call
    action?: string;       // ROS action to execute
    payloadMapping: {      // How parameters map to ROS messages
      type: "direct" | "transform";
      map?: Record<string, string>;
      transformFn?: string;  // Name of transform function if type="transform"
    };
    qos?: {               // Quality of Service settings
      reliability: "reliable" | "best_effort";
      durability: "volatile" | "transient_local";
      history: "keep_last" | "keep_all";
      depth?: number;
    };
  };
}
```

### Example Plugin JSON

```json
{
  "robotId": "turtlebot3-burger",
  "name": "TurtleBot3 Burger",
  "description": "A compact, affordable, programmable, ROS2-based mobile robot for education and research",
  "platform": "ROS2",
  "version": "2.0.0",
  
  "manufacturer": {
    "name": "ROBOTIS",
    "website": "https://www.robotis.com/",
    "support": "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/"
  },

  "documentation": {
    "mainUrl": "https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/",
    "apiReference": "https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_manipulation/",
    "wikiUrl": "https://wiki.ros.org/turtlebot3",
    "videoUrl": "https://www.youtube.com/watch?v=rVM994ZhsEM"
  },

  "assets": {
    "thumbnailUrl": "/robots/turtlebot3-burger-thumb.png",
    "images": {
      "main": "/robots/turtlebot3-burger-main.png",
      "angles": {
        "front": "/robots/turtlebot3-burger-front.png",
        "side": "/robots/turtlebot3-burger-side.png",
        "top": "/robots/turtlebot3-burger-top.png"
      },
      "dimensions": "/robots/turtlebot3-burger-dimensions.png"
    },
    "model": {
      "format": "URDF",
      "url": "https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/master/turtlebot3_description/urdf/turtlebot3_burger.urdf"
    }
  },

  "specs": {
    "dimensions": {
      "length": 0.138,
      "width": 0.178,
      "height": 0.192,
      "weight": 1.0
    },
    "capabilities": [
      "differential_drive",
      "lidar",
      "imu",
      "odometry"
    ],
    "maxSpeed": 0.22,
    "batteryLife": 2.5
  },

  "ros2Config": {
    "namespace": "turtlebot3",
    "nodePrefix": "hri_studio",
    "defaultTopics": {
      "cmd_vel": "/cmd_vel",
      "odom": "/odom",
      "scan": "/scan",
      "imu": "/imu",
      "joint_states": "/joint_states"
    }
  },

  "actions": [
    {
      "actionId": "move-velocity",
      "type": "move",
      "title": "Set Velocity",
      "description": "Control the robot's linear and angular velocity",
      "icon": "navigation",
      "parameters": {
        "type": "object",
        "properties": {
          "linear": {
            "type": "number",
            "title": "Linear Velocity",
            "description": "Forward/backward velocity",
            "default": 0,
            "minimum": -0.22,
            "maximum": 0.22,
            "unit": "m/s"
          },
          "angular": {
            "type": "number",
            "title": "Angular Velocity",
            "description": "Rotational velocity",
            "default": 0,
            "minimum": -2.84,
            "maximum": 2.84,
            "unit": "rad/s"
          }
        },
        "required": ["linear", "angular"]
      },
      "ros2": {
        "messageType": "geometry_msgs/msg/Twist",
        "topic": "/cmd_vel",
        "payloadMapping": {
          "type": "transform",
          "transformFn": "transformToTwist"
        },
        "qos": {
          "reliability": "reliable",
          "durability": "volatile",
          "history": "keep_last",
          "depth": 1
        }
      }
    },
    {
      "actionId": "move-to-pose",
      "type": "move",
      "title": "Move to Position",
      "description": "Navigate to a specific position on the map",
      "icon": "target",
      "parameters": {
        "type": "object",
        "properties": {
          "x": {
            "type": "number",
            "title": "X Position",
            "description": "X coordinate in meters",
            "default": 0,
            "unit": "m"
          },
          "y": {
            "type": "number",
            "title": "Y Position",
            "description": "Y coordinate in meters",
            "default": 0,
            "unit": "m"
          },
          "theta": {
            "type": "number",
            "title": "Orientation",
            "description": "Final orientation",
            "default": 0,
            "unit": "rad"
          }
        },
        "required": ["x", "y", "theta"]
      },
      "ros2": {
        "messageType": "geometry_msgs/msg/PoseStamped",
        "action": "/navigate_to_pose",
        "payloadMapping": {
          "type": "transform",
          "transformFn": "transformToPoseStamped"
        }
      }
    }
  ]
}
```

## Implementation Plan

### 1. Plugin Store Module

Create a TypeScript module to manage plugins:

```typescript
// src/lib/plugin-store/types.ts
export interface RobotPlugin { ... }
export interface ActionDefinition { ... }

// src/lib/plugin-store/store.ts
export class PluginStore {
  private plugins: Map<string, RobotPlugin>;
  
  async loadPlugins(): Promise<void>;
  async getPlugin(robotId: string): Promise<RobotPlugin>;
  async getAllPlugins(): Promise<RobotPlugin[]>;
}
```

### 2. Admin Interface

Build an admin panel for managing plugins:
- Upload/edit JSON plugin definitions
- Validate plugin schema
- Version management
- Preview plugin details

### 3. API Routes

Create API endpoints for plugin management:

```typescript
// GET /api/plugins
// GET /api/plugins/:robotId
// POST /api/plugins (with auth)
// PUT /api/plugins/:robotId (with auth)
// DELETE /api/plugins/:robotId (with auth)
```

### 4. Experiment Designer Integration

Update the experiment designer to:
- Allow robot selection
- Load appropriate actions
- Configure ROS2 bridge settings

### 5. ROS2 Bridge Integration

Create a bridge module for executing actions:

```typescript
// src/lib/ros2-bridge/bridge.ts
export class ROS2Bridge {
  async executeAction(
    robotId: string,
    actionId: string,
    parameters: Record<string, any>
  ): Promise<void>;
}
```

## Development Phases

1. **Phase 1: Core Plugin Store**
   - Implement plugin schema and validation
   - Build plugin loader
   - Create basic API endpoints

2. **Phase 2: Admin Interface**
   - Build plugin management UI
   - Implement plugin upload/edit
   - Add version control

3. **Phase 3: Experiment Designer Integration**
   - Add robot selection
   - Update action library based on selection
   - Enhance action configuration

4. **Phase 4: ROS2 Bridge**
   - Implement ROS2 connection
   - Add message transformation
   - Test with real robots

5. **Phase 5: Documentation & Testing**
   - Write contributor guidelines
   - Add comprehensive tests
   - Create example plugins

## Contributing

To add a new robot to the plugin store:

1. Create a new JSON file following the plugin schema
2. Test the plugin using the validation tools
3. Submit a pull request with:
   - Plugin JSON
   - Any custom transformation functions
   - Documentation updates
   - Test cases

## Future Enhancements

- Plugin marketplace for sharing robot definitions
- Visual plugin builder in admin interface
- Real-time plugin updates
- Plugin dependency management
- Custom action visualization components 