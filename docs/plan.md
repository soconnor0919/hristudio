# HRIStudio Development Plan

## Immediate Goal: Paper Submission (1 Month)
Focus on delivering a functional experiment designer that demonstrates the platform's capabilities for Wizard-of-Oz HRI studies.

### 1. Experiment Designer Core
- [x] Basic flow-based designer UI
- [ ] Step containers with drag-and-drop, that can contain sets of actions
- [ ] Action node system
  - [ ] Action schema definition
  - [ ] Visual node editor
  - [ ] Connection validation
  - [ ] Parameter configuration UI

### 2. Plugin System
- [x] Plugin store infrastructure
- [x] Basic plugin loading mechanism
- [ ] Action Libraries
  - [ ] Wizard Actions
    - [ ] Robot movement control
    - [ ] Speech synthesis
    - [ ] Gesture control
  - [ ] TurtleBot3 Integration
    - [ ] ROS2 message types
    - [ ] Movement actions
    - [ ] Sensor feedback
  - [ ] Experiment Flow
    - [ ] Timing controls
    - [ ] Wait conditions
    - [ ] Participant input handling
    - [ ] Data recording triggers

### 3. Execution Engine
- [ ] Step execution pipeline
- [ ] Action validation
- [ ] Real-time monitoring
- [ ] Data collection
  - [ ] Action logs
  - [ ] Timing data
  - [ ] Participant responses

## Future Extensions

### 1. Enhanced Plugin Ecosystem
- Community plugin repository
- Plugin versioning and compatibility
- Custom action development tools

### 2. Advanced Experiment Features
- Conditional branching
- Dynamic parameter adjustment
- Multi-robot coordination
- Real-time visualization

### 3. Data Analysis Tools
- Session replay
- Data export
- Analysis templates
- Visualization tools

## Technical Requirements

### Action Schema
```typescript
interface ActionDefinition {
  actionId: string;
  type: ActionType;
  title: string;
  description: string;
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
      unit?: string;
    }>;
    required: string[];
  };
  ros2?: {
    messageType: string;
    topic?: string;
    service?: string;
    action?: string;
    payloadMapping: {
      type: "direct" | "transform";
      map?: Record<string, string>;
      transformFn?: string;
    };
    qos?: {
      reliability: "reliable" | "best_effort";
      durability: "volatile" | "transient_local";
      history: "keep_last" | "keep_all";
      depth?: number;
    };
  };
}
```

### Plugin Structure
```
plugin-name/
├── plugin.json       # Plugin metadata and action definitions
├── transforms.ts     # Custom transform functions
├── validators.ts     # Parameter validation
└── assets/          # Icons and documentation
```

## Implementation Priority
1. Core action system and visual editor
2. Basic wizard actions (movement, speech)
3. TurtleBot3 integration
4. Flow control actions
5. Data collection
6. Analysis tools