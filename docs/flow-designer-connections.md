# Flow Designer Connections & Ordering System

## Overview

The HRIStudio Flow Designer uses React Flow to provide an intuitive visual interface for connecting and ordering experiment steps. This document explains how the connection system works and why React Flow is the optimal choice for this functionality.

## Why React Flow?

### ✅ **Advantages of React Flow**

1. **Visual Clarity**: Users can see the experiment flow at a glance
2. **Intuitive Interaction**: Drag-and-drop connections feel natural
3. **Professional UI**: Industry-standard flow editor interface
4. **Flexible Layouts**: Non-linear arrangements for complex experiments
5. **Real-time Feedback**: Immediate visual confirmation of connections
6. **Zoom & Pan**: Handle large, complex experiments easily
7. **Accessibility**: Built-in keyboard navigation and screen reader support

### 🔄 **Alternative Approaches Considered**

| Approach | Pros | Cons | Verdict |
|----------|------|------|---------|
| **List-based ordering** | Simple to implement | Limited to linear flows | ❌ Too restrictive |
| **Tree structure** | Good for hierarchies | Complex for parallel flows | ❌ Not flexible enough |
| **Graph-based UI** | Very flexible | Higher learning curve | ⚠️ React Flow provides this |
| **Timeline interface** | Good for time-based flows | Poor for conditional logic | ❌ Wrong metaphor |

**Conclusion**: React Flow provides the best balance of power, usability, and visual clarity.

## Connection System

### 🔗 **How Connections Work**

#### **1. Visual Connection Handles**
```typescript
// Each step node has input/output handles
<Handle
  type="target"
  position={Position.Left}
  className="!bg-primary !border-background !h-3 !w-3 !border-2"
  id="input"
/>
<Handle
  type="source"
  position={Position.Right}
  className="!bg-primary !border-background !h-3 !w-3 !border-2"
  id="output"
/>
```

#### **2. Connection Logic**
- **Source Handle**: Right side of each step (output)
- **Target Handle**: Left side of each step (input)
- **Visual Feedback**: Handles highlight during connection attempts
- **Theme Integration**: Handles use `!bg-primary` for consistent theming

#### **3. Auto-Positioning**
When steps are connected, the system automatically adjusts positions:
```typescript
const handleConnect = useCallback((params: Connection) => {
  // Automatically position target step to the right of source
  const updatedPosition = {
    x: Math.max(sourceStep.position.x + 300, step.position.x),
    y: step.position.y,
  };
});
```

### 📋 **Connection Rules**

1. **One Input per Step**: Each step can have multiple inputs but should logically follow one primary path
2. **Multiple Outputs**: Steps can connect to multiple subsequent steps (for parallel or conditional flows)
3. **No Circular Dependencies**: System prevents creating loops that could cause infinite execution
4. **Automatic Spacing**: Connected steps maintain minimum 300px horizontal spacing

## Ordering System

### 🔢 **How Ordering Works**

#### **1. Position-Based Ordering**
```typescript
// Steps are ordered based on X position (left to right)
const sortedSteps = [...design.steps].sort(
  (a, b) => a.position.x - b.position.x
);
```

#### **2. Auto-Connection Logic**
```typescript
// Automatically connect steps that are close horizontally
const distance = Math.abs(targetStep.position.x - sourceStep.position.x);
if (distance < 400) {
  // Create automatic connection
}
```

#### **3. Manual Reordering**
- **Drag Steps**: Users can drag steps to reposition them
- **Visual Feedback**: Connections update in real-time
- **Smart Snapping**: 20px grid snapping for clean alignment

### 🎯 **Ordering Strategies**

#### **Linear Flow** (Most Common)
```
[Start] → [Step 1] → [Step 2] → [Step 3] → [End]
```
- Simple left-to-right arrangement
- Auto-connection between adjacent steps
- Perfect for basic experimental protocols

#### **Parallel Flow** (Advanced)
```
[Start] → [Parallel] → [Step A]
                    → [Step B] → [Merge] → [End]
```
- Multiple paths from one step
- Useful for simultaneous robot/wizard actions
- Visual branching with clear convergence points

#### **Conditional Flow** (Complex)
```
[Start] → [Decision] → [Path A] → [End A]
                    → [Path B] → [End B]
```
- Branching based on conditions
- Different outcomes based on participant responses
- Clear visual representation of decision points

## User Interaction Guide

### 🖱️ **Connecting Steps**

1. **Hover over Source Handle**: Right side of a step highlights
2. **Click and Drag**: Start connection from source handle
3. **Drop on Target Handle**: Left side of destination step
4. **Visual Confirmation**: Animated line appears between steps
5. **Auto-Positioning**: Target step repositions if needed

### ⌨️ **Keyboard Shortcuts**

| Shortcut | Action |
|----------|--------|
| `Delete` | Delete selected step/connection |
| `Ctrl/Cmd + D` | Duplicate selected step |
| `Ctrl/Cmd + Z` | Undo last action |
| `Ctrl/Cmd + Y` | Redo last action |
| `Space + Drag` | Pan canvas |
| `Ctrl/Cmd + Scroll` | Zoom in/out |

### 🎨 **Visual Feedback**

#### **Connection States**
- **Default**: Muted gray lines (`stroke: hsl(var(--muted-foreground))`)
- **Animated**: Flowing dashes during execution
- **Hover**: Highlighted with primary color
- **Selected**: Thicker stroke with selection indicators

#### **Handle States**
- **Default**: Primary color background
- **Hover**: Pulsing animation
- **Connecting**: Enlarged with glow effect
- **Invalid Target**: Red color with error indication

## Technical Implementation

### 🔧 **React Flow Configuration**

```typescript
<ReactFlow
  nodes={nodes}
  edges={edges}
  nodeTypes={nodeTypes}
  connectionLineType="smoothstep"
  snapToGrid={true}
  snapGrid={[20, 20]}
  defaultEdgeOptions={{
    type: "smoothstep",
    animated: true,
    style: { strokeWidth: 2 },
  }}
  onConnect={handleConnect}
  onNodesChange={handleNodesChange}
/>
```

### 🎨 **Theme Integration**

```css
/* Custom theming for React Flow components */
.react-flow__handle {
  background-color: hsl(var(--primary));
  border: 2px solid hsl(var(--background));
}

.react-flow__edge-path {
  stroke: hsl(var(--muted-foreground));
  stroke-width: 2;
}

.react-flow__connection-line {
  stroke: hsl(var(--primary));
  stroke-dasharray: 5;
}
```

### 📊 **Data Structure**

```typescript
interface FlowDesign {
  id: string;
  name: string;
  steps: FlowStep[];
  version: number;
}

interface FlowStep {
  id: string;
  type: StepType;
  name: string;
  position: { x: number; y: number };
  actions: FlowAction[];
}

// Connections are implicit through React Flow edges
interface Edge {
  id: string;
  source: string;
  target: string;
  sourceHandle: string;
  targetHandle: string;
}
```

## Best Practices

### 🎯 **For Researchers**

1. **Start Simple**: Begin with linear flows, add complexity gradually
2. **Clear Naming**: Use descriptive step names that explain their purpose
3. **Logical Flow**: Arrange steps left-to-right in execution order
4. **Group Related Steps**: Use visual proximity for related actions
5. **Test Connections**: Verify flow logic before running trials

### 🛠️ **For Developers**

1. **Handle Edge Cases**: Validate connections to prevent loops
2. **Performance**: Optimize for large flows (100+ steps)
3. **Accessibility**: Ensure keyboard navigation works properly
4. **Mobile Support**: Test touch interactions on tablets
5. **Error Recovery**: Graceful handling of malformed flows

## Advanced Features

### 🔮 **Future Enhancements**

#### **Smart Auto-Layout**
- Automatic optimal positioning of connected steps
- Hierarchical layout algorithms for complex flows
- Conflict resolution for overlapping connections

#### **Connection Types**
- **Sequential**: Normal step-to-step execution
- **Conditional**: Based on runtime conditions
- **Parallel**: Simultaneous execution paths
- **Loop**: Repeat sections based on criteria

#### **Visual Enhancements**
- **Step Previews**: Hover to see step details
- **Execution Trace**: Visual playback of completed trials
- **Error Highlighting**: Red indicators for problematic connections
- **Performance Metrics**: Timing information on connections

## Troubleshooting

### ❓ **Common Issues**

#### **Steps Won't Connect**
- Check that handles are properly positioned
- Ensure no circular dependencies
- Verify both nodes are valid connection targets

#### **Connections Disappear**
- May indicate data consistency issues
- Check that both source and target steps exist
- Verify connection IDs are unique

#### **Poor Performance**
- Large flows (50+ steps) may need optimization
- Consider pagination or virtualization
- Check for memory leaks in event handlers

### 🔧 **Debug Tools**

```typescript
// Enable React Flow debug mode
const DEBUG_MODE = process.env.NODE_ENV === 'development';

<ReactFlow
  {...props}
  onError={DEBUG_MODE ? console.error : undefined}
  onInit={DEBUG_MODE ? console.log : undefined}
/>
```

## Conclusion

The React Flow-based connection system provides HRIStudio with a professional, intuitive interface for designing complex experimental workflows. The combination of visual clarity, flexible layout options, and robust connection handling makes it the ideal solution for HRI researchers who need to create sophisticated experimental protocols.

The system successfully balances ease of use for simple linear experiments with the power needed for complex branching and parallel execution flows, making it suitable for researchers at all skill levels.