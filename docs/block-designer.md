# HRIStudio Block Designer

## Overview

The HRIStudio Block Designer is a visual programming interface for creating experiment protocols in Human-Robot Interaction research. It provides an intuitive, drag-and-drop environment where researchers can design complex experimental workflows without programming knowledge.

**Status**: Production ready - Fully implemented with database integration

## Features

### **Dense, Structured Interface**
- **Three-panel layout**: Block Library | Experiment Flow | Properties
- **Linear block sequencing** with clear top-to-bottom execution order
- **Resizable panels** to fit different workflow preferences
- **Compact, efficient design** maximizing information density

### **Visual Block System**
- **Color-coded categories** for easy identification
- **Shape-based functionality** indicating block behavior
- **Parameter preview** showing current values inline
- **Execution state indicators** during trial playback

### **Advanced Drag & Drop**
- **Powered by dnd-kit** for reliable, cross-platform operation
- **Reorder blocks** by dragging in the main sequence
- **Nest blocks** by dropping into control structures
- **Visual feedback** with drop zones and hover states
- **Touch support** for tablet and mobile devices

### **Control Flow & Nesting**
- **Control blocks** can contain other blocks for complex logic
- **Visual hierarchy** with indentation and connecting lines
- **Easy removal** from nested structures
- **Drop zones** clearly indicate where blocks can be placed

## Block Categories

### **Events** (Green - Play icon)
Entry points that trigger experiment sequences.

#### `when trial starts`
- **Shape**: Hat (distinctive top curve)
- **Purpose**: Marks the beginning of experiment execution
- **Parameters**: None
- **Usage**: Every experiment should start with an event block

### **Wizard Actions** (Purple - Users icon)
Human-operated actions performed by the experiment wizard.

#### `say`
- **Shape**: Rounded rectangle
- **Purpose**: Wizard speaks to participant
- **Parameters**: 
  - `message` (text): What the wizard should say
- **Example**: "Please take a seat and get comfortable"

#### `gesture`
- **Shape**: Rounded rectangle  
- **Purpose**: Wizard performs physical gesture
- **Parameters**:
  - `type` (select): wave, point, nod, thumbs_up
- **Example**: Wave to greet participant

### **Robot Actions** (Blue - Bot icon)
Automated behaviors performed by the robot system.

#### `say`
- **Shape**: Rounded rectangle
- **Purpose**: Robot speaks using text-to-speech
- **Parameters**:
  - `text` (text): Message for robot to speak
- **Example**: "Hello, I'm ready to help you today"

#### `move`
- **Shape**: Rounded rectangle
- **Purpose**: Robot moves in specified direction
- **Parameters**:
  - `direction` (select): forward, backward, left, right
  - `distance` (number): Distance in meters (0.1-5.0)
- **Example**: Move forward 1.5 meters

#### `look at`
- **Shape**: Rounded rectangle
- **Purpose**: Robot orients gaze toward target
- **Parameters**:
  - `target` (select): participant, object, door
- **Example**: Look at participant during conversation

### **Control Flow** (Orange - GitBranch icon)
Logic and timing blocks that control experiment flow.

#### `wait`
- **Shape**: Rounded rectangle
- **Purpose**: Pause execution for specified time
- **Parameters**:
  - `seconds` (number): Duration to wait (0.1-60)
- **Example**: Wait 3 seconds between actions

#### `repeat`
- **Shape**: Control block (C-shaped with nesting area)
- **Purpose**: Execute contained blocks multiple times
- **Parameters**:
  - `times` (number): Number of repetitions (1-20)
- **Nesting**: Can contain other blocks
- **Example**: Repeat greeting sequence 3 times

#### `if`
- **Shape**: Control block (C-shaped with nesting area)
- **Purpose**: Conditional execution based on conditions
- **Parameters**:
  - `condition` (select): participant speaks, object detected, timer expired
- **Nesting**: Can contain other blocks
- **Example**: If participant speaks, respond with acknowledgment

### **Sensors** (Green - Activity icon)
Data collection and observation tools.

#### `observe`
- **Shape**: Rounded rectangle
- **Purpose**: Record behavioral observations
- **Parameters**:
  - `what` (text): Description of what to observe
  - `duration` (number): Observation time in seconds (1-60)
- **Example**: Observe participant engagement for 10 seconds

## User Interface

### **Block Library Panel (Left)**
- **Category tabs**: Click to switch between block categories
- **Block cards**: Click to add blocks to experiment
- **Visual previews**: Icons and descriptions for each block type
- **Smooth animations**: Hover effects and visual feedback

### **Experiment Flow Panel (Middle)**
- **Linear sequence**: Blocks arranged vertically in execution order
- **Drag handles**: Grip icons for reordering blocks
- **Selection states**: Click blocks to select for editing
- **Nesting support**: Control blocks show contained blocks indented
- **Drop zones**: Dashed areas for dropping blocks into control structures

### **Properties Panel (Right)**
- **Block details**: Name, description, and icon
- **Parameter editing**: Form controls for block configuration
- **Live updates**: Changes reflected immediately in block preview
- **Type-appropriate inputs**: Text fields, numbers, dropdowns as needed

## Workflow Examples

### **Simple Linear Sequence**
```
1. [when trial starts]
2. [robot say] "Welcome to our study"
3. [wait] 2 seconds
4. [wizard say] "Please introduce yourself"
5. [observe] "participant response" for 10 seconds
```

### **Repeated Actions**
```
1. [when trial starts]
2. [robot say] "I'll demonstrate this movement 3 times"
3. [repeat] 3 times
   ├─ [robot move] forward 0.5 meters
   ├─ [wait] 1 second
   ├─ [robot move] backward 0.5 meters
   └─ [wait] 1 second
4. [wizard say] "Now you try it"
```

### **Conditional Logic**
```
1. [when trial starts]
2. [robot say] "Do you have any questions?"
3. [if] participant speaks
   ├─ [robot say] "Let me address that"
   ├─ [wait] 3 seconds
   └─ [wizard say] "Please elaborate if needed"
4. [robot say] "Let's begin the main task"
```

### **Complex Multi-Modal Interaction**
```
1. [when trial starts]
2. [robot look at] participant
3. [robot say] "Hello! I'm going to help you today"
4. [wizard gesture] wave
5. [repeat] 5 times
   ├─ [robot move] forward 0.3 meters
   ├─ [if] object detected
   │  ├─ [robot say] "I see something interesting"
   │  ├─ [robot look at] object
   │  └─ [observe] "participant attention" for 5 seconds
   └─ [wait] 2 seconds
6. [wizard say] "Great job! That completes our session"
```

## Technical Implementation

### **Data Structure**
```typescript
interface ExperimentBlock {
  id: string;                    // Unique identifier
  type: string;                  // Block type (e.g., 'robot_speak')
  category: BlockCategory;       // Visual category
  shape: BlockShape;             // Visual shape
  displayName: string;           // User-friendly name
  description: string;           // Help text
  icon: string;                  // Lucide icon name
  color: string;                 // Category color
  parameters: BlockParameter[]; // Configurable values
  children?: ExperimentBlock[];  // Nested blocks (for control)
  nestable?: boolean;           // Can contain children
  order: number;                // Sequence position
}
```

### **Plugin Architecture**
The block system supports extensible plugins for different robot platforms:

```typescript
interface PluginBlockDefinition {
  type: string;                  // Unique block identifier
  shape: BlockShape;             // Visual representation
  category: BlockCategory;       // Palette category
  displayName: string;           // User-visible name
  description: string;           // Help description
  icon: string;                  // Icon identifier
  color: string;                 // Category color
  parameters: ParameterSchema[]; // Configuration schema
  nestable?: boolean;           // Supports nesting
}
```

### **Execution Integration**
Visual blocks compile to executable trial sequences:

1. **Design Phase**: Visual blocks stored as JSON in database
2. **Compilation**: Blocks converted to execution graph
3. **Runtime**: Trial executor processes blocks sequentially
4. **Monitoring**: Real-time status updates back to visual blocks

## Best Practices

### **For Simple Experiments**
- Start with a clear event trigger (`when trial starts`)
- Use linear sequences for straightforward protocols
- Add timing blocks (`wait`) for natural pacing
- Include observation blocks for data collection
- Keep nesting minimal for clarity

### **For Complex Experiments**
- Group related actions in control blocks (`repeat`, `if`)
- Use descriptive parameter values
- Test conditional logic thoroughly before trials
- Document unusual configurations in experiment notes
- Break complex flows into smaller, testable segments

### **For Team Collaboration**
- Use consistent naming conventions across experiments
- Export and share protocol designs
- Review block sequences visually before implementation
- Maintain version history of experimental protocols
- Train team members on block meanings and usage

### **Parameter Configuration**
- Use clear, descriptive text for speech blocks
- Set appropriate timing for wait blocks (not too fast/slow)
- Choose realistic movement distances for robot actions
- Configure observation durations based on expected behaviors
- Test parameter values in pilot sessions

### **Parameters in Block Drawer**
Parameter names are currently shown as badges in the block library for preview:
- **Parameter badges**: Shows first 2 parameter names under each block
- **Overflow indicator**: Shows "+X more" for blocks with many parameters  
- **Visual preview**: Helps identify block configuration needs
- **Future enhancement**: Could support inline editing for rapid prototyping

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Delete` | Remove selected block |
| `Escape` | Deselect all blocks |
| `↑/↓ Arrow` | Navigate block selection |
| `Enter` | Edit selected block parameters |
| `Ctrl/Cmd + S` | Save experiment design |
| `Ctrl/Cmd + Z` | Undo last action |

## Accessibility Features

- **Keyboard navigation** through all interface elements
- **Screen reader support** with proper ARIA labels
- **High contrast** color schemes for visibility
- **Touch-friendly** sizing for tablet interfaces
- **Clear visual hierarchy** with consistent typography

## Database Integration

### **Storage Schema**
Experiment designs are stored in the `experiments` table:
- `visual_design` (JSONB): Complete block layout and configuration
- `execution_graph` (JSONB): Compiled execution sequence
- `plugin_dependencies` (TEXT[]): Required robot platform plugins

### **Performance Optimization**
- **GIN indexes** on JSONB columns for fast queries
- **Lazy loading** of large block libraries
- **Efficient rendering** with React virtualization
- **Minimal re-renders** using optimized state management

## Implementation Status

### **Completed Features**
- **Dense three-panel interface** with resizable panels
- **Six block categories** with color coding and icons
- **dnd-kit powered drag and drop** with nesting support
- **Control flow blocks** (repeat, if) with visual hierarchy
- **Parameter editing** in dedicated properties panel
- **Database integration** with JSONB storage
- **Breadcrumb navigation** using dashboard system
- **Plugin architecture** ready for robot platform extensions

### **Technical Implementation**
- **Database**: PostgreSQL with JSONB columns for visual designs
- **Frontend**: React with TypeScript, dnd-kit, shadcn/ui
- **State management**: React hooks with optimistic updates
- **Performance**: Efficient rendering for experiments up to 50+ blocks

## Troubleshooting

### **Common Issues**

**Blocks won't drag:**
- Ensure you're dragging from the grip handle (not the block body)
- Check that browser supports modern drag and drop APIs
- Try refreshing the page if drag state gets stuck

**Parameters not saving:**
- Click outside parameter fields to trigger save
- Check network connection for auto-save functionality
- Verify you have edit permissions for the experiment

**Control blocks not nesting:**
- Drag blocks specifically onto the dashed drop zone
- Ensure control blocks are expanded (not collapsed)
- Check that target block supports nesting

**Missing blocks in palette:**
- Verify required robot plugins are installed and active
- Check that you have access to the block category
- Refresh page to reload block registry

### **Breadcrumb Navigation** 
The block designer integrates with the existing dashboard breadcrumb system:
- **Path**: Dashboard → Experiments → [Experiment Name] → Designer
- **Header integration**: Breadcrumbs appear in dashboard header (not duplicated)
- **Context preservation**: Maintains navigation state during design sessions
- **Automatic cleanup**: Breadcrumbs reset when leaving designer

### **Performance Tips**
- Keep experiments under 50 blocks for optimal performance
- Use control blocks to organize complex sequences
- Regularly save work to prevent data loss
- Close unused browser tabs to free memory

## Development Notes

### **File Locations**
- **Main component**: `src/components/experiments/designer/EnhancedBlockDesigner.tsx`
- **Page route**: `src/app/(dashboard)/experiments/[id]/designer/page.tsx`
- **Database schema**: Enhanced experiments table with `visual_design` JSONB column
- **Documentation**: `docs/block-designer.md` (this file)

### **Key Dependencies**
- **@dnd-kit/core**: Drag and drop functionality
- **@dnd-kit/sortable**: Block reordering and nesting
- **lucide-react**: All icons throughout interface
- **shadcn/ui**: UI components and theming
- **PostgreSQL**: JSONB storage for block designs

---

*The HRIStudio Block Designer makes complex experimental protocols accessible to researchers regardless of programming background, while maintaining the flexibility needed for cutting-edge HRI research.*