# Experiment Designer

## Overview

The Experiment Designer is a core feature of HRIStudio that enables researchers to create and configure robot experiments using a visual, flow-based interface. It supports drag-and-drop functionality, real-time updates, and integration with the plugin system.

## Architecture

### Core Components

```typescript
interface ExperimentDesignerProps {
  className?: string;
  defaultSteps?: Step[];
  onChange?: (steps: Step[]) => void;
  readOnly?: boolean;
}

export function ExperimentDesigner({
  className,
  defaultSteps = [],
  onChange,
  readOnly = false,
}: ExperimentDesignerProps) {
  // Implementation
}
```

### Data Types

```typescript
export type ActionType = 
  | "move"      // Robot movement
  | "speak"     // Robot speech
  | "wait"      // Wait for a duration
  | "input"     // Wait for user input
  | "gesture"   // Robot gesture
  | "record"    // Start/stop recording
  | "condition" // Conditional branching
  | "loop";     // Repeat actions

export interface Action {
  id: string;
  type: ActionType;
  parameters: Record<string, any>;
  order: number;
}

export interface Step {
  id: string;
  title: string;
  description?: string;
  actions: Action[];
  order: number;
}

export interface Experiment {
  id: number;
  studyId: number;
  title: string;
  description?: string;
  version: number;
  status: "draft" | "active" | "archived";
  steps: Step[];
  createdAt: Date;
  updatedAt: Date;
}
```

## Visual Components

### Action Node

```typescript
interface ActionNodeData {
  type: string;
  parameters: Record<string, any>;
  onChange?: (parameters: Record<string, any>) => void;
}

export const ActionNode = memo(({ data, selected }: NodeProps<ActionNodeData>) => {
  const [configOpen, setConfigOpen] = useState(false);
  const [isHovered, setIsHovered] = useState(false);
  const actionConfig = AVAILABLE_ACTIONS.find((a) => a.type === data.type);

  return (
    <motion.div
      initial={{ scale: 0.8, opacity: 0 }}
      animate={{ scale: 1, opacity: 1 }}
      transition={{ duration: 0.2 }}
      className={cn(
        "relative",
        "before:absolute before:inset-[-2px] before:rounded-xl before:bg-gradient-to-br",
        selected && "before:from-primary/50 before:to-primary/20"
      )}
    >
      <Card>
        <CardHeader>
          <div className="flex items-center gap-2">
            <div className="flex h-8 w-8 items-center justify-center rounded-md bg-gradient-to-br">
              {actionConfig?.icon}
            </div>
            <CardTitle>{actionConfig?.title}</CardTitle>
          </div>
        </CardHeader>
        <CardContent>
          <CardDescription>{actionConfig?.description}</CardDescription>
        </CardContent>
      </Card>
    </motion.div>
  );
});
```

### Flow Edge

```typescript
export function FlowEdge({
  id,
  sourceX,
  sourceY,
  targetX,
  targetY,
  sourcePosition,
  targetPosition,
  style = {},
  markerEnd,
}: EdgeProps) {
  const [edgePath] = getBezierPath({
    sourceX,
    sourceY,
    sourcePosition,
    targetX,
    targetY,
    targetPosition,
  });

  return (
    <>
      <BaseEdge path={edgePath} markerEnd={markerEnd} style={style} />
      <motion.path
        id={id}
        style={{
          strokeWidth: 3,
          fill: "none",
          stroke: "hsl(var(--primary))",
          strokeDasharray: "5,5",
          opacity: 0.5,
        }}
        d={edgePath}
        animate={{
          strokeDashoffset: [0, -10],
        }}
        transition={{
          duration: 1,
          repeat: Infinity,
          ease: "linear",
        }}
      />
    </>
  );
}
```

## Action Configuration

### Available Actions

```typescript
export const AVAILABLE_ACTIONS: ActionConfig[] = [
  {
    type: "move",
    title: "Move Robot",
    description: "Move the robot to a specific position",
    icon: <Move className="h-4 w-4" />,
    defaultParameters: {
      position: { x: 0, y: 0, z: 0 },
      speed: 1,
      easing: "linear",
    },
  },
  {
    type: "speak",
    title: "Robot Speech",
    description: "Make the robot say something",
    icon: <MessageSquare className="h-4 w-4" />,
    defaultParameters: {
      text: "",
      speed: 1,
      pitch: 1,
      volume: 1,
    },
  },
  // Additional actions...
];
```

### Parameter Configuration Dialog

```typescript
interface ActionConfigDialogProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  type: ActionType;
  parameters: Record<string, any>;
  onSubmit: (parameters: Record<string, any>) => void;
}

export function ActionConfigDialog({
  open,
  onOpenChange,
  type,
  parameters,
  onSubmit,
}: ActionConfigDialogProps) {
  const actionConfig = AVAILABLE_ACTIONS.find(a => a.type === type);
  
  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Configure {actionConfig?.title}</DialogTitle>
          <DialogDescription>
            {actionConfig?.description}
          </DialogDescription>
        </DialogHeader>
        <Form>
          {/* Parameter fields */}
        </Form>
      </DialogContent>
    </Dialog>
  );
}
```

## Database Schema

```typescript
export const experiments = createTable("experiment", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id")
    .notNull()
    .references(() => studies.id, { onDelete: "cascade" }),
  title: varchar("title", { length: 256 }).notNull(),
  description: text("description"),
  version: integer("version").notNull().default(1),
  status: experimentStatusEnum("status").notNull().default("draft"),
  steps: jsonb("steps").$type<Step[]>().default([]),
  createdById: varchar("created_by", { length: 255 })
    .notNull()
    .references(() => users.id),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});
```

## Integration with Plugin System

### Action Transformation

```typescript
interface ActionTransform {
  type: "direct" | "transform";
  transformFn?: string;
  map?: Record<string, string>;
}

function transformActionParameters(
  parameters: Record<string, any>,
  transform: ActionTransform
): unknown {
  if (transform.type === "direct") {
    return parameters;
  }

  const transformFn = getTransformFunction(transform.transformFn!);
  return transformFn(parameters);
}
```

### Plugin Action Integration

```typescript
function getAvailableActions(plugin: RobotPlugin): ActionConfig[] {
  return plugin.actions.map(action => ({
    type: action.type,
    title: action.title,
    description: action.description,
    icon: getActionIcon(action.type),
    defaultParameters: getDefaultParameters(action.parameters),
    transform: action.ros2?.payloadMapping,
  }));
}
```

## User Interface Features

### Drag and Drop

```typescript
function onDragStart(event: DragEvent, nodeType: string) {
  event.dataTransfer.setData("application/reactflow", nodeType);
  event.dataTransfer.effectAllowed = "move";
}

function onDrop(event: DragEvent) {
  event.preventDefault();

  const type = event.dataTransfer.getData("application/reactflow");
  const position = project({
    x: event.clientX,
    y: event.clientY,
  });

  const newNode = {
    id: getId(),
    type,
    position,
    data: { label: `${type} node` },
  };

  setNodes((nds) => nds.concat(newNode));
}
```

### Step Organization

```typescript
function reorderSteps(steps: Step[], sourceIndex: number, targetIndex: number): Step[] {
  const result = Array.from(steps);
  const [removed] = result.splice(sourceIndex, 1);
  result.splice(targetIndex, 0, removed);

  return result.map((step, index) => ({
    ...step,
    order: index,
  }));
}
```

## Best Practices

1. **Performance:**
   - Use React.memo for expensive components
   - Implement virtualization for large flows
   - Optimize drag and drop operations

2. **User Experience:**
   - Provide clear visual feedback
   - Implement undo/redo functionality
   - Show validation errors inline

3. **Data Management:**
   - Validate experiment data
   - Implement auto-save
   - Version control experiments

4. **Error Handling:**
   - Validate action parameters
   - Handle plugin loading errors
   - Provide clear error messages

## Future Enhancements

1. **Advanced Flow Control:**
   - Conditional branching
   - Parallel execution
   - Loop constructs

2. **Visual Improvements:**
   - Custom node themes
   - Animation preview
   - Mini-map navigation

3. **Collaboration:**
   - Real-time collaboration
   - Comment system
   - Version history

4. **Analysis Tools:**
   - Flow validation
   - Performance analysis
   - Debug tools 