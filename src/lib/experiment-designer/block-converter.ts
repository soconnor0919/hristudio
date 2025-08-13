import type {
  ExperimentStep,
  ExperimentAction,
  ExecutionDescriptor,
} from "./types";

// Convert step-based design to database records
export function convertStepsToDatabase(
  steps: ExperimentStep[],
): ConvertedStep[] {
  return steps.map((step, index) => ({
    name: step.name,
    description: step.description,
    type: mapStepTypeToDatabase(step.type),
    orderIndex: index,
    durationEstimate: calculateStepDuration(step.actions),
    required: true,
    conditions: step.trigger.conditions,
    actions: step.actions.map((action, actionIndex) =>
      convertActionToDatabase(action, actionIndex),
    ),
  }));
}

// Map designer step types to database step types
function mapStepTypeToDatabase(
  stepType: ExperimentStep["type"],
): "wizard" | "robot" | "parallel" | "conditional" {
  switch (stepType) {
    case "sequential":
      return "wizard";
    case "parallel":
      return "parallel";
    case "conditional":
    case "loop":
      return "conditional";
    default:
      return "wizard";
  }
}

// Calculate step duration from actions
function calculateStepDuration(actions: ExperimentAction[]): number {
  let total = 0;

  for (const action of actions) {
    switch (action.type) {
      case "wizard_speak":
      case "robot_speak":
        // Estimate based on text length if available
        const text = action.parameters.text as string;
        if (text) {
          total += Math.max(2, text.length / 10); // ~10 chars per second
        } else {
          total += 3;
        }
        break;
      case "wait":
        total += (action.parameters.duration as number) || 2;
        break;
      case "robot_move":
        total += 5; // Movement takes longer
        break;
      case "wizard_gesture":
        total += 2;
        break;
      case "observe":
        total += (action.parameters.duration as number) || 5;
        break;
      default:
        total += 2; // Default duration
    }
  }

  return Math.max(1, Math.round(total));
}

// Estimate action timeout
function estimateActionTimeout(action: ExperimentAction): number {
  switch (action.type) {
    case "wizard_speak":
    case "robot_speak":
      return 30;
    case "robot_move":
      return 60;
    case "wait": {
      const duration = action.parameters.duration as number | undefined;
      return (duration ?? 2) + 10; // Add buffer
    }
    case "observe":
      return 120; // Observation can take longer
    default:
      return 30;
  }
}

// Database conversion types (same as before)
export interface ConvertedStep {
  name: string;
  description?: string;
  type: "wizard" | "robot" | "parallel" | "conditional";
  orderIndex: number;
  durationEstimate?: number;
  required: boolean;
  conditions: Record<string, unknown>;
  actions: ConvertedAction[];
}

export interface ConvertedAction {
  name: string;
  description?: string;
  type: string;
  orderIndex: number;
  parameters: Record<string, unknown>;
  timeout?: number;
  // Provenance & execution metadata (flattened for now)
  pluginId?: string;
  pluginVersion?: string;
  robotId?: string | null;
  baseActionId?: string;
  transport?: ExecutionDescriptor["transport"];
  ros2?: ExecutionDescriptor["ros2"];
  rest?: ExecutionDescriptor["rest"];
  retryable?: boolean;
  parameterSchemaRaw?: unknown;
  sourceKind?: "core" | "plugin";
  category?: string;
}

// Deprecated legacy compatibility function removed to eliminate unsafe any usage.
// If needed, implement a proper migration path elsewhere.

/**
 * Convert a single experiment action into a ConvertedAction (DB shape),
 * preserving provenance and execution metadata for reproducibility.
 */
export function convertActionToDatabase(
  action: ExperimentAction,
  orderIndex: number,
): ConvertedAction {
  return {
    name: action.name,
    description: `${action.type} action`,
    type: action.type,
    orderIndex,
    parameters: action.parameters,
    timeout: estimateActionTimeout(action),
    pluginId: action.source.pluginId,
    pluginVersion: action.source.pluginVersion,
    robotId: action.source.robotId,
    baseActionId: action.source.baseActionId,
    transport: action.execution.transport,
    ros2: action.execution.ros2,
    rest: action.execution.rest,
    retryable: action.execution.retryable,
    parameterSchemaRaw: action.parameterSchemaRaw,
    sourceKind: action.source.kind,
    category: action.category,
  };
}
