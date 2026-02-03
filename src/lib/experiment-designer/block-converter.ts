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
    transport: action.execution?.transport,
    ros2: action.execution?.ros2,
    rest: action.execution?.rest,
    retryable: action.execution?.retryable,
    parameterSchemaRaw: action.parameterSchemaRaw,
    sourceKind: action.source.kind,
    category: action.category,
  };
}

// Reconstruct designer steps from database records
export function convertDatabaseToSteps(
  dbSteps: any[] // Typing as any[] because Drizzle types are complex to import here without circular deps
): ExperimentStep[] {
  // Paranoid Sort: Ensure steps are strictly ordered by index before assigning Triggers.
  // This safeguards against API returning unsorted data.
  const sortedSteps = [...dbSteps].sort((a, b) => (a.orderIndex ?? 0) - (b.orderIndex ?? 0));

  return sortedSteps.map((dbStep, idx) => {
    // console.log(`[block-converter] Step ${dbStep.name} OrderIndex:`, dbStep.orderIndex, dbStep.order_index);
    return {
      id: dbStep.id,
      name: dbStep.name,
      description: dbStep.description ?? undefined,
      type: mapDatabaseToStepType(dbStep.type),
      order: dbStep.orderIndex ?? idx, // Fallback to array index if missing
      trigger: {
        // Enforce Sequential Architecture: Validated by user requirement.
        // Index 0 is Trial Start, all others are Previous Step.
        type: idx === 0 ? "trial_start" : "previous_step",
        conditions: (dbStep.conditions as Record<string, unknown>) || {},
      },
      expanded: true, // Default to expanded in designer
      actions: (dbStep.actions || []).map((dbAction: any) =>
        convertDatabaseToAction(dbAction)
      ),
    };
  });
}

function mapDatabaseToStepType(type: string): ExperimentStep["type"] {
  switch (type) {
    case "wizard":
      return "sequential";
    case "parallel":
      return "parallel";
    case "conditional":
      return "conditional"; // Loop is also stored as conditional, distinction lost unless encoded in metadata
    default:
      return "sequential";
  }
}

export function convertDatabaseToAction(dbAction: any): ExperimentAction {
  // Reconstruct nested source object
  const source: ExperimentAction["source"] = {
    kind: (dbAction.sourceKind || dbAction.source_kind || "core") as "core" | "plugin",
    pluginId: dbAction.pluginId || dbAction.plugin_id || undefined,
    pluginVersion: dbAction.pluginVersion || dbAction.plugin_version || undefined,
    robotId: dbAction.robotId || dbAction.robot_id || undefined,
    baseActionId: dbAction.baseActionId || dbAction.base_action_id || undefined,
  };

  // Robust Inference: If properties are missing but Type suggests a plugin (e.g., "nao6-ros2.say_text"),
  // assume/infer the pluginId to ensure validation passes.
  if (dbAction.type && dbAction.type.includes(".") && !source.pluginId) {
    const parts = dbAction.type.split(".");
    if (parts.length === 2) {
      source.kind = "plugin";
      source.pluginId = parts[0];
      // Fallback robotId if missing
      if (!source.robotId) source.robotId = parts[0];
    }
  }

  // Reconstruct execution object
  const execution: ExecutionDescriptor = {
    transport: dbAction.transport as ExecutionDescriptor["transport"],
    ros2: dbAction.ros2 as ExecutionDescriptor["ros2"],
    rest: dbAction.rest as ExecutionDescriptor["rest"],
    retryable: dbAction.retryable ?? false,
  };

  return {
    id: dbAction.id,
    name: dbAction.name,
    description: dbAction.description ?? undefined,
    type: dbAction.type,
    category: dbAction.category ?? "general",
    parameters: (dbAction.parameters as Record<string, unknown>) || {},
    source,
    execution,
    parameterSchemaRaw: dbAction.parameterSchemaRaw,
  };
}
