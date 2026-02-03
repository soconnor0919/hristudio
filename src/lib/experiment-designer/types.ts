// Core experiment designer types

export type StepType = "sequential" | "parallel" | "conditional" | "loop";

export type ActionCategory = "wizard" | "robot" | "observation" | "control";

export type ActionType =
  | "wizard_speak"
  | "wizard_gesture"
  | "robot_move"
  | "robot_speak"
  | "wait"
  | "observe"
  | "collect_data"
  // Namespaced plugin action types will use pattern: pluginId.actionId
  | (string & {});

export type TriggerType =
  | "trial_start"
  | "participant_action"
  | "timer"
  | "previous_step";

export interface ActionParameter {
  id: string;
  name: string;
  type: "text" | "number" | "select" | "boolean";
  placeholder?: string;
  options?: string[];
  min?: number;
  max?: number;
  value?: string | number | boolean;
  required?: boolean;
  description?: string;
  step?: number; // numeric increment if relevant
}

export interface ActionDefinition {
  id: string;
  type: ActionType;
  name: string;
  description: string;
  category: ActionCategory;
  icon: string;
  color: string;
  parameters: ActionParameter[];
  source: {
    kind: "core" | "plugin";
    pluginId?: string;
    pluginVersion?: string;
    robotId?: string | null;
    baseActionId?: string; // original internal action id inside plugin repo
  };
  execution?: ExecutionDescriptor;
  parameterSchemaRaw?: unknown; // snapshot of original schema for validation/audit
  nestable?: boolean; // If true, this action can contain child actions
}

export interface ExperimentAction {
  id: string;
  type: string; // e.g. "wizard_speak", "robot_move"
  name: string;
  description?: string; // Optional description
  parameters: Record<string, unknown>;
  duration?: number; // Estimated duration in seconds
  category: ActionCategory;
  // Provenance (where did this come from?)
  source: {
    kind: "core" | "plugin";
    pluginId?: string;
    pluginVersion?: string;
    robotId?: string | null;
    baseActionId?: string;
  };
  // Execution (how do we run this?)
  execution?: ExecutionDescriptor;

  // Snapshot of parameter schema at the time of addition (for drift detection)
  parameterSchemaRaw?: unknown;

  // Nested actions (control flow)
  children?: ExperimentAction[];
}

export interface StepTrigger {
  type: TriggerType;
  conditions: Record<string, unknown>;
}

export interface ExperimentStep {
  id: string;
  name: string;
  description?: string;
  type: StepType;
  order: number;
  trigger: StepTrigger;
  actions: ExperimentAction[];
  estimatedDuration?: number;
  expanded: boolean;
}

export interface ExperimentDesign {
  id: string;
  name: string;
  description: string;
  steps: ExperimentStep[];
  version: number;
  lastSaved: Date;
  compiledAt?: Date; // when an execution plan was compiled
  integrityHash?: string; // hash of action definitions for reproducibility
}

// Trigger options for UI
export const TRIGGER_OPTIONS = [
  { value: "trial_start" as const, label: "Trial starts" },
  { value: "participant_action" as const, label: "Participant acts" },
  { value: "timer" as const, label: "After timer" },
  { value: "previous_step" as const, label: "Previous step completes" },
];

// Step type options for UI
// IMPORTANT: Steps should ALWAYS execute sequentially
// Parallel execution, conditionals, and loops should be implemented via control flow ACTIONS
export const STEP_TYPE_OPTIONS = [
  {
    value: "sequential" as const,
    label: "Sequential",
    description: "Actions run one after another (enforced for all steps)",
  },
];

// Execution descriptors (appended)

export interface ExecutionDescriptor {
  transport: "ros2" | "rest" | "internal";
  timeoutMs?: number;
  retryable?: boolean;
  ros2?: Ros2Execution;
  rest?: RestExecution;
}

export interface Ros2Execution {
  topic?: string;
  messageType?: string;
  service?: string;
  action?: string;
  qos?: {
    reliability?: string;
    durability?: string;
    history?: string;
    depth?: number;
  };
  payloadMapping?: unknown; // mapping definition retained for transform at runtime
}

export interface RestExecution {
  method: "GET" | "POST" | "PUT" | "PATCH" | "DELETE";
  path: string;
  headers?: Record<string, string>;
  query?: Record<string, string | number | boolean>;
  bodyTemplate?: unknown;
}
