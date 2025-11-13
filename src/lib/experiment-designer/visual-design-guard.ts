import { z } from "zod";
import type {
  ExperimentStep,
  ExperimentAction,
  StepType,
  TriggerType,
  ActionCategory,
  ExecutionDescriptor,
} from "./types";

/**
 * Visual Design Guard
 *
 * Provides a robust Zod-based parsing/normalization pipeline that:
 *  - Accepts a loosely-typed visualDesign.steps payload coming from the client
 *  - Normalizes and validates it into strongly typed arrays for internal processing
 *  - Strips unknown fields (preserves only what we rely on)
 *  - Ensures provenance + execution descriptors are structurally sound
 *
 * This replaces ad-hoc runtime filtering in the experiments update mutation.
 *
 * Usage:
 *   const { steps, issues } = parseVisualDesignSteps(rawSteps);
 *   if (issues.length) -> reject request
 *   else -> steps (ExperimentStep[]) is now safe for conversion & compilation
 */

// Enumerations (reuse domain model semantics without hard binding to future expansions)
const stepTypeEnum = z.enum(["sequential", "parallel", "conditional", "loop"]);
const triggerTypeEnum = z.enum([
  "trial_start",
  "participant_action",
  "timer",
  "previous_step",
]);
const actionCategoryEnum = z.enum([
  "wizard",
  "robot",
  "observation",
  "control",
]);

// Provenance
const actionSourceSchema = z
  .object({
    kind: z.enum(["core", "plugin"]),
    pluginId: z.string().min(1).optional(),
    pluginVersion: z.string().min(1).optional(),
    robotId: z.string().min(1).nullable().optional(),
    baseActionId: z.string().min(1).optional(),
  })
  .strict();

// Execution descriptor
const executionDescriptorSchema = z
  .object({
    transport: z.enum(["ros2", "rest", "internal"]),
    timeoutMs: z.number().int().positive().optional(),
    retryable: z.boolean().optional(),
    ros2: z
      .object({
        topic: z.string().min(1).optional(),
        messageType: z.string().min(1).optional(),
        service: z.string().min(1).optional(),
        action: z.string().min(1).optional(),
        qos: z
          .object({
            reliability: z.string().optional(),
            durability: z.string().optional(),
            history: z.string().optional(),
            depth: z.number().int().optional(),
          })
          .strict()
          .optional(),
        payloadMapping: z.unknown().optional(),
      })
      .strict()
      .optional(),
    rest: z
      .object({
        method: z.enum(["GET", "POST", "PUT", "PATCH", "DELETE"]),
        path: z.string().min(1),
        headers: z.record(z.string(), z.string()).optional(),
      })
      .strict()
      .optional(),
  })
  .strict();

// Action parameter snapshot is a free-form structure retained for audit
const parameterSchemaRawSchema = z.unknown().optional();

// Action schema (loose input → normalized internal)
const visualActionInputSchema = z
  .object({
    id: z.string().min(1),
    type: z.string().min(1),
    name: z.string().min(1),
    category: actionCategoryEnum.optional(),
    parameters: z.record(z.string(), z.unknown()).default({}),
    source: actionSourceSchema.optional(),
    execution: executionDescriptorSchema.optional(),
    parameterSchemaRaw: parameterSchemaRawSchema,
  })
  .strict();

// Trigger schema
const triggerSchema = z
  .object({
    type: triggerTypeEnum,
    conditions: z.record(z.string(), z.unknown()).default({}),
  })
  .strict();

// Step schema
const visualStepInputSchema = z
  .object({
    id: z.string().min(1),
    name: z.string().min(1),
    description: z.string().optional(),
    type: stepTypeEnum,
    order: z.number().int().nonnegative().optional(),
    trigger: triggerSchema.optional(),
    actions: z.array(visualActionInputSchema),
    expanded: z.boolean().optional(),
  })
  .strict();

// Array schema root
const visualDesignStepsSchema = z.array(visualStepInputSchema);

/**
 * Parse & normalize raw steps payload.
 */
export function parseVisualDesignSteps(raw: unknown): {
  steps: ExperimentStep[];
  issues: string[];
} {
  const issues: string[] = [];

  const parsed = visualDesignStepsSchema.safeParse(raw);
  if (!parsed.success) {
    const zodErr = parsed.error;
    issues.push(
      ...zodErr.issues.map(
        (issue) =>
          `steps${
            issue.path.length ? "." + issue.path.join(".") : ""
          }: ${issue.message} (code=${issue.code})`,
      ),
    );
    return { steps: [], issues };
  }

  // Normalize to internal ExperimentStep[] shape
  const inputSteps = parsed.data;

  const normalized: ExperimentStep[] = inputSteps.map((s, idx) => {
    const actions: ExperimentAction[] = s.actions.map((a) => {
      // Default provenance
      const source: {
        kind: "core" | "plugin";
        pluginId?: string;
        pluginVersion?: string;
        robotId?: string | null;
        baseActionId?: string;
      } = a.source
        ? {
            kind: a.source.kind,
            pluginId: a.source.pluginId,
            pluginVersion: a.source.pluginVersion,
            robotId: a.source.robotId ?? null,
            baseActionId: a.source.baseActionId,
          }
        : { kind: "core" };

      // Default execution
      const execution: ExecutionDescriptor = a.execution
        ? {
            transport: a.execution.transport,
            timeoutMs: a.execution.timeoutMs,
            retryable: a.execution.retryable,
            ros2: a.execution.ros2,
            rest: a.execution.rest
              ? {
                  method: a.execution.rest.method,
                  path: a.execution.rest.path,
                  headers: a.execution.rest.headers
                    ? Object.fromEntries(
                        Object.entries(a.execution.rest.headers).filter(
                          (kv): kv is [string, string] =>
                            typeof kv[1] === "string",
                        ),
                      )
                    : undefined,
                }
              : undefined,
          }
        : { transport: "internal" };

      return {
        id: a.id,
        type: a.type, // dynamic (pluginId.actionId)
        name: a.name,
        parameters: a.parameters ?? {},
        duration: undefined,
        category: (a.category ?? "wizard") as ActionCategory,
        source: {
          kind: source.kind,
          pluginId: source.kind === "plugin" ? source.pluginId : undefined,
          pluginVersion:
            source.kind === "plugin" ? source.pluginVersion : undefined,
          robotId: source.kind === "plugin" ? (source.robotId ?? null) : null,
          baseActionId:
            source.kind === "plugin" ? source.baseActionId : undefined,
        },
        execution,
        parameterSchemaRaw: a.parameterSchemaRaw,
      };
    });

    // Construct step
    return {
      id: s.id,
      name: s.name,
      description: s.description,
      type: s.type as StepType,
      order: typeof s.order === "number" ? s.order : idx,
      trigger: {
        type: (s.trigger?.type ?? "previous_step") as TriggerType,
        conditions: s.trigger?.conditions ?? {},
      },
      actions,
      estimatedDuration: undefined,
      expanded: s.expanded ?? true,
    };
  });

  // Basic structural checks
  const seenStepIds = new Set<string>();
  for (const st of normalized) {
    if (seenStepIds.has(st.id)) {
      issues.push(`Duplicate step id: ${st.id}`);
    }
    seenStepIds.add(st.id);
    if (!st.actions.length) {
      issues.push(`Step "${st.name}" has no actions`);
    }
    const seenActionIds = new Set<string>();
    for (const act of st.actions) {
      if (seenActionIds.has(act.id)) {
        issues.push(`Duplicate action id in step "${st.name}": ${act.id}`);
      }
      seenActionIds.add(act.id);
      if (!act.source.kind) {
        issues.push(`Action "${act.id}" missing source.kind`);
      }
      if (!act.execution.transport) {
        issues.push(`Action "${act.id}" missing execution transport`);
      }
    }
  }

  return { steps: normalized, issues };
}

/**
 * Estimate aggregate duration (in seconds) from normalized steps.
 * Uses simple additive heuristic: sum each step's summed action durations
 * if present; falls back to rough defaults for certain action patterns.
 */
export function estimateDesignDurationSeconds(steps: ExperimentStep[]): number {
  let total = 0;
  for (const step of steps) {
    let stepSum = 0;
    for (const action of step.actions) {
      const t = classifyDuration(action);
      stepSum += t;
    }
    total += stepSum;
  }
  return Math.max(1, Math.round(total));
}

function classifyDuration(action: ExperimentAction): number {
  // Heuristic mapping (could be evolved to plugin-provided estimates)
  switch (true) {
    case action.type.startsWith("wizard_speak"):
    case action.type.startsWith("robot_speak"): {
      const text = action.parameters.text as string | undefined;
      if (text && text.length > 0) {
        return Math.max(2, Math.round(text.length / 10));
      }
      return 3;
    }
    case action.type.startsWith("wait"): {
      const d = action.parameters.duration as number | undefined;
      return d && d > 0 ? d : 2;
    }
    case action.type.startsWith("robot_move"):
      return 5;
    case action.type.startsWith("wizard_gesture"):
      return 2;
    case action.type.startsWith("observe"): {
      const d = action.parameters.duration as number | undefined;
      return d && d > 0 ? d : 5;
    }
    default:
      return 2;
  }
}

/**
 * Convenience wrapper: validates, returns steps or throws with issues attached.
 */
export function assertVisualDesignSteps(raw: unknown): ExperimentStep[] {
  const { steps, issues } = parseVisualDesignSteps(raw);
  if (issues.length) {
    const err = new Error(
      `Visual design validation failed:\n- ${issues.join("\n- ")}`,
    );
    (err as { issues?: string[] }).issues = issues;
    throw err;
  }
  return steps;
}
