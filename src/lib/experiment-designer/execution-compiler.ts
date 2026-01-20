/**
 * Execution Compiler Utilities
 *
 * Purpose:
 *  - Produce a deterministic execution graph snapshot from the visual design
 *  - Generate an integrity hash capturing provenance & structural identity
 *  - Extract normalized plugin dependency list (pluginId@version)
 *
 * These utilities are used on the server prior to saving an experiment so that
 * trial execution can rely on an immutable compiled artifact. This helps ensure
 * reproducibility by decoupling future plugin updates from already designed
 * experiment protocols.
 *
 * NOTE:
 *  - This module intentionally performs only pure / synchronous operations.
 *  - Any plugin resolution or database queries should happen in a higher layer
 *    before invoking these functions.
 */

import type {
  ExperimentDesign,
  ExperimentStep,
  ExperimentAction,
  ExecutionDescriptor,
} from "./types";

/* ---------- Public Types ---------- */

export interface CompiledExecutionGraph {
  version: number;
  generatedAt: string; // ISO timestamp
  steps: CompiledExecutionStep[];
  pluginDependencies: string[];
  hash: string;
}

export interface CompiledExecutionStep {
  id: string;
  name: string;
  order: number;
  type: ExperimentStep["type"];
  trigger: {
    type: string;
    conditions: Record<string, unknown>;
  };
  actions: CompiledExecutionAction[];
  estimatedDuration?: number;
}

export interface CompiledExecutionAction {
  id: string;
  name: string;
  type: string;
  category: string;
  provenance: {
    sourceKind: "core" | "plugin";
    pluginId?: string;
    pluginVersion?: string;
    robotId?: string | null;
    baseActionId?: string;
  };
  execution: ExecutionDescriptor;
  parameters: Record<string, unknown>;
  parameterSchemaRaw?: unknown;
  timeout?: number;
  retryable?: boolean;
  children?: CompiledExecutionAction[];
}

/* ---------- Compile Entry Point ---------- */

/**
 * Compile an ExperimentDesign into a reproducible execution graph + hash.
 */
export function compileExecutionDesign(
  design: ExperimentDesign,
  opts: { hashAlgorithm?: "sha256" | "sha1" } = {},
): CompiledExecutionGraph {
  const pluginDependencies = collectPluginDependencies(design);

  const compiledSteps: CompiledExecutionStep[] = design.steps
    .slice()
    .sort((a, b) => a.order - b.order)
    .map((step) => compileStep(step));

  const structuralSignature = buildStructuralSignature(
    design,
    compiledSteps,
    pluginDependencies,
  );

  const hash = stableHash(structuralSignature, opts.hashAlgorithm ?? "sha256");

  return {
    version: 1,
    generatedAt: new Date().toISOString(),
    steps: compiledSteps,
    pluginDependencies,
    hash,
  };
}

/* ---------- Step / Action Compilation ---------- */

function compileStep(step: ExperimentStep): CompiledExecutionStep {
  const compiledActions: CompiledExecutionAction[] = step.actions.map(
    (action, index) => compileAction(action, index),
  );

  return {
    id: step.id,
    name: step.name,
    order: step.order,
    type: step.type,
    trigger: {
      type: step.trigger.type,
      conditions: step.trigger.conditions ?? {},
    },
    actions: compiledActions,
    estimatedDuration: step.estimatedDuration,
  };
}

function compileAction(
  action: ExperimentAction,
  _index: number, // index currently unused (reserved for future ordering diagnostics)
): CompiledExecutionAction {
  return {
    id: action.id,
    name: action.name,
    type: action.type,
    category: action.category,
    provenance: {
      sourceKind: action.source.kind,
      pluginId: action.source.pluginId,
      pluginVersion: action.source.pluginVersion,
      robotId: action.source.robotId,
      baseActionId: action.source.baseActionId,
    },
    execution: action.execution!, // Assumes validation passed
    parameters: action.parameters,
    parameterSchemaRaw: action.parameterSchemaRaw,
    timeout: action.execution?.timeoutMs,
    retryable: action.execution?.retryable,
    children: action.children?.map((child, i) => compileAction(child, i)),
  };
}

/* ---------- Plugin Dependency Extraction ---------- */

export function collectPluginDependencies(design: ExperimentDesign): string[] {
  const set = new Set<string>();
  for (const step of design.steps) {
    collectDependenciesFromActions(step.actions, set);
  }
  return Array.from(set).sort();
}
// Helper to recursively collect from actions list directly would be cleaner
function collectDependenciesFromActions(actions: ExperimentAction[], set: Set<string>) {
  for (const action of actions) {
    if (action.source.kind === "plugin" && action.source.pluginId) {
      const versionPart = action.source.pluginVersion
        ? `@${action.source.pluginVersion}`
        : "";
      set.add(`${action.source.pluginId}${versionPart}`);
    }
    if (action.children) {
      collectDependenciesFromActions(action.children, set);
    }
  }
}

/* ---------- Integrity Hash Generation ---------- */

/**
 * Build a minimal, deterministic JSON-serializable representation capturing:
 *  - Step ordering, ids, types, triggers
 *  - Action ordering, ids, types, provenance, execution transport, parameters (keys only for hash)
 *  - Plugin dependency list
 *
 * Parameter values are not fully included (only key presence) to avoid hash churn
 * on mutable text fields while preserving structural identity. If full parameter
 * value hashing is desired, adjust `summarizeParametersForHash`.
 */
function buildStructuralSignature(
  design: ExperimentDesign,
  steps: CompiledExecutionStep[],
  pluginDependencies: string[],
): unknown {
  return {
    experimentId: design.id,
    version: design.version,
    steps: steps.map((s) => ({
      id: s.id,
      order: s.order,
      type: s.type,
      trigger: {
        type: s.trigger.type,
        // Include condition keys only for stability
        conditionKeys: Object.keys(s.trigger.conditions).sort(),
      },
      actions: s.actions.map((a) => ({
        id: a.id,
        type: a.type,
        category: a.category,
        provenance: a.provenance,
        transport: a.execution.transport,
        timeout: a.timeout,
        retryable: a.retryable ?? false,
        parameterKeys: summarizeParametersForHash(a.parameters),
        children: a.children?.map(c => ({
          id: c.id,
          // Recurse structural signature for children
          type: c.type,
          parameterKeys: summarizeParametersForHash(c.parameters),
        })),
      })),
    })),
    pluginDependencies,
  };
}

function summarizeParametersForHash(params: Record<string, unknown>): string[] {
  return Object.keys(params).sort();
}

/* ---------- Stable Hash Implementation ---------- */

/**
 * Simple stable hash using built-in Web Crypto if available; falls back
 * to a lightweight JS implementation (FNV-1a) for environments without
 * crypto.subtle (e.g. some test runners).
 *
 * This is synchronous; if crypto.subtle is present it still uses
 * a synchronous wrapper by blocking on the Promise with deasync style
 * simulation (not implemented) so we default to FNV-1a here for portability.
 */
function stableHash(value: unknown, algorithm: "sha256" | "sha1"): string {
  // Use a deterministic JSON stringify
  const json = JSON.stringify(value);
  // FNV-1a 64-bit (represented as hex)
  let hashHigh = 0xcbf29ce4;
  let hashLow = 0x84222325; // Split 64-bit for simple JS accumulation

  for (let i = 0; i < json.length; i++) {
    const c = json.charCodeAt(i);
    // XOR low part
    hashLow ^= c;
    // 64-bit FNV prime: 1099511628211 -> split multiply
    // (hash * prime) mod 2^64
    // Multiply low
    let low =
      (hashLow & 0xffff) * 0x1b3 +
      (((hashLow >>> 16) * 0x1b3) & 0xffff) * 0x10000;
    // Include high
    low +=
      ((hashHigh & 0xffff) * 0x1b3 +
        (((hashHigh >>> 16) * 0x1b3) & 0xffff) * 0x10000) &
      0xffffffff;
    // Rotate values (approximate 64-bit handling)
    hashHigh ^= low >>> 13;
    hashHigh &= 0xffffffff;
    hashLow = low & 0xffffffff;
  }

  // Combine into hex; algorithm param reserved for future (differing strategies)
  const highHex = (hashHigh >>> 0).toString(16).padStart(8, "0");
  const lowHex = (hashLow >>> 0).toString(16).padStart(8, "0");
  return `${algorithm}-${highHex}${lowHex}`;
}

/* ---------- Validation Helpers (Optional Use) ---------- */

/**
 * Lightweight structural sanity checks prior to compilation.
 * Returns array of issues; empty array means pass.
 */
export function validateDesignStructure(design: ExperimentDesign): string[] {
  const issues: string[] = [];
  if (!design.steps.length) {
    issues.push("No steps defined");
  }
  const seenStepIds = new Set<string>();
  for (const step of design.steps) {
    if (seenStepIds.has(step.id)) {
      issues.push(`Duplicate step id: ${step.id}`);
    } else {
      seenStepIds.add(step.id);
    }
    if (!step.actions.length) {
      issues.push(`Step "${step.name}" has no actions`);
    }
    const seenActionIds = new Set<string>();
    for (const action of step.actions) {
      if (seenActionIds.has(action.id)) {
        issues.push(`Duplicate action id in step "${step.name}": ${action.id}`);
      } else {
        seenActionIds.add(action.id);
      }
      if (!action.type) {
        issues.push(`Action "${action.id}" missing type`);
      }
      if (!action.source?.kind) {
        issues.push(`Action "${action.id}" missing provenance`);
      }
      if (!action.execution?.transport) {
        issues.push(`Action "${action.id}" missing execution transport`);
      }
    }
  }
  return issues;
}

/**
 * High-level convenience wrapper: validate + compile; throws on issues.
 */
export function validateAndCompile(
  design: ExperimentDesign,
): CompiledExecutionGraph {
  const issues = validateDesignStructure(design);
  if (issues.length) {
    const error = new Error(
      `Design validation failed:\n- ${issues.join("\n- ")}`,
    );
    (error as { issues?: string[] }).issues = issues;
    throw error;
  }
  return compileExecutionDesign(design);
}
