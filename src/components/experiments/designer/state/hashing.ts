/**
 * Hashing utilities for the Experiment Designer.
 *
 * Implements deterministic, canonical, incremental hashing per the redesign spec:
 * - Stable structural hashing for steps and actions
 * - Optional inclusion of parameter VALUES vs only parameter KEYS
 * - Incremental hash computation to avoid recomputing entire design on small changes
 * - Action signature hashing (schema/provenance sensitive) for drift detection
 *
 * Default behavior excludes parameter values from the design hash to reduce false-positive drift
 * caused by content edits (reproducibility concerns focus on structure + provenance).
 */

import type {
  ExperimentAction,
  ExperimentStep,
  ExecutionDescriptor,
} from "~/lib/experiment-designer/types";

/* -------------------------------------------------------------------------- */
/* Canonicalization                                                           */
/* -------------------------------------------------------------------------- */

type CanonicalPrimitive = string | number | boolean | null;
type CanonicalValue =
  | CanonicalPrimitive
  | CanonicalValue[]
  | { [key: string]: CanonicalValue };

/**
 * Recursively canonicalize an unknown value:
 * - Removes undefined properties
 * - Sorts object keys
 * - Leaves arrays in existing (semantic) order
 */
function canonicalize(value: unknown): CanonicalValue {
  if (
    value === null ||
    typeof value === "string" ||
    typeof value === "number" ||
    typeof value === "boolean"
  ) {
    return value;
  }
  if (Array.isArray(value)) {
    return value.map((v) => canonicalize(v));
  }
  if (typeof value === "object") {
    const obj = value as Record<string, unknown>;
    const out: Record<string, CanonicalValue> = {};
    Object.keys(obj)
      .filter((k) => obj[k] !== undefined)
      .sort()
      .forEach((k) => {
        out[k] = canonicalize(obj[k]);
      });
    return out;
  }
  // Unsupported types (symbol, function, bigint) replaced with null
  return null;
}

/* -------------------------------------------------------------------------- */
/* Hashing Primitives                                                         */
/* -------------------------------------------------------------------------- */

/**
 * Convert an ArrayBuffer to a lowercase hex string.
 */
function bufferToHex(buffer: ArrayBuffer): string {
  const bytes = new Uint8Array(buffer);
  let hex = "";
  for (const byte of bytes) {
    const b = byte.toString(16).padStart(2, "0");
    hex += b;
  }
  return hex;
}

/**
 * Hash a UTF-8 string using Web Crypto if available, else Node's crypto.
 */
async function hashString(input: string): Promise<string> {
  // Prefer Web Crypto subtle (Edge/Browser compatible)
  if (typeof globalThis.crypto?.subtle?.digest === "function") {
    const enc = new TextEncoder().encode(input);
    const digest = await globalThis.crypto.subtle.digest("SHA-256", enc);
    return bufferToHex(digest);
  }

  // Fallback to Node (should not execute in Edge runtime)
  try {
    // eslint-disable-next-line @typescript-eslint/no-require-imports, @typescript-eslint/no-unsafe-assignment
    const nodeCrypto = require("crypto");
    // eslint-disable-next-line @typescript-eslint/no-unsafe-return, @typescript-eslint/no-unsafe-call, @typescript-eslint/no-unsafe-member-access
    return nodeCrypto.createHash("sha256").update(input).digest("hex");
  } catch {
    throw new Error("No suitable crypto implementation available for hashing.");
  }
}

/**
 * Hash an object using canonical JSON serialization (no whitespace, sorted keys).
 */
export async function hashObject(obj: unknown): Promise<string> {
  const canonical = canonicalize(obj);
  return hashString(JSON.stringify(canonical));
}

/* -------------------------------------------------------------------------- */
/* Structural Projections                                                     */
/* -------------------------------------------------------------------------- */

export interface DesignHashOptions {
  /**
   * Include parameter VALUES in hash rather than only parameter KEY sets.
   * Defaults to false (only parameter keys) to focus on structural reproducibility.
   */
  includeParameterValues?: boolean;
  /**
   * Include action descriptive user-facing metadata (e.g. action.name) in hash.
   * Defaults to true - set false if wanting purely behavioral signature.
   */
  includeActionNames?: boolean;
  /**
   * Include step descriptive fields (step.name, step.description).
   * Defaults to true.
   */
  includeStepNames?: boolean;
}

const DEFAULT_OPTIONS: Required<DesignHashOptions> = {
  includeParameterValues: true, // Changed to true so parameter changes trigger hash updates
  includeActionNames: true,
  includeStepNames: true,
};

/**
 * Projection of an action for design hash purposes.
 */
function projectActionForDesign(
  action: ExperimentAction,
  options: Required<DesignHashOptions>,
): Record<string, unknown> {
  const parameterProjection = options.includeParameterValues
    ? canonicalize(action.parameters)
    : Object.keys(action.parameters).sort();

  const base: Record<string, unknown> = {
    id: action.id,
    type: action.type,
    source: {
      kind: action.source.kind,
      pluginId: action.source.pluginId,
      pluginVersion: action.source.pluginVersion,
      baseActionId: action.source.baseActionId,
    },
    execution: action.execution ? projectExecutionDescriptor(action.execution) : null,
    parameterKeysOrValues: parameterProjection,
    children: action.children?.map(c => projectActionForDesign(c, options)) ?? [],
  };

  if (options.includeActionNames) {
    base.name = action.name;
  }

  return base;
}

function projectExecutionDescriptor(
  exec: ExecutionDescriptor,
): Record<string, unknown> {
  return {
    transport: exec.transport,
    retryable: exec.retryable ?? false,
    timeoutMs: exec.timeoutMs ?? null,
    ros2: exec.ros2
      ? {
        topic: exec.ros2.topic ?? null,
        service: exec.ros2.service ?? null,
        action: exec.ros2.action ?? null,
      }
      : null,
    rest: exec.rest
      ? {
        method: exec.rest.method,
        path: exec.rest.path,
      }
      : null,
  };
}

/**
 * Projection of a step for design hash purposes.
 */
function projectStepForDesign(
  step: ExperimentStep,
  options: Required<DesignHashOptions>,
): Record<string, unknown> {
  const base: Record<string, unknown> = {
    id: step.id,
    type: step.type,
    order: step.order,
    trigger: {
      type: step.trigger.type,
      // Only the sorted keys of conditions (structural presence)
      conditionKeys: Object.keys(step.trigger.conditions).sort(),
    },
    actions: step.actions.map((a) => projectActionForDesign(a, options)),
  };

  if (options.includeStepNames) {
    base.name = step.name;
  }

  return base;
}

/* -------------------------------------------------------------------------- */
/* Action Signature Hash (Schema / Provenance Drift)                          */
/* -------------------------------------------------------------------------- */

export interface ActionSignatureInput {
  type: string;
  category: string;
  parameterSchemaRaw?: unknown;
  execution?: ExecutionDescriptor;
  baseActionId?: string;
  pluginVersion?: string;
  pluginId?: string;
}

/**
 * Hash that uniquely identifies the structural/schema definition of an action definition.
 * Used for plugin drift detection: if signature changes, existing action instances require inspection.
 */
export async function computeActionSignature(
  def: ActionSignatureInput,
): Promise<string> {
  const projection = {
    type: def.type,
    category: def.category,
    pluginId: def.pluginId ?? null,
    pluginVersion: def.pluginVersion ?? null,
    baseActionId: def.baseActionId ?? null,
    execution: def.execution
      ? {
        transport: def.execution.transport,
        retryable: def.execution.retryable ?? false,
        timeoutMs: def.execution.timeoutMs ?? null,
      }
      : null,
    schema: def.parameterSchemaRaw ? canonicalize(def.parameterSchemaRaw) : null,
  };
  return hashObject(projection);
}

/* -------------------------------------------------------------------------- */
/* Design Hash                                                                */
/* -------------------------------------------------------------------------- */

/**
 * Compute a deterministic hash for the entire design (steps + actions) under given options.
 */
export async function computeDesignHash(
  steps: ExperimentStep[],
  opts: DesignHashOptions = {},
): Promise<string> {
  const options = { ...DEFAULT_OPTIONS, ...opts };
  const projected = steps
    .slice()
    .sort((a, b) => a.order - b.order)
    .map((s) => projectStepForDesign(s, options));
  return hashObject({ steps: projected });
}

/* -------------------------------------------------------------------------- */
/* Incremental Hashing                                                        */
/* -------------------------------------------------------------------------- */

export interface IncrementalHashMaps {
  actionHashes: Map<string, string>;
  stepHashes: Map<string, string>;
}

export interface IncrementalHashResult extends IncrementalHashMaps {
  designHash: string;
}

/**
 * Compute or reuse action/step hashes to avoid re-hashing unchanged branches.
 */
export async function computeIncrementalDesignHash(
  steps: ExperimentStep[],
  previous?: IncrementalHashMaps,
  opts: DesignHashOptions = {},
): Promise<IncrementalHashResult> {
  const options = { ...DEFAULT_OPTIONS, ...opts };
  const actionHashes = new Map<string, string>();
  const stepHashes = new Map<string, string>();

  // First compute per-action hashes
  for (const step of steps) {
    for (const action of step.actions) {
      // Only reuse cached hash if we're NOT including parameter values
      // (because parameter values can change without changing the action ID)
      const existing = !options.includeParameterValues
        ? previous?.actionHashes.get(action.id)
        : undefined;

      if (existing) {
        // Simple heuristic: if shallow structural keys unchanged, reuse
        // (We still project to confirm minimal structure; deeper diff omitted for performance.)
        actionHashes.set(action.id, existing);
        continue;
      }
      const projectedAction = projectActionForDesign(action, options);
      const h = await hashObject(projectedAction);
      actionHashes.set(action.id, h);
    }
  }

  // Then compute step hashes (including ordered list of action hashes)
  for (const step of steps) {
    // Only reuse cached hash if we're NOT including parameter values
    // (because parameter values in actions can change without changing the step ID)
    const existing = !options.includeParameterValues
      ? previous?.stepHashes.get(step.id)
      : undefined;

    if (existing) {
      stepHashes.set(step.id, existing);
      continue;
    }
    const projectedStep = {
      id: step.id,
      type: step.type,
      order: step.order,
      trigger: {
        type: step.trigger.type,
        conditionKeys: Object.keys(step.trigger.conditions).sort(),
      },
      actions: step.actions.map((a) => actionHashes.get(a.id) ?? ""),
      ...(options.includeStepNames ? { name: step.name } : {}),
    };
    const h = await hashObject(projectedStep);
    stepHashes.set(step.id, h);
  }

  // Aggregate design hash from ordered step hashes + minimal meta
  const orderedStepHashes = steps
    .slice()
    .sort((a, b) => a.order - b.order)
    .map((s) => stepHashes.get(s.id));

  const designHash = await hashObject({
    steps: orderedStepHashes,
    count: steps.length,
  });

  return { designHash, actionHashes, stepHashes };
}

/* -------------------------------------------------------------------------- */
/* Utility Helpers                                                            */
/* -------------------------------------------------------------------------- */

/**
 * Convenience helper to check if design hash matches a known validated hash.
 */
export function isDesignHashValidated(
  currentHash: string | undefined | null,
  validatedHash: string | undefined | null,
): boolean {
  return Boolean(currentHash && validatedHash && currentHash === validatedHash);
}

/**
 * Determine structural drift given last validated snapshot hash and current.
 */
export function hasStructuralDrift(
  currentHash: string | undefined | null,
  validatedHash: string | undefined | null,
): boolean {
  if (!validatedHash) return false;
  if (!currentHash) return false;
  return currentHash !== validatedHash;
}

/* -------------------------------------------------------------------------- */
/* Exports                                                                    */
/* -------------------------------------------------------------------------- */

export const Hashing = {
  canonicalize,
  hashObject,
  computeDesignHash,
  computeIncrementalDesignHash,
  computeActionSignature,
  isDesignHashValidated,
  hasStructuralDrift,
};
export default Hashing;
