/* Minimal runtime sanity test for block-converter.
 *
 * Purpose:
 *  - Ensures the module can be imported and basic conversion executes without
 *    throwing (guards against accidental breaking structural changes).
 *  - Avoids introducing any testing framework dependency assumptions.
 *
 * This is intentionally lightweight: if expectations fail it will throw,
 * causing any test runner / build process that executes the file to surface
 * the issue.
 */

import {
  convertStepsToDatabase,
  convertActionToDatabase,
} from "../block-converter";
import type { ExperimentStep, ExperimentAction } from "../types";

// Construct a minimal valid ExperimentAction (fields inferred from converter usage)
const speakAction: ExperimentAction = {
  id: "a1",
  name: "Wizard Speak Greeting",
  type: "wizard_speak",
  category: "wizard",
  parameters: { text: "Hello participant, welcome!" },
  parameterSchemaRaw: {
    type: "object",
    properties: { text: { type: "string" } },
  },
  source: {
    kind: "core",
    pluginId: undefined,
    pluginVersion: undefined,
    robotId: undefined,
    baseActionId: "core_wizard_speak",
  },
  execution: {
    transport: "internal",
    ros2: undefined,
    rest: undefined,
    retryable: false,
  },
  // Additional metadata sometimes present in richer designs could go here
};

// Secondary action to exercise duration + timeout estimation differences
const waitAction: ExperimentAction = {
  id: "a2",
  name: "Wait 2s",
  type: "wait",
  category: "control",
  parameters: { duration: 2 },
  parameterSchemaRaw: {
    type: "object",
    properties: { duration: { type: "number" } },
  },
  source: {
    kind: "core",
    pluginId: undefined,
    pluginVersion: undefined,
    robotId: undefined,
    baseActionId: "core_wait",
  },
  execution: {
    transport: "internal",
    ros2: undefined,
    rest: undefined,
    retryable: false,
  },
};

// Compose a single step using the actions
const steps: ExperimentStep[] = [
  {
    id: "step-1",
    name: "Introduction",
    description: "Wizard greets the participant, then brief pause.",
    type: "sequential",
    order: 0,
    trigger: {
      type: "trial_start",
      conditions: {},
    },
    actions: [speakAction, waitAction],
    expanded: true,
  },
];

// Runtime smoke test
(() => {
  const converted = convertStepsToDatabase(steps);

  if (converted.length !== steps.length) {
    throw new Error(
      `convertStepsToDatabase length mismatch: expected ${steps.length}, got ${converted.length}`,
    );
  }

  const first = converted[0];
  if (!first || first.actions.length !== 2) {
    throw new Error(
      `Expected first converted step to contain 2 actions, got ${first?.actions.length ?? "undefined"}`,
    );
  }

  // Also directly test single action conversion (structural presence)
  const convertedAction = convertActionToDatabase(speakAction, 0);
  if (
    convertedAction.name !== speakAction.name ||
    convertedAction.orderIndex !== 0
  ) {
    throw new Error(
      "convertActionToDatabase did not preserve basic fields correctly",
    );
  }

  // Basic invariants that should hold true given estimation logic
  if (
    typeof first.durationEstimate !== "number" ||
    Number.isNaN(first.durationEstimate) ||
    first.durationEstimate < 1
  ) {
    throw new Error(
      `durationEstimate invalid: ${String(first.durationEstimate)} (expected positive integer)`,
    );
  }
})();
