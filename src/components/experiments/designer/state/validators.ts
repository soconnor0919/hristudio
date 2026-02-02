/**
 * Validation utilities for the Experiment Designer.
 *
 * Implements comprehensive validation rules per the redesign spec:
 * - Structural validation (step names, types, trigger configurations)
 * - Parameter validation (required fields, type checking, bounds)
 * - Semantic validation (uniqueness, dependencies, best practices)
 * - Cross-step validation (workflow integrity, execution feasibility)
 *
 * Each validator returns an array of ValidationIssue objects with severity levels.
 */

import type {
  ExperimentStep,
  ActionDefinition,
  TriggerType,
  StepType,
} from "~/lib/experiment-designer/types";

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

export interface ValidationIssue {
  severity: "error" | "warning" | "info";
  message: string;
  category: "structural" | "parameter" | "semantic" | "execution";
  field?: string;
  suggestion?: string;
  actionId?: string;
  stepId?: string;
}

export interface ValidationContext {
  steps: ExperimentStep[];
  actionDefinitions: ActionDefinition[];
  allowPartialValidation?: boolean;
}

export interface ValidationResult {
  valid: boolean;
  issues: ValidationIssue[];
  errorCount: number;
  warningCount: number;
  infoCount: number;
}

/* -------------------------------------------------------------------------- */
/* Validation Rule Sets                                                       */
/* -------------------------------------------------------------------------- */

const VALID_STEP_TYPES: StepType[] = [
  "sequential",
  "parallel",
  "conditional",
  "loop",
];
const VALID_TRIGGER_TYPES: TriggerType[] = [
  "trial_start",
  "participant_action",
  "timer",
  "previous_step",
];

/* -------------------------------------------------------------------------- */
/* Structural Validation                                                      */
/* -------------------------------------------------------------------------- */

export function validateStructural(
  steps: ExperimentStep[],
  _context: ValidationContext,
): ValidationIssue[] {
  const issues: ValidationIssue[] = [];

  // Global structural checks
  if (steps.length === 0) {
    issues.push({
      severity: "error",
      message: "Experiment must contain at least one step",
      category: "structural",
      suggestion: "Add a step to begin designing your experiment",
    });
    return issues; // Early return for empty experiment
  }

  // Step-level validation
  steps.forEach((step, stepIndex) => {
    const stepId = step.id;

    // Step name validation
    if (!step.name?.trim()) {
      issues.push({
        severity: "error",
        message: "Step name cannot be empty",
        category: "structural",
        field: "name",
        stepId,
        suggestion: "Provide a descriptive name for this step",
      });
    } else if (step.name.length > 100) {
      issues.push({
        severity: "warning",
        message: "Step name is very long and may be truncated in displays",
        category: "structural",
        field: "name",
        stepId,
        suggestion: "Consider shortening the step name",
      });
    }

    // Step type validation
    if (!VALID_STEP_TYPES.includes(step.type)) {
      issues.push({
        severity: "error",
        message: `Invalid step type: ${step.type}`,
        category: "structural",
        field: "type",
        stepId,
        suggestion: `Valid types are: ${VALID_STEP_TYPES.join(", ")}`,
      });
    }

    // Step order validation
    if (step.order !== stepIndex) {
      issues.push({
        severity: "error",
        message: `Step order mismatch: expected ${stepIndex}, got ${step.order}`,
        category: "structural",
        field: "order",
        stepId,
        suggestion: "Step order must be sequential starting from 0",
      });
    }

    // Trigger validation
    if (!VALID_TRIGGER_TYPES.includes(step.trigger.type)) {
      issues.push({
        severity: "error",
        message: `Invalid trigger type: ${step.trigger.type}`,
        category: "structural",
        field: "trigger.type",
        stepId,
        suggestion: `Valid trigger types are: ${VALID_TRIGGER_TYPES.join(", ")}`,
      });
    }

    // Conditional step must have conditions
    if (step.type === "conditional") {
      const conditionKeys = Object.keys(step.trigger.conditions || {});
      if (conditionKeys.length === 0) {
        issues.push({
          severity: "error",
          message: "Conditional step must define at least one condition",
          category: "structural",
          field: "trigger.conditions",
          stepId,
          suggestion: "Add conditions to define when this step should execute",
        });
      }
    }

    // Loop step should have termination conditions
    if (step.type === "loop") {
      const conditionKeys = Object.keys(step.trigger.conditions || {});
      if (conditionKeys.length === 0) {
        issues.push({
          severity: "warning",
          message:
            "Loop step should define termination conditions to prevent infinite loops",
          category: "structural",
          field: "trigger.conditions",
          stepId,
          suggestion: "Add conditions to control when the loop should exit",
        });
      }
    }

    // Parallel step should have multiple actions
    if (step.type === "parallel" && step.actions.length < 2) {
      issues.push({
        severity: "warning",
        message:
          "Parallel step has fewer than 2 actions - consider using sequential type",
        category: "structural",
        stepId,
        suggestion: "Add more actions or change to sequential execution",
      });
    }

    // Action-level structural validation
    step.actions.forEach((action) => {
      const actionId = action.id;

      // Action name validation
      if (!action.name?.trim()) {
        issues.push({
          severity: "error",
          message: "Action name cannot be empty",
          category: "structural",
          field: "name",
          stepId,
          actionId,
          suggestion: "Provide a descriptive name for this action",
        });
      }

      // Action type validation
      if (!action.type?.trim()) {
        issues.push({
          severity: "error",
          message: "Action type cannot be empty",
          category: "structural",
          field: "type",
          stepId,
          actionId,
          suggestion: "Select a valid action type from the library",
        });
      }

      // Note: Action order validation removed as orderIndex is not in the type definition
      // Actions are ordered by their position in the array

      // Source validation
      if (!action.source?.kind) {
        issues.push({
          severity: "error",
          message: "Action source kind is required",
          category: "structural",
          field: "source.kind",
          stepId,
          actionId,
          suggestion: "Action must specify if it's from core or plugin source",
        });
      }

      // Plugin actions need plugin metadata
      if (action.source?.kind === "plugin") {
        if (!action.source.pluginId) {
          issues.push({
            severity: "error",
            message: "Plugin action must specify pluginId",
            category: "structural",
            field: "source.pluginId",
            stepId,
            actionId,
            suggestion: "Plugin actions require valid plugin identification",
          });
        }
        if (!action.source.pluginVersion) {
          issues.push({
            severity: "warning",
            message: "Plugin action should specify version for reproducibility",
            category: "structural",
            field: "source.pluginVersion",
            stepId,
            actionId,
            suggestion: "Pin plugin version to ensure consistent behavior",
          });
        }
      }

      // Execution descriptor validation
      if (!action.execution?.transport) {
        issues.push({
          severity: "error",
          message: "Action must specify execution transport",
          category: "structural",
          field: "execution.transport",
          stepId,
          actionId,
          suggestion:
            "Define how this action should be executed (rest, ros2, etc.)",
        });
      }
    });
  });

  return issues;
}

/* -------------------------------------------------------------------------- */
/* Parameter Validation                                                       */
/* -------------------------------------------------------------------------- */

export function validateParameters(
  steps: ExperimentStep[],
  context: ValidationContext,
): ValidationIssue[] {
  const issues: ValidationIssue[] = [];
  const { actionDefinitions } = context;

  steps.forEach((step) => {
    step.actions.forEach((action) => {
      const stepId = step.id;
      const actionId = action.id;

      // Find action definition
      const definition = actionDefinitions.find(
        (def) => def.type === action.type,
      );

      if (!definition) {
        issues.push({
          severity: "error",
          message: `Action definition not found for type: ${action.type}`,
          category: "parameter",
          stepId,
          actionId,
          suggestion: "Check if the required plugin is installed and loaded",
        });
        return; // Skip parameter validation for missing definitions
      }

      // Validate each parameter
      definition.parameters.forEach((paramDef) => {
        const paramId = paramDef.id;
        const value = action.parameters[paramId];
        const field = `parameters.${paramId}`;

        // Required parameter check
        if (paramDef.required) {
          const isEmpty =
            value === undefined ||
            value === null ||
            (typeof value === "string" && value.trim() === "");

          if (isEmpty) {
            issues.push({
              severity: "error",
              message: `Required parameter '${paramDef.name}' is missing`,
              category: "parameter",
              field,
              stepId,
              actionId,
              suggestion: "Provide a value for this required parameter",
            });
            return; // Skip type validation for missing required params
          }
        }

        // Skip validation for optional empty parameters
        if (value === undefined || value === null) return;

        // Type validation
        switch (paramDef.type) {
          case "text":
            if (typeof value !== "string") {
              issues.push({
                severity: "error",
                message: `Parameter '${paramDef.name}' must be text`,
                category: "parameter",
                field,
                stepId,
                actionId,
                suggestion: "Enter a text value",
              });
              // Note: maxLength validation removed as it's not in the ActionParameter type
            }
            break;

          case "number":
            if (typeof value !== "number" || isNaN(value)) {
              issues.push({
                severity: "error",
                message: `Parameter '${paramDef.name}' must be a valid number`,
                category: "parameter",
                field,
                stepId,
                actionId,
                suggestion: "Enter a numeric value",
              });
            } else {
              // Range validation
              if (paramDef.min !== undefined && value < paramDef.min) {
                issues.push({
                  severity: "error",
                  message: `Parameter '${paramDef.name}' must be at least ${paramDef.min}`,
                  category: "parameter",
                  field,
                  stepId,
                  actionId,
                  suggestion: `Enter a value >= ${paramDef.min}`,
                });
              }
              if (paramDef.max !== undefined && value > paramDef.max) {
                issues.push({
                  severity: "error",
                  message: `Parameter '${paramDef.name}' must be at most ${paramDef.max}`,
                  category: "parameter",
                  field,
                  stepId,
                  actionId,
                  suggestion: `Enter a value <= ${paramDef.max}`,
                });
              }
            }
            break;

          case "boolean":
            if (typeof value !== "boolean") {
              issues.push({
                severity: "error",
                message: `Parameter '${paramDef.name}' must be true or false`,
                category: "parameter",
                field,
                stepId,
                actionId,
                suggestion: "Use the toggle switch to set this value",
              });
            }
            break;

          case "select":
            if (
              paramDef.options &&
              !paramDef.options.includes(value as string)
            ) {
              issues.push({
                severity: "error",
                message: `Parameter '${paramDef.name}' has invalid value`,
                category: "parameter",
                field,
                stepId,
                actionId,
                suggestion:
                  Array.isArray(paramDef.options) && paramDef.options.length
                    ? `Choose from: ${paramDef.options.join(", ")}`
                    : "Choose a valid option",
              });
            }
            break;

          default:
            // Unknown parameter type
            issues.push({
              severity: "warning",
              message: `Unknown parameter type '${String(paramDef.type)}' for '${paramDef.name}'`,
              category: "parameter",
              field,
              stepId,
              actionId,
              suggestion: "Check action definition for correct parameter types",
            });
        }
      });

      // Check for unexpected parameters
      Object.keys(action.parameters).forEach((paramId) => {
        const isDefinedParam = definition.parameters.some(
          (def) => def.id === paramId,
        );
        if (!isDefinedParam) {
          issues.push({
            severity: "warning",
            message: `Unexpected parameter '${paramId}' - not defined in action schema`,
            category: "parameter",
            field: `parameters.${paramId}`,
            stepId,
            actionId,
            suggestion:
              "Remove this parameter or check if action definition is outdated",
          });
        }
      });
    });
  });

  return issues;
}

/* -------------------------------------------------------------------------- */
/* Semantic Validation                                                        */
/* -------------------------------------------------------------------------- */

export function validateSemantic(
  steps: ExperimentStep[],
  _context: ValidationContext,
): ValidationIssue[] {
  const issues: ValidationIssue[] = [];

  // Check for duplicate step IDs
  const stepIds = new Set<string>();
  const duplicateStepIds = new Set<string>();

  steps.forEach((step) => {
    if (stepIds.has(step.id)) {
      duplicateStepIds.add(step.id);
    }
    stepIds.add(step.id);
  });

  duplicateStepIds.forEach((stepId) => {
    issues.push({
      severity: "error",
      message: `Duplicate step ID: ${stepId}`,
      category: "semantic",
      stepId,
      suggestion: "Step IDs must be unique throughout the experiment",
    });
  });

  // Check for duplicate action IDs globally
  const actionIds = new Set<string>();
  const duplicateActionIds = new Set<string>();

  steps.forEach((step) => {
    step.actions.forEach((action) => {
      if (actionIds.has(action.id)) {
        duplicateActionIds.add(action.id);
      }
      actionIds.add(action.id);
    });
  });

  duplicateActionIds.forEach((actionId) => {
    const containingSteps = steps.filter((s) =>
      s.actions.some((a) => a.id === actionId),
    );

    containingSteps.forEach((step) => {
      issues.push({
        severity: "error",
        message: `Duplicate action ID: ${actionId}`,
        category: "semantic",
        stepId: step.id,
        actionId,
        suggestion: "Action IDs must be unique throughout the experiment",
      });
    });
  });

  // Check for empty steps
  steps.forEach((step) => {
    if (step.actions.length === 0) {
      const severity = step.type === "parallel" ? "error" : "warning";
      issues.push({
        severity,
        message: `${step.type} step has no actions`,
        category: "semantic",
        stepId: step.id,
        suggestion: "Add actions to this step or remove it",
      });
    }
  });

  // Documentation suggestions
  steps.forEach((step) => {
    // Missing step descriptions
    if (!step.description?.trim()) {
      issues.push({
        severity: "info",
        message: "Consider adding a description to document step purpose",
        category: "semantic",
        field: "description",
        stepId: step.id,
        suggestion:
          "Descriptions improve experiment documentation and reproducibility",
      });
    }

    // Actions without meaningful names
    step.actions.forEach((action) => {
      if (
        action.name === action.type ||
        action.name.toLowerCase().includes("untitled")
      ) {
        issues.push({
          severity: "info",
          message: "Consider providing a more descriptive action name",
          category: "semantic",
          field: "name",
          stepId: step.id,
          actionId: action.id,
          suggestion:
            "Descriptive names help with experiment understanding and debugging",
        });
      }
    });
  });

  // Workflow logic suggestions
  steps.forEach((step, index) => {
    // First step should typically use trial_start trigger
    if (index === 0 && step.trigger.type !== "trial_start") {
      issues.push({
        severity: "info",
        message: "First step typically uses trial_start trigger",
        category: "semantic",
        field: "trigger.type",
        stepId: step.id,
        suggestion: "Consider using trial_start trigger for the initial step",
      });
    }

    // Timer triggers without reasonable durations
    if (step.trigger.type === "timer") {
      const duration = step.trigger.conditions?.duration;
      if (typeof duration === "number") {
        if (duration < 100) {
          issues.push({
            severity: "warning",
            message: "Very short timer duration may cause timing issues",
            category: "semantic",
            field: "trigger.conditions.duration",
            stepId: step.id,
            suggestion: "Consider using at least 100ms for reliable timing",
          });
        }
        if (duration > 300000) {
          // 5 minutes
          issues.push({
            severity: "info",
            message: "Long timer duration - ensure this is intentional",
            category: "semantic",
            field: "trigger.conditions.duration",
            stepId: step.id,
            suggestion:
              "Verify the timer duration is correct for your use case",
          });
        }
      }
    }
  });

  return issues;
}

/* -------------------------------------------------------------------------- */
/* Cross-Step Execution Validation                                           */
/* -------------------------------------------------------------------------- */

export function validateExecution(
  steps: ExperimentStep[],
  _context: ValidationContext,
): ValidationIssue[] {
  const issues: ValidationIssue[] = [];

  // Check for unreachable steps (basic heuristic)
  if (steps.length > 1) {
    const trialStartSteps = steps.filter(
      (s) => s.trigger.type === "trial_start",
    );
    if (trialStartSteps.length > 1) {
      trialStartSteps.slice(1).forEach((step) => {
        issues.push({
          severity: "info",
          message:
            "This step will start immediately at trial start. For sequential flow, use 'Previous Step' trigger.",
          category: "execution",
          field: "trigger.type",
          stepId: step.id,
          suggestion: "Change trigger to 'Previous Step' if this step should follow the previous one",
        });
      });
    }
  }

  // Check for missing robot dependencies
  const robotActions = steps.flatMap((step) =>
    step.actions.filter(
      (action) =>
        action.execution?.transport === "ros2" ||
        action.execution?.transport === "rest",
    ),
  );

  if (robotActions.length > 0) {
    // This would need robot registry integration in full implementation
    issues.push({
      severity: "info",
      message:
        "Experiment contains robot actions - ensure robot connections are configured",
      category: "execution",
      suggestion:
        "Verify robot plugins are installed and robots are accessible",
    });
  }

  return issues;
}

/* -------------------------------------------------------------------------- */
/* Main Validation Function                                                   */
/* -------------------------------------------------------------------------- */

export function validateExperimentDesign(
  steps: ExperimentStep[],
  context: ValidationContext,
): ValidationResult {
  const issues: ValidationIssue[] = [];

  // Run all validation rule sets
  issues.push(...validateStructural(steps, context));
  issues.push(...validateParameters(steps, context));
  issues.push(...validateSemantic(steps, context));
  issues.push(...validateExecution(steps, context));

  // Count issues by severity
  const errorCount = issues.filter((i) => i.severity === "error").length;
  const warningCount = issues.filter((i) => i.severity === "warning").length;
  const infoCount = issues.filter((i) => i.severity === "info").length;

  // Experiment is valid if no errors (warnings and info are allowed)
  const valid = errorCount === 0;

  return {
    valid,
    issues,
    errorCount,
    warningCount,
    infoCount,
  };
}

/* -------------------------------------------------------------------------- */
/* Issue Grouping Utilities                                                   */
/* -------------------------------------------------------------------------- */

export function groupIssuesByEntity(
  issues: ValidationIssue[],
): Record<string, ValidationIssue[]> {
  const grouped: Record<string, ValidationIssue[]> = {};

  issues.forEach((issue) => {
    const entityId = issue.actionId ?? issue.stepId ?? "experiment";
    grouped[entityId] ??= [];
    grouped[entityId].push(issue);
  });

  return grouped;
}

export function getIssuesByStep(
  issues: ValidationIssue[],
  stepId: string,
): ValidationIssue[] {
  return issues.filter((issue) => issue.stepId === stepId);
}

export function getIssuesByAction(
  issues: ValidationIssue[],
  actionId: string,
): ValidationIssue[] {
  return issues.filter((issue) => issue.actionId === actionId);
}

/* -------------------------------------------------------------------------- */
/* Exports                                                                    */
/* -------------------------------------------------------------------------- */

export const Validators = {
  validateStructural,
  validateParameters,
  validateSemantic,
  validateExecution,
  validateExperimentDesign,
  groupIssuesByEntity,
  getIssuesByStep,
  getIssuesByAction,
};

export default Validators;
