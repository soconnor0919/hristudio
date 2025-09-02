/* eslint-disable @typescript-eslint/no-explicit-any */
/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-unsafe-member-access */

/* eslint-disable @typescript-eslint/no-unsafe-call */
/* eslint-disable @typescript-eslint/no-unsafe-return */
/* eslint-disable @typescript-eslint/prefer-nullish-coalescing */
/* eslint-disable @typescript-eslint/restrict-template-expressions */
/* eslint-disable @typescript-eslint/no-base-to-string */

import { type db } from "~/server/db";
import { trials, steps, actions, trialEvents } from "~/server/db/schema";
import { eq, asc } from "drizzle-orm";

export type TrialStatus =
  | "scheduled"
  | "in_progress"
  | "completed"
  | "aborted"
  | "failed";

export interface ExecutionContext {
  trialId: string;
  experimentId: string;
  participantId: string;
  wizardId?: string;
  currentStepIndex: number;
  startTime: Date;
  variables: Record<string, unknown>;
}

export interface StepDefinition {
  id: string;
  name: string;
  description?: string;
  type: string;
  orderIndex: number;
  condition?: string;
  actions: ActionDefinition[];
}

export interface ActionDefinition {
  id: string;
  stepId: string;
  name: string;
  description?: string;
  type: string;
  orderIndex: number;
  parameters: Record<string, unknown>;
  timeout?: number;
  required: boolean;
  condition?: string;
}

export interface ExecutionResult {
  success: boolean;
  error?: string;
  data?: Record<string, unknown>;
  duration?: number;
  nextStepIndex?: number;
}

export interface ActionExecutionResult {
  success: boolean;
  error?: string;
  data?: Record<string, unknown>;
  duration: number;
  completed: boolean;
}

export class TrialExecutionEngine {
  private db: typeof db;
  private activeTrials = new Map<string, ExecutionContext>();
  private stepDefinitions = new Map<string, StepDefinition[]>();

  constructor(database: typeof db) {
    this.db = database;
  }

  /**
   * Initialize a trial for execution
   */
  async initializeTrial(trialId: string): Promise<ExecutionContext> {
    // Get trial details
    const [trial] = await this.db
      .select()
      .from(trials)
      .where(eq(trials.id, trialId));

    if (!trial) {
      throw new Error(`Trial ${trialId} not found`);
    }

    if (trial.status === "completed" || trial.status === "aborted") {
      throw new Error(`Trial ${trialId} is already ${trial.status}`);
    }

    // Load experiment steps and actions
    const experimentSteps = await this.loadExperimentProtocol(
      trial.experimentId,
    );
    this.stepDefinitions.set(trialId, experimentSteps);

    // Create execution context
    const context: ExecutionContext = {
      trialId,
      experimentId: trial.experimentId,
      participantId: trial.participantId || "",
      wizardId: trial.wizardId || undefined,
      currentStepIndex: 0,
      startTime: new Date(),
      variables: {},
    };

    this.activeTrials.set(trialId, context);
    return context;
  }

  /**
   * Load experiment protocol (steps and actions) from database
   */
  private async loadExperimentProtocol(
    experimentId: string,
  ): Promise<StepDefinition[]> {
    // Get all steps for the experiment
    const stepRecords = await this.db
      .select()
      .from(steps)
      .where(eq(steps.experimentId, experimentId))
      .orderBy(asc(steps.orderIndex));

    const stepDefinitions: StepDefinition[] = [];

    for (const step of stepRecords) {
      // Get all actions for this step
      const actionRecords = await this.db
        .select()
        .from(actions)
        .where(eq(actions.stepId, step.id))
        .orderBy(asc(actions.orderIndex));

      const actionDefinitions: ActionDefinition[] = actionRecords.map(
        (action: any) => ({
          id: action.id,
          stepId: action.stepId,
          name: action.name,
          description: action.description || undefined,
          type: action.type,
          orderIndex: action.orderIndex,
          parameters: (action.parameters as Record<string, unknown>) || {},
          timeout: action.timeout || undefined,
          required: action.required || true,
          condition: action.condition || undefined,
        }),
      );

      stepDefinitions.push({
        id: step.id,
        name: step.name,
        description: step.description || undefined,
        type: step.type,
        orderIndex: step.orderIndex,
        condition: (step.conditions as string) || undefined,
        actions: actionDefinitions,
      });
    }

    return stepDefinitions;
  }

  /**
   * Start trial execution
   */
  async startTrial(
    trialId: string,
    wizardId?: string,
  ): Promise<ExecutionResult> {
    try {
      let context = this.activeTrials.get(trialId);
      if (!context) {
        context = await this.initializeTrial(trialId);
      }

      if (wizardId) {
        context.wizardId = wizardId;
      }

      // Update trial status in database
      await this.db
        .update(trials)
        .set({
          status: "in_progress",
          startedAt: context.startTime,
          wizardId: context.wizardId,
        })
        .where(eq(trials.id, trialId));

      // Log trial start event
      await this.logTrialEvent(trialId, "trial_started", {
        wizardId: context.wizardId,
        startTime: context.startTime.toISOString(),
      });

      return {
        success: true,
        data: {
          trialId,
          status: "in_progress",
          currentStepIndex: context.currentStepIndex,
        },
      };
    } catch (error) {
      return {
        success: false,
        error:
          error instanceof Error
            ? error.message
            : "Unknown error starting trial",
      };
    }
  }

  /**
   * Execute the current step
   */
  async executeCurrentStep(trialId: string): Promise<ExecutionResult> {
    const context = this.activeTrials.get(trialId);
    if (!context) {
      return { success: false, error: "Trial not initialized" };
    }

    const steps = this.stepDefinitions.get(trialId);
    if (!steps || context.currentStepIndex >= steps.length) {
      return await this.completeTrial(trialId);
    }

    const step = steps[context.currentStepIndex];
    if (!step) {
      return { success: false, error: "Invalid step index" };
    }

    try {
      // Check step condition
      if (step.condition && !this.evaluateCondition(step.condition, context)) {
        // Skip this step
        return await this.advanceToNextStep(trialId);
      }

      // Log step start
      await this.logTrialEvent(trialId, "step_started", {
        stepId: step.id,
        stepName: step.name,
        stepIndex: context.currentStepIndex,
      });

      // Execute all actions in the step
      const actionResults = await this.executeStepActions(trialId, step);

      const failedActions = actionResults.filter(
        (result) => !result.success && result.required,
      );
      if (failedActions.length > 0) {
        throw new Error(
          `Step failed: ${failedActions.map((f) => f.error).join(", ")}`,
        );
      }

      // Log step completion
      await this.logTrialEvent(trialId, "step_completed", {
        stepId: step.id,
        stepName: step.name,
        stepIndex: context.currentStepIndex,
        actionResults: actionResults.map((r) => ({
          success: r.success,
          duration: r.duration,
        })),
      });

      return {
        success: true,
        data: {
          stepId: step.id,
          stepName: step.name,
          actionResults,
        },
      };
    } catch (error) {
      await this.logTrialEvent(trialId, "step_failed", {
        stepId: step.id,
        stepName: step.name,
        stepIndex: context.currentStepIndex,
        error: error instanceof Error ? error.message : "Unknown error",
      });

      return {
        success: false,
        error:
          error instanceof Error
            ? error.message
            : "Unknown error executing step",
      };
    }
  }

  /**
   * Execute all actions within a step
   */
  private async executeStepActions(
    trialId: string,
    step: StepDefinition,
  ): Promise<Array<ActionExecutionResult & { required: boolean }>> {
    const context = this.activeTrials.get(trialId)!;
    const results: Array<ActionExecutionResult & { required: boolean }> = [];

    for (const action of step.actions) {
      // Check action condition
      if (
        action.condition &&
        !this.evaluateCondition(action.condition, context)
      ) {
        results.push({
          success: true,
          completed: false,
          duration: 0,
          data: { skipped: true, reason: "condition not met" },
          required: action.required,
        });
        continue;
      }

      const startTime = Date.now();

      try {
        const result = await this.executeAction(trialId, action);
        const duration = Date.now() - startTime;

        await this.logTrialEvent(trialId, "action_executed", {
          actionId: action.id,
          actionName: action.name,
          actionType: action.type,
          stepId: step.id,
          duration,
          success: result.success,
          data: result.data,
        });

        results.push({
          ...result,
          duration,
          required: action.required,
        });
      } catch (error) {
        const duration = Date.now() - startTime;

        await this.logTrialEvent(trialId, "action_failed", {
          actionId: action.id,
          actionName: action.name,
          actionType: action.type,
          stepId: step.id,
          duration,
          error: error instanceof Error ? error.message : "Unknown error",
        });

        results.push({
          success: false,
          completed: false,
          duration,
          error: error instanceof Error ? error.message : "Unknown error",
          required: action.required,
        });
      }
    }

    return results;
  }

  /**
   * Execute a single action
   */
  private async executeAction(
    trialId: string,
    action: ActionDefinition,
  ): Promise<ActionExecutionResult> {
    // This is where we'd dispatch to different action executors based on action.type
    // For now, we'll implement basic action types and mock robot actions

    switch (action.type) {
      case "wait":
        return await this.executeWaitAction(action);

      case "wizard_say":
        return await this.executeWizardAction(trialId, action);

      case "wizard_gesture":
        return await this.executeWizardAction(trialId, action);

      case "observe_behavior":
        return await this.executeObservationAction(trialId, action);

      default:
        // Check if it's a robot action (contains plugin prefix)
        if (action.type.includes(".")) {
          return await this.executeRobotAction(trialId, action);
        }

        // Unknown action type - log and continue
        return {
          success: true,
          completed: true,
          duration: 0,
          data: {
            message: `Action type '${action.type}' not implemented yet`,
            parameters: action.parameters,
          },
        };
    }
  }

  /**
   * Execute wait action
   */
  private async executeWaitAction(
    action: ActionDefinition,
  ): Promise<ActionExecutionResult> {
    const duration = (action.parameters.duration as number) || 1000;

    return new Promise((resolve) => {
      setTimeout(() => {
        resolve({
          success: true,
          completed: true,
          duration,
          data: { waitDuration: duration },
        });
      }, duration);
    });
  }

  /**
   * Execute wizard action (requires human input)
   */
  private async executeWizardAction(
    trialId: string,
    action: ActionDefinition,
  ): Promise<ActionExecutionResult> {
    // For wizard actions, we return immediately but mark as requiring wizard input
    // The wizard interface will handle the actual execution

    return {
      success: true,
      completed: false, // Requires wizard confirmation
      duration: 0,
      data: {
        requiresWizardInput: true,
        actionType: action.type,
        parameters: action.parameters,
        instructions: this.getWizardInstructions(action),
      },
    };
  }

  /**
   * Execute observation action
   */
  private async executeObservationAction(
    trialId: string,
    action: ActionDefinition,
  ): Promise<ActionExecutionResult> {
    // Observation actions typically require wizard input to record observations

    return {
      success: true,
      completed: false,
      duration: 0,
      data: {
        requiresWizardInput: true,
        actionType: action.type,
        parameters: action.parameters,
        observationType: action.parameters.type || "behavior",
      },
    };
  }

  /**
   * Execute robot action through plugin system
   */
  private async executeRobotAction(
    trialId: string,
    action: ActionDefinition,
  ): Promise<ActionExecutionResult> {
    try {
      // Parse plugin.action format
      const [pluginId, actionType] = action.type.split(".");

      // TODO: Integrate with actual robot plugin system
      // For now, simulate robot action execution

      const simulationDelay = Math.random() * 2000 + 500; // 500ms - 2.5s

      return new Promise((resolve) => {
        setTimeout(() => {
          // Simulate success/failure
          const success = Math.random() > 0.1; // 90% success rate

          resolve({
            success,
            completed: true,
            duration: simulationDelay,
            data: {
              pluginId,
              actionType,
              parameters: action.parameters,
              robotResponse: success
                ? "Action completed successfully"
                : "Robot action failed",
            },
            error: success ? undefined : "Simulated robot failure",
          });
        }, simulationDelay);
      });
    } catch (error) {
      return {
        success: false,
        completed: false,
        duration: 0,
        error:
          error instanceof Error
            ? error.message
            : "Robot action execution failed",
      };
    }
  }

  /**
   * Advance to the next step
   */
  async advanceToNextStep(trialId: string): Promise<ExecutionResult> {
    const context = this.activeTrials.get(trialId);
    if (!context) {
      return { success: false, error: "Trial not initialized" };
    }

    const steps = this.stepDefinitions.get(trialId);
    if (!steps) {
      return { success: false, error: "No steps loaded for trial" };
    }

    const previousStepIndex = context.currentStepIndex;
    context.currentStepIndex++;

    await this.logTrialEvent(trialId, "step_transition", {
      fromStepIndex: previousStepIndex,
      toStepIndex: context.currentStepIndex,
    });

    // Check if we've completed all steps
    if (context.currentStepIndex >= steps.length) {
      return await this.completeTrial(trialId);
    }

    return {
      success: true,
      nextStepIndex: context.currentStepIndex,
      data: { previousStepIndex, currentStepIndex: context.currentStepIndex },
    };
  }

  /**
   * Complete the trial
   */
  async completeTrial(trialId: string): Promise<ExecutionResult> {
    const context = this.activeTrials.get(trialId);
    if (!context) {
      return { success: false, error: "Trial not initialized" };
    }

    const endTime = new Date();
    const duration = endTime.getTime() - context.startTime.getTime();

    try {
      // Update trial in database
      await this.db
        .update(trials)
        .set({
          status: "completed",
          completedAt: endTime,
          duration: Math.round(duration / 1000), // Convert to seconds
        })
        .where(eq(trials.id, trialId));

      // Log completion
      await this.logTrialEvent(trialId, "trial_completed", {
        endTime: endTime.toISOString(),
        duration,
        totalSteps: this.stepDefinitions.get(trialId)?.length || 0,
      });

      // Clean up
      this.activeTrials.delete(trialId);
      this.stepDefinitions.delete(trialId);

      return {
        success: true,
        data: {
          trialId,
          status: "completed",
          duration,
          endTime: endTime.toISOString(),
        },
      };
    } catch (error) {
      return {
        success: false,
        error:
          error instanceof Error ? error.message : "Failed to complete trial",
      };
    }
  }

  /**
   * Abort the trial
   */
  async abortTrial(trialId: string, reason?: string): Promise<ExecutionResult> {
    const context = this.activeTrials.get(trialId);
    if (!context) {
      return { success: false, error: "Trial not initialized" };
    }

    const endTime = new Date();
    const duration = endTime.getTime() - context.startTime.getTime();

    try {
      await this.db
        .update(trials)
        .set({
          status: "aborted",
          completedAt: endTime,
          duration: Math.round(duration / 1000),
        })
        .where(eq(trials.id, trialId));

      await this.logTrialEvent(trialId, "trial_aborted", {
        reason: reason || "Manual abort",
        endTime: endTime.toISOString(),
        duration,
        stepIndex: context.currentStepIndex,
      });

      // Clean up
      this.activeTrials.delete(trialId);
      this.stepDefinitions.delete(trialId);

      return {
        success: true,
        data: {
          trialId,
          status: "aborted",
          reason,
          duration,
        },
      };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : "Failed to abort trial",
      };
    }
  }

  /**
   * Get current execution status
   */
  getTrialStatus(trialId: string): ExecutionContext | null {
    return this.activeTrials.get(trialId) || null;
  }

  /**
   * Get current step definition
   */
  getCurrentStep(trialId: string): StepDefinition | null {
    const context = this.activeTrials.get(trialId);
    const steps = this.stepDefinitions.get(trialId);

    if (!context || !steps || context.currentStepIndex >= steps.length) {
      return null;
    }

    return steps[context.currentStepIndex] || null;
  }

  /**
   * Log trial event to database
   */
  private async logTrialEvent(
    trialId: string,
    eventType: string,
    data: Record<string, unknown> = {},
  ): Promise<void> {
    try {
      await this.db.insert(trialEvents).values({
        trialId,
        eventType,
        data: data as any, // TODO: Fix typing
        timestamp: new Date(),
        createdBy: this.activeTrials.get(trialId)?.wizardId,
      });
    } catch (error) {
      console.error("Failed to log trial event:", error);
      // Don't throw - logging failures shouldn't stop execution
    }
  }

  /**
   * Evaluate condition (simple implementation)
   */
  private evaluateCondition(
    condition: string,
    context: ExecutionContext,
  ): boolean {
    try {
      // Simple condition evaluation - in production, use a safer evaluator
      // For now, support basic variable checks
      if (condition.includes("variables.")) {
        // Replace variables in condition with actual values
        let evaluableCondition = condition;
        Object.entries(context.variables).forEach(([key, value]) => {
          evaluableCondition = evaluableCondition.replace(
            `variables.${key}`,
            JSON.stringify(value),
          );
        });

        // Basic evaluation - in production, use a proper expression evaluator
        // eslint-disable-next-line @typescript-eslint/no-implied-eval
        return new Function("return " + evaluableCondition)();
      }

      return true; // Default to true if condition can't be evaluated
    } catch (error) {
      console.warn("Failed to evaluate condition:", condition, error);
      return true; // Fail open
    }
  }

  /**
   * Get wizard instructions for an action
   */
  private getWizardInstructions(action: ActionDefinition): string {
    switch (action.type) {
      case "wizard_say":
        return `Say: "${action.parameters.text || "Please speak to the participant"}"`;

      case "wizard_gesture":
        return `Perform gesture: ${action.parameters.gesture || "as specified in the protocol"}`;

      case "observe_behavior":
        return `Observe and record: ${action.parameters.behavior || "participant behavior"}`;

      default:
        return `Execute: ${action.name}`;
    }
  }
}
