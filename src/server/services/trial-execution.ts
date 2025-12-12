/* eslint-disable @typescript-eslint/no-explicit-any */
/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-unsafe-member-access */

/* eslint-disable @typescript-eslint/no-unsafe-call */
/* eslint-disable @typescript-eslint/no-unsafe-return */
/* eslint-disable @typescript-eslint/prefer-nullish-coalescing */
/* eslint-disable @typescript-eslint/restrict-template-expressions */
/* eslint-disable @typescript-eslint/no-base-to-string */

import { type db } from "~/server/db";
import {
  trials,
  steps,
  actions,
  trialEvents,
  plugins,
} from "~/server/db/schema";
import { eq, asc } from "drizzle-orm";
import {
  getRobotCommunicationService,
  type RobotAction,
  type RobotActionResult,
} from "./robot-communication";

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
  private pluginCache = new Map<string, any>();
  private robotComm = getRobotCommunicationService();

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
  async executeAction(
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
    const startTime = Date.now();

    try {
      // Parse plugin.action format
      const [pluginName, actionId] = action.type.split(".");

      console.log(`[TrialExecution] Parsed action: pluginName=${pluginName}, actionId=${actionId}`);

      if (!pluginName || !actionId) {
        throw new Error(
          `Invalid robot action format: ${action.type}. Expected format: plugin.action`,
        );
      }

      // Get plugin configuration from database
      const plugin = await this.getPluginDefinition(pluginName);
      if (!plugin) {
        throw new Error(`Plugin '${pluginName}' not found`);
      }

      console.log(`[TrialExecution] Plugin loaded: ${plugin.name} (ID: ${plugin.id})`);
      console.log(`[TrialExecution] Available actions: ${plugin.actions?.map((a: any) => a.id).join(", ")}`);

      // Find action definition in plugin
      const actionDefinition = plugin.actions?.find(
        (a: any) => a.id === actionId,
      );
      if (!actionDefinition) {
        throw new Error(
          `Action '${actionId}' not found in plugin '${pluginName}'`,
        );
      }

      // Validate parameters
      const validatedParams = this.validateActionParameters(
        actionDefinition,
        action.parameters,
      );

      // Execute action through robot communication service
      const result = await this.executeRobotActionWithComm(
        plugin,
        actionDefinition,
        validatedParams,
        trialId,
      );

      const duration = Date.now() - startTime;

      return {
        success: true,
        completed: true,
        duration,
        data: {
          pluginName,
          actionId,
          parameters: validatedParams,
          robotResponse: result,
          platform: plugin.platform,
        },
      };
    } catch (error) {
      const duration = Date.now() - startTime;

      return {
        success: false,
        completed: false,
        duration,
        error:
          error instanceof Error
            ? error.message
            : "Robot action execution failed",
      };
    }
  }

  /**
   * Get plugin definition from database with caching
   */
  private async getPluginDefinition(pluginName: string): Promise<any> {
    // Check cache first
    if (this.pluginCache.has(pluginName)) {
      return this.pluginCache.get(pluginName);
    }

    try {
      // Check if pluginName is a UUID
      const isUuid =
        /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i.test(
          pluginName,
        );

      const query = isUuid
        ? eq(plugins.id, pluginName)
        : eq(plugins.name, pluginName);

      const [plugin] = await this.db
        .select()
        .from(plugins)
        .where(query)
        .limit(1);

      if (plugin) {
        // Cache the plugin definition
        // Use the actual name for cache key if we looked up by ID
        const cacheKey = isUuid ? plugin.name : pluginName;

        const pluginData = {
          ...plugin,
          actions: plugin.actionDefinitions,
          platform: (plugin.metadata as any)?.platform,
          ros2Config: (plugin.metadata as any)?.ros2Config,
        };

        this.pluginCache.set(cacheKey, pluginData);
        // Also cache by ID if accessible
        if (plugin.id) {
          this.pluginCache.set(plugin.id, pluginData);
        }

        return pluginData;
      }

      return null;
    } catch (error) {
      console.error(`Failed to load plugin ${pluginName}:`, error);
      return null;
    }
  }

  /**
   * Validate action parameters against plugin schema
   */
  private validateActionParameters(
    actionDefinition: any,
    parameters: Record<string, unknown>,
  ): Record<string, unknown> {
    const validated: Record<string, unknown> = {};

    if (!actionDefinition.parameters) {
      return parameters;
    }

    for (const paramDef of actionDefinition.parameters) {
      const paramName = paramDef.name;
      const paramValue = parameters[paramName];

      // Required parameter check
      if (
        paramDef.required &&
        (paramValue === undefined || paramValue === null)
      ) {
        throw new Error(`Required parameter '${paramName}' is missing`);
      }

      // Use default value if parameter not provided
      if (paramValue === undefined && paramDef.default !== undefined) {
        validated[paramName] = paramDef.default;
        continue;
      }

      if (paramValue !== undefined) {
        // Type validation
        switch (paramDef.type) {
          case "number":
            const numValue = Number(paramValue);
            if (isNaN(numValue)) {
              throw new Error(`Parameter '${paramName}' must be a number`);
            }
            if (paramDef.min !== undefined && numValue < paramDef.min) {
              throw new Error(
                `Parameter '${paramName}' must be >= ${paramDef.min}`,
              );
            }
            if (paramDef.max !== undefined && numValue > paramDef.max) {
              throw new Error(
                `Parameter '${paramName}' must be <= ${paramDef.max}`,
              );
            }
            validated[paramName] = numValue;
            break;

          case "boolean":
            validated[paramName] = Boolean(paramValue);
            break;

          case "select":
            if (paramDef.options) {
              const validOptions = paramDef.options.map(
                (opt: any) => opt.value,
              );
              if (!validOptions.includes(paramValue)) {
                throw new Error(
                  `Parameter '${paramName}' must be one of: ${validOptions.join(", ")}`,
                );
              }
            }
            validated[paramName] = paramValue;
            break;

          default:
            validated[paramName] = paramValue;
        }
      }
    }

    return validated;
  }

  /**
   * Execute robot action through robot communication service
   */
  private async executeRobotActionWithComm(
    plugin: any,
    actionDefinition: any,
    parameters: Record<string, unknown>,
    trialId: string,
  ): Promise<string> {
    // Ensure robot communication service is available
    if (!this.robotComm.getConnectionStatus()) {
      try {
        await this.robotComm.connect();
      } catch (error) {
        throw new Error(
          `Failed to connect to robot: ${error instanceof Error ? error.message : "Unknown error"}`,
        );
      }
    }

    // Prepare robot action
    const robotAction: RobotAction = {
      pluginName: plugin.name,
      actionId: actionDefinition.id,
      parameters,
      implementation: actionDefinition.implementation,
    };

    // Execute action through robot communication service
    const result: RobotActionResult =
      await this.robotComm.executeAction(robotAction);

    if (!result.success) {
      throw new Error(result.error || "Robot action failed");
    }

    // Log the successful action execution
    await this.logTrialEvent(trialId, "robot_action_executed", {
      actionId: actionDefinition.id,
      parameters,
      platform: plugin.platform,
      topic: actionDefinition.implementation?.topic,
      messageType: actionDefinition.implementation?.messageType,
      duration: result.duration,
      robotResponse: result.data,
    });

    // Return human-readable result
    return this.formatRobotActionResult(
      plugin,
      actionDefinition,
      parameters,
      result,
    );
  }

  /**
   * Format robot action result for human readability
   */
  private formatRobotActionResult(
    plugin: any,
    actionDefinition: any,
    parameters: Record<string, unknown>,
    result: RobotActionResult,
  ): string {
    const actionType = actionDefinition.id;
    const platform = plugin.platform || "Robot";

    switch (actionType) {
      case "say_text":
        return `${platform} said: "${parameters.text}"`;

      case "walk_forward":
        return `${platform} walked forward at speed ${parameters.speed} for ${parameters.duration || "indefinite"} seconds`;

      case "walk_backward":
        return `${platform} walked backward at speed ${parameters.speed} for ${parameters.duration || "indefinite"} seconds`;

      case "turn_left":
      case "turn_right":
        const direction = actionType.split("_")[1];
        return `${platform} turned ${direction} at speed ${parameters.speed}`;

      case "move_head":
        return `${platform} moved head to yaw=${parameters.yaw}, pitch=${parameters.pitch}`;

      case "move_arm":
        return `${platform} moved ${parameters.arm} arm to specified position`;

      case "stop_movement":
        return `${platform} stopped all movement`;

      case "set_volume":
        return `${platform} set volume to ${parameters.volume}`;

      case "set_language":
        return `${platform} set language to ${parameters.language}`;

      default:
        return `${platform} executed action: ${actionType} (${result.duration}ms)`;
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
