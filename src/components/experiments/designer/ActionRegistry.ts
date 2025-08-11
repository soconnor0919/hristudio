"use client";

import { useState, useEffect } from "react";
import type { ActionDefinition } from "~/lib/experiment-designer/types";

/**
 * ActionRegistry
 *
 * Central singleton for loading and serving action definitions from:
 * - Core system action JSON manifests (served from /hristudio-core/plugins/*.json)
 * - Study-installed plugin action definitions (ROS2 / REST / internal transports)
 *
 * Responsibilities:
 * - Lazy, idempotent loading of core and plugin actions
 * - Provenance retention (core vs plugin, plugin id/version, robot id)
 * - Parameter schema → UI parameter mapping (primitive only for now)
 * - Fallback action population if core load fails (ensures minimal functionality)
 *
 * Notes:
 * - The registry is client-side only (designer runtime); server performs its own
 *   validation & compilation using persisted action instances (never trusts client).
 * - Action IDs for plugins are namespaced: `${plugin.id}.${action.id}`.
 * - Core actions retain their base IDs (e.g., wait, wizard_speak) for clarity.
 */
export class ActionRegistry {
  private static instance: ActionRegistry;
  private actions = new Map<string, ActionDefinition>();
  private coreActionsLoaded = false;
  private pluginActionsLoaded = false;
  private loadedStudyId: string | null = null;
  private listeners = new Set<() => void>();

  static getInstance(): ActionRegistry {
    if (!ActionRegistry.instance) {
      ActionRegistry.instance = new ActionRegistry();
    }
    return ActionRegistry.instance;
  }

  /* ---------------- Reactivity ---------------- */

  subscribe(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  private notifyListeners(): void {
    this.listeners.forEach((listener) => listener());
  }

  /* ---------------- Core Actions ---------------- */

  async loadCoreActions(): Promise<void> {
    if (this.coreActionsLoaded) return;

    interface CoreBlockParam {
      id: string;
      name: string;
      type: string;
      placeholder?: string;
      options?: string[];
      min?: number;
      max?: number;
      value?: string | number | boolean;
      required?: boolean;
      description?: string;
      step?: number;
    }

    interface CoreBlock {
      id: string;
      name: string;
      description?: string;
      category: string;
      icon?: string;
      color?: string;
      parameters?: CoreBlockParam[];
      timeoutMs?: number;
      retryable?: boolean;
    }

    try {
      const coreActionSets = [
        "wizard-actions",
        "control-flow",
        "observation",
        "events",
      ];

      for (const actionSetId of coreActionSets) {
        try {
          const response = await fetch(
            `/hristudio-core/plugins/${actionSetId}.json`,
          );
          // Non-blocking skip if not found
          if (!response.ok) continue;

          const rawActionSet = (await response.json()) as unknown;
          const actionSet = rawActionSet as { blocks?: CoreBlock[] };
          if (!actionSet.blocks || !Array.isArray(actionSet.blocks)) continue;

          // Register each block as an ActionDefinition
          actionSet.blocks.forEach((block) => {
            if (!block.id || !block.name) return;

            const actionDef: ActionDefinition = {
              id: block.id,
              type: block.id,
              name: block.name,
              description: block.description ?? "",
              category: this.mapBlockCategoryToActionCategory(block.category),
              icon: block.icon ?? "Zap",
              color: block.color ?? "#6b7280",
              parameters: (block.parameters ?? []).map((param) => ({
                id: param.id,
                name: param.name,
                type:
                  (param.type as "text" | "number" | "select" | "boolean") ||
                  "text",
                placeholder: param.placeholder,
                options: param.options,
                min: param.min,
                max: param.max,
                value: param.value,
                required: param.required !== false,
                description: param.description,
                step: param.step,
              })),
              source: {
                kind: "core",
                baseActionId: block.id,
              },
              execution: {
                transport: "internal",
                timeoutMs: block.timeoutMs,
                retryable: block.retryable,
              },
              parameterSchemaRaw: {
                parameters: block.parameters ?? [],
              },
            };

            this.actions.set(actionDef.id, actionDef);
          });
        } catch (error) {
          // Non-fatal: we will fallback later
          console.warn(`Failed to load core action set ${actionSetId}:`, error);
        }
      }

      this.coreActionsLoaded = true;
      this.notifyListeners();
    } catch (error) {
      console.error("Failed to load core actions:", error);
      this.loadFallbackActions();
    }
  }

  private mapBlockCategoryToActionCategory(
    category: string,
  ): ActionDefinition["category"] {
    switch (category) {
      case "wizard":
        return "wizard";
      case "event":
        return "wizard"; // Events are wizard-initiated triggers
      case "robot":
        return "robot";
      case "control":
        return "control";
      case "sensor":
      case "observation":
        return "observation";
      default:
        return "wizard";
    }
  }

  private loadFallbackActions(): void {
    const fallbackActions: ActionDefinition[] = [
      {
        id: "wizard_speak",
        type: "wizard_speak",
        name: "Wizard Says",
        description: "Wizard speaks to participant",
        category: "wizard",
        icon: "MessageSquare",
        color: "#3b82f6",
        parameters: [
          {
            id: "text",
            name: "Text to say",
            type: "text",
            placeholder: "Hello, participant!",
            required: true,
          },
        ],
        source: { kind: "core", baseActionId: "wizard_speak" },
        execution: { transport: "internal", timeoutMs: 30000 },
        parameterSchemaRaw: {
          type: "object",
          properties: {
            text: { type: "string" },
          },
          required: ["text"],
        },
      },
      {
        id: "wait",
        type: "wait",
        name: "Wait",
        description: "Wait for specified time",
        category: "control",
        icon: "Clock",
        color: "#f59e0b",
        parameters: [
          {
            id: "duration",
            name: "Duration (seconds)",
            type: "number",
            min: 0.1,
            max: 300,
            value: 2,
            required: true,
          },
        ],
        source: { kind: "core", baseActionId: "wait" },
        execution: { transport: "internal", timeoutMs: 60000 },
        parameterSchemaRaw: {
          type: "object",
          properties: {
            duration: {
              type: "number",
              minimum: 0.1,
              maximum: 300,
              default: 2,
            },
          },
          required: ["duration"],
        },
      },
      {
        id: "observe",
        type: "observe",
        name: "Observe",
        description: "Record participant behavior",
        category: "observation",
        icon: "Eye",
        color: "#8b5cf6",
        parameters: [
          {
            id: "behavior",
            name: "Behavior to observe",
            type: "select",
            options: ["facial_expression", "body_language", "verbal_response"],
            required: true,
          },
        ],
        source: { kind: "core", baseActionId: "observe" },
        execution: { transport: "internal", timeoutMs: 120000 },
        parameterSchemaRaw: {
          type: "object",
          properties: {
            behavior: {
              type: "string",
              enum: ["facial_expression", "body_language", "verbal_response"],
            },
          },
          required: ["behavior"],
        },
      },
    ];

    fallbackActions.forEach((action) => this.actions.set(action.id, action));
    this.notifyListeners();
  }

  /* ---------------- Plugin Actions ---------------- */

  loadPluginActions(
    studyId: string,
    studyPlugins: Array<{
      plugin: {
        id: string;
        robotId: string | null;
        version: string | null;
        actionDefinitions?: Array<{
          id: string;
          name: string;
          description?: string;
          category?: string;
          icon?: string;
          timeout?: number;
          retryable?: boolean;
          parameterSchema?: unknown;
          ros2?: {
            topic?: string;
            messageType?: string;
            service?: string;
            action?: string;
            payloadMapping?: unknown;
            qos?: {
              reliability?: string;
              durability?: string;
              history?: string;
              depth?: number;
            };
          };
          rest?: {
            method: "GET" | "POST" | "PUT" | "PATCH" | "DELETE";
            path: string;
            headers?: Record<string, string>;
          };
        }>;
      };
    }>,
  ): void {
    console.log("ActionRegistry.loadPluginActions called with:", {
      studyId,
      pluginCount: studyPlugins?.length ?? 0,
      plugins: studyPlugins?.map((sp) => ({
        id: sp.plugin.id,
        actionCount: Array.isArray(sp.plugin.actionDefinitions)
          ? sp.plugin.actionDefinitions.length
          : 0,
        hasActionDefs: !!sp.plugin.actionDefinitions,
      })),
    });

    if (this.pluginActionsLoaded && this.loadedStudyId === studyId) return;

    if (this.loadedStudyId !== studyId) {
      this.resetPluginActions();
    }

    let totalActionsLoaded = 0;

    (studyPlugins ?? []).forEach((studyPlugin) => {
      const { plugin } = studyPlugin;
      const actionDefs = Array.isArray(plugin.actionDefinitions)
        ? plugin.actionDefinitions
        : undefined;

      console.log(`Plugin ${plugin.id}:`, {
        actionDefinitions: plugin.actionDefinitions,
        isArray: Array.isArray(plugin.actionDefinitions),
        actionCount: actionDefs?.length ?? 0,
      });

      if (!actionDefs) return;

      actionDefs.forEach((action) => {
        const rawCategory =
          typeof action.category === "string"
            ? action.category.toLowerCase().trim()
            : "";
        const categoryMap: Record<string, ActionDefinition["category"]> = {
          wizard: "wizard",
          robot: "robot",
          control: "control",
          observation: "observation",
        };
        const category = categoryMap[rawCategory] ?? "robot";

        const execution = action.ros2
          ? {
              transport: "ros2" as const,
              timeoutMs: action.timeout,
              retryable: action.retryable,
              ros2: {
                topic: action.ros2.topic,
                messageType: action.ros2.messageType,
                service: action.ros2.service,
                action: action.ros2.action,
                qos: action.ros2.qos,
                payloadMapping: action.ros2.payloadMapping,
              },
            }
          : action.rest
            ? {
                transport: "rest" as const,
                timeoutMs: action.timeout,
                retryable: action.retryable,
                rest: {
                  method: action.rest.method,
                  path: action.rest.path,
                  headers: action.rest.headers,
                },
              }
            : {
                transport: "internal" as const,
                timeoutMs: action.timeout,
                retryable: action.retryable,
              };

        const actionDef: ActionDefinition = {
          id: `${plugin.id}.${action.id}`,
          type: `${plugin.id}.${action.id}`,
          name: action.name,
          description: action.description ?? "",
          category,
          icon: action.icon ?? "Bot",
          color: "#10b981",
          parameters: this.convertParameterSchemaToParameters(
            action.parameterSchema,
          ),
          source: {
            kind: "plugin",
            pluginId: plugin.id,
            robotId: plugin.robotId,
            pluginVersion: plugin.version ?? undefined,
            baseActionId: action.id,
          },
          execution,
          parameterSchemaRaw: action.parameterSchema ?? undefined,
        };
        this.actions.set(actionDef.id, actionDef);
        totalActionsLoaded++;
      });
    });

    console.log(
      `ActionRegistry: Loaded ${totalActionsLoaded} plugin actions for study ${studyId}`,
    );
    console.log("Current action registry state:", {
      totalActions: this.actions.size,
      actionsByCategory: {
        wizard: this.getActionsByCategory("wizard").length,
        robot: this.getActionsByCategory("robot").length,
        control: this.getActionsByCategory("control").length,
        observation: this.getActionsByCategory("observation").length,
      },
    });

    this.pluginActionsLoaded = true;
    this.loadedStudyId = studyId;
    this.notifyListeners();
  }

  private convertParameterSchemaToParameters(
    parameterSchema: unknown,
  ): ActionDefinition["parameters"] {
    interface JsonSchemaProperty {
      type?: string;
      title?: string;
      description?: string;
      enum?: string[];
      default?: string | number | boolean;
      minimum?: number;
      maximum?: number;
    }
    interface JsonSchema {
      properties?: Record<string, JsonSchemaProperty>;
      required?: string[];
    }
    const schema = parameterSchema as JsonSchema | undefined;
    if (!schema?.properties) return [];

    return Object.entries(schema.properties).map(([key, paramDef]) => {
      let type: "text" | "number" | "select" | "boolean" = "text";

      if (paramDef.type === "number") {
        type = "number";
      } else if (paramDef.type === "boolean") {
        type = "boolean";
      } else if (paramDef.enum && Array.isArray(paramDef.enum)) {
        type = "select";
      }

      return {
        id: key,
        name: paramDef.title ?? key.charAt(0).toUpperCase() + key.slice(1),
        type,
        value: paramDef.default,
        placeholder: paramDef.description,
        options: paramDef.enum,
        min: paramDef.minimum,
        max: paramDef.maximum,
        required: true,
      };
    });
  }

  private resetPluginActions(): void {
    this.pluginActionsLoaded = false;
    this.loadedStudyId = null;
    // Remove existing plugin actions (retain known core ids + fallback ids)
    const pluginActionIds = Array.from(this.actions.keys()).filter(
      (id) =>
        !id.startsWith("wizard_") &&
        !id.startsWith("when_") &&
        !id.startsWith("wait") &&
        !id.startsWith("observe") &&
        !id.startsWith("repeat") &&
        !id.startsWith("if_") &&
        !id.startsWith("parallel") &&
        !id.startsWith("sequence") &&
        !id.startsWith("random_") &&
        !id.startsWith("try_") &&
        !id.startsWith("break") &&
        !id.startsWith("measure_") &&
        !id.startsWith("count_") &&
        !id.startsWith("record_") &&
        !id.startsWith("capture_") &&
        !id.startsWith("log_") &&
        !id.startsWith("survey_") &&
        !id.startsWith("physiological_"),
    );
    pluginActionIds.forEach((id) => this.actions.delete(id));
  }

  /* ---------------- Query Helpers ---------------- */

  getActionsByCategory(
    category: ActionDefinition["category"],
  ): ActionDefinition[] {
    return Array.from(this.actions.values()).filter(
      (action) => action.category === category,
    );
  }

  getAllActions(): ActionDefinition[] {
    return Array.from(this.actions.values());
  }

  getAction(id: string): ActionDefinition | undefined {
    return this.actions.get(id);
  }

  /* ---------------- Debug Helpers ---------------- */

  getDebugInfo(): {
    coreActionsLoaded: boolean;
    pluginActionsLoaded: boolean;
    loadedStudyId: string | null;
    totalActions: number;
    actionsByCategory: Record<ActionDefinition["category"], number>;
    sampleActionIds: string[];
  } {
    return {
      coreActionsLoaded: this.coreActionsLoaded,
      pluginActionsLoaded: this.pluginActionsLoaded,
      loadedStudyId: this.loadedStudyId,
      totalActions: this.actions.size,
      actionsByCategory: {
        wizard: this.getActionsByCategory("wizard").length,
        robot: this.getActionsByCategory("robot").length,
        control: this.getActionsByCategory("control").length,
        observation: this.getActionsByCategory("observation").length,
      },
      sampleActionIds: Array.from(this.actions.keys()).slice(0, 10),
    };
  }
}

export const actionRegistry = ActionRegistry.getInstance();

/* ---------------- React Hook ---------------- */

export function useActionRegistry(): ActionRegistry {
  const [, forceUpdate] = useState({});

  useEffect(() => {
    const unsubscribe = actionRegistry.subscribe(() => {
      forceUpdate({});
    });
    return unsubscribe;
  }, []);

  return actionRegistry;
}
