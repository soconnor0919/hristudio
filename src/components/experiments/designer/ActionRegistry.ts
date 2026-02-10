"use client";

import { useState, useEffect } from "react";
import type { ActionDefinition, ExperimentAction } from "~/lib/experiment-designer/types";
import corePluginDef from "~/plugins/definitions/hristudio-core.json";
import wozPluginDef from "~/plugins/definitions/hristudio-woz.json";

/**
 * ActionRegistry
 *
 * Central singleton for loading and serving action definitions from:
 * - Core system action JSON manifests (hristudio-core, hristudio-woz)
 * - Study-installed plugin action definitions (ROS2 / REST / internal transports)
 *
 * Responsibilities:
 * - Lazy, idempotent loading of core and plugin actions
 * - Provenance retention (core vs plugin, plugin id/version, robot id)
 * - Parameter schema → UI parameter mapping (primitive only for now)
 * - Fallback action population if core load fails (ensures minimal functionality)
 */
export class ActionRegistry {
  private static instance: ActionRegistry;
  private actions = new Map<string, ActionDefinition>();
  private aliasIndex = new Map<string, string>();
  private coreActionsLoaded = false;
  private pluginActionsLoaded = false;
  private loadedStudyId: string | null = null;
  private listeners = new Set<() => void>();

  private readonly SYSTEM_PLUGIN_IDS = ["hristudio-core", "hristudio-woz"];

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

  /* ---------------- Core / System Actions ---------------- */

  async loadCoreActions(): Promise<void> {
    if (this.coreActionsLoaded) return;

    // Load System Plugins (Core & WoZ)
    this.registerPluginDefinition(corePluginDef);
    this.registerPluginDefinition(wozPluginDef);

    console.log(`[ActionRegistry] Loaded system plugins: ${this.SYSTEM_PLUGIN_IDS.join(", ")}`);

    this.coreActionsLoaded = true;
    this.notifyListeners();
  }

  /* ---------------- Plugin Actions ---------------- */

  loadPluginActions(
    studyId: string,
    studyPlugins: any[],
  ): void {
    if (this.pluginActionsLoaded && this.loadedStudyId === studyId) return;

    if (this.loadedStudyId !== studyId) {
      this.resetPluginActions();
    }

    let totalActionsLoaded = 0;

    (studyPlugins ?? []).forEach((plugin) => {
      this.registerPluginDefinition(plugin);
      totalActionsLoaded += (plugin.actionDefinitions?.length || 0);
    });

    console.log(
      `ActionRegistry: Loaded ${totalActionsLoaded} plugin actions for study ${studyId}`,
    );

    this.pluginActionsLoaded = true;
    this.loadedStudyId = studyId;
    this.notifyListeners();
  }

  /* ---------------- Shared Registration Logic ---------------- */

  private registerPluginDefinition(plugin: any) {
    const actionDefs = Array.isArray(plugin.actionDefinitions)
      ? plugin.actionDefinitions
      : undefined;

    if (!actionDefs) return;

    actionDefs.forEach((action: any) => {
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

      // Default category based on plugin type or explicit category
      let category = categoryMap[rawCategory];
      if (!category) {
        if (plugin.id === 'hristudio-woz') category = 'wizard';
        else if (plugin.id === 'hristudio-core') category = 'control';
        else category = 'robot';
      }

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

      // Extract semantic ID from metadata if available, otherwise fall back to database IDs
      // Priority: metadata.robotId > metadata.id (for system plugins) > robotId > id
      const semanticRobotId =
        plugin.metadata?.robotId ||
        plugin.metadata?.id ||
        plugin.robotId ||
        plugin.id;

      // For system plugins, we want to keep the short IDs (wait, branch) to avoid breaking existing save data
      // For robot plugins, we namespace them (nao6-ros2.say_text)
      const isSystem = this.SYSTEM_PLUGIN_IDS.includes(semanticRobotId);
      const actionId = isSystem ? action.id : `${semanticRobotId}.${action.id}`;
      const actionType = actionId; // Type is usually same as ID

      const actionDef: ActionDefinition = {
        id: actionId,
        type: actionType,
        name: action.name,
        description: action.description ?? "",
        category,
        icon: action.icon ?? "Bot",
        color: action.color || "#10b981",
        parameters: this.convertParameterSchemaToParameters(
          action.parameterSchema,
        ),
        source: {
          kind: isSystem ? "core" : "plugin", // Maintain 'core' distinction for UI grouping if needed
          pluginId: semanticRobotId,
          robotId: plugin.robotId,
          pluginVersion: plugin.version ?? undefined,
          baseActionId: action.id,
        },
        execution,
        parameterSchemaRaw: action.parameterSchema ?? undefined,
        nestable: action.nestable
      };

      // Prevent overwriting if it already exists (first-come-first-served, usually core first)
      if (!this.actions.has(actionId)) {
        this.actions.set(actionId, actionDef);
      }

      // Register aliases
      const aliases = Array.isArray(action.aliases) ? action.aliases : undefined;
      if (aliases) {
        for (const alias of aliases) {
          if (typeof alias === "string" && alias.trim()) {
            this.aliasIndex.set(alias, actionDef.id);
          }
        }
      }
    });
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
      let type: "text" | "number" | "select" | "boolean" | "json" | "array" = "text";

      if (paramDef.type === "number") {
        type = "number";
      } else if (paramDef.type === "boolean") {
        type = "boolean";
      } else if (paramDef.enum && Array.isArray(paramDef.enum)) {
        type = "select";
      } else if (paramDef.type === "array") {
        type = "array";
      } else if (paramDef.type === "object") {
        type = "json";
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

    // Robust Reset: Remove valid plugin actions, BUT protect system plugins.
    const idsToDelete: string[] = [];
    this.actions.forEach((action, id) => {
      if (action.source.kind === "plugin" && !this.SYSTEM_PLUGIN_IDS.includes(action.source.pluginId || "")) {
        idsToDelete.push(id);
      }
    });

    idsToDelete.forEach((id) => this.actions.delete(id));
    this.notifyListeners();
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
    const direct = this.actions.get(id);
    if (direct) return direct;
    const mapped = this.aliasIndex.get(id);
    return mapped ? this.actions.get(mapped) : undefined;
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
