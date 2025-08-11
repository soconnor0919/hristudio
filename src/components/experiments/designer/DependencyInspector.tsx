"use client";

import React, { useMemo } from "react";
import {
  Package,
  AlertTriangle,
  CheckCircle,
  RefreshCw,
  AlertCircle,
  Zap,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Separator } from "~/components/ui/separator";
import { cn } from "~/lib/utils";
import type {
  ExperimentStep,
  ActionDefinition,
} from "~/lib/experiment-designer/types";

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

export interface PluginDependency {
  pluginId: string;
  version: string;
  robotId?: string;
  name?: string;
  status: "available" | "missing" | "outdated" | "error";
  installedVersion?: string;
  actionCount: number;
  driftedActionCount: number;
}

export interface ActionSignatureDrift {
  actionId: string;
  actionName: string;
  stepId: string;
  stepName: string;
  type: string;
  pluginId?: string;
  pluginVersion?: string;
  driftType: "missing_definition" | "schema_changed" | "version_mismatch";
  details?: string;
}

export interface DependencyInspectorProps {
  steps: ExperimentStep[];
  /**
   * Map of action instance ID to signature drift information
   */
  actionSignatureDrift: Set<string>;
  /**
   * Available action definitions from registry
   */
  actionDefinitions: ActionDefinition[];
  /**
   * Called when user wants to reconcile a drifted action
   */
  onReconcileAction?: (actionId: string) => void;
  /**
   * Called when user wants to refresh plugin dependencies
   */
  onRefreshDependencies?: () => void;
  /**
   * Called when user wants to install a missing plugin
   */
  onInstallPlugin?: (pluginId: string) => void;
  className?: string;
}

/* -------------------------------------------------------------------------- */
/* Utility Functions                                                          */
/* -------------------------------------------------------------------------- */

function extractPluginDependencies(
  steps: ExperimentStep[],
  actionDefinitions: ActionDefinition[],
  driftedActions: Set<string>,
): PluginDependency[] {
  const dependencyMap = new Map<string, PluginDependency>();

  // Collect all plugin actions used in the experiment
  steps.forEach((step) => {
    step.actions.forEach((action) => {
      if (action.source.kind === "plugin" && action.source.pluginId) {
        const key = `${action.source.pluginId}@${action.source.pluginVersion}`;

        if (!dependencyMap.has(key)) {
          dependencyMap.set(key, {
            pluginId: action.source.pluginId,
            version: action.source.pluginVersion ?? "unknown",
            status: "available", // Will be updated below
            actionCount: 0,
            driftedActionCount: 0,
          });
        }

        const dep = dependencyMap.get(key)!;
        dep.actionCount++;

        if (driftedActions.has(action.id)) {
          dep.driftedActionCount++;
        }
      }
    });
  });

  // Update status based on available definitions
  dependencyMap.forEach((dep) => {
    const availableActions = actionDefinitions.filter(
      (def) =>
        def.source.kind === "plugin" && def.source.pluginId === dep.pluginId,
    );

    if (availableActions.length === 0) {
      dep.status = "missing";
    } else {
      // Check if we have the exact version
      const exactVersion = availableActions.find(
        (def) => def.source.pluginVersion === dep.version,
      );

      if (!exactVersion) {
        dep.status = "outdated";
        // Get the installed version
        const anyVersion = availableActions[0];
        dep.installedVersion = anyVersion?.source.pluginVersion;
      } else {
        dep.status = "available";
        dep.installedVersion = dep.version;
      }

      // Set plugin name from first available definition
      if (availableActions[0]) {
        dep.name = availableActions[0].source.pluginId; // Could be enhanced with actual plugin name
      }
    }
  });

  return Array.from(dependencyMap.values()).sort((a, b) =>
    a.pluginId.localeCompare(b.pluginId),
  );
}

function extractActionDrifts(
  steps: ExperimentStep[],
  actionDefinitions: ActionDefinition[],
  driftedActions: Set<string>,
): ActionSignatureDrift[] {
  const drifts: ActionSignatureDrift[] = [];

  steps.forEach((step) => {
    step.actions.forEach((action) => {
      if (driftedActions.has(action.id)) {
        const definition = actionDefinitions.find(
          (def) => def.type === action.type,
        );

        let driftType: ActionSignatureDrift["driftType"] = "missing_definition";
        let details = "";

        if (!definition) {
          driftType = "missing_definition";
          details = `Action definition for type '${action.type}' not found`;
        } else if (
          action.source.pluginId &&
          action.source.pluginVersion !== definition.source.pluginVersion
        ) {
          driftType = "version_mismatch";
          details = `Expected v${action.source.pluginVersion}, found v${definition.source.pluginVersion}`;
        } else {
          driftType = "schema_changed";
          details = "Action schema or execution parameters have changed";
        }

        drifts.push({
          actionId: action.id,
          actionName: action.name,
          stepId: step.id,
          stepName: step.name,
          type: action.type,
          pluginId: action.source.pluginId,
          pluginVersion: action.source.pluginVersion,
          driftType,
          details,
        });
      }
    });
  });

  return drifts;
}

/* -------------------------------------------------------------------------- */
/* Plugin Dependency Item                                                     */
/* -------------------------------------------------------------------------- */

interface PluginDependencyItemProps {
  dependency: PluginDependency;
  onInstall?: (pluginId: string) => void;
}

function PluginDependencyItem({
  dependency,
  onInstall,
}: PluginDependencyItemProps) {
  const statusConfig = {
    available: {
      icon: CheckCircle,
      color: "text-green-600 dark:text-green-400",
      badgeVariant: "outline" as const,
      badgeColor: "border-green-300 text-green-700 dark:text-green-300",
    },
    missing: {
      icon: AlertCircle,
      color: "text-red-600 dark:text-red-400",
      badgeVariant: "destructive" as const,
      badgeColor: "",
    },
    outdated: {
      icon: AlertTriangle,
      color: "text-amber-600 dark:text-amber-400",
      badgeVariant: "secondary" as const,
      badgeColor: "",
    },
    error: {
      icon: AlertTriangle,
      color: "text-red-600 dark:text-red-400",
      badgeVariant: "destructive" as const,
      badgeColor: "",
    },
  };

  const config = statusConfig[dependency.status];
  const IconComponent = config.icon;

  return (
    <div className="flex items-center justify-between rounded-md border p-3">
      <div className="flex items-center gap-3">
        <div className="flex-shrink-0">
          <IconComponent className={cn("h-4 w-4", config.color)} />
        </div>

        <div className="min-w-0 flex-1">
          <div className="flex items-center gap-2">
            <span className="text-sm font-medium">{dependency.pluginId}</span>
            <Badge
              variant={config.badgeVariant}
              className={cn("h-4 text-[10px]", config.badgeColor)}
            >
              {dependency.status}
            </Badge>
          </div>

          <div className="text-muted-foreground mt-1 text-xs">
            v{dependency.version}
            {dependency.installedVersion &&
              dependency.installedVersion !== dependency.version && (
                <span> (installed: v{dependency.installedVersion})</span>
              )}
            • {dependency.actionCount} actions
            {dependency.driftedActionCount > 0 && (
              <span className="text-amber-600 dark:text-amber-400">
                • {dependency.driftedActionCount} drifted
              </span>
            )}
          </div>
        </div>
      </div>

      {dependency.status === "missing" && onInstall && (
        <Button
          variant="outline"
          size="sm"
          className="h-6 text-xs"
          onClick={() => onInstall(dependency.pluginId)}
        >
          Install
        </Button>
      )}
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* Action Drift Item                                                          */
/* -------------------------------------------------------------------------- */

interface ActionDriftItemProps {
  drift: ActionSignatureDrift;
  onReconcile?: (actionId: string) => void;
}

function ActionDriftItem({ drift, onReconcile }: ActionDriftItemProps) {
  const driftConfig = {
    missing_definition: {
      icon: AlertCircle,
      color: "text-red-600 dark:text-red-400",
      badgeVariant: "destructive" as const,
      label: "Missing",
    },
    schema_changed: {
      icon: AlertTriangle,
      color: "text-amber-600 dark:text-amber-400",
      badgeVariant: "secondary" as const,
      label: "Schema Changed",
    },
    version_mismatch: {
      icon: AlertTriangle,
      color: "text-blue-600 dark:text-blue-400",
      badgeVariant: "outline" as const,
      label: "Version Mismatch",
    },
  };

  const config = driftConfig[drift.driftType];
  const IconComponent = config.icon;

  return (
    <div className="flex items-start justify-between rounded-md border p-3">
      <div className="flex items-start gap-3">
        <div className="flex-shrink-0">
          <IconComponent className={cn("h-4 w-4", config.color)} />
        </div>

        <div className="min-w-0 flex-1">
          <div className="flex items-start gap-2">
            <div className="min-w-0 flex-1">
              <p className="text-sm font-medium">{drift.actionName}</p>
              <p className="text-muted-foreground text-xs">
                in {drift.stepName} • {drift.type}
              </p>
            </div>
            <Badge
              variant={config.badgeVariant}
              className="h-4 flex-shrink-0 text-[10px]"
            >
              {config.label}
            </Badge>
          </div>

          {drift.details && (
            <p className="text-muted-foreground mt-1 text-xs leading-relaxed">
              {drift.details}
            </p>
          )}

          {drift.pluginId && (
            <div className="mt-1 flex flex-wrap gap-1">
              <Badge variant="outline" className="h-4 text-[10px]">
                {drift.pluginId}
                {drift.pluginVersion && `@${drift.pluginVersion}`}
              </Badge>
            </div>
          )}
        </div>
      </div>

      {onReconcile && (
        <Button
          variant="outline"
          size="sm"
          className="h-6 text-xs"
          onClick={() => onReconcile(drift.actionId)}
        >
          Fix
        </Button>
      )}
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* DependencyInspector Component                                              */
/* -------------------------------------------------------------------------- */

export function DependencyInspector({
  steps,
  actionSignatureDrift,
  actionDefinitions,
  onReconcileAction,
  onRefreshDependencies,
  onInstallPlugin,
  className,
}: DependencyInspectorProps) {
  const dependencies = useMemo(
    () =>
      extractPluginDependencies(steps, actionDefinitions, actionSignatureDrift),
    [steps, actionDefinitions, actionSignatureDrift],
  );

  const drifts = useMemo(
    () => extractActionDrifts(steps, actionDefinitions, actionSignatureDrift),
    [steps, actionDefinitions, actionSignatureDrift],
  );

  // Count core vs plugin actions
  const actionCounts = useMemo(() => {
    let core = 0;
    let plugin = 0;

    steps.forEach((step) => {
      step.actions.forEach((action) => {
        if (action.source.kind === "plugin") {
          plugin++;
        } else {
          core++;
        }
      });
    });

    return { core, plugin, total: core + plugin };
  }, [steps]);

  const hasIssues =
    dependencies.some((d) => d.status !== "available") || drifts.length > 0;

  return (
    <Card className={cn("h-[calc(100vh-12rem)]", className)}>
      <CardHeader className="pb-2">
        <CardTitle className="flex items-center justify-between text-sm">
          <div className="flex items-center gap-2">
            <Package className="h-4 w-4" />
            Dependencies
          </div>
          <div className="flex items-center gap-1">
            {hasIssues ? (
              <Badge variant="destructive" className="h-4 text-[10px]">
                Issues
              </Badge>
            ) : (
              <Badge
                variant="outline"
                className="h-4 border-green-300 text-[10px] text-green-700 dark:text-green-300"
              >
                Healthy
              </Badge>
            )}
            {onRefreshDependencies && (
              <Button
                variant="ghost"
                size="sm"
                className="h-6 w-6 p-0"
                onClick={onRefreshDependencies}
              >
                <RefreshCw className="h-3 w-3" />
              </Button>
            )}
          </div>
        </CardTitle>
      </CardHeader>

      <CardContent className="p-0">
        <ScrollArea className="h-full">
          <div className="space-y-4 p-3">
            {/* Action Summary */}
            <div className="space-y-2">
              <h4 className="text-muted-foreground text-xs font-medium tracking-wide uppercase">
                Action Summary
              </h4>
              <div className="flex flex-wrap gap-1">
                <Badge variant="outline" className="h-4 text-[10px]">
                  <Zap className="mr-1 h-2 w-2" />
                  {actionCounts.core} core
                </Badge>
                <Badge variant="outline" className="h-4 text-[10px]">
                  <Package className="mr-1 h-2 w-2" />
                  {actionCounts.plugin} plugin
                </Badge>
                <Badge variant="secondary" className="h-4 text-[10px]">
                  {actionCounts.total} total
                </Badge>
              </div>
            </div>

            {/* Plugin Dependencies */}
            {dependencies.length > 0 && (
              <>
                <Separator />
                <div className="space-y-2">
                  <h4 className="text-muted-foreground text-xs font-medium tracking-wide uppercase">
                    Plugin Dependencies ({dependencies.length})
                  </h4>
                  <div className="space-y-2">
                    {dependencies.map((dep) => (
                      <PluginDependencyItem
                        key={`${dep.pluginId}@${dep.version}`}
                        dependency={dep}
                        onInstall={onInstallPlugin}
                      />
                    ))}
                  </div>
                </div>
              </>
            )}

            {/* Action Signature Drifts */}
            {drifts.length > 0 && (
              <>
                <Separator />
                <div className="space-y-2">
                  <h4 className="text-muted-foreground text-xs font-medium tracking-wide uppercase">
                    Action Drift ({drifts.length})
                  </h4>
                  <div className="space-y-2">
                    {drifts.map((drift) => (
                      <ActionDriftItem
                        key={drift.actionId}
                        drift={drift}
                        onReconcile={onReconcileAction}
                      />
                    ))}
                  </div>
                </div>
              </>
            )}

            {/* Empty State */}
            {dependencies.length === 0 && drifts.length === 0 && (
              <div className="py-8 text-center">
                <div className="bg-muted mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full">
                  <Package className="h-4 w-4" />
                </div>
                <p className="text-sm font-medium">No plugin dependencies</p>
                <p className="text-muted-foreground text-xs">
                  This experiment uses only core actions
                </p>
              </div>
            )}

            {/* Healthy State */}
            {dependencies.length > 0 && !hasIssues && (
              <div className="py-4 text-center">
                <div className="mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full bg-green-100 dark:bg-green-950/20">
                  <CheckCircle className="h-4 w-4 text-green-600 dark:text-green-400" />
                </div>
                <p className="text-sm font-medium text-green-700 dark:text-green-300">
                  All dependencies healthy
                </p>
                <p className="text-muted-foreground text-xs">
                  No drift or missing plugins detected
                </p>
              </div>
            )}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  );
}
