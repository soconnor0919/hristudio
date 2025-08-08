"use client";

import React from "react";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectTrigger,
  SelectValue,
  SelectContent,
  SelectItem,
} from "~/components/ui/select";
import { Switch } from "~/components/ui/switch";
import { Slider } from "~/components/ui/slider";
import { Badge } from "~/components/ui/badge";
import { cn } from "~/lib/utils";
import {
  TRIGGER_OPTIONS,
  type ExperimentAction,
  type ExperimentStep,
  type StepType,
  type TriggerType,
  type ExperimentDesign,
} from "~/lib/experiment-designer/types";
import { actionRegistry } from "./ActionRegistry";
import {
  Settings,
  Zap,
  MessageSquare,
  Hand,
  Navigation,
  Volume2,
  Clock,
  Eye,
  Bot,
  User,
  Timer,
  MousePointer,
  Mic,
  Activity,
  Play,
} from "lucide-react";

/**
 * PropertiesPanel
 *
 * Extracted modular panel for editing either:
 * - Action properties (when an action is selected)
 * - Step properties (when a step is selected and no action selected)
 * - Empty instructional state otherwise
 *
 * Enhancements:
 * - Boolean parameters render as Switch
 * - Number parameters with min/max render as Slider (with live value)
 * - Number parameters without bounds fall back to numeric input
 * - Select and text remain standard controls
 * - Provenance + category badges retained
 */

export interface PropertiesPanelProps {
  design: ExperimentDesign;
  selectedStep?: ExperimentStep;
  selectedAction?: ExperimentAction;
  onActionUpdate: (
    stepId: string,
    actionId: string,
    updates: Partial<ExperimentAction>,
  ) => void;
  onStepUpdate: (stepId: string, updates: Partial<ExperimentStep>) => void;
  className?: string;
}

export function PropertiesPanel({
  design,
  selectedStep,
  selectedAction,
  onActionUpdate,
  onStepUpdate,
  className,
}: PropertiesPanelProps) {
  const registry = actionRegistry;

  // Find containing step for selected action (if any)
  const containingStep =
    selectedAction &&
    design.steps.find((s) => s.actions.some((a) => a.id === selectedAction.id));

  /* -------------------------- Action Properties View -------------------------- */
  if (selectedAction && containingStep) {
    const def = registry.getAction(selectedAction.type);
    const categoryColors = {
      wizard: "bg-blue-500",
      robot: "bg-emerald-500",
      control: "bg-amber-500",
      observation: "bg-purple-500",
    } as const;
    // Icon resolution uses statically imported lucide icons (no dynamic require)

    // Icon resolution uses statically imported lucide icons (no dynamic require)
    const iconComponents: Record<
      string,
      React.ComponentType<{ className?: string }>
    > = {
      Zap,
      MessageSquare,
      Hand,
      Navigation,
      Volume2,
      Clock,
      Eye,
      Bot,
      User,
      Timer,
      MousePointer,
      Mic,
      Activity,
      Play,
    };
    const ResolvedIcon: React.ComponentType<{ className?: string }> =
      def?.icon && iconComponents[def.icon]
        ? (iconComponents[def.icon] as React.ComponentType<{
            className?: string;
          }>)
        : Zap;

    return (
      <div className={cn("space-y-3", className)}>
        {/* Header / Metadata */}
        <div className="border-b pb-3">
          <div className="mb-2 flex items-center gap-2">
            {def && (
              <div
                className={cn(
                  "flex h-6 w-6 items-center justify-center rounded text-white",
                  categoryColors[def.category],
                )}
              >
                <ResolvedIcon className="h-3 w-3" />
              </div>
            )}
            <div className="min-w-0">
              <h3 className="truncate text-sm font-medium">
                {selectedAction.name}
              </h3>
              <p className="text-muted-foreground text-xs">
                {def?.category} • {selectedAction.type}
              </p>
            </div>
          </div>
          <div className="flex flex-wrap gap-1">
            <Badge variant="outline" className="h-4 text-[10px]">
              {selectedAction.source.kind === "plugin" ? "Plugin" : "Core"}
            </Badge>
            {selectedAction.source.pluginId && (
              <Badge variant="secondary" className="h-4 text-[10px]">
                {selectedAction.source.pluginId}
                {selectedAction.source.pluginVersion
                  ? `@${selectedAction.source.pluginVersion}`
                  : ""}
              </Badge>
            )}
            <Badge variant="outline" className="h-4 text-[10px]">
              {selectedAction.execution.transport}
            </Badge>
            {selectedAction.execution.retryable && (
              <Badge variant="outline" className="h-4 text-[10px]">
                retryable
              </Badge>
            )}
          </div>
          {def?.description && (
            <p className="text-muted-foreground mt-2 text-xs leading-relaxed">
              {def.description}
            </p>
          )}
        </div>

        {/* General Action Fields */}
        <div className="space-y-2">
          <div>
            <Label className="text-xs">Display Name</Label>
            <Input
              value={selectedAction.name}
              onChange={(e) =>
                onActionUpdate(containingStep.id, selectedAction.id, {
                  name: e.target.value,
                })
              }
              className="mt-1 h-7 text-xs"
            />
          </div>
        </div>

        {/* Parameters */}
        {def?.parameters.length ? (
          <div className="space-y-3">
            <div className="text-muted-foreground text-[10px] tracking-wide uppercase">
              Parameters
            </div>
            <div className="space-y-3">
              {def.parameters.map((param) => {
                const rawValue = selectedAction.parameters[param.id];
                const commonLabel = (
                  <Label className="flex items-center gap-2 text-xs">
                    {param.name}
                    <span className="text-muted-foreground font-normal">
                      {param.type === "number" &&
                        (param.min !== undefined || param.max !== undefined) &&
                        typeof rawValue === "number" &&
                        `( ${rawValue} )`}
                    </span>
                  </Label>
                );

                /* ---- Handlers ---- */
                const updateParamValue = (value: unknown) => {
                  onActionUpdate(containingStep.id, selectedAction.id, {
                    parameters: {
                      ...selectedAction.parameters,
                      [param.id]: value,
                    },
                  });
                };

                /* ---- Control Rendering ---- */
                let control: React.ReactNode = null;

                if (param.type === "text") {
                  control = (
                    <Input
                      value={(rawValue as string) ?? ""}
                      placeholder={param.placeholder}
                      onChange={(e) => updateParamValue(e.target.value)}
                      className="mt-1 h-7 text-xs"
                    />
                  );
                } else if (param.type === "select") {
                  control = (
                    <Select
                      value={(rawValue as string) ?? ""}
                      onValueChange={(val) => updateParamValue(val)}
                    >
                      <SelectTrigger className="mt-1 h-7 text-xs">
                        <SelectValue placeholder="Select…" />
                      </SelectTrigger>
                      <SelectContent>
                        {param.options?.map((opt) => (
                          <SelectItem key={opt} value={opt}>
                            {opt}
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                  );
                } else if (param.type === "boolean") {
                  control = (
                    <div className="mt-1 flex h-7 items-center">
                      <Switch
                        checked={Boolean(rawValue)}
                        onCheckedChange={(val) => updateParamValue(val)}
                        aria-label={param.name}
                      />
                      <span className="text-muted-foreground ml-2 text-[11px]">
                        {Boolean(rawValue) ? "Enabled" : "Disabled"}
                      </span>
                    </div>
                  );
                } else if (param.type === "number") {
                  const numericVal =
                    typeof rawValue === "number"
                      ? rawValue
                      : typeof param.value === "number"
                        ? param.value
                        : (param.min ?? 0);

                  if (param.min !== undefined || param.max !== undefined) {
                    const min = param.min ?? 0;
                    const max =
                      param.max ??
                      Math.max(
                        min + 1,
                        Number.isFinite(numericVal) ? numericVal : min + 1,
                      );
                    // Step heuristic
                    const range = max - min;
                    const step =
                      param.step ??
                      (range <= 5
                        ? 0.1
                        : range <= 50
                          ? 0.5
                          : Math.max(1, Math.round(range / 100)));
                    control = (
                      <div className="mt-1">
                        <div className="flex items-center gap-2">
                          <Slider
                            min={min}
                            max={max}
                            step={step}
                            value={[Number(numericVal)]}
                            onValueChange={(vals: number[]) =>
                              updateParamValue(vals[0])
                            }
                          />
                          <span className="text-muted-foreground min-w-[2.5rem] text-right text-[10px] tabular-nums">
                            {step < 1
                              ? Number(numericVal).toFixed(2)
                              : Number(numericVal).toString()}
                          </span>
                        </div>
                        <div className="text-muted-foreground mt-1 flex justify-between text-[10px]">
                          <span>{min}</span>
                          <span>{max}</span>
                        </div>
                      </div>
                    );
                  } else {
                    control = (
                      <Input
                        type="number"
                        value={numericVal}
                        onChange={(e) =>
                          updateParamValue(parseFloat(e.target.value) || 0)
                        }
                        className="mt-1 h-7 text-xs"
                      />
                    );
                  }
                }

                return (
                  <div key={param.id} className="space-y-1">
                    {commonLabel}
                    {param.description && (
                      <div className="text-muted-foreground text-[10px]">
                        {param.description}
                      </div>
                    )}
                    {control}
                  </div>
                );
              })}
            </div>
          </div>
        ) : (
          <div className="text-muted-foreground text-xs">
            No parameters for this action.
          </div>
        )}
      </div>
    );
  }

  /* --------------------------- Step Properties View --------------------------- */
  if (selectedStep) {
    return (
      <div className={cn("space-y-3", className)}>
        <div className="border-b pb-2">
          <h3 className="flex items-center gap-2 text-sm font-medium">
            <div
              className={cn("h-3 w-3 rounded", {
                "bg-blue-500": selectedStep.type === "sequential",
                "bg-emerald-500": selectedStep.type === "parallel",
                "bg-amber-500": selectedStep.type === "conditional",
                "bg-purple-500": selectedStep.type === "loop",
              })}
            />
            Step Settings
          </h3>
        </div>
        <div className="space-y-2">
          <div>
            <Label className="text-xs">Name</Label>
            <Input
              value={selectedStep.name}
              onChange={(e) =>
                onStepUpdate(selectedStep.id, { name: e.target.value })
              }
              className="mt-1 h-7 text-xs"
            />
          </div>
          <div>
            <Label className="text-xs">Description</Label>
            <Input
              value={selectedStep.description ?? ""}
              placeholder="Optional step description"
              onChange={(e) =>
                onStepUpdate(selectedStep.id, {
                  description: e.target.value,
                })
              }
              className="mt-1 h-7 text-xs"
            />
          </div>
          <div>
            <Label className="text-xs">Type</Label>
            <Select
              value={selectedStep.type}
              onValueChange={(val) =>
                onStepUpdate(selectedStep.id, { type: val as StepType })
              }
            >
              <SelectTrigger className="mt-1 h-7 text-xs">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="sequential">Sequential</SelectItem>
                <SelectItem value="parallel">Parallel</SelectItem>
                <SelectItem value="conditional">Conditional</SelectItem>
                <SelectItem value="loop">Loop</SelectItem>
              </SelectContent>
            </Select>
          </div>
          <div>
            <Label className="text-xs">Trigger</Label>
            <Select
              value={selectedStep.trigger.type}
              onValueChange={(val) =>
                onStepUpdate(selectedStep.id, {
                  trigger: {
                    ...selectedStep.trigger,
                    type: val as TriggerType,
                  },
                })
              }
            >
              <SelectTrigger className="mt-1 h-7 text-xs">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {TRIGGER_OPTIONS.map((opt) => (
                  <SelectItem key={opt.value} value={opt.value}>
                    {opt.label}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>
        </div>
      </div>
    );
  }

  /* ------------------------------- Empty State ------------------------------- */
  return (
    <div
      className={cn(
        "flex h-24 items-center justify-center text-center",
        className,
      )}
    >
      <div>
        <Settings className="text-muted-foreground/50 mx-auto mb-2 h-6 w-6" />
        <h3 className="mb-1 text-sm font-medium">Select Step or Action</h3>
        <p className="text-muted-foreground text-xs">
          Click in the flow to edit properties
        </p>
      </div>
    </div>
  );
}
