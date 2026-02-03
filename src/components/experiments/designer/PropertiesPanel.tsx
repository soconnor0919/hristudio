"use client";

import React, { useState, useEffect, useCallback, useRef } from "react";
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

export function PropertiesPanelBase({
  design,
  selectedStep,
  selectedAction,
  onActionUpdate,
  onStepUpdate,
  className,
}: PropertiesPanelProps) {
  const registry = actionRegistry;

  // Local state for controlled inputs
  const [localActionName, setLocalActionName] = useState("");
  const [localStepName, setLocalStepName] = useState("");
  const [localStepDescription, setLocalStepDescription] = useState("");
  const [localParams, setLocalParams] = useState<Record<string, unknown>>({});

  // Debounce timers
  const actionUpdateTimer = useRef<NodeJS.Timeout | undefined>(undefined);
  const stepUpdateTimer = useRef<NodeJS.Timeout | undefined>(undefined);
  const paramUpdateTimers = useRef(new Map<string, NodeJS.Timeout>());

  // Sync local state when selection ID changes (not on every object recreation)
  useEffect(() => {
    if (selectedAction) {
      setLocalActionName(selectedAction.name);
      setLocalParams(selectedAction.parameters);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedAction?.id]);

  useEffect(() => {
    if (selectedStep) {
      setLocalStepName(selectedStep.name);
      setLocalStepDescription(selectedStep.description ?? "");
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedStep?.id]);

  // Cleanup timers on unmount
  useEffect(() => {
    const timersMap = paramUpdateTimers.current;
    return () => {
      if (actionUpdateTimer.current) clearTimeout(actionUpdateTimer.current);
      if (stepUpdateTimer.current) clearTimeout(stepUpdateTimer.current);
      timersMap.forEach((timer) => clearTimeout(timer));
    };
  }, []);

  // Debounced update handlers
  const debouncedActionUpdate = useCallback(
    (stepId: string, actionId: string, updates: Partial<ExperimentAction>) => {
      if (actionUpdateTimer.current) clearTimeout(actionUpdateTimer.current);
      actionUpdateTimer.current = setTimeout(() => {
        onActionUpdate(stepId, actionId, updates);
      }, 300);
    },
    [onActionUpdate],
  );

  const debouncedStepUpdate = useCallback(
    (stepId: string, updates: Partial<ExperimentStep>) => {
      if (stepUpdateTimer.current) clearTimeout(stepUpdateTimer.current);
      stepUpdateTimer.current = setTimeout(() => {
        onStepUpdate(stepId, updates);
      }, 300);
    },
    [onStepUpdate],
  );

  const debouncedParamUpdate = useCallback(
    (stepId: string, actionId: string, paramId: string, value: unknown) => {
      const existing = paramUpdateTimers.current.get(paramId);
      if (existing) clearTimeout(existing);

      const timer = setTimeout(() => {
        onActionUpdate(stepId, actionId, {
          parameters: {
            ...selectedAction?.parameters,
            [paramId]: value,
          },
        });
        paramUpdateTimers.current.delete(paramId);
      }, 300);

      paramUpdateTimers.current.set(paramId, timer);
    },
    [onActionUpdate, selectedAction?.parameters],
  );

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
      <div className={cn("w-full min-w-0 space-y-3 px-3", className)} id="tour-designer-properties">
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
              <p className="text-muted-foreground text-xs capitalize">
                {def?.category}
              </p>
            </div>
          </div>
          <div className="flex flex-wrap gap-1">
            <Badge variant="outline" className="h-4 text-[10px]">
              {selectedAction.source.kind === "plugin" ? "Plugin" : "Core"}
            </Badge>
            {/* internal plugin identifiers hidden from UI */}
            <Badge variant="outline" className="h-4 text-[10px]">
              {selectedAction.execution?.transport}
            </Badge>
            {selectedAction.execution?.retryable && (
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

        {/* General */}
        <div className="space-y-2">
          <div className="text-muted-foreground text-[10px] tracking-wide uppercase">
            General
          </div>
          <div>
            <Label className="text-xs">Display Name</Label>
            <Input
              value={localActionName}
              onChange={(e) => {
                const newName = e.target.value;
                setLocalActionName(newName);
                debouncedActionUpdate(containingStep.id, selectedAction.id, {
                  name: newName,
                });
              }}
              onBlur={() => {
                if (localActionName !== selectedAction.name) {
                  onActionUpdate(containingStep.id, selectedAction.id, {
                    name: localActionName,
                  });
                }
              }}
              className="mt-1 h-7 w-full text-xs"
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
              {def.parameters.map((param) => (
                <ParameterEditor
                  key={param.id}
                  param={param}
                  value={selectedAction.parameters[param.id]}
                  onUpdate={(val) => {
                    onActionUpdate(containingStep.id, selectedAction.id, {
                      parameters: {
                        ...selectedAction.parameters,
                        [param.id]: val,
                      },
                    });
                  }}
                  onCommit={() => { }}
                />
              ))}
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
      <div className={cn("w-full min-w-0 space-y-3 px-3", className)} id="tour-designer-properties">
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
        <div className="space-y-3">
          <div>
            <div className="text-muted-foreground text-[10px] tracking-wide uppercase">
              General
            </div>
            <div className="mt-2 space-y-2">
              <div>
                <Label className="text-xs">Name</Label>
                <Input
                  value={localStepName}
                  onChange={(e) => {
                    const newName = e.target.value;
                    setLocalStepName(newName);
                    debouncedStepUpdate(selectedStep.id, { name: newName });
                  }}
                  onBlur={() => {
                    if (localStepName !== selectedStep.name) {
                      onStepUpdate(selectedStep.id, { name: localStepName });
                    }
                  }}
                  className="mt-1 h-7 w-full text-xs"
                />
              </div>
              <div>
                <Label className="text-xs">Description</Label>
                <Input
                  value={localStepDescription}
                  placeholder="Optional step description"
                  onChange={(e) => {
                    const newDesc = e.target.value;
                    setLocalStepDescription(newDesc);
                    debouncedStepUpdate(selectedStep.id, {
                      description: newDesc,
                    });
                  }}
                  onBlur={() => {
                    if (
                      localStepDescription !== (selectedStep.description ?? "")
                    ) {
                      onStepUpdate(selectedStep.id, {
                        description: localStepDescription,
                      });
                    }
                  }}
                  className="mt-1 h-7 w-full text-xs"
                />
              </div>
            </div>
          </div>

          <div>
            <div className="text-muted-foreground text-[10px] tracking-wide uppercase">
              Behavior
            </div>
            <div className="mt-2 space-y-2">
              <div>
                <Label className="text-xs">Type</Label>
                <Select
                  value={selectedStep.type}
                  onValueChange={(val) => {
                    onStepUpdate(selectedStep.id, { type: val as StepType });
                  }}
                  disabled={true}
                >
                  <SelectTrigger className="mt-1 h-7 w-full text-xs">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="sequential">Sequential</SelectItem>
                  </SelectContent>
                </Select>
                <p className="text-muted-foreground mt-1 text-[10px]">
                  Steps always execute sequentially. Use control flow actions for parallel/conditional logic.
                </p>
              </div>
              <div>
                <Label className="text-xs">Trigger</Label>
                <Select
                  value={selectedStep.trigger.type}
                  onValueChange={(val) => {
                    onStepUpdate(selectedStep.id, {
                      trigger: {
                        ...selectedStep.trigger,
                        type: val as TriggerType,
                      },
                    });
                  }}
                >
                  <SelectTrigger className="mt-1 h-7 w-full text-xs">
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
      id="tour-designer-properties"
    >
      <div>
        <Settings className="text-muted-foreground/50 mx-auto mb-2 h-6 w-6" />
        <h3 className="mb-1 text-sm font-medium">No selection</h3>
        <p className="text-muted-foreground text-xs">
          Select a step or action in the flow to edit its properties.
        </p>
      </div>
    </div>
  );
}

export const PropertiesPanel = React.memo(PropertiesPanelBase);

/* -------------------------------------------------------------------------- */
/* Isolated Parameter Editor (Optimized)                                      */
/* -------------------------------------------------------------------------- */

interface ParameterEditorProps {
  param: any;
  value: unknown;
  onUpdate: (value: unknown) => void;
  onCommit: () => void;
}

const ParameterEditor = React.memo(function ParameterEditor({
  param,
  value: rawValue,
  onUpdate,
  onCommit
}: ParameterEditorProps) {
  // Local state for immediate feedback
  const [localValue, setLocalValue] = useState<unknown>(rawValue);
  const debounceRef = useRef<NodeJS.Timeout | undefined>(undefined);

  // Sync from prop if it changes externally
  useEffect(() => {
    setLocalValue(rawValue);
  }, [rawValue]);

  const handleUpdate = useCallback((newVal: unknown, immediate = false) => {
    setLocalValue(newVal);

    if (debounceRef.current) clearTimeout(debounceRef.current);

    if (immediate) {
      onUpdate(newVal);
    } else {
      debounceRef.current = setTimeout(() => {
        onUpdate(newVal);
      }, 300);
    }
  }, [onUpdate]);

  const handleCommit = useCallback(() => {
    if (localValue !== rawValue) {
      onUpdate(localValue);
    }
  }, [localValue, rawValue, onUpdate]);

  let control: React.ReactNode = null;

  if (param.type === "text") {
    control = (
      <Input
        value={(localValue as string) ?? ""}
        placeholder={param.placeholder}
        onChange={(e) => handleUpdate(e.target.value)}
        onBlur={handleCommit}
        className="mt-1 h-7 w-full text-xs"
      />
    );
  } else if (param.type === "select") {
    control = (
      <Select
        value={(localValue as string) ?? ""}
        onValueChange={(val) => handleUpdate(val, true)}
      >
        <SelectTrigger className="mt-1 h-7 w-full text-xs">
          <SelectValue placeholder="Select…" />
        </SelectTrigger>
        <SelectContent>
          {param.options?.map((opt: string) => (
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
          checked={Boolean(localValue)}
          onCheckedChange={(val) => handleUpdate(val, true)}
          aria-label={param.name}
        />
        <span className="text-muted-foreground ml-2 text-[11px]">
          {Boolean(localValue) ? "Enabled" : "Disabled"}
        </span>
      </div>
    );
  } else if (param.type === "number") {
    const numericVal = typeof localValue === "number" ? localValue : (param.min ?? 0);

    if (param.min !== undefined || param.max !== undefined) {
      const min = param.min ?? 0;
      const max = param.max ?? Math.max(min + 1, Number.isFinite(numericVal) ? numericVal : min + 1);
      const range = max - min;
      const step = param.step ?? (range <= 5 ? 0.1 : range <= 50 ? 0.5 : Math.max(1, Math.round(range / 100)));

      control = (
        <div className="mt-1">
          <div className="flex items-center gap-2">
            <Slider
              min={min}
              max={max}
              step={step}
              value={[Number(numericVal)]}
              onValueChange={(vals) => setLocalValue(vals[0])} // Update only local visual
              onPointerUp={() => handleUpdate(localValue)} // Commit on release
            />
            <span className="text-muted-foreground min-w-[2.5rem] text-right text-[10px] tabular-nums">
              {step < 1 ? Number(numericVal).toFixed(2) : Number(numericVal).toString()}
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
          onChange={(e) => handleUpdate(parseFloat(e.target.value) || 0)}
          onBlur={handleCommit}
          className="mt-1 h-7 w-full text-xs"
        />
      );
    }
  }

  return (
    <div className="space-y-1">
      <Label className="flex items-center gap-2 text-xs">
        {param.name}
        <span className="text-muted-foreground font-normal">
          {param.type === "number" &&
            (param.min !== undefined || param.max !== undefined) &&
            typeof rawValue === "number" &&
            `( ${rawValue} )`}
        </span>
      </Label>
      {param.description && (
        <div className="text-muted-foreground text-[10px]">
          {param.description}
        </div>
      )}
      {control}
    </div>
  );
});
