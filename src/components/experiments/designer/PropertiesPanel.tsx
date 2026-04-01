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
import { Button } from "~/components/ui/button";
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
  Plus,
  GitBranch,
  Trash2,
  PlayCircle,
  Square,
  Loader2,
  CheckCircle2,
  XCircle,
} from "lucide-react";
import { toast } from "sonner";
import { getWizardRosService, initWizardRosService, resetWizardRosService } from "~/lib/ros/wizard-ros-service";

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

  // Test action state
  const [isTesting, setIsTesting] = useState(false);
  const [testStatus, setTestStatus] = useState<"idle" | "running" | "success" | "error">("idle");

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

  // Test action handler
  const handleTestAction = useCallback(async () => {
    if (!selectedAction || !containingStep) return;

    setIsTesting(true);
    setTestStatus("running");

    try {
      console.log("[Test Action] Starting test for action:", selectedAction.name, selectedAction.type);
      console.log("[Test Action] Execution config:", JSON.stringify(selectedAction.execution, null, 2));
      console.log("[Test Action] Parameters:", selectedAction.parameters);

      // Reset service to ensure clean state for testing
      resetWizardRosService();
      
      // Initialize with actual robot connection (not simulation)
      const rosService = await initWizardRosService(false);
      console.log("[Test Action] ROS service initialized, connected:", rosService.getConnectionStatus());

      // Build action config from execution descriptor
      const execution = selectedAction.execution;
      let actionConfig: {
        topic: string;
        messageType: string;
        payloadMapping: {
          type: string;
          payload?: Record<string, unknown>;
          transformFn?: string;
        };
      } | undefined;

      if (execution?.transport === "ros2" && execution.ros2) {
        const ros2 = execution.ros2 as any;
        actionConfig = {
          topic: ros2.topic || "/speech",
          messageType: ros2.messageType || "std_msgs/msg/String",
          payloadMapping: {
            type: ros2.payloadMapping?.type || "static",
            payload: ros2.payloadMapping?.payload,
            transformFn: ros2.payloadMapping?.transformFn,
          },
        };
        console.log("[Test Action] Action config built:", JSON.stringify(actionConfig, null, 2));
      }

      // Execute the action on the real robot
      const result = await rosService.executeRobotAction(
        selectedAction.source?.kind === "plugin" ? (selectedAction.source?.pluginId || "core") : "core",
        selectedAction.type,
        selectedAction.parameters,
        actionConfig,
      );
      console.log("[Test Action] Execution result:", result);

      setTestStatus("success");
      toast.success(`Action "${selectedAction.name}" executed on robot`);
    } catch (error) {
      setTestStatus("error");
      const message = error instanceof Error ? error.message : "Action execution failed";
      toast.error(message);
      console.error("Test action error:", error);
    } finally {
      setIsTesting(false);
      // Reset status after a delay
      setTimeout(() => setTestStatus("idle"), 2000);
    }
  }, [selectedAction, containingStep]);

  /* -------------------------- Action Properties View -------------------------- */
  if (selectedAction && containingStep) {
    let def = registry.getAction(selectedAction.type);

    // Fallback: If action not found in registry, try without plugin prefix
    if (!def && selectedAction.type.includes(".")) {
      const baseType = selectedAction.type.split(".").pop();
      if (baseType) {
        def = registry.getAction(baseType);
      }
    }

    // Final fallback: Create minimal definition from action data
    if (!def) {
      def = {
        id: selectedAction.type,
        type: selectedAction.type,
        name: selectedAction.name,
        description: `Action type: ${selectedAction.type}`,
        category: selectedAction.category || "control",
        icon: "Zap",
        color: "#6366f1",
        parameters: [],
        source: selectedAction.source,
      };
    }
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
      <div
        className={cn("w-full min-w-0 space-y-3 px-3", className)}
        id="tour-designer-properties"
      >
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

        {/* Test Action Button */}
        {selectedAction.execution?.transport !== "internal" && (
          <div className="flex items-center gap-2">
            <Button
              variant="outline"
              size="sm"
              className="w-full gap-1.5"
              onClick={handleTestAction}
              disabled={isTesting}
            >
              {testStatus === "running" ? (
                <>
                  <Loader2 className="h-4 w-4 animate-spin" />
                  Running...
                </>
              ) : testStatus === "success" ? (
                <>
                  <CheckCircle2 className="h-4 w-4 text-green-500" />
                  Success!
                </>
              ) : testStatus === "error" ? (
                <>
                  <XCircle className="h-4 w-4 text-red-500" />
                  Failed
                </>
              ) : (
                <>
                  <PlayCircle className="h-4 w-4" />
                  Test Action
                </>
              )}
            </Button>
          </div>
        )}

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

        {/* Branching Configuration (Special Case) */}
        {selectedAction.type === "branch" ? (
          <div className="space-y-3">
            <div className="text-muted-foreground flex items-center justify-between text-[10px] tracking-wide uppercase">
              <span>Branch Options</span>
              <Button
                variant="ghost"
                size="sm"
                className="h-5 w-5 p-0"
                onClick={() => {
                  const currentOptions =
                    ((containingStep.trigger.conditions as any)
                      .options as any[]) || [];
                  const newOptions = [
                    ...currentOptions,
                    {
                      label: "New Option",
                      nextStepId: design.steps[containingStep.order + 1]?.id,
                      variant: "default",
                    },
                  ];

                  // Sync to Step Trigger (Source of Truth)
                  onStepUpdate(containingStep.id, {
                    trigger: {
                      ...containingStep.trigger,
                      conditions: {
                        ...containingStep.trigger.conditions,
                        options: newOptions,
                      },
                    },
                  });
                  // Sync to Action Params (for consistency)
                  onActionUpdate(containingStep.id, selectedAction.id, {
                    parameters: {
                      ...selectedAction.parameters,
                      options: newOptions,
                    },
                  });
                }}
              >
                <Plus className="h-3.5 w-3.5" />
              </Button>
            </div>

            <div className="space-y-3">
              {(
                ((containingStep.trigger.conditions as any).options as any[]) ||
                []
              ).map((opt: any, idx: number) => (
                <div
                  key={idx}
                  className="bg-muted/50 space-y-2 rounded border p-2"
                >
                  <div className="grid grid-cols-5 gap-2">
                    <div className="col-span-3">
                      <Label className="text-[10px]">Label</Label>
                      <Input
                        value={opt.label}
                        onChange={(e) => {
                          const currentOptions =
                            ((containingStep.trigger.conditions as any)
                              .options as any[]) || [];
                          const newOpts = [...currentOptions];
                          newOpts[idx] = {
                            ...newOpts[idx],
                            label: e.target.value,
                          };

                          onStepUpdate(containingStep.id, {
                            trigger: {
                              ...containingStep.trigger,
                              conditions: {
                                ...containingStep.trigger.conditions,
                                options: newOpts,
                              },
                            },
                          });
                          onActionUpdate(containingStep.id, selectedAction.id, {
                            parameters: {
                              ...selectedAction.parameters,
                              options: newOpts,
                            },
                          });
                        }}
                        className="h-7 text-xs"
                      />
                    </div>
                    <div className="col-span-2">
                      <Label className="text-[10px]">Target Step</Label>
                      {design.steps.length <= 1 ? (
                        <div
                          className="text-muted-foreground bg-muted/50 flex h-7 items-center truncate rounded border px-2 text-[10px]"
                          title="Add more steps to link"
                        >
                          No linkable steps
                        </div>
                      ) : (
                        <Select
                          value={opt.nextStepId ?? ""}
                          onValueChange={(val) => {
                            const currentOptions =
                              ((containingStep.trigger.conditions as any)
                                .options as any[]) || [];
                            const newOpts = [...currentOptions];
                            newOpts[idx] = { ...newOpts[idx], nextStepId: val };

                            onStepUpdate(containingStep.id, {
                              trigger: {
                                ...containingStep.trigger,
                                conditions: {
                                  ...containingStep.trigger.conditions,
                                  options: newOpts,
                                },
                              },
                            });
                            onActionUpdate(
                              containingStep.id,
                              selectedAction.id,
                              {
                                parameters: {
                                  ...selectedAction.parameters,
                                  options: newOpts,
                                },
                              },
                            );
                          }}
                        >
                          <SelectTrigger className="h-7 w-full text-xs">
                            <SelectValue placeholder="Select..." />
                          </SelectTrigger>
                          <SelectContent className="min-w-[180px]">
                            {design.steps.map((s) => (
                              <SelectItem
                                key={s.id}
                                value={s.id}
                                disabled={s.id === containingStep.id}
                              >
                                {s.order + 1}. {s.name}
                              </SelectItem>
                            ))}
                          </SelectContent>
                        </Select>
                      )}
                    </div>
                  </div>
                  <div className="flex items-center justify-between">
                    <Select
                      value={opt.variant || "default"}
                      onValueChange={(val) => {
                        const currentOptions =
                          ((containingStep.trigger.conditions as any)
                            .options as any[]) || [];
                        const newOpts = [...currentOptions];
                        newOpts[idx] = { ...newOpts[idx], variant: val };

                        onStepUpdate(containingStep.id, {
                          trigger: {
                            ...containingStep.trigger,
                            conditions: {
                              ...containingStep.trigger.conditions,
                              options: newOpts,
                            },
                          },
                        });
                        onActionUpdate(containingStep.id, selectedAction.id, {
                          parameters: {
                            ...selectedAction.parameters,
                            options: newOpts,
                          },
                        });
                      }}
                    >
                      <SelectTrigger className="h-6 w-[120px] text-[10px]">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="default">Default (Next)</SelectItem>
                        <SelectItem value="destructive">
                          Destructive (Red)
                        </SelectItem>
                        <SelectItem value="outline">Outline</SelectItem>
                      </SelectContent>
                    </Select>

                    <Button
                      variant="ghost"
                      size="sm"
                      className="text-muted-foreground h-6 w-6 p-0 hover:text-red-500"
                      onClick={() => {
                        const currentOptions =
                          ((containingStep.trigger.conditions as any)
                            .options as any[]) || [];
                        const newOpts = [...currentOptions];
                        newOpts.splice(idx, 1);

                        onStepUpdate(containingStep.id, {
                          trigger: {
                            ...containingStep.trigger,
                            conditions: {
                              ...containingStep.trigger.conditions,
                              options: newOpts,
                            },
                          },
                        });
                        onActionUpdate(containingStep.id, selectedAction.id, {
                          parameters: {
                            ...selectedAction.parameters,
                            options: newOpts,
                          },
                        });
                      }}
                    >
                      <Trash2 className="h-3 w-3" />
                    </Button>
                  </div>
                </div>
              ))}
              {!((containingStep.trigger.conditions as any).options as any[])
                ?.length && (
                <div className="text-muted-foreground rounded border border-dashed py-4 text-center text-xs">
                  No options defined.
                  <br />
                  Click + to add a branch.
                </div>
              )}
            </div>
          </div>
        ) : selectedAction.type === "loop" ? (
          /* Loop Configuration */
          <div className="space-y-3">
            <div className="text-muted-foreground text-[10px] tracking-wide uppercase">
              Loop Configuration
            </div>

            <div className="space-y-4">
              {/* Iterations */}
              <div>
                <Label className="text-xs">Iterations</Label>
                <div className="mt-1 flex items-center gap-2">
                  <Slider
                    min={1}
                    max={20}
                    step={1}
                    value={[Number(selectedAction.parameters.iterations || 1)]}
                    onValueChange={(vals) => {
                      onActionUpdate(containingStep.id, selectedAction.id, {
                        parameters: {
                          ...selectedAction.parameters,
                          iterations: vals[0],
                        },
                      });
                    }}
                  />
                  <span className="w-8 text-right font-mono text-xs">
                    {Number(selectedAction.parameters.iterations || 1)}
                  </span>
                </div>
              </div>
            </div>
          </div>
        ) : /* Standard Parameters */
        def?.parameters.length ? (
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
                  onCommit={() => {}}
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
      <div
        className={cn("w-full min-w-0 space-y-3 px-3", className)}
        id="tour-designer-properties"
      >
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
                  Steps always execute sequentially. Use control flow actions
                  for parallel/conditional logic.
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
  onCommit,
}: ParameterEditorProps) {
  // Local state for immediate feedback
  const [localValue, setLocalValue] = useState<unknown>(rawValue);
  const debounceRef = useRef<NodeJS.Timeout | undefined>(undefined);

  // Sync from prop if it changes externally
  useEffect(() => {
    setLocalValue(rawValue);
  }, [rawValue]);

  const handleUpdate = useCallback(
    (newVal: unknown, immediate = false) => {
      setLocalValue(newVal);

      if (debounceRef.current) clearTimeout(debounceRef.current);

      if (immediate) {
        onUpdate(newVal);
      } else {
        debounceRef.current = setTimeout(() => {
          onUpdate(newVal);
        }, 300);
      }
    },
    [onUpdate],
  );

  const handleCommit = useCallback(() => {
    if (localValue !== rawValue) {
      onUpdate(localValue);
    }
  }, [localValue, rawValue, onUpdate]);

  let control: React.ReactNode = null;

  if (param.type === "text") {
    control = (
      <textarea
        value={(localValue as string) ?? ""}
        placeholder={param.placeholder}
        onChange={(e) => handleUpdate(e.target.value)}
        onBlur={handleCommit}
        rows={3}
        className="mt-1 w-full rounded-md border border-input bg-transparent px-3 py-2 text-xs shadow-sm placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring disabled:cursor-not-allowed disabled:opacity-50"
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
    const numericVal =
      typeof localValue === "number" ? localValue : (param.min ?? 0);

    if (param.min !== undefined || param.max !== undefined) {
      const min = param.min ?? 0;
      const max =
        param.max ??
        Math.max(min + 1, Number.isFinite(numericVal) ? numericVal : min + 1);
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
              onValueChange={(vals) => setLocalValue(vals[0])}
              onPointerUp={() => handleUpdate(localValue)}
            />
            <Input
              type="number"
              value={Number(numericVal)}
              min={min}
              max={max}
              step={step}
              onChange={(e) => {
                const v = parseFloat(e.target.value);
                if (!isNaN(v)) {
                  setLocalValue(Math.max(min, Math.min(max, v)));
                }
              }}
              onBlur={handleCommit}
              className="h-7 w-16 text-xs tabular-nums"
            />
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
