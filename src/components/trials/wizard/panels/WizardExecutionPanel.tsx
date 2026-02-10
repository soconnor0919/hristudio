"use client";

import React from "react";
import {
  Play,
  SkipForward,
  CheckCircle,
  AlertCircle,
  ArrowRight,
  Zap,
  Loader2,
  Clock,
  RotateCcw,
  AlertTriangle,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { ScrollArea } from "~/components/ui/scroll-area";

interface StepData {
  id: string;
  name: string;
  description: string | null;
  type:
  | "wizard_action"
  | "robot_action"
  | "parallel_steps"
  | "conditional";
  parameters: Record<string, unknown>;
  conditions?: {
    options?: {
      label: string;
      value: string;
      nextStepId?: string;
      nextStepIndex?: number;
      variant?: "default" | "destructive" | "outline" | "secondary" | "ghost" | "link";
    }[];
  };
  order: number;
  actions?: {
    id: string;
    name: string;
    description: string | null;
    type: string;
    parameters: Record<string, unknown>;
    order: number;
    pluginId: string | null;
  }[];
}

interface TrialData {
  id: string;
  status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
  scheduledAt: Date | null;
  startedAt: Date | null;
  completedAt: Date | null;
  duration: number | null;
  sessionNumber: number | null;
  notes: string | null;
  experimentId: string;
  participantId: string | null;
  wizardId: string | null;
  experiment: {
    id: string;
    name: string;
    description: string | null;
    studyId: string;
  };
  participant: {
    id: string;
    participantCode: string;
    demographics: Record<string, unknown> | null;
  };
}

interface TrialEvent {
  type: string;
  timestamp: Date;
  data?: unknown;
  message?: string;
}

interface WizardExecutionPanelProps {
  trial: TrialData;
  currentStep: StepData | null;
  steps: StepData[];
  currentStepIndex: number;
  trialEvents: TrialEvent[];
  onStepSelect: (index: number) => void;
  onExecuteAction: (
    actionId: string,
    parameters?: Record<string, unknown>,
  ) => void;
  onExecuteRobotAction: (
    pluginName: string,
    actionId: string,
    parameters: Record<string, unknown>,
    options?: { autoAdvance?: boolean },
  ) => Promise<void>;
  activeTab: "current" | "timeline" | "events"; // Deprecated/Ignored
  onTabChange: (tab: "current" | "timeline" | "events") => void; // Deprecated/Ignored
  onSkipAction: (
    pluginName: string,
    actionId: string,
    parameters: Record<string, unknown>,
    options?: { autoAdvance?: boolean },
  ) => Promise<void>;
  isExecuting?: boolean;
  onNextStep?: () => void;
  onCompleteTrial?: () => void;
  completedActionsCount: number;
  onActionCompleted: () => void;
  readOnly?: boolean;
}

export function WizardExecutionPanel({
  trial,
  currentStep,
  steps,
  currentStepIndex,
  trialEvents,
  onStepSelect,
  onExecuteAction,
  onExecuteRobotAction,
  activeTab,
  onTabChange,
  onSkipAction,
  isExecuting = false,
  onNextStep,
  onCompleteTrial,
  completedActionsCount,
  onActionCompleted,
  readOnly = false,
}: WizardExecutionPanelProps) {
  // Local state removed in favor of parent state to prevent reset on re-render
  // const [completedCount, setCompletedCount] = React.useState(0);

  const activeActionIndex = completedActionsCount;

  // Auto-scroll to active action
  const activeActionRef = React.useRef<HTMLDivElement>(null);

  React.useEffect(() => {
    if (activeActionRef.current) {
      activeActionRef.current.scrollIntoView({
        behavior: "smooth",
        block: "center",
      });
    }
  }, [activeActionIndex, currentStepIndex]);

  // Pre-trial state
  if (trial.status === "scheduled") {
    return (
      <div className="flex h-full flex-col">
        <div className="flex-1 flex items-center justify-center p-6">
          <div className="w-full max-w-md space-y-4 text-center">
            <Clock className="text-muted-foreground mx-auto h-12 w-12 opacity-20" />
            <div>
              <h4 className="text-lg font-medium">Ready to Begin</h4>
              <p className="text-muted-foreground text-sm">
                {steps.length} steps prepared. Use controls to start.
              </p>
            </div>
          </div>
        </div>
      </div>
    );
  }

  // Post-trial state
  if (
    trial.status === "completed" ||
    trial.status === "aborted" ||
    trial.status === "failed"
  ) {
    return (
      <div className="flex h-full flex-col">
        <div className="border-b p-3">
          <h3 className="text-sm font-medium">
            Trial {trial.status === "completed" ? "Completed" : "Ended"}
          </h3>
          <p className="text-muted-foreground text-xs">
            {trial.completedAt &&
              `Ended at ${new Date(trial.completedAt).toLocaleTimeString()} `}
          </p>
        </div>

        <div className="flex h-full flex-1 items-center justify-center p-6">
          <div className="w-full max-w-md space-y-3 text-center">
            <CheckCircle className="text-muted-foreground mx-auto h-8 w-8" />
            <div>
              <h4 className="text-sm font-medium">Execution Complete</h4>
              <p className="text-muted-foreground text-xs">
                Review results and captured data
              </p>
            </div>
            <div className="text-muted-foreground text-xs">
              {trialEvents.length} events recorded
            </div>
          </div>
        </div>
      </div>
    );
  }

  // Active trial state
  return (
    <div className="flex h-full flex-col overflow-hidden">
      <div className="flex-1 min-h-0 relative">
        <ScrollArea className="h-full w-full">
          <div className="pr-4">
            {currentStep ? (
              <div className="flex flex-col gap-4 p-4 max-w-2xl mx-auto">
                {/* Header Info */}
                <div className="space-y-1 pb-4 border-b">
                  <h2 className="text-xl font-bold tracking-tight">{currentStep.name}</h2>
                  {currentStep.description && (
                    <div className="text-muted-foreground">{currentStep.description}</div>
                  )}
                </div>

                {/* Action Sequence */}
                {currentStep.actions && currentStep.actions.length > 0 && (
                  <div className="relative ml-3 space-y-0 pt-2">
                    {currentStep.actions.map((action, idx) => {
                      const isCompleted = idx < activeActionIndex;
                      const isActive = idx === activeActionIndex;
                      const isLast = idx === currentStep.actions!.length - 1;

                      return (
                        <div
                          key={action.id}
                          className="relative pl-8 pb-10 last:pb-0"
                          ref={isActive ? activeActionRef : undefined}
                        >
                          {/* Connecting Line */}
                          {!isLast && (
                            <div
                              className={`absolute left-[11px] top-8 bottom-0 w-[2px] ${isCompleted ? "bg-primary/20" : "bg-border/40"}`}
                            />
                          )}

                          {/* Marker */}
                          <div
                            className={`absolute left-0 top-1 h-6 w-6 rounded-full border-2 flex items-center justify-center z-10 bg-background transition-all duration-300 ${isCompleted
                              ? "border-primary bg-primary text-primary-foreground"
                              : isActive
                                ? "border-primary ring-4 ring-primary/10 scale-110"
                                : "border-muted-foreground/30 text-muted-foreground"
                              }`}
                          >
                            {isCompleted ? (
                              <CheckCircle className="h-3.5 w-3.5" />
                            ) : (
                              <span className="text-[10px] font-bold">{idx + 1}</span>
                            )}
                          </div>

                          {/* Content Card */}
                          <div
                            className={`rounded-lg border transition-all duration-300 ${isActive
                              ? "bg-card border-primary/50 shadow-md p-5 translate-x-1"
                              : "bg-muted/5 border-transparent p-3 opacity-70 hover:opacity-100"
                              }`}
                          >
                            <div className="space-y-2">
                              <div className="flex items-start justify-between gap-4">
                                <div
                                  className={`text-base font-medium leading-none ${isCompleted ? "line-through text-muted-foreground" : ""
                                    }`}
                                >
                                  {action.name}
                                </div>
                              </div>

                              {action.description && (
                                <div className="text-sm text-muted-foreground">
                                  {action.description}
                                </div>
                              )}

                              {/* Active Action Controls */}
                              {isActive && (
                                <div className="pt-3 flex items-center gap-3">
                                  {action.pluginId && !["hristudio-core", "hristudio-woz"].includes(action.pluginId) ? (
                                    <>
                                      <Button
                                        size="sm"
                                        className="shadow-sm min-w-[100px]"
                                        onClick={(e) => {
                                          e.preventDefault();
                                          onExecuteRobotAction(
                                            action.pluginId!,
                                            action.type.includes(".")
                                              ? action.type.split(".").pop()!
                                              : action.type,
                                            action.parameters || {},
                                            { autoAdvance: false }
                                          );
                                          onActionCompleted();
                                        }}
                                        disabled={readOnly || isExecuting}
                                      >
                                        <Play className="mr-2 h-3.5 w-3.5" />
                                        Execute
                                      </Button>
                                      <Button
                                        size="sm"
                                        variant="ghost"
                                        className="text-muted-foreground hover:text-foreground"
                                        onClick={(e) => {
                                          e.preventDefault();
                                          onSkipAction(
                                            action.pluginId!,
                                            action.type.includes(".")
                                              ? action.type.split(".").pop()!
                                              : action.type,
                                            action.parameters || {},
                                            { autoAdvance: false }
                                          );
                                          onActionCompleted();
                                        }}
                                        disabled={readOnly}
                                      >
                                        Skip
                                      </Button>
                                    </>
                                  ) : (
                                    <Button
                                      size="sm"
                                      onClick={(e) => {
                                        e.preventDefault();
                                        onActionCompleted();
                                      }}
                                      disabled={readOnly || isExecuting}
                                    >
                                      Mark Done
                                    </Button>
                                  )}
                                </div>
                              )}

                              {/* Wizard Wait For Response / Branching UI */}
                              {isActive && action.type === 'wizard_wait_for_response' && action.parameters?.options && Array.isArray(action.parameters.options) && (
                                <div className="pt-3 grid grid-cols-1 sm:grid-cols-2 gap-2">
                                  {(action.parameters.options as any[]).map((opt, optIdx) => {
                                    // Handle both string options and object options
                                    const label = typeof opt === 'string' ? opt : opt.label;
                                    const value = typeof opt === 'string' ? opt : opt.value;
                                    const nextStepId = typeof opt === 'object' ? opt.nextStepId : undefined;

                                    return (
                                      <Button
                                        key={optIdx}
                                        variant="outline"
                                        className="justify-start h-auto py-3 px-4 text-left border-primary/20 hover:border-primary hover:bg-primary/5"
                                        onClick={(e) => {
                                          e.preventDefault();
                                          onExecuteAction(
                                            action.id,
                                            {
                                              value,
                                              label,
                                              nextStepId
                                            }
                                          );
                                          onActionCompleted();
                                        }}
                                        disabled={readOnly || isExecuting}
                                      >
                                        <div className="flex flex-col items-start gap-1">
                                          <span className="font-medium">{String(label)}</span>
                                          {typeof opt !== 'string' && value && <span className="text-xs text-muted-foreground font-mono bg-muted px-1.5 py-0.5 rounded-sm">{String(value)}</span>}
                                        </div>
                                      </Button>
                                    );
                                  })}
                                </div>
                              )}

                              {/* Completed State Actions */}
                              {isCompleted && action.pluginId && (
                                <div className="pt-1 flex items-center gap-1">
                                  <Button
                                    size="sm"
                                    variant="ghost"
                                    className="h-7 px-2 text-xs text-muted-foreground hover:text-primary"
                                    onClick={(e) => {
                                      e.preventDefault();
                                      onExecuteRobotAction(
                                        action.pluginId!,
                                        action.type.includes(".") ? action.type.split(".").pop()! : action.type,
                                        action.parameters || {},
                                        { autoAdvance: false },
                                      );
                                    }}
                                    disabled={readOnly || isExecuting}
                                  >
                                    <RotateCcw className="mr-1.5 h-3 w-3" />
                                    Retry
                                  </Button>
                                </div>
                              )}
                            </div>
                          </div>
                        </div>
                      );
                    })}
                  </div>
                )
                }

                {/* Manual Advance Button */}
                {activeActionIndex >= (currentStep.actions?.length || 0) && (
                  <div className="mt-6 flex justify-center pb-8">
                    <Button
                      size="lg"
                      onClick={currentStepIndex === steps.length - 1 ? onCompleteTrial : onNextStep}
                      className={`w-full max-w-sm text-white shadow-lg transition-all hover:scale-[1.02] ${currentStepIndex === steps.length - 1
                        ? "bg-blue-600 hover:bg-blue-700"
                        : "bg-green-600 hover:bg-green-700"
                        }`}
                      disabled={readOnly || isExecuting}
                    >
                      {currentStepIndex === steps.length - 1 ? "Complete Trial" : "Complete Step"}
                      <ArrowRight className="ml-2 h-5 w-5" />
                    </Button>
                  </div>
                )}
              </div>
            ) : (
              <div className="flex h-full flex-col items-center justify-center text-muted-foreground space-y-3">
                <Loader2 className="h-8 w-8 animate-spin opacity-50" />
                <div className="text-sm">Waiting for trial to start...</div>
              </div>
            )}
          </div>
        </ScrollArea>
      </div>
    </div>
  );
}
