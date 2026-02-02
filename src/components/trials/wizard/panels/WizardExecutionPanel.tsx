"use client";

import React from "react";
import {
  Play,
  Clock,
  CheckCircle,
  AlertCircle,
  Bot,
  User,
  Activity,
  Zap,
  ArrowRight,
  AlertTriangle,
  RotateCcw,
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
  | "conditional_branch";
  parameters: Record<string, unknown>;
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

  // Pre-trial state
  if (trial.status === "scheduled") {
    return (
      <div className="flex h-full flex-col">
        <div className="border-b p-3">
          <h3 className="text-sm font-medium">Trial Ready</h3>
          <p className="text-muted-foreground text-xs">
            {steps.length} steps prepared for execution
          </p>
        </div>

        <div className="flex h-full flex-1 items-center justify-center p-6">
          <div className="w-full max-w-md space-y-3 text-center">
            <Clock className="text-muted-foreground mx-auto h-8 w-8" />
            <div>
              <h4 className="text-sm font-medium">Ready to Begin</h4>
              <p className="text-muted-foreground text-xs">
                Use the control panel to start this trial
              </p>
            </div>
            <div className="text-muted-foreground space-y-1 text-xs">
              <div>Experiment: {trial.experiment.name}</div>
              <div>Participant: {trial.participant.participantCode}</div>
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
    <div className="flex h-full flex-col">
      {/* Header */}
      <div className="border-b p-3">
        <div className="flex items-center justify-between">
          <h3 className="text-sm font-medium">Trial Execution</h3>
          <Badge variant="secondary" className="text-xs">
            {currentStepIndex + 1} / {steps.length}
          </Badge>
        </div>
        {currentStep && (
          <p className="text-muted-foreground mt-1 text-xs">
            {currentStep.name}
          </p>
        )}
      </div>

      {/* Simplified Content - Sequential Focus */}
      <div className="relative flex-1 overflow-hidden">
        <ScrollArea className="h-full">
          {currentStep ? (
            <div className="flex flex-col gap-6 p-6">
              {/* Header Info (Simplified) */}
              <div className="space-y-4">
                <div className="flex items-start justify-between">
                  <div>
                    <h2 className="text-xl font-bold tracking-tight">{currentStep.name}</h2>
                    {currentStep.description && (
                      <div className="text-muted-foreground text-sm mt-1">{currentStep.description}</div>
                    )}
                  </div>
                </div>
              </div>

              {/* Action Sequence */}
              {currentStep.actions && currentStep.actions.length > 0 && (
                <div className="space-y-4">
                  <div className="flex items-center justify-between">
                    <h3 className="text-sm font-semibold text-muted-foreground uppercase tracking-wider">
                      Execution Sequence
                    </h3>
                  </div>

                  <div className="grid gap-3">
                    {currentStep.actions.map((action, idx) => {
                      const isCompleted = idx < activeActionIndex;
                      const isActive = idx === activeActionIndex;

                      return (
                        <div
                          key={action.id}
                          className={`group relative flex items-center gap-4 rounded-xl border p-5 transition-all ${isActive ? "bg-card border-primary ring-1 ring-primary shadow-md" :
                            isCompleted ? "bg-muted/30 border-transparent opacity-70" :
                              "bg-card border-border opacity-50"
                            }`}
                        >
                          <div className={`flex h-8 w-8 flex-shrink-0 items-center justify-center rounded-full border text-sm font-medium ${isCompleted ? "bg-transparent text-green-600 border-green-600" :
                            isActive ? "bg-transparent text-primary border-primary font-bold shadow-sm" :
                              "bg-transparent text-muted-foreground border-transparent"
                            }`}>
                            {isCompleted ? <CheckCircle className="h-5 w-5" /> : idx + 1}
                          </div>

                          <div className="flex-1 min-w-0">
                            <div className={`font - medium truncate ${isCompleted ? "line-through text-muted-foreground" : ""} `}>{action.name}</div>
                            {action.description && (
                              <div className="text-xs text-muted-foreground line-clamp-1">
                                {action.description}
                              </div>
                            )}
                          </div>

                          {action.pluginId && isActive && (
                            <div className="flex items-center gap-2">
                              <Button
                                size="sm"
                                variant="ghost"
                                className="h-9 px-3 text-muted-foreground hover:text-foreground"
                                onClick={(e) => {
                                  e.preventDefault();
                                  e.stopPropagation();
                                  console.log("Skip clicked");
                                  // Fire and forget
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
                              <Button
                                size="default"
                                className="h-10 px-4 shadow-sm"
                                onClick={(e) => {
                                  e.preventDefault();
                                  e.stopPropagation();
                                  console.log("Execute clicked");
                                  onExecuteRobotAction(
                                    action.pluginId!,
                                    action.type.includes(".")
                                      ? action.type.split(".").pop()!
                                      : action.type,
                                    action.parameters || {},
                                    { autoAdvance: false },
                                  );
                                  onActionCompleted();
                                }}
                                disabled={readOnly || isExecuting}
                              >
                                <Play className="mr-2 h-4 w-4" />
                                Execute
                              </Button>
                            </div>
                          )}

                          {/* Fallback for actions with no plugin ID (e.g. manual steps) */}
                          {!action.pluginId && isActive && (
                            <div className="flex items-center gap-2">
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
                            </div>
                          )}

                          {/* Completed State Indicator */}
                          {isCompleted && (
                            <div className="flex items-center gap-2 px-3">
                              <div className="text-xs font-medium text-green-600">
                                Done
                              </div>
                              {action.pluginId && (
                                <>
                                  <Button
                                    size="icon"
                                    variant="ghost"
                                    className="h-7 w-7 text-muted-foreground hover:text-foreground"
                                    title="Retry Action"
                                    onClick={(e) => {
                                      e.preventDefault();
                                      e.stopPropagation();
                                      // Execute again without advancing count
                                      onExecuteRobotAction(
                                        action.pluginId!,
                                        action.type.includes(".") ? action.type.split(".").pop()! : action.type,
                                        action.parameters || {},
                                        { autoAdvance: false },
                                      );
                                    }}
                                    disabled={readOnly || isExecuting}
                                  >
                                    <RotateCcw className="h-3.5 w-3.5" />
                                  </Button>
                                  <Button
                                    size="icon"
                                    variant="ghost"
                                    className="h-7 w-7 text-amber-500 hover:text-amber-600 hover:bg-amber-100"
                                    title="Mark Issue"
                                    onClick={(e) => {
                                      e.preventDefault();
                                      e.stopPropagation();
                                      onExecuteAction("note", {
                                        content: `Reported issue with action: ${action.name}`,
                                        category: "system_issue"
                                      });
                                    }}
                                    disabled={readOnly}
                                  >
                                    <AlertTriangle className="h-3.5 w-3.5" />
                                  </Button>
                                </>
                              )}
                            </div>
                          )}
                        </div>
                      )
                    })}
                  </div>

                  {/* Manual Advance Button */}
                  {activeActionIndex >= (currentStep.actions?.length || 0) && (
                    <div className="mt-6 flex justify-end">
                      <Button
                        size="lg"
                        onClick={currentStepIndex === steps.length - 1 ? onCompleteTrial : onNextStep}
                        className={`w-full text-white shadow-md transition-all hover:scale-[1.02] ${currentStepIndex === steps.length - 1
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
              )}

              {/* Manual Wizard Controls (If applicable) */}
              {currentStep.type === "wizard_action" && (
                <div className="rounded-xl border border-dashed p-6 space-y-4">
                  <h3 className="text-sm font-medium text-muted-foreground">Manual Controls</h3>
                  <div className="grid grid-cols-1 gap-3">
                    <Button
                      variant="outline"
                      className="h-12 justify-start border-yellow-200 bg-yellow-50 text-yellow-700 hover:bg-yellow-100 hover:text-yellow-800"
                      onClick={() => onExecuteAction("intervene")}
                      disabled={readOnly}
                    >
                      <Zap className="mr-2 h-4 w-4" />
                      Flag Issue / Intervention
                    </Button>
                  </div>
                </div>
              )}
            </div>
          ) : (
            <div className="flex h-full items-center justify-center text-muted-foreground">
              No active step
            </div>
          )}
        </ScrollArea>
        {/* Scroll Hint Fade */}
        <div className="pointer-events-none absolute inset-x-0 bottom-0 h-20 bg-gradient-to-t from-background to-transparent z-10" />
      </div>
    </div >
  );
}
