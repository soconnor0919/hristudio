"use client";


import React from "react";
import { WizardActionItem } from "./WizardActionItem";
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
  isPaused?: boolean;
  rosConnected?: boolean;
  completedStepIndices?: Set<number>;
  skippedStepIndices?: Set<number>;
  onLogEvent?: (type: string, data?: any) => void;
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
  isPaused = false,
  rosConnected,
  completedStepIndices = new Set(),
  skippedStepIndices = new Set(),
  onLogEvent,
}: WizardExecutionPanelProps) {
  // Local state removed in favor of parent state to prevent reset on re-render
  // const [completedCount, setCompletedCount] = React.useState(0);

  const isStepCompleted = completedStepIndices.has(currentStepIndex);
  const activeActionIndex = isStepCompleted ? 9999 : completedActionsCount;

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
    <div className="flex h-full flex-col overflow-hidden relative">
      {/* Paused Overlay */}
      {isPaused && (
        <div className="absolute inset-0 z-50 bg-background/60 backdrop-blur-[2px] flex items-center justify-center">
          <div className="bg-background border shadow-lg rounded-xl p-8 flex flex-col items-center max-w-sm text-center space-y-4">
            <AlertCircle className="h-12 w-12 text-muted-foreground" />
            <div>
              <h2 className="text-xl font-bold tracking-tight">Trial Paused</h2>
              <p className="text-sm text-muted-foreground mt-1">
                The trial execution has been paused. Resume from the control bar to continue interacting.
              </p>
            </div>
          </div>
        </div>
      )}

      {/* Horizontal Step Progress Bar */}
      <div className="flex-none border-b bg-muted/30 p-3">
        <div className="flex items-center gap-2 overflow-x-auto pb-2">
          {steps.map((step, idx) => {
            const isCurrent = idx === currentStepIndex;
            const isSkipped = skippedStepIndices.has(idx);
            const isCompleted = completedStepIndices.has(idx) || (!isSkipped && idx < currentStepIndex);
            const isUpcoming = idx > currentStepIndex;

            return (
              <div
                key={step.id}
                className="flex items-center gap-2 flex-shrink-0"
              >
                <button
                  onClick={() => onStepSelect(idx)}
                  disabled={readOnly}
                  className={`
                    group relative flex items-center gap-2 rounded-lg border-2 px-3 py-2 transition-all
                    ${isCurrent
                      ? "border-primary bg-primary/10 shadow-sm"
                      : isCompleted
                        ? "border-primary/30 bg-primary/5 hover:bg-primary/10"
                        : isSkipped
                          ? "border-muted-foreground/30 bg-muted/20 border-dashed"
                          : "border-muted-foreground/20 bg-background hover:bg-muted/50"
                    }
                    ${readOnly ? "cursor-default" : "cursor-pointer"}
                  `}
                >
                  {/* Step Number/Icon */}
                  <div
                    className={`
                      flex h-6 w-6 items-center justify-center rounded-full text-xs font-bold
                      ${isCompleted
                        ? "bg-primary text-primary-foreground"
                        : isSkipped
                          ? "bg-transparent border border-muted-foreground/40 text-muted-foreground"
                          : isCurrent
                            ? "bg-primary text-primary-foreground ring-2 ring-primary/20"
                            : "bg-muted text-muted-foreground"
                      }
                    `}
                  >
                    {isCompleted ? (
                      <CheckCircle className="h-3.5 w-3.5" />
                    ) : (
                      idx + 1
                    )}
                  </div>

                  {/* Step Name */}
                  <span
                    className={`text-xs font-medium max-w-[120px] truncate ${isCurrent
                      ? "text-foreground"
                      : isCompleted
                        ? "text-muted-foreground"
                        : "text-muted-foreground/60"
                      }`}
                    title={step.name}
                  >
                    {step.name}
                  </span>
                </button>

                {/* Arrow Connector */}
                {idx < steps.length - 1 && (
                  <ArrowRight
                    className={`h-4 w-4 flex-shrink-0 ${isCompleted ? "text-primary/40" : "text-muted-foreground/30"
                      }`}
                  />
                )}
              </div>
            );
          })}
        </div>
      </div>

      {/* Current Step Details - NO SCROLL */}
      <div className="flex-1 min-h-0 overflow-hidden">
        <div className="h-full overflow-y-auto">
          <div className="pr-4">
            {currentStep ? (
              <div className="flex flex-col gap-4 p-4 max-w-5xl mx-auto w-full">
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
                      const isActive: boolean = idx === activeActionIndex;
                      const isLast = idx === (currentStep.actions?.length || 0) - 1;

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

                          {/* Action Content */}
                          <WizardActionItem
                            action={action as any} // Cast to ActionData
                            index={idx}
                            isActive={isActive}
                            isCompleted={isCompleted}
                            onExecute={onExecuteAction}
                            onExecuteRobot={onExecuteRobotAction}
                            onSkip={onSkipAction}
                            onCompleted={onActionCompleted}
                            readOnly={readOnly}
                            isExecuting={isExecuting}
                            isRobotConnected={rosConnected}
                            onLogEvent={onLogEvent}
                          />
                        </div>
                      );
                    })}
                  </div>
                )}

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
        </div>
      </div>
    </div>
  );
}
