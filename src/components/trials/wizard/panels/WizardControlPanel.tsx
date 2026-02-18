"use client";

import React from "react";
import {
  Play,
  Pause,
  SkipForward,
  CheckCircle,
  X,
  Clock,
  AlertCircle,
  Settings,
  Zap,
  User,
  Bot,
  Eye,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Progress } from "~/components/ui/progress";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { ScrollArea } from "~/components/ui/scroll-area";

interface StepData {
  id: string;
  name: string;
  description: string | null;
  type:
  | "wizard_action"
  | "robot_action"
  | "parallel_steps"
  | "conditional"; // Updated to match DB enum
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

interface WizardControlPanelProps {
  trial: TrialData;
  currentStep: StepData | null;
  steps: StepData[];
  currentStepIndex: number;
  onStartTrial: () => void;
  onPauseTrial: () => void;
  onNextStep: (targetIndex?: number) => void;
  onCompleteTrial: () => void;
  onAbortTrial: () => void;
  onExecuteAction: (
    actionId: string,
    parameters?: Record<string, unknown>,
  ) => void;
  isStarting?: boolean;
  readOnly?: boolean;
}

export const WizardControlPanel = React.memo(function WizardControlPanel({
  trial,
  currentStep,
  steps,
  currentStepIndex,
  onStartTrial,
  onPauseTrial,
  onNextStep,
  onCompleteTrial,
  onAbortTrial,
  onExecuteAction,
  isStarting = false,
  readOnly = false,
}: WizardControlPanelProps) {


  return (
    <div className="flex h-full flex-col" id="tour-wizard-controls">


      <div className="min-h-0 flex-1">
        <ScrollArea className="h-full">
          <div className="space-y-4 p-3">
            <div className="space-y-3">
              {/* Decision Point UI removed as per user request (handled in Execution Panel) */}

              {trial.status === "in_progress" ? (
                <div className="space-y-2" id="tour-wizard-action-list">
                  <Button
                    variant="outline"
                    size="sm"
                    className="w-full justify-start"
                    onClick={() => onExecuteAction("acknowledge")}
                    disabled={readOnly}
                  >
                    <CheckCircle className="mr-2 h-3 w-3" />
                    Acknowledge
                  </Button>

                  <Button
                    variant="outline"
                    size="sm"
                    className="w-full justify-start border-yellow-200 bg-yellow-50 text-yellow-700 hover:bg-yellow-100 hover:text-yellow-800 dark:bg-yellow-900/20 dark:text-yellow-300 dark:border-yellow-700/50 dark:hover:bg-yellow-900/40"
                    onClick={() => onExecuteAction("intervene")}
                    disabled={readOnly}
                  >
                    <AlertCircle className="mr-2 h-3 w-3" />
                    Flag Intervention
                  </Button>

                  <Button
                    variant="outline"
                    size="sm"
                    className="w-full justify-start"
                    onClick={() => onExecuteAction("note", { content: "Wizard note" })}
                    disabled={readOnly}
                  >
                    <User className="mr-2 h-3 w-3" />
                    Add Note
                  </Button>

                  {currentStep?.type === "wizard_action" && (
                    <Button
                      variant="outline"
                      size="sm"
                      className="w-full justify-start"
                      onClick={() => onExecuteAction("step_complete")}
                      disabled={readOnly}
                    >
                      <CheckCircle className="mr-2 h-3 w-3" />
                      Mark Step Complete
                    </Button>
                  )}
                </div>
              ) : (
                <div className="text-xs text-muted-foreground p-2 text-center border border-dashed rounded-md bg-muted/20">
                  Controls available during trial
                </div>
              )}

              {/* Step Navigation */}
              <div className="pt-4 border-t space-y-2">
                <span className="text-xs font-semibold text-muted-foreground uppercase tracking-wider">Navigation</span>
                <select
                  className="w-full text-xs p-2 rounded-md border bg-background"
                  value={currentStepIndex}
                  onChange={(e) => onNextStep(parseInt(e.target.value, 10))}
                  disabled={readOnly}
                >
                  {steps.map((step, idx) => (
                    <option key={step.id} value={idx}>
                      {idx + 1}. {step.name}
                    </option>
                  ))}
                </select>
              </div>
            </div>
          </div>
        </ScrollArea>
      </div>
    </div>
  );
});
