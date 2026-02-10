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
import { Separator } from "~/components/ui/separator";
import { Switch } from "~/components/ui/switch";
import { Label } from "~/components/ui/label";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { ScrollArea } from "~/components/ui/scroll-area";
import { RobotActionsPanel } from "../RobotActionsPanel";

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
  onExecuteRobotAction?: (
    pluginName: string,
    actionId: string,
    parameters: Record<string, unknown>,
  ) => Promise<void>;
  studyId?: string;
  _isConnected: boolean;

  isStarting?: boolean;
  onSetAutonomousLife?: (enabled: boolean) => Promise<boolean | void>;
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
  onExecuteRobotAction,
  studyId,
  _isConnected,
  isStarting = false,
  onSetAutonomousLife,
  readOnly = false,
}: WizardControlPanelProps) {
  const [autonomousLife, setAutonomousLife] = React.useState(true);

  const handleAutonomousLifeChange = async (checked: boolean) => {
    setAutonomousLife(checked); // Optimistic update
    if (onSetAutonomousLife) {
      try {
        const result = await onSetAutonomousLife(checked);
        if (result === false) {
          throw new Error("Service unavailable");
        }
      } catch (error) {
        console.error("Failed to set autonomous life:", error);
        setAutonomousLife(!checked); // Revert on failure
        // Optional: Toast error?
      }
    }
  };

  return (
    <div className="flex h-full flex-col">


      <div className="min-h-0 flex-1">
        <ScrollArea className="h-full">
          <div className="space-y-4 p-3">
            <div className="space-y-3">
              {/* Decision Point UI removed as per user request (handled in Execution Panel) */}

              {trial.status === "in_progress" ? (
                <div className="space-y-2">
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
                    className="w-full justify-start border-yellow-200 bg-yellow-50 text-yellow-700 hover:bg-yellow-100 hover:text-yellow-800"
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
            </div>

            <Separator />

            {/* Robot Controls (Merged from System & Robot Tab) */}
            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <span className="text-muted-foreground text-xs">Connection</span>
                {_isConnected ? (
                  <Badge variant="default" className="bg-green-600 text-xs">Connected</Badge>
                ) : (
                  <Badge variant="outline" className="text-muted-foreground border-muted-foreground/30 text-xs">Offline</Badge>
                )}
              </div>

              <div className="flex items-center justify-between">
                <Label htmlFor="autonomous-life" className="text-xs font-normal text-muted-foreground">Autonomous Life</Label>
                <Switch
                  id="autonomous-life"
                  checked={!!autonomousLife}
                  onCheckedChange={handleAutonomousLifeChange}
                  disabled={!_isConnected || readOnly}
                  className="scale-75"
                />
              </div>

              <Separator />

              {/* Robot Actions Panel Integration */}
              {studyId && onExecuteRobotAction ? (
                <div className={readOnly ? "pointer-events-none opacity-50" : ""}>
                  <RobotActionsPanel
                    studyId={studyId}
                    trialId={trial.id}
                    onExecuteAction={onExecuteRobotAction}
                  />
                </div>
              ) : (
                <div className="text-xs text-muted-foreground text-center py-2">Robot actions unavailable</div>
              )}
            </div>
          </div>
        </ScrollArea>
      </div >
    </div >
  );
});
