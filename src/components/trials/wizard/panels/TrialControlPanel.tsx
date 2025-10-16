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
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Progress } from "~/components/ui/progress";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import { Alert, AlertDescription } from "~/components/ui/alert";

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

interface TrialControlPanelProps {
  trial: TrialData;
  currentStep: StepData | null;
  steps: StepData[];
  currentStepIndex: number;
  onStartTrial: () => void;
  onPauseTrial: () => void;
  onNextStep: () => void;
  onCompleteTrial: () => void;
  onAbortTrial: () => void;
  onExecuteAction: (
    actionId: string,
    parameters?: Record<string, unknown>,
  ) => void;
  isConnected: boolean;
}

export function TrialControlPanel({
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
  isConnected,
}: TrialControlPanelProps) {
  const progress =
    steps.length > 0 ? ((currentStepIndex + 1) / steps.length) * 100 : 0;

  const getStatusConfig = (status: string) => {
    switch (status) {
      case "scheduled":
        return { variant: "outline" as const, icon: Clock };
      case "in_progress":
        return { variant: "default" as const, icon: Play };
      case "completed":
        return { variant: "secondary" as const, icon: CheckCircle };
      case "aborted":
      case "failed":
        return { variant: "destructive" as const, icon: X };
      default:
        return { variant: "outline" as const, icon: Clock };
    }
  };

  const statusConfig = getStatusConfig(trial.status);
  const StatusIcon = statusConfig.icon;

  return (
    <div className="flex h-full flex-col space-y-4 p-4">
      {/* Trial Status Card */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center justify-between text-sm">
            <span>Trial Status</span>
            <Badge
              variant={statusConfig.variant}
              className="flex items-center gap-1"
            >
              <StatusIcon className="h-3 w-3" />
              {trial.status.replace("_", " ")}
            </Badge>
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Session</span>
              <span>#{trial.sessionNumber}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Participant</span>
              <span className="font-mono">
                {trial.participant.participantCode}
              </span>
            </div>
            {trial.status === "in_progress" && (
              <>
                <Separator />
                <div className="flex justify-between">
                  <span className="text-muted-foreground">Progress</span>
                  <span>
                    {currentStepIndex + 1} of {steps.length}
                  </span>
                </div>
                <Progress value={progress} className="h-2" />
              </>
            )}
          </div>

          {/* Connection Status */}
          <div className="flex items-center justify-between pt-2">
            <span className="text-muted-foreground text-sm">Connection</span>
            <Badge
              variant={isConnected ? "default" : "outline"}
              className="text-xs"
            >
              {isConnected ? "Live" : "Polling"}
            </Badge>
          </div>
        </CardContent>
      </Card>

      {/* Trial Controls */}
      <Card className="flex-1">
        <CardHeader className="pb-3">
          <CardTitle className="text-sm">Controls</CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          {trial.status === "scheduled" && (
            <Button onClick={onStartTrial} className="w-full" size="sm">
              <Play className="mr-2 h-4 w-4" />
              Start Trial
            </Button>
          )}

          {trial.status === "in_progress" && (
            <>
              <div className="grid grid-cols-2 gap-2">
                <Button
                  onClick={onPauseTrial}
                  variant="outline"
                  size="sm"
                  disabled={!isConnected}
                >
                  <Pause className="mr-1 h-3 w-3" />
                  Pause
                </Button>
                <Button
                  onClick={onNextStep}
                  disabled={currentStepIndex >= steps.length - 1}
                  size="sm"
                >
                  <SkipForward className="mr-1 h-3 w-3" />
                  Next
                </Button>
              </div>

              <Separator />

              <div className="space-y-2">
                <Button
                  onClick={onCompleteTrial}
                  variant="outline"
                  className="w-full"
                  size="sm"
                >
                  <CheckCircle className="mr-2 h-4 w-4" />
                  Complete Trial
                </Button>
                <Button
                  onClick={onAbortTrial}
                  variant="destructive"
                  className="w-full"
                  size="sm"
                >
                  <X className="mr-2 h-4 w-4" />
                  Abort Trial
                </Button>
              </div>
            </>
          )}

          {(trial.status === "completed" || trial.status === "aborted") && (
            <Alert>
              <CheckCircle className="h-4 w-4" />
              <AlertDescription className="text-sm">
                Trial has ended. All controls are disabled.
              </AlertDescription>
            </Alert>
          )}
        </CardContent>
      </Card>

      {/* Current Step Info */}
      {currentStep && trial.status === "in_progress" && (
        <Card>
          <CardHeader className="pb-3">
            <CardTitle className="text-sm">Current Step</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="space-y-2">
              <div className="text-sm font-medium">{currentStep.name}</div>
              {currentStep.description && (
                <p className="text-muted-foreground line-clamp-3 text-xs">
                  {currentStep.description}
                </p>
              )}
              <div className="flex items-center justify-between pt-1">
                <Badge variant="outline" className="text-xs">
                  {currentStep.type.replace("_", " ")}
                </Badge>
                <span className="text-muted-foreground text-xs">
                  Step {currentStepIndex + 1}
                </span>
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Quick Actions */}
      {trial.status === "in_progress" &&
        currentStep?.type === "wizard_action" && (
          <Card>
            <CardHeader className="pb-3">
              <CardTitle className="text-sm">Quick Actions</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-2">
                <Button
                  variant="outline"
                  size="sm"
                  className="w-full justify-start"
                  onClick={() => onExecuteAction("acknowledge")}
                >
                  <CheckCircle className="mr-2 h-3 w-3" />
                  Acknowledge
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  className="w-full justify-start"
                  onClick={() => onExecuteAction("intervene")}
                >
                  <AlertCircle className="mr-2 h-3 w-3" />
                  Intervene
                </Button>
              </div>
            </CardContent>
          </Card>
        )}
    </div>
  );
}
