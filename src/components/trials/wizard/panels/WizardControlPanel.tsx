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
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Progress } from "~/components/ui/progress";
import { Separator } from "~/components/ui/separator";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";
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
  onNextStep: () => void;
  onCompleteTrial: () => void;
  onAbortTrial: () => void;
  onExecuteAction: (
    actionId: string,
    parameters?: Record<string, unknown>,
  ) => void;
  _isConnected: boolean;
  activeTab: "control" | "step" | "actions";
  onTabChange: (tab: "control" | "step" | "actions") => void;
  isStarting?: boolean;
}

export function WizardControlPanel({
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
  _isConnected,
  activeTab,
  onTabChange,
  isStarting = false,
}: WizardControlPanelProps) {
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
    <div className="flex h-full flex-col">
      {/* Trial Info Header */}
      <div className="border-b p-3">
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <Badge
              variant={statusConfig.variant}
              className="flex items-center gap-1"
            >
              <StatusIcon className="h-3 w-3" />
              {trial.status.replace("_", " ")}
            </Badge>
            <span className="text-muted-foreground text-xs">
              Session #{trial.sessionNumber}
            </span>
          </div>

          <div className="text-sm font-medium">
            {trial.participant.participantCode}
          </div>

          {trial.status === "in_progress" && steps.length > 0 && (
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-muted-foreground">Progress</span>
                <span>
                  {currentStepIndex + 1} of {steps.length}
                </span>
              </div>
              <Progress value={progress} className="h-1.5" />
            </div>
          )}
        </div>
      </div>

      {/* Tabbed Content */}
      <Tabs
        value={activeTab}
        onValueChange={(value: string) => {
          if (value === "control" || value === "step" || value === "actions") {
            onTabChange(value);
          }
        }}
        className="flex min-h-0 flex-1 flex-col"
      >
        <div className="border-b px-2 py-1">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="control" className="text-xs">
              <Settings className="mr-1 h-3 w-3" />
              Control
            </TabsTrigger>
            <TabsTrigger value="step" className="text-xs">
              <Play className="mr-1 h-3 w-3" />
              Step
            </TabsTrigger>
            <TabsTrigger value="actions" className="text-xs">
              <Zap className="mr-1 h-3 w-3" />
              Actions
            </TabsTrigger>
          </TabsList>
        </div>

        <div className="min-h-0 flex-1">
          {/* Trial Control Tab */}
          <TabsContent
            value="control"
            className="m-0 h-full data-[state=active]:flex data-[state=active]:flex-col"
          >
            <ScrollArea className="h-full">
              <div className="space-y-3 p-3">
                {trial.status === "scheduled" && (
                  <Button
                    onClick={() => {
                      console.log("[WizardControlPanel] Start Trial clicked");
                      onStartTrial();
                    }}
                    className="w-full"
                    size="sm"
                    disabled={isStarting}
                  >
                    <Play className="mr-2 h-4 w-4" />
                    {isStarting ? "Starting..." : "Start Trial"}
                  </Button>
                )}

                {trial.status === "in_progress" && (
                  <div className="space-y-2">
                    <div className="grid grid-cols-2 gap-2">
                      <Button
                        onClick={onPauseTrial}
                        variant="outline"
                        size="sm"
                        disabled={false}
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
                )}

                {(trial.status === "completed" ||
                  trial.status === "aborted") && (
                  <Alert>
                    <CheckCircle className="h-4 w-4" />
                    <AlertDescription className="text-sm">
                      Trial has ended. All controls are disabled.
                    </AlertDescription>
                  </Alert>
                )}

                {/* Connection Status */}
                <Separator />
                <div className="space-y-2">
                  <div className="text-xs font-medium">Connection</div>
                  <div className="flex items-center justify-between">
                    <span className="text-muted-foreground text-xs">
                      Status
                    </span>
                    <Badge variant="default" className="text-xs">
                      Polling
                    </Badge>
                  </div>
                </div>
              </div>
            </ScrollArea>
          </TabsContent>

          {/* Current Step Tab */}
          <TabsContent
            value="step"
            className="m-0 h-full data-[state=active]:flex data-[state=active]:flex-col"
          >
            <ScrollArea className="h-full">
              <div className="p-3">
                {currentStep && trial.status === "in_progress" ? (
                  <div className="space-y-3">
                    <div className="space-y-2">
                      <div className="text-sm font-medium">
                        {currentStep.name}
                      </div>
                      <Badge variant="outline" className="text-xs">
                        {currentStep.type.replace("_", " ")}
                      </Badge>
                    </div>

                    {currentStep.description && (
                      <div className="text-muted-foreground text-xs">
                        {currentStep.description}
                      </div>
                    )}

                    <Separator />

                    <div className="space-y-2">
                      <div className="text-xs font-medium">Step Progress</div>
                      <div className="flex items-center justify-between text-xs">
                        <span className="text-muted-foreground">Current</span>
                        <span>Step {currentStepIndex + 1}</span>
                      </div>
                      <div className="flex items-center justify-between text-xs">
                        <span className="text-muted-foreground">Remaining</span>
                        <span>{steps.length - currentStepIndex - 1} steps</span>
                      </div>
                    </div>

                    {currentStep.type === "robot_action" && (
                      <Alert>
                        <AlertCircle className="h-4 w-4" />
                        <AlertDescription className="text-xs">
                          Robot is executing this step. Monitor progress in the
                          monitoring panel.
                        </AlertDescription>
                      </Alert>
                    )}
                  </div>
                ) : (
                  <div className="text-muted-foreground flex h-32 items-center justify-center text-center text-xs">
                    {trial.status === "scheduled"
                      ? "Start trial to see current step"
                      : trial.status === "in_progress"
                        ? "No current step"
                        : "Trial has ended"}
                  </div>
                )}
              </div>
            </ScrollArea>
          </TabsContent>

          {/* Quick Actions Tab */}
          <TabsContent
            value="actions"
            className="m-0 h-full data-[state=active]:flex data-[state=active]:flex-col"
          >
            <ScrollArea className="h-full">
              <div className="space-y-2 p-3">
                {trial.status === "in_progress" ? (
                  <>
                    <div className="mb-2 text-xs font-medium">
                      Quick Actions
                    </div>

                    <Button
                      variant="outline"
                      size="sm"
                      className="w-full justify-start"
                      onClick={() => {
                        console.log("[WizardControlPanel] Acknowledge clicked");
                        onExecuteAction("acknowledge");
                      }}
                      disabled={false}
                    >
                      <CheckCircle className="mr-2 h-3 w-3" />
                      Acknowledge
                    </Button>

                    <Button
                      variant="outline"
                      size="sm"
                      className="w-full justify-start"
                      onClick={() => {
                        console.log("[WizardControlPanel] Intervene clicked");
                        onExecuteAction("intervene");
                      }}
                      disabled={false}
                    >
                      <AlertCircle className="mr-2 h-3 w-3" />
                      Intervene
                    </Button>

                    <Button
                      variant="outline"
                      size="sm"
                      className="w-full justify-start"
                      onClick={() => {
                        console.log("[WizardControlPanel] Add Note clicked");
                        onExecuteAction("note", { content: "Wizard note" });
                      }}
                      disabled={false}
                    >
                      <User className="mr-2 h-3 w-3" />
                      Add Note
                    </Button>

                    <Separator />

                    {currentStep?.type === "wizard_action" && (
                      <div className="space-y-2">
                        <div className="text-xs font-medium">Step Actions</div>
                        <Button
                          variant="outline"
                          size="sm"
                          className="w-full justify-start"
                          onClick={() => onExecuteAction("step_complete")}
                          disabled={false}
                        >
                          <CheckCircle className="mr-2 h-3 w-3" />
                          Mark Complete
                        </Button>
                      </div>
                    )}
                  </>
                ) : (
                  <div className="flex h-32 items-center justify-center">
                    <div className="text-muted-foreground text-center text-xs">
                      {trial.status === "scheduled"
                        ? "Start trial to access actions"
                        : "Actions unavailable - trial not active"}
                    </div>
                  </div>
                )}
              </div>
            </ScrollArea>
          </TabsContent>
        </div>
      </Tabs>
    </div>
  );
}
