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
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
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

interface TrialEvent {
  type: string;
  timestamp: Date;
  data?: unknown;
  message?: string;
}

interface ExecutionPanelProps {
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
}

export function ExecutionPanel({
  trial,
  currentStep,
  steps,
  currentStepIndex,
  trialEvents,
  onStepSelect,
  onExecuteAction,
}: ExecutionPanelProps) {
  const getStepIcon = (type: string) => {
    switch (type) {
      case "wizard_action":
        return User;
      case "robot_action":
        return Bot;
      case "parallel_steps":
        return Activity;
      case "conditional_branch":
        return AlertCircle;
      default:
        return Play;
    }
  };

  const getStepStatus = (stepIndex: number) => {
    if (stepIndex < currentStepIndex) return "completed";
    if (stepIndex === currentStepIndex && trial.status === "in_progress")
      return "active";
    return "pending";
  };

  const getStepVariant = (status: string) => {
    switch (status) {
      case "completed":
        return "default";
      case "active":
        return "secondary";
      case "pending":
        return "outline";
      default:
        return "outline";
    }
  };

  if (trial.status === "scheduled") {
    return (
      <div className="flex h-full items-center justify-center p-8">
        <Card className="w-full max-w-md">
          <CardContent className="pt-6 text-center">
            <Clock className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
            <h3 className="mb-2 text-lg font-semibold">Trial Ready to Start</h3>
            <p className="text-muted-foreground mb-4">
              This trial is scheduled and ready to begin. Use the controls in
              the left panel to start execution.
            </p>
            <div className="text-muted-foreground space-y-1 text-sm">
              <div>Experiment: {trial.experiment.name}</div>
              <div>Participant: {trial.participant.participantCode}</div>
              <div>Session: #{trial.sessionNumber}</div>
              {steps.length > 0 && <div>{steps.length} steps to execute</div>}
            </div>
          </CardContent>
        </Card>
      </div>
    );
  }

  if (
    trial.status === "completed" ||
    trial.status === "aborted" ||
    trial.status === "failed"
  ) {
    return (
      <div className="flex h-full items-center justify-center p-8">
        <Card className="w-full max-w-md">
          <CardContent className="pt-6 text-center">
            <CheckCircle className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
            <h3 className="mb-2 text-lg font-semibold">
              Trial {trial.status === "completed" ? "Completed" : "Ended"}
            </h3>
            <p className="text-muted-foreground mb-4">
              The trial execution has finished. You can review the results and
              captured data.
            </p>
            {trial.completedAt && (
              <div className="text-muted-foreground text-sm">
                Ended at {new Date(trial.completedAt).toLocaleString()}
              </div>
            )}
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <div className="flex h-full flex-col p-6">
      {/* Current Step Header */}
      {currentStep && (
        <Card className="mb-6">
          <CardHeader className="pb-4">
            <CardTitle className="flex items-center gap-3">
              <div className="bg-primary/10 flex h-10 w-10 items-center justify-center rounded-full">
                {React.createElement(getStepIcon(currentStep.type), {
                  className: "h-5 w-5 text-primary",
                })}
              </div>
              <div className="flex-1">
                <div className="font-semibold">{currentStep.name}</div>
                <div className="text-muted-foreground text-sm">
                  Step {currentStepIndex + 1} of {steps.length}
                </div>
              </div>
              <Badge variant="secondary" className="ml-auto">
                {currentStep.type.replace("_", " ")}
              </Badge>
            </CardTitle>
          </CardHeader>
          <CardContent>
            {currentStep.description && (
              <p className="text-muted-foreground mb-4">
                {currentStep.description}
              </p>
            )}

            {currentStep.type === "wizard_action" && (
              <div className="space-y-3">
                <div className="text-sm font-medium">Available Actions:</div>
                <div className="flex flex-wrap gap-2">
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={() => onExecuteAction("acknowledge")}
                  >
                    <CheckCircle className="mr-2 h-4 w-4" />
                    Acknowledge
                  </Button>
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={() => onExecuteAction("intervene")}
                  >
                    <Zap className="mr-2 h-4 w-4" />
                    Intervene
                  </Button>
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={() =>
                      onExecuteAction("note", { content: "Wizard observation" })
                    }
                  >
                    Note
                  </Button>
                </div>
              </div>
            )}

            {currentStep.type === "robot_action" && (
              <div className="rounded-lg bg-blue-50 p-3 text-sm">
                <div className="flex items-center gap-2 font-medium text-blue-900">
                  <Bot className="h-4 w-4" />
                  Robot Action in Progress
                </div>
                <div className="mt-1 text-blue-700">
                  The robot is executing this step. Monitor progress in the
                  right panel.
                </div>
              </div>
            )}
          </CardContent>
        </Card>
      )}

      {/* Steps Timeline */}
      <Card className="flex-1">
        <CardHeader className="pb-4">
          <CardTitle className="flex items-center gap-2">
            <Activity className="h-5 w-5" />
            Experiment Timeline
          </CardTitle>
        </CardHeader>
        <CardContent>
          <ScrollArea className="h-full">
            <div className="space-y-3">
              {steps.map((step, index) => {
                const status = getStepStatus(index);
                const StepIcon = getStepIcon(step.type);
                const isActive = index === currentStepIndex;

                return (
                  <div
                    key={step.id}
                    className={`hover:bg-muted/50 flex cursor-pointer items-start gap-3 rounded-lg p-3 transition-colors ${
                      isActive ? "bg-primary/5 border-primary/20 border" : ""
                    }`}
                    onClick={() => onStepSelect(index)}
                  >
                    {/* Step Number and Status */}
                    <div className="flex flex-col items-center">
                      <div
                        className={`flex h-8 w-8 items-center justify-center rounded-full text-sm font-medium ${
                          status === "completed"
                            ? "bg-green-100 text-green-700"
                            : status === "active"
                              ? "bg-primary/10 text-primary"
                              : "bg-muted text-muted-foreground"
                        }`}
                      >
                        {status === "completed" ? (
                          <CheckCircle className="h-4 w-4" />
                        ) : (
                          index + 1
                        )}
                      </div>
                      {index < steps.length - 1 && (
                        <div
                          className={`mt-2 h-6 w-0.5 ${
                            status === "completed"
                              ? "bg-green-200"
                              : "bg-border"
                          }`}
                        />
                      )}
                    </div>

                    {/* Step Content */}
                    <div className="min-w-0 flex-1">
                      <div className="flex items-center gap-2">
                        <StepIcon className="text-muted-foreground h-4 w-4" />
                        <div className="font-medium">{step.name}</div>
                        <Badge
                          variant={getStepVariant(status)}
                          className="ml-auto text-xs"
                        >
                          {step.type.replace("_", " ")}
                        </Badge>
                      </div>
                      {step.description && (
                        <p className="text-muted-foreground mt-1 line-clamp-2 text-sm">
                          {step.description}
                        </p>
                      )}
                      {isActive && trial.status === "in_progress" && (
                        <div className="mt-2 flex items-center gap-2">
                          <div className="bg-primary h-2 w-2 animate-pulse rounded-full" />
                          <span className="text-primary text-xs">
                            Currently executing
                          </span>
                        </div>
                      )}
                    </div>
                  </div>
                );
              })}
            </div>
          </ScrollArea>
        </CardContent>
      </Card>

      {/* Recent Events */}
      {trialEvents.length > 0 && (
        <Card className="mt-4">
          <CardHeader className="pb-3">
            <CardTitle className="text-sm">Recent Activity</CardTitle>
          </CardHeader>
          <CardContent>
            <ScrollArea className="h-24">
              <div className="space-y-2">
                {trialEvents.slice(-5).map((event, index) => (
                  <div
                    key={index}
                    className="flex items-center justify-between text-xs"
                  >
                    <span className="font-medium">{event.type}</span>
                    <span className="text-muted-foreground">
                      {new Date(event.timestamp).toLocaleTimeString()}
                    </span>
                  </div>
                ))}
              </div>
            </ScrollArea>
          </CardContent>
        </Card>
      )}
    </div>
  );
}
