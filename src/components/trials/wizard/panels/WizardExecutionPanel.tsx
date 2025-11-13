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
  Eye,
  List,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";
import { ScrollArea } from "~/components/ui/scroll-area";
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
  activeTab: "current" | "timeline" | "events";
  onTabChange: (tab: "current" | "timeline" | "events") => void;
}

export function WizardExecutionPanel({
  trial,
  currentStep,
  steps,
  currentStepIndex,
  trialEvents,
  onStepSelect,
  onExecuteAction,
  activeTab,
  onTabChange,
}: WizardExecutionPanelProps) {
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
              `Ended at ${new Date(trial.completedAt).toLocaleTimeString()}`}
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

      {/* Tabbed Content */}
      <Tabs
        value={activeTab}
        onValueChange={(value: string) => {
          if (
            value === "current" ||
            value === "timeline" ||
            value === "events"
          ) {
            onTabChange(value);
          }
        }}
        className="flex min-h-0 flex-1 flex-col"
      >
        <div className="border-b px-2 py-1">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="current" className="text-xs">
              <Eye className="mr-1 h-3 w-3" />
              Current
            </TabsTrigger>
            <TabsTrigger value="timeline" className="text-xs">
              <List className="mr-1 h-3 w-3" />
              Timeline
            </TabsTrigger>
            <TabsTrigger value="events" className="text-xs">
              <Activity className="mr-1 h-3 w-3" />
              Events
              {trialEvents.length > 0 && (
                <Badge variant="secondary" className="ml-1 text-xs">
                  {trialEvents.length}
                </Badge>
              )}
            </TabsTrigger>
          </TabsList>
        </div>

        <div className="min-h-0 flex-1">
          {/* Current Step Tab */}
          <TabsContent value="current" className="m-0 h-full">
            <div className="h-full">
              {currentStep ? (
                <div className="flex h-full flex-col p-4">
                  {/* Current Step Display */}
                  <div className="flex-1 space-y-4 text-left">
                    <div className="flex items-start gap-3">
                      <div className="bg-primary/10 flex h-10 w-10 flex-shrink-0 items-center justify-center rounded-full">
                        {React.createElement(getStepIcon(currentStep.type), {
                          className: "h-5 w-5 text-primary",
                        })}
                      </div>
                      <div className="min-w-0 flex-1">
                        <h4 className="text-sm font-medium">
                          {currentStep.name}
                        </h4>
                        <Badge variant="outline" className="mt-1 text-xs">
                          {currentStep.type.replace("_", " ")}
                        </Badge>
                      </div>
                    </div>

                    {currentStep.description && (
                      <div className="text-muted-foreground text-sm">
                        {currentStep.description}
                      </div>
                    )}

                    {/* Step-specific content */}
                    {currentStep.type === "wizard_action" && (
                      <div className="space-y-3">
                        <div className="text-sm font-medium">
                          Available Actions
                        </div>
                        <div className="space-y-2">
                          <Button
                            size="sm"
                            variant="outline"
                            className="w-full justify-start"
                            onClick={() => onExecuteAction("acknowledge")}
                          >
                            <CheckCircle className="mr-2 h-4 w-4" />
                            Acknowledge Step
                          </Button>
                          <Button
                            size="sm"
                            variant="outline"
                            className="w-full justify-start"
                            onClick={() => onExecuteAction("intervene")}
                          >
                            <Zap className="mr-2 h-4 w-4" />
                            Manual Intervention
                          </Button>
                          <Button
                            size="sm"
                            variant="outline"
                            className="w-full justify-start"
                            onClick={() =>
                              onExecuteAction("note", {
                                content: "Step observation",
                              })
                            }
                          >
                            <User className="mr-2 h-4 w-4" />
                            Add Observation
                          </Button>
                        </div>
                      </div>
                    )}

                    {currentStep.type === "robot_action" && (
                      <Alert>
                        <Bot className="h-4 w-4" />
                        <AlertDescription className="text-sm">
                          <div className="font-medium">
                            Robot Action in Progress
                          </div>
                          <div className="mt-1 text-xs">
                            The robot is executing this step. Monitor status in
                            the monitoring panel.
                          </div>
                        </AlertDescription>
                      </Alert>
                    )}

                    {currentStep.type === "parallel_steps" && (
                      <Alert>
                        <Activity className="h-4 w-4" />
                        <AlertDescription className="text-sm">
                          <div className="font-medium">Parallel Execution</div>
                          <div className="mt-1 text-xs">
                            Multiple actions are running simultaneously.
                          </div>
                        </AlertDescription>
                      </Alert>
                    )}
                  </div>
                </div>
              ) : (
                <div className="flex h-full items-center justify-center p-6">
                  <div className="w-full max-w-md text-center">
                    <div className="text-muted-foreground text-sm">
                      No current step available
                    </div>
                  </div>
                </div>
              )}
            </div>
          </TabsContent>

          {/* Timeline Tab */}
          <TabsContent value="timeline" className="m-0 h-full">
            <ScrollArea className="h-full">
              <div className="space-y-2 p-3">
                {steps.map((step, index) => {
                  const status = getStepStatus(index);
                  const StepIcon = getStepIcon(step.type);
                  const isActive = index === currentStepIndex;

                  return (
                    <div
                      key={step.id}
                      className={`hover:bg-muted/50 flex cursor-pointer items-start gap-3 rounded-lg p-2 transition-colors ${
                        isActive ? "bg-primary/5 border-primary/20 border" : ""
                      }`}
                      onClick={() => onStepSelect(index)}
                    >
                      {/* Step Number and Status */}
                      <div className="flex flex-col items-center">
                        <div
                          className={`flex h-6 w-6 flex-shrink-0 items-center justify-center rounded-full text-xs font-medium ${
                            status === "completed"
                              ? "bg-green-100 text-green-700"
                              : status === "active"
                                ? "bg-primary/10 text-primary"
                                : "bg-muted text-muted-foreground"
                          }`}
                        >
                          {status === "completed" ? (
                            <CheckCircle className="h-3 w-3" />
                          ) : (
                            index + 1
                          )}
                        </div>
                        {index < steps.length - 1 && (
                          <div
                            className={`mt-1 h-4 w-0.5 ${
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
                          <StepIcon className="text-muted-foreground h-3 w-3 flex-shrink-0" />
                          <div className="truncate text-sm font-medium">
                            {step.name}
                          </div>
                          <Badge
                            variant={getStepVariant(status)}
                            className="ml-auto flex-shrink-0 text-xs"
                          >
                            {step.type.replace("_", " ")}
                          </Badge>
                        </div>

                        {step.description && (
                          <p className="text-muted-foreground mt-1 line-clamp-2 text-xs">
                            {step.description}
                          </p>
                        )}

                        {isActive && trial.status === "in_progress" && (
                          <div className="mt-1 flex items-center gap-1">
                            <div className="bg-primary h-1.5 w-1.5 animate-pulse rounded-full" />
                            <span className="text-primary text-xs">
                              Executing
                            </span>
                          </div>
                        )}
                      </div>
                    </div>
                  );
                })}
              </div>
            </ScrollArea>
          </TabsContent>

          {/* Events Tab */}
          <TabsContent value="events" className="m-0 h-full">
            <ScrollArea className="h-full">
              <div className="p-3">
                {trialEvents.length === 0 ? (
                  <div className="flex h-32 items-center justify-center">
                    <div className="text-muted-foreground text-center text-sm">
                      No events recorded yet
                    </div>
                  </div>
                ) : (
                  <div className="space-y-2">
                    {trialEvents
                      .slice()
                      .reverse()
                      .map((event, index) => (
                        <div
                          key={`${event.timestamp.getTime()}-${index}`}
                          className="border-border/50 flex items-start gap-2 rounded-lg border p-2"
                        >
                          <div className="bg-muted flex h-6 w-6 flex-shrink-0 items-center justify-center rounded">
                            <Activity className="h-3 w-3" />
                          </div>
                          <div className="min-w-0 flex-1">
                            <div className="text-sm font-medium capitalize">
                              {event.type.replace(/_/g, " ")}
                            </div>
                            {event.message && (
                              <div className="text-muted-foreground mt-1 text-xs">
                                {event.message}
                              </div>
                            )}
                            <div className="text-muted-foreground mt-1 text-xs">
                              {event.timestamp.toLocaleTimeString()}
                            </div>
                          </div>
                        </div>
                      ))}
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
