"use client";

import React, { useState, useEffect } from "react";
import { useRouter } from "next/navigation";
import {
  Play,
  SkipForward,
  CheckCircle,
  X,
  Clock,
  AlertCircle,
  Bot,
  User,
  Activity,
  Zap,
  Settings,
} from "lucide-react";

import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Progress } from "~/components/ui/progress";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import { PageHeader } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";

import { PanelsContainer } from "~/components/experiments/designer/layout/PanelsContainer";
import { ActionControls } from "./ActionControls";
import { RobotStatus } from "./RobotStatus";
import { ParticipantInfo } from "./ParticipantInfo";
import { EventsLogSidebar } from "./EventsLogSidebar";

import { api } from "~/trpc/react";
import { useTrialWebSocket } from "~/hooks/useWebSocket";

interface WizardInterfaceProps {
  trial: {
    id: string;
    status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
    scheduledAt: Date | null;
    startedAt: Date | null;
    completedAt: Date | null;
    duration: number | null;
    sessionNumber: number | null;
    notes: string | null;
    metadata: Record<string, unknown> | null;
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
  };
  userRole: string;
}

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

export function WizardInterface({
  trial: initialTrial,
  userRole: _userRole,
}: WizardInterfaceProps) {
  const router = useRouter();
  const [trial, setTrial] = useState(initialTrial);
  const [currentStepIndex, setCurrentStepIndex] = useState(0);
  const [trialStartTime, setTrialStartTime] = useState<Date | null>(
    initialTrial.startedAt ? new Date(initialTrial.startedAt) : null,
  );
  const [elapsedTime, setElapsedTime] = useState(0);

  // Get experiment steps from API
  const { data: experimentSteps } = api.experiments.getSteps.useQuery(
    { experimentId: trial.experimentId },
    {
      enabled: !!trial.experimentId,
      staleTime: 30000,
    },
  );

  // Get study data for breadcrumbs
  const { data: studyData } = api.studies.get.useQuery(
    { id: trial.experiment.studyId },
    { enabled: !!trial.experiment.studyId },
  );

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    ...(studyData
      ? [
          { label: studyData.name, href: `/studies/${studyData.id}` },
          { label: "Trials", href: `/studies/${studyData.id}/trials` },
        ]
      : []),
    {
      label: `Trial ${trial.participant.participantCode}`,
      href: `/trials/${trial.id}`,
    },
    { label: "Wizard Control" },
  ]);

  // Map database step types to component step types
  const mapStepType = (dbType: string) => {
    switch (dbType) {
      case "wizard":
        return "wizard_action" as const;
      case "robot":
        return "robot_action" as const;
      case "parallel":
        return "parallel_steps" as const;
      case "conditional":
        return "conditional_branch" as const;
      default:
        return "wizard_action" as const;
    }
  };

  // Real-time WebSocket connection
  const {
    isConnected: wsConnected,
    isConnecting: wsConnecting,
    connectionError: wsError,
    trialEvents,
    executeTrialAction,
    transitionStep,
  } = useTrialWebSocket(trial.id);

  // Fallback polling for trial updates when WebSocket is not available
  api.trials.get.useQuery(
    { id: trial.id },
    {
      refetchInterval: wsConnected ? 10000 : 2000,
      refetchOnWindowFocus: true,
      enabled: !wsConnected,
    },
  );

  // Mutations for trial control
  const startTrialMutation = api.trials.start.useMutation({
    onSuccess: (data) => {
      setTrial((prev) => ({
        ...prev,
        status: data.status,
        startedAt: data.startedAt,
      }));
      setTrialStartTime(new Date());
    },
  });

  const completeTrialMutation = api.trials.complete.useMutation({
    onSuccess: (data) => {
      if (data) {
        setTrial((prev) => ({
          ...prev,
          status: data.status,
          completedAt: data.completedAt,
        }));
      }
      router.push(`/trials/${trial.id}/analysis`);
    },
  });

  const abortTrialMutation = api.trials.abort.useMutation({
    onSuccess: (data) => {
      if (data) {
        setTrial((prev) => ({
          ...prev,
          status: data.status,
          completedAt: data.completedAt,
        }));
      }
      router.push(`/trials/${trial.id}`);
    },
  });

  // Process steps from API response
  const steps: StepData[] = React.useMemo(() => {
    if (!experimentSteps) return [];
    return experimentSteps.map((step) => ({
      id: step.id,
      name: step.name,
      description: step.description,
      type: mapStepType(step.type),
      parameters:
        typeof step.parameters === "object" && step.parameters !== null
          ? step.parameters
          : {},
      order: step.order,
    }));
  }, [experimentSteps]);

  const currentStep = steps[currentStepIndex] ?? null;
  const progress =
    steps.length > 0 ? ((currentStepIndex + 1) / steps.length) * 100 : 0;

  // Update elapsed time
  useEffect(() => {
    if (!trialStartTime || trial.status !== "in_progress") return;

    const interval = setInterval(() => {
      setElapsedTime(
        Math.floor((Date.now() - trialStartTime.getTime()) / 1000),
      );
    }, 1000);

    return () => clearInterval(interval);
  }, [trialStartTime, trial.status]);

  // Format elapsed time
  const formatElapsedTime = (seconds: number) => {
    const minutes = Math.floor(seconds / 60);
    const remainingSeconds = seconds % 60;
    return `${minutes}:${remainingSeconds.toString().padStart(2, "0")}`;
  };

  // Trial control handlers
  const handleStartTrial = async () => {
    try {
      await startTrialMutation.mutateAsync({ id: trial.id });
    } catch (error) {
      console.error("Failed to start trial:", error);
    }
  };

  const handleCompleteTrial = async () => {
    try {
      await completeTrialMutation.mutateAsync({ id: trial.id });
    } catch (error) {
      console.error("Failed to complete trial:", error);
    }
  };

  const handleAbortTrial = async () => {
    if (window.confirm("Are you sure you want to abort this trial?")) {
      try {
        await abortTrialMutation.mutateAsync({ id: trial.id });
      } catch (error) {
        console.error("Failed to abort trial:", error);
      }
    }
  };

  const handleNextStep = () => {
    if (currentStepIndex < steps.length - 1) {
      setCurrentStepIndex(currentStepIndex + 1);
      if (transitionStep) {
        void transitionStep({
          to_step: currentStepIndex + 1,
          from_step: currentStepIndex,
          step_name: steps[currentStepIndex + 1]?.name,
        });
      }
    }
  };

  const handlePreviousStep = () => {
    if (currentStepIndex > 0) {
      setCurrentStepIndex(currentStepIndex - 1);
      if (transitionStep) {
        void transitionStep({
          to_step: currentStepIndex - 1,
          from_step: currentStepIndex,
          step_name: steps[currentStepIndex - 1]?.name,
        });
      }
    }
  };

  const handleCompleteWizardAction = (
    actionId: string,
    actionData: Record<string, unknown>,
  ) => {
    if (executeTrialAction) {
      void executeTrialAction(actionId, actionData);
    }
  };

  // Left panel - Trial controls and step navigation
  const leftPanel = (
    <div className="h-full space-y-4 p-4">
      {/* Trial Status */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center justify-between text-sm">
            <span>Trial Status</span>
            <Badge
              variant={
                trial.status === "in_progress"
                  ? "default"
                  : trial.status === "completed"
                    ? "secondary"
                    : "outline"
              }
            >
              {trial.status.replace("_", " ")}
            </Badge>
          </CardTitle>
        </CardHeader>
        <CardContent>
          {trial.status === "in_progress" && (
            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span>Elapsed</span>
                <span>{formatElapsedTime(elapsedTime)}</span>
              </div>
              <div className="flex justify-between text-sm">
                <span>Step</span>
                <span>
                  {currentStepIndex + 1} of {steps.length}
                </span>
              </div>
              <Progress value={progress} className="h-2" />
            </div>
          )}

          {trial.status === "scheduled" && (
            <Button
              onClick={handleStartTrial}
              disabled={startTrialMutation.isPending}
              className="w-full"
            >
              <Play className="mr-2 h-4 w-4" />
              Start Trial
            </Button>
          )}
        </CardContent>
      </Card>

      {/* Trial Controls */}
      {trial.status === "in_progress" && (
        <Card>
          <CardHeader className="pb-3">
            <CardTitle className="text-sm">Trial Controls</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2">
            <Button
              onClick={handleNextStep}
              disabled={currentStepIndex >= steps.length - 1}
              className="w-full"
              size="sm"
            >
              <SkipForward className="mr-2 h-4 w-4" />
              Next Step
            </Button>
            <Button
              onClick={handleCompleteTrial}
              disabled={completeTrialMutation.isPending}
              variant="outline"
              className="w-full"
              size="sm"
            >
              <CheckCircle className="mr-2 h-4 w-4" />
              Complete Trial
            </Button>
            <Button
              onClick={handleAbortTrial}
              disabled={abortTrialMutation.isPending}
              variant="destructive"
              className="w-full"
              size="sm"
            >
              <X className="mr-2 h-4 w-4" />
              Abort Trial
            </Button>
          </CardContent>
        </Card>
      )}

      {/* Step List */}
      {steps.length > 0 && (
        <Card>
          <CardHeader className="pb-3">
            <CardTitle className="text-sm">Experiment Steps</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="max-h-64 space-y-2 overflow-y-auto">
              {steps.map((step, index) => (
                <div
                  key={step.id}
                  className={`flex items-center gap-2 rounded-lg p-2 text-sm ${
                    index === currentStepIndex
                      ? "bg-primary/10 border-primary/20 border"
                      : index < currentStepIndex
                        ? "border border-green-200 bg-green-50"
                        : "bg-muted/50"
                  }`}
                >
                  <div className="bg-background flex h-5 w-5 items-center justify-center rounded-full text-xs font-medium">
                    {index + 1}
                  </div>
                  <div className="min-w-0 flex-1">
                    <div className="truncate font-medium">{step.name}</div>
                    {step.description && (
                      <div className="text-muted-foreground truncate text-xs">
                        {step.description}
                      </div>
                    )}
                  </div>
                </div>
              ))}
            </div>
          </CardContent>
        </Card>
      )}
    </div>
  );

  // Center panel - Main execution area
  const centerPanel = (
    <div className="h-full space-y-6 p-6">
      {/* Connection Status Alert */}
      {wsError && wsError.length > 0 && !wsConnecting && (
        <Alert
          variant={wsError.includes("polling mode") ? "default" : "destructive"}
        >
          <AlertCircle className="h-4 w-4" />
          <AlertDescription>
            {wsError.includes("polling mode")
              ? "Real-time connection unavailable - using polling for updates"
              : wsError}
          </AlertDescription>
        </Alert>
      )}

      {trial.status === "scheduled" ? (
        // Trial not started
        <Card>
          <CardContent className="py-12 text-center">
            <Clock className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
            <h3 className="mb-2 text-lg font-semibold">Trial Scheduled</h3>
            <p className="text-muted-foreground mb-4">
              This trial is scheduled and ready to begin. Click &quot;Start
              Trial&quot; in the left panel to begin execution.
            </p>
          </CardContent>
        </Card>
      ) : trial.status === "in_progress" ? (
        // Trial in progress - Current step and controls
        <div className="space-y-6">
          {/* Current Step */}
          {currentStep && (
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  <Play className="h-5 w-5" />
                  Current Step: {currentStep.name}
                </CardTitle>
              </CardHeader>
              <CardContent>
                <p className="text-muted-foreground mb-4">
                  {currentStep.description}
                </p>
                <div className="flex gap-2">
                  {currentStepIndex > 0 && (
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={handlePreviousStep}
                    >
                      Previous
                    </Button>
                  )}
                  <Button
                    size="sm"
                    onClick={handleNextStep}
                    disabled={currentStepIndex >= steps.length - 1}
                  >
                    Next Step
                  </Button>
                </div>
              </CardContent>
            </Card>
          )}

          {/* Wizard Actions */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Zap className="h-5 w-5" />
                Wizard Actions
              </CardTitle>
            </CardHeader>
            <CardContent>
              <ActionControls
                trialId={trial.id}
                currentStep={
                  currentStep
                    ? {
                        id: currentStep.id,
                        name: currentStep.name,
                        type: currentStep.type,
                        description: currentStep.description ?? undefined,
                        parameters: currentStep.parameters,
                      }
                    : null
                }
                onActionComplete={handleCompleteWizardAction}
                isConnected={wsConnected}
              />
            </CardContent>
          </Card>
        </div>
      ) : (
        // Trial completed/aborted
        <Card>
          <CardContent className="py-12 text-center">
            <CheckCircle className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
            <h3 className="mb-2 text-lg font-semibold">
              Trial {trial.status === "completed" ? "Completed" : "Ended"}
            </h3>
            <p className="text-muted-foreground mb-4">
              This trial has{" "}
              {trial.status === "completed"
                ? "completed successfully"
                : "ended"}
              . You can view the results and analysis data.
            </p>
            <Button asChild>
              <a href={`/trials/${trial.id}/analysis`}>View Analysis</a>
            </Button>
          </CardContent>
        </Card>
      )}
    </div>
  );

  // Right panel - Monitoring and context
  const rightPanel = (
    <div className="h-full space-y-4 p-4">
      {/* Robot Status */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Bot className="h-4 w-4" />
            Robot Status
          </CardTitle>
        </CardHeader>
        <CardContent>
          <RobotStatus trialId={trial.id} />
        </CardContent>
      </Card>

      {/* Participant Info */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <User className="h-4 w-4" />
            Participant
          </CardTitle>
        </CardHeader>
        <CardContent>
          <ParticipantInfo
            participant={trial.participant}
            trialStatus={trial.status}
          />
        </CardContent>
      </Card>

      {/* Live Events */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Activity className="h-4 w-4" />
            Live Events
          </CardTitle>
        </CardHeader>
        <CardContent>
          <EventsLogSidebar
            events={trialEvents}
            maxEvents={15}
            showTimestamps={true}
          />
        </CardContent>
      </Card>

      {/* Connection Status */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Settings className="h-4 w-4" />
            Connection
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-sm">Status</span>
              <Badge variant={wsConnected ? "default" : "secondary"}>
                {wsConnected ? "Connected" : "Polling"}
              </Badge>
            </div>
            <Separator />
            <div className="text-muted-foreground space-y-1 text-xs">
              <div>Trial ID: {trial.id.slice(-8)}</div>
              <div>Experiment: {trial.experiment.name}</div>
              <div>Participant: {trial.participant.participantCode}</div>
              {trialStartTime && (
                <div>Started: {trialStartTime.toLocaleTimeString()}</div>
              )}
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );

  return (
    <div className="flex h-screen flex-col">
      {/* Page Header */}
      <PageHeader
        title="Wizard Control"
        description={`${trial.experiment.name} • ${trial.participant.participantCode}`}
        icon={Activity}
      />

      {/* Main Panel Layout */}
      <PanelsContainer
        left={leftPanel}
        center={centerPanel}
        right={rightPanel}
        showDividers={true}
        className="min-h-0 flex-1"
      />
    </div>
  );
}
