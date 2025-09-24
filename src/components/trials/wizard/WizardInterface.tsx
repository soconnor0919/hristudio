"use client";

import React, { useState, useEffect } from "react";

import { Play, CheckCircle, X, Clock, AlertCircle } from "lucide-react";

import { Badge } from "~/components/ui/badge";
import { Progress } from "~/components/ui/progress";
import { Alert, AlertDescription } from "~/components/ui/alert";

import { PanelsContainer } from "~/components/experiments/designer/layout/PanelsContainer";
import { TrialControlPanel } from "./panels/TrialControlPanel";
import { ExecutionPanel } from "./panels/ExecutionPanel";
import { MonitoringPanel } from "./panels/MonitoringPanel";

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
  const { data: pollingData } = api.trials.get.useQuery(
    { id: trial.id },
    {
      enabled: !wsConnected && !wsConnecting,
      refetchInterval: wsConnected ? false : 5000,
    },
  );

  // Update trial data from polling
  React.useEffect(() => {
    if (pollingData && !wsConnected) {
      setTrial({
        ...pollingData,
        metadata: pollingData.metadata as Record<string, unknown> | null,
        participant: {
          ...pollingData.participant,
          demographics: pollingData.participant.demographics as Record<
            string,
            unknown
          > | null,
        },
      });
    }
  }, [pollingData, wsConnected]);

  // Transform experiment steps to component format
  const steps: StepData[] =
    experimentSteps?.map((step, index) => ({
      id: step.id,
      name: step.name ?? `Step ${index + 1}`,
      description: step.description,
      type: mapStepType(step.type),
      parameters: step.parameters ?? {},
      order: step.order ?? index,
    })) ?? [];

  const currentStep = steps[currentStepIndex] ?? null;
  const totalSteps = steps.length;
  const progressPercentage =
    totalSteps > 0 ? (currentStepIndex / totalSteps) * 100 : 0;

  // Timer effect for elapsed time
  useEffect(() => {
    if (!trialStartTime || trial.status !== "in_progress") return;

    const interval = setInterval(() => {
      const now = new Date();
      const elapsed = Math.floor(
        (now.getTime() - trialStartTime.getTime()) / 1000,
      );
      setElapsedTime(elapsed);
    }, 1000);

    return () => clearInterval(interval);
  }, [trialStartTime, trial.status]);

  // Format elapsed time
  const formatElapsedTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  // Status badge configuration
  const getStatusConfig = (status: string) => {
    switch (status) {
      case "scheduled":
        return { variant: "outline" as const, color: "blue", icon: Clock };
      case "in_progress":
        return { variant: "default" as const, color: "green", icon: Play };
      case "completed":
        return {
          variant: "secondary" as const,
          color: "gray",
          icon: CheckCircle,
        };
      case "aborted":
        return { variant: "destructive" as const, color: "orange", icon: X };
      case "failed":
        return {
          variant: "destructive" as const,
          color: "red",
          icon: AlertCircle,
        };
      default:
        return { variant: "outline" as const, color: "gray", icon: Clock };
    }
  };

  const statusConfig = getStatusConfig(trial.status);
  const StatusIcon = statusConfig.icon;

  // Mutations for trial actions
  const startTrialMutation = api.trials.start.useMutation({
    onSuccess: (data) => {
      setTrial({ ...trial, status: data.status, startedAt: data.startedAt });
      setTrialStartTime(new Date());
    },
  });

  const completeTrialMutation = api.trials.complete.useMutation({
    onSuccess: (data) => {
      if (data) {
        setTrial({
          ...trial,
          status: data.status,
          completedAt: data.completedAt,
        });
      }
    },
  });

  const abortTrialMutation = api.trials.abort.useMutation({
    onSuccess: (data) => {
      setTrial({ ...trial, status: data.status });
    },
  });

  // Action handlers
  const handleStartTrial = async () => {
    try {
      await startTrialMutation.mutateAsync({ id: trial.id });
    } catch (error) {
      console.error("Failed to start trial:", error);
    }
  };

  const handlePauseTrial = async () => {
    // TODO: Implement pause functionality
    console.log("Pause trial");
  };

  const handleNextStep = () => {
    if (currentStepIndex < steps.length - 1) {
      setCurrentStepIndex(currentStepIndex + 1);
      transitionStep?.({
        to_step: currentStepIndex + 1,
        from_step: currentStepIndex,
        step_name: steps[currentStepIndex + 1]?.name,
      });
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
    try {
      await abortTrialMutation.mutateAsync({ id: trial.id });
    } catch (error) {
      console.error("Failed to abort trial:", error);
    }
  };

  const handleExecuteAction = async (
    actionId: string,
    parameters?: Record<string, unknown>,
  ) => {
    try {
      executeTrialAction?.(actionId, parameters ?? {});
    } catch (error) {
      console.error("Failed to execute action:", error);
    }
  };

  return (
    <div className="flex h-full flex-col">
      {/* Status Bar */}
      <div className="bg-background border-b px-4 py-2">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <Badge
              variant={statusConfig.variant}
              className="flex items-center gap-1"
            >
              <StatusIcon className="h-3 w-3" />
              {trial.status.replace("_", " ")}
            </Badge>

            {trial.status === "in_progress" && (
              <div className="flex items-center gap-1 font-mono text-sm">
                <Clock className="h-3 w-3" />
                {formatElapsedTime(elapsedTime)}
              </div>
            )}

            {steps.length > 0 && (
              <div className="flex items-center gap-2 text-sm">
                <span className="text-muted-foreground">
                  Step {currentStepIndex + 1} of {totalSteps}
                </span>
                <div className="w-16">
                  <Progress value={progressPercentage} className="h-2" />
                </div>
              </div>
            )}
          </div>

          <div className="text-muted-foreground text-sm">
            {trial.experiment.name} • {trial.participant.participantCode}
          </div>
        </div>
      </div>

      {/* WebSocket Connection Status */}
      {wsError && (
        <Alert className="mx-4 mt-4">
          <AlertCircle className="h-4 w-4" />
          <AlertDescription>
            WebSocket connection failed. Using fallback polling. Some features
            may be limited.
          </AlertDescription>
        </Alert>
      )}

      {/* Main Content - Three Panel Layout */}
      <div className="min-h-0 flex-1">
        <PanelsContainer
          left={
            <TrialControlPanel
              trial={trial}
              currentStep={currentStep}
              steps={steps}
              currentStepIndex={currentStepIndex}
              onStartTrial={handleStartTrial}
              onPauseTrial={handlePauseTrial}
              onNextStep={handleNextStep}
              onCompleteTrial={handleCompleteTrial}
              onAbortTrial={handleAbortTrial}
              onExecuteAction={handleExecuteAction}
              isConnected={wsConnected}
            />
          }
          center={
            <ExecutionPanel
              trial={trial}
              currentStep={currentStep}
              steps={steps}
              currentStepIndex={currentStepIndex}
              trialEvents={trialEvents.map((event) => ({
                type: event.type ?? "unknown",
                timestamp:
                  "data" in event &&
                  event.data &&
                  typeof event.data === "object" &&
                  "timestamp" in event.data &&
                  typeof event.data.timestamp === "number"
                    ? new Date(event.data.timestamp)
                    : new Date(),
                data: "data" in event ? event.data : undefined,
                message:
                  "data" in event &&
                  event.data &&
                  typeof event.data === "object" &&
                  "message" in event.data &&
                  typeof event.data.message === "string"
                    ? event.data.message
                    : undefined,
              }))}
              onStepSelect={(index) => setCurrentStepIndex(index)}
              onExecuteAction={handleExecuteAction}
            />
          }
          right={
            <MonitoringPanel
              trial={trial}
              trialEvents={trialEvents.map((event) => ({
                type: event.type ?? "unknown",
                timestamp:
                  "data" in event &&
                  event.data &&
                  typeof event.data === "object" &&
                  "timestamp" in event.data &&
                  typeof event.data.timestamp === "number"
                    ? new Date(event.data.timestamp)
                    : new Date(),
                data: "data" in event ? event.data : undefined,
                message:
                  "data" in event &&
                  event.data &&
                  typeof event.data === "object" &&
                  "message" in event.data &&
                  typeof event.data.message === "string"
                    ? event.data.message
                    : undefined,
              }))}
              isConnected={wsConnected}
              wsError={wsError ?? undefined}
            />
          }
          showDividers={true}
          className="h-full"
        />
      </div>
    </div>
  );
}

export default WizardInterface;
