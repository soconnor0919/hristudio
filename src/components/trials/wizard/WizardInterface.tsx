"use client";

import {
    Activity, AlertTriangle, CheckCircle, Play, SkipForward, Square, Timer, Wifi,
    WifiOff
} from "lucide-react";
import { useRouter } from "next/navigation";
import { useCallback, useEffect, useState } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Progress } from "~/components/ui/progress";
import { useTrialWebSocket } from "~/hooks/useWebSocket";
import { api } from "~/trpc/react";
import { EventsLog } from "../execution/EventsLog";
import { ActionControls } from "./ActionControls";
import { ParticipantInfo } from "./ParticipantInfo";
import { RobotStatus } from "./RobotStatus";
import { StepDisplay } from "./StepDisplay";
import { TrialProgress } from "./TrialProgress";

interface WizardInterfaceProps {
  trial: {
    id: string;
    participantId: string | null;
    experimentId: string;
    status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
    startedAt: Date | null;
    completedAt: Date | null;
    duration: number | null;
    notes: string | null;
    metadata: any;
    createdAt: Date;
    updatedAt: Date;
    experiment: {
      id: string;
      name: string;
      description: string | null;
      studyId: string;
    };
    participant: {
      id: string;
      participantCode: string;
      demographics: any;
    };
  };
  userRole: string;
}

export function WizardInterface({
  trial: initialTrial,
  userRole,
}: WizardInterfaceProps) {
  const router = useRouter();
  const [trial, setTrial] = useState(initialTrial);
  const [currentStepIndex, setCurrentStepIndex] = useState(0);
  const [trialStartTime, setTrialStartTime] = useState<Date | null>(
    initialTrial.startedAt ? new Date(initialTrial.startedAt) : null,
  );
  const [refreshKey, setRefreshKey] = useState(0);

  // Real-time WebSocket connection
  const {
    isConnected: wsConnected,
    isConnecting: wsConnecting,
    connectionError: wsError,
    currentTrialStatus,
    trialEvents,
    wizardActions,
    executeTrialAction,
    logWizardIntervention,
    transitionStep,
  } = useTrialWebSocket(trial.id);

  // Fallback polling for trial updates when WebSocket is not available
  const { data: trialUpdates } = api.trials.get.useQuery(
    { id: trial.id },
    {
      refetchInterval: wsConnected ? 10000 : 2000, // Less frequent polling when WebSocket is active
      refetchOnWindowFocus: true,
      enabled: !wsConnected, // Disable when WebSocket is connected
    },
  );

  // Mutations for trial control
  const startTrialMutation = api.trials.start.useMutation({
    onSuccess: (data) => {
      setTrial((prev) => ({ ...prev, ...data }));
      setTrialStartTime(new Date());
      setRefreshKey((prev) => prev + 1);
    },
  });

  const completeTrialMutation = api.trials.complete.useMutation({
    onSuccess: (data) => {
      setTrial((prev) => ({ ...prev, ...data }));
      setRefreshKey((prev) => prev + 1);
      // Redirect to analysis page after completion
      setTimeout(() => {
        router.push(`/trials/${trial.id}/analysis`);
      }, 2000);
    },
  });

  const abortTrialMutation = api.trials.abort.useMutation({
    onSuccess: (data) => {
      setTrial((prev) => ({ ...prev, ...data }));
      setRefreshKey((prev) => prev + 1);
    },
  });

  const logEventMutation = api.trials.logEvent.useMutation({
    onSuccess: () => {
      setRefreshKey((prev) => prev + 1);
    },
  });

  // Update trial state when data changes (WebSocket has priority)
  useEffect(() => {
    const latestTrial = currentTrialStatus || trialUpdates;
    if (latestTrial) {
      setTrial(latestTrial);
      if (latestTrial.startedAt && !trialStartTime) {
        setTrialStartTime(new Date(latestTrial.startedAt));
      }
    }
  }, [currentTrialStatus, trialUpdates, trialStartTime]);

  // Mock experiment steps for now - in real implementation, fetch from experiment API
  const experimentSteps = [
    {
      id: "step1",
      name: "Initial Greeting",
      type: "wizard_action" as const,
      description: "Greet the participant and explain the task",
      duration: 60,
    },
    {
      id: "step2",
      name: "Robot Introduction",
      type: "robot_action" as const,
      description: "Robot introduces itself to participant",
      duration: 30,
    },
    {
      id: "step3",
      name: "Task Demonstration",
      type: "wizard_action" as const,
      description: "Demonstrate the task to the participant",
      duration: 120,
    },
  ];
  const currentStep = experimentSteps[currentStepIndex];
  const progress =
    experimentSteps.length > 0
      ? ((currentStepIndex + 1) / experimentSteps.length) * 100
      : 0;

  // Trial control handlers using WebSocket when available
  const handleStartTrial = useCallback(async () => {
    try {
      if (wsConnected) {
        executeTrialAction("start_trial", {
          step_index: 0,
          data: { notes: "Trial started by wizard" },
        });
      } else {
        await startTrialMutation.mutateAsync({ id: trial.id });
        await logEventMutation.mutateAsync({
          trialId: trial.id,
          type: "trial_start",
          data: { step_index: 0, notes: "Trial started by wizard" },
        });
      }
    } catch (_error) {
      console.error("Failed to start trial:", _error);
    }
  }, [
    trial.id,
    wsConnected,
    executeTrialAction,
    startTrialMutation,
    logEventMutation,
  ]);

  const handleCompleteTrial = useCallback(async () => {
    try {
      if (wsConnected) {
        executeTrialAction("complete_trial", {
          final_step_index: currentStepIndex,
          completion_type: "wizard_completed",
          notes: "Trial completed successfully via wizard interface",
        });
      } else {
        await completeTrialMutation.mutateAsync({
          id: trial.id,
          notes: "Trial completed successfully via wizard interface",
        });
        await logEventMutation.mutateAsync({
          trialId: trial.id,
          type: "trial_end",
          data: {
            final_step_index: currentStepIndex,
            completion_type: "wizard_completed",
            notes: "Trial completed by wizard",
          },
        });
      }
    } catch (_error) {
      console.error("Failed to complete trial:", _error);
    }
  }, [
    trial.id,
    currentStepIndex,
    wsConnected,
    executeTrialAction,
    completeTrialMutation,
    logEventMutation,
  ]);

  const handleAbortTrial = useCallback(async () => {
    try {
      if (wsConnected) {
        executeTrialAction("abort_trial", {
          abort_step_index: currentStepIndex,
          abort_reason: "wizard_abort",
          reason: "Aborted via wizard interface",
        });
      } else {
        await abortTrialMutation.mutateAsync({
          id: trial.id,
          reason: "Aborted via wizard interface",
        });
        await logEventMutation.mutateAsync({
          trialId: trial.id,
          type: "trial_end",
          data: {
            abort_step_index: currentStepIndex,
            abort_reason: "wizard_abort",
            notes: "Trial aborted by wizard",
          },
        });
      }
    } catch (_error) {
      console.error("Failed to abort trial:", _error);
    }
  }, [
    trial.id,
    currentStepIndex,
    wsConnected,
    executeTrialAction,
    abortTrialMutation,
    logEventMutation,
  ]);

  const handleNextStep = useCallback(async () => {
    if (currentStepIndex < experimentSteps.length - 1) {
      const nextIndex = currentStepIndex + 1;
      setCurrentStepIndex(nextIndex);

      if (wsConnected) {
        transitionStep({
          from_step: currentStepIndex,
          to_step: nextIndex,
          step_name: experimentSteps[nextIndex]?.name,
                      data: { notes: `Advanced to step ${nextIndex + 1}: ${experimentSteps[nextIndex]?.name}` },
        });
      } else {
        await logEventMutation.mutateAsync({
          trialId: trial.id,
          type: "step_start",
          data: {
            from_step: currentStepIndex,
            to_step: nextIndex,
            step_name: experimentSteps[nextIndex]?.name,
            notes: `Advanced to step ${nextIndex + 1}: ${experimentSteps[nextIndex]?.name}`,
          },
        });
      }
    }
  }, [
    currentStepIndex,
    experimentSteps,
    trial.id,
    wsConnected,
    transitionStep,
    logEventMutation,
  ]);

  const handleExecuteAction = useCallback(
    async (actionType: string, actionData: any) => {
      if (wsConnected) {
        logWizardIntervention({
          action_type: actionType,
          step_index: currentStepIndex,
          step_name: currentStep?.name,
          action_data: actionData,
                      data: { notes: `Wizard executed ${actionType} action` },
        });
      } else {
        await logEventMutation.mutateAsync({
          trialId: trial.id,
          type: "wizard_intervention",
          data: {
            action_type: actionType,
            step_index: currentStepIndex,
            step_name: currentStep?.name,
            action_data: actionData,
            notes: `Wizard executed ${actionType} action`,
          },
        });
      }
    },
    [
      trial.id,
      currentStepIndex,
      currentStep?.name,
      wsConnected,
      logWizardIntervention,
      logEventMutation,
    ],
  );

  // Calculate elapsed time
  const elapsedTime = trialStartTime
    ? Math.floor((Date.now() - trialStartTime.getTime()) / 1000)
    : 0;

  const formatElapsedTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  return (
    <div className="flex h-[calc(100vh-120px)] bg-slate-50">
      {/* Left Panel - Main Control */}
      <div className="flex flex-1 flex-col space-y-6 overflow-y-auto p-6">
        {/* Trial Controls */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center justify-between">
              <div className="flex items-center space-x-2">
                <Activity className="h-5 w-5" />
                <span>Trial Control</span>
              </div>
              {/* WebSocket Connection Status */}
              <div className="flex items-center space-x-2">
                {wsConnected ? (
                  <Badge className="bg-green-100 text-green-800">
                    <Wifi className="mr-1 h-3 w-3" />
                    Real-time
                  </Badge>
                ) : wsConnecting ? (
                  <Badge className="bg-yellow-100 text-yellow-800">
                    <Activity className="mr-1 h-3 w-3 animate-spin" />
                    Connecting...
                  </Badge>
                ) : (
                  <Badge className="bg-red-100 text-red-800">
                    <WifiOff className="mr-1 h-3 w-3" />
                    Offline
                  </Badge>
                )}
              </div>
            </CardTitle>
            {wsError && (
              <Alert className="mt-2">
                <AlertTriangle className="h-4 w-4" />
                <AlertDescription className="text-sm">
                  Connection issue: {wsError}
                </AlertDescription>
              </Alert>
            )}
          </CardHeader>
          <CardContent className="space-y-4">
            {/* Status and Timer */}
            <div className="flex items-center justify-between">
              <div className="flex items-center space-x-4">
                <Badge
                  className={
                    trial.status === "in_progress"
                      ? "bg-green-100 text-green-800"
                      : trial.status === "scheduled"
                        ? "bg-blue-100 text-blue-800"
                        : "bg-gray-100 text-gray-800"
                  }
                >
                  {trial.status === "in_progress"
                    ? "Active"
                    : trial.status === "scheduled"
                      ? "Ready"
                      : "Inactive"}
                </Badge>
                {trial.status === "in_progress" && (
                  <div className="flex items-center space-x-2 text-sm text-slate-600">
                    <Timer className="h-4 w-4" />
                    <span className="font-mono text-lg">
                      {formatElapsedTime(elapsedTime)}
                    </span>
                  </div>
                )}
              </div>
            </div>

            {/* Progress Bar */}
            {experimentSteps.length > 0 && (
              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span>Progress</span>
                  <span>
                    {currentStepIndex + 1} of {experimentSteps.length} steps
                  </span>
                </div>
                <Progress value={progress} className="h-2" />
              </div>
            )}

            {/* Main Action Buttons */}
            <div className="flex space-x-2">
              {trial.status === "scheduled" && (
                <Button
                  onClick={handleStartTrial}
                  disabled={startTrialMutation.isPending}
                  className="flex-1"
                >
                  <Play className="mr-2 h-4 w-4" />
                  Start Trial
                </Button>
              )}

              {trial.status === "in_progress" && (
                <>
                  <Button
                    onClick={handleNextStep}
                    disabled={currentStepIndex >= experimentSteps.length - 1}
                    className="flex-1"
                  >
                    <SkipForward className="mr-2 h-4 w-4" />
                    Next Step
                  </Button>
                  <Button
                    onClick={handleCompleteTrial}
                    disabled={completeTrialMutation.isPending}
                    variant="outline"
                  >
                    <CheckCircle className="mr-2 h-4 w-4" />
                    Complete
                  </Button>
                  <Button
                    onClick={handleAbortTrial}
                    disabled={abortTrialMutation.isPending}
                    variant="destructive"
                  >
                    <Square className="mr-2 h-4 w-4" />
                    Abort
                  </Button>
                </>
              )}
            </div>
          </CardContent>
        </Card>

        {/* Current Step Display */}
        {currentStep && (
          <StepDisplay
            step={currentStep}
            stepIndex={currentStepIndex}
            totalSteps={experimentSteps.length}
            isActive={trial.status === "in_progress"}
            onExecuteAction={handleExecuteAction}
          />
        )}

        {/* Action Controls */}
        {trial.status === "in_progress" && (
          <ActionControls
            currentStep={currentStep ?? null}
            onExecuteAction={handleExecuteAction}
            trialId={trial.id}
          />
        )}

        {/* Trial Progress Overview */}
        <TrialProgress
          steps={experimentSteps}
          currentStepIndex={currentStepIndex}
          trialStatus={trial.status}
        />
      </div>

      {/* Right Panel - Info & Monitoring */}
      <div className="flex w-96 flex-col border-l border-slate-200 bg-white">
        {/* Participant Info */}
        <div className="border-b border-slate-200 p-4">
          <ParticipantInfo participant={{...trial.participant, email: null, name: null}} />
        </div>

        {/* Robot Status */}
        <div className="border-b border-slate-200 p-4">
          <RobotStatus trialId={trial.id} />
        </div>

        {/* Live Events Log */}
        <div className="flex-1 overflow-hidden">
          <EventsLog
            trialId={trial.id}
            refreshKey={refreshKey}
            isLive={trial.status === "in_progress"}
            realtimeEvents={trialEvents}
            isWebSocketConnected={wsConnected}
          />
        </div>
      </div>
    </div>
  );
}
