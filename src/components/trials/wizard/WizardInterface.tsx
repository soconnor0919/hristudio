"use client";

import React, { useState, useEffect, useCallback, useMemo } from "react";
import { Play, CheckCircle, X, Clock, AlertCircle, HelpCircle } from "lucide-react";
import { useRouter } from "next/navigation";
import { Badge } from "~/components/ui/badge";
import { Progress } from "~/components/ui/progress";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { PanelsContainer } from "~/components/experiments/designer/layout/PanelsContainer";
import { WizardControlPanel } from "./panels/WizardControlPanel";
import { WizardExecutionPanel } from "./panels/WizardExecutionPanel";
import { WizardMonitoringPanel } from "./panels/WizardMonitoringPanel";
import { WizardObservationPane } from "./panels/WizardObservationPane";
import {
  ResizableHandle,
  ResizablePanel,
  ResizablePanelGroup,
} from "~/components/ui/resizable";
import { api } from "~/trpc/react";
import { useWizardRos } from "~/hooks/useWizardRos";
import { toast } from "sonner";
import { useTour } from "~/components/onboarding/TourProvider";

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

interface ActionData {
  id: string;
  name: string;
  description: string | null;
  type: string;
  parameters: Record<string, unknown>;
  order: number;
  pluginId: string | null;
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
  actions: ActionData[];
}

export const WizardInterface = React.memo(function WizardInterface({
  trial: initialTrial,
  userRole: _userRole,
}: WizardInterfaceProps) {
  const { startTour } = useTour();
  const [trial, setTrial] = useState(initialTrial);
  const [currentStepIndex, setCurrentStepIndex] = useState(0);
  const [trialStartTime, setTrialStartTime] = useState<Date | null>(
    initialTrial.startedAt ? new Date(initialTrial.startedAt) : null,
  );
  const [elapsedTime, setElapsedTime] = useState(0);
  const router = useRouter();

  // Persistent tab states to prevent resets from parent re-renders
  const [controlPanelTab, setControlPanelTab] = useState<
    "control" | "step" | "actions" | "robot"
  >("control");
  const [executionPanelTab, setExecutionPanelTab] = useState<
    "current" | "timeline" | "events"
  >(trial.status === "in_progress" ? "current" : "timeline");
  const [isExecutingAction, setIsExecutingAction] = useState(false);
  const [monitoringPanelTab, setMonitoringPanelTab] = useState<
    "status" | "robot" | "events"
  >("status");
  const [completedActionsCount, setCompletedActionsCount] = useState(0);

  // Reset completed actions when step changes
  useEffect(() => {
    setCompletedActionsCount(0);
  }, [currentStepIndex]);

  // Get experiment steps from API
  const { data: experimentSteps } = api.experiments.getSteps.useQuery(
    { experimentId: trial.experimentId },
    {
      enabled: !!trial.experimentId,
      staleTime: 30000,
    },
  );

  // Robot action execution mutation
  const executeRobotActionMutation = api.trials.executeRobotAction.useMutation({
    onSuccess: (result) => {
      toast.success("Robot action executed successfully", {
        description: `Completed in ${result.duration}ms`,
      });
    },
    onError: (error) => {
      toast.error("Failed to execute robot action", {
        description: error.message,
      });
    },
  });

  // Log robot action mutation (for client-side execution)
  const logRobotActionMutation = api.trials.logRobotAction.useMutation({
    onError: (error) => {
      console.error("Failed to log robot action:", error);
    },
  });

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

  // Memoized callbacks to prevent infinite re-renders
  const onActionCompleted = useCallback((execution: { actionId: string }) => {
    toast.success(`Robot action completed: ${execution.actionId}`);
  }, []);

  const onActionFailed = useCallback((execution: { actionId: string; error?: string }) => {
    toast.error(`Robot action failed: ${execution.actionId}`, {
      description: execution.error,
    });
  }, []);

  // ROS WebSocket connection for robot control
  const {
    isConnected: rosConnected,
    isConnecting: rosConnecting,
    connectionError: rosError,
    robotStatus,
    connect: connectRos,
    disconnect: disconnectRos,
    executeRobotAction: executeRosAction,
    setAutonomousLife,
  } = useWizardRos({
    autoConnect: true,
    onActionCompleted,
    onActionFailed,
  });

  // Use polling for trial status updates (no trial WebSocket server exists)
  const { data: pollingData } = api.trials.get.useQuery(
    { id: trial.id },
    {
      refetchInterval: trial.status === "in_progress" ? 5000 : 15000,
      staleTime: 2000,
      refetchOnWindowFocus: false,
    },
  );

  // Poll for trial events
  const { data: fetchedEvents } = api.trials.getEvents.useQuery(
    { trialId: trial.id, limit: 100 },
    {
      refetchInterval: 3000,
      staleTime: 1000,
    }
  );

  // Update local trial state from polling only if changed
  useEffect(() => {
    if (pollingData && JSON.stringify(pollingData) !== JSON.stringify(trial)) {
      // Only update if specific fields we care about have changed to avoid
      // unnecessary re-renders that might cause UI flashing
      if (pollingData.status !== trial.status ||
        pollingData.startedAt?.getTime() !== trial.startedAt?.getTime() ||
        pollingData.completedAt?.getTime() !== trial.completedAt?.getTime()) {

        setTrial((prev) => ({
          ...prev,
          status: pollingData.status,
          startedAt: pollingData.startedAt
            ? new Date(pollingData.startedAt)
            : prev.startedAt,
          completedAt: pollingData.completedAt
            ? new Date(pollingData.completedAt)
            : prev.completedAt,
        }));
      }
    }
  }, [pollingData, trial]);

  // Auto-start trial on mount if scheduled
  useEffect(() => {
    if (trial.status === "scheduled") {
      handleStartTrial();
    }
  }, []); // Run once on mount

  // Trial events from robot actions

  const trialEvents = useMemo<
    Array<{
      type: string;
      timestamp: Date;
      data?: unknown;
      message?: string;
    }>
  >(() => {
    return (fetchedEvents ?? []).map(event => {
      let message: string | undefined;
      const eventData = event.data as any;

      // Extract or generate message based on event type
      if (event.eventType.startsWith('annotation_')) {
        message = eventData?.description || eventData?.label || 'Annotation added';
      } else if (event.eventType.startsWith('robot_action_')) {
        const actionName = event.eventType.replace('robot_action_', '').replace(/_/g, ' ');
        message = `Robot action: ${actionName}`;
      } else if (event.eventType === 'trial_started') {
        message = 'Trial started';
      } else if (event.eventType === 'trial_completed') {
        message = 'Trial completed';
      } else if (event.eventType === 'step_changed') {
        message = `Step changed to: ${eventData?.stepName || 'next step'}`;
      } else if (event.eventType.startsWith('wizard_')) {
        message = eventData?.notes || eventData?.message || event.eventType.replace('wizard_', '').replace(/_/g, ' ');
      } else {
        // Generic fallback
        message = eventData?.notes || eventData?.message || eventData?.description || event.eventType.replace(/_/g, ' ');
      }

      return {
        type: event.eventType,
        timestamp: new Date(event.timestamp),
        data: event.data,
        message,
      };
    }).sort((a, b) => b.timestamp.getTime() - a.timestamp.getTime()); // Newest first
  }, [fetchedEvents]);

  // Transform experiment steps to component format
  const steps: StepData[] =
    experimentSteps?.map((step, index) => ({
      id: step.id,
      name: step.name ?? `Step ${index + 1}`,
      description: step.description,
      type: mapStepType(step.type),
      parameters: step.parameters ?? {},
      order: step.order ?? index,
      actions: step.actions?.map((action) => ({
        id: action.id,
        name: action.name,
        description: action.description,
        type: action.type,
        parameters: action.parameters ?? {},
        order: action.order,
        pluginId: action.pluginId,
      })) ?? [],
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
        toast.success("Trial completed! Redirecting to analysis...");
        router.push(`/studies/${trial.experiment.studyId}/trials/${trial.id}/analysis`);
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
    console.log(
      "[WizardInterface] Starting trial:",
      trial.id,
      "Current status:",
      trial.status,
    );

    // Check if trial can be started
    if (trial.status !== "scheduled") {
      toast.error("Trial can only be started from scheduled status");
      return;
    }

    try {
      const result = await startTrialMutation.mutateAsync({ id: trial.id });
      console.log("[WizardInterface] Trial started successfully", result);

      // Update local state immediately
      setTrial((prev) => ({
        ...prev,
        status: "in_progress",
        startedAt: new Date(),
      }));
      setTrialStartTime(new Date());

      toast.success("Trial started successfully");
    } catch (error) {
      console.error("Failed to start trial:", error);
      toast.error(
        `Failed to start trial: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    }
  };

  const handlePauseTrial = async () => {
    // TODO: Implement pause functionality
    console.log("Pause trial");
  };

  const handleNextStep = () => {
    if (currentStepIndex < steps.length - 1) {
      setCompletedActionsCount(0); // Reset immediately to prevent flickering/double-click issues
      setCurrentStepIndex(currentStepIndex + 1);
      // Note: Step transitions can be enhanced later with database logging
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

  // Mutations for annotations
  const addAnnotationMutation = api.trials.addAnnotation.useMutation({
    onSuccess: () => {
      toast.success("Note added");
    },
    onError: (error) => {
      toast.error("Failed to add note", { description: error.message });
    },
  });

  const handleAddAnnotation = async (
    description: string,
    category?: string,
    tags?: string[],
  ) => {
    await addAnnotationMutation.mutateAsync({
      trialId: trial.id,
      description,
      category,
      tags,
    });
  };

  // Mutation for events (Acknowledge)
  const logEventMutation = api.trials.logEvent.useMutation({
    onSuccess: () => toast.success("Event logged"),
  });

  // Mutation for interventions
  const addInterventionMutation = api.trials.addIntervention.useMutation({
    onSuccess: () => toast.success("Intervention logged"),
  });

  const handleExecuteAction = async (
    actionId: string,
    parameters?: Record<string, unknown>,
  ) => {
    try {
      console.log("Executing action:", actionId, parameters);

      if (actionId === "acknowledge") {
        await logEventMutation.mutateAsync({
          trialId: trial.id,
          type: "wizard_acknowledge",
          data: parameters,
        });
        handleNextStep();
      } else if (actionId === "intervene") {
        await addInterventionMutation.mutateAsync({
          trialId: trial.id,
          type: "manual_intervention",
          description: "Wizard manual intervention triggered",
          data: parameters,
        });
      } else if (actionId === "note") {
        await addAnnotationMutation.mutateAsync({
          trialId: trial.id,
          description: String(parameters?.content || "Quick note"),
          category: String(parameters?.category || "quick_note")
        });
      }

      // Note: Action execution can be enhanced later with tRPC mutations
    } catch (error) {
      console.error("Failed to execute action:", error);
      toast.error("Failed to execute action");
    }
  };

  const handleExecuteRobotAction = useCallback(
    async (
      pluginName: string,
      actionId: string,
      parameters: Record<string, unknown>,
      options?: { autoAdvance?: boolean },
    ) => {
      try {
        setIsExecutingAction(true);
        // Try direct WebSocket execution first for better performance
        if (rosConnected) {
          try {
            const result = await executeRosAction(pluginName, actionId, parameters);

            const duration =
              result.endTime && result.startTime
                ? result.endTime.getTime() - result.startTime.getTime()
                : 0;

            // Log to trial events for data capture
            await logRobotActionMutation.mutateAsync({
              trialId: trial.id,
              pluginName,
              actionId,
              parameters,
              duration,
              result: { status: result.status },
            });

            toast.success(`Robot action executed: ${actionId}`);
            if (options?.autoAdvance) {
              handleNextStep();
            }
          } catch (rosError) {
            console.warn(
              "WebSocket execution failed, falling back to tRPC:",
              rosError,
            );

            // Fallback to tRPC-only execution
            await executeRobotActionMutation.mutateAsync({
              trialId: trial.id,
              pluginName,
              actionId,
              parameters,
            });

            toast.success(`Robot action executed via fallback: ${actionId}`);
            if (options?.autoAdvance) {
              handleNextStep();
            }
          }
        } else {
          // Use tRPC execution if WebSocket not connected
          await executeRobotActionMutation.mutateAsync({
            trialId: trial.id,
            pluginName,
            actionId,
            parameters,
          });

          toast.success(`Robot action executed: ${actionId}`);
          if (options?.autoAdvance) {
            handleNextStep();
          }
        }
      } catch (error) {
        console.error("Failed to execute robot action:", error);
        toast.error(`Failed to execute robot action: ${actionId}`, {
          description: error instanceof Error ? error.message : "Unknown error",
        });
      } finally {
        setIsExecutingAction(false);
      }
    },
    [rosConnected, executeRosAction, executeRobotActionMutation, trial.id],
  );

  const handleSkipAction = useCallback(
    async (
      pluginName: string,
      actionId: string,
      parameters: Record<string, unknown>,
      options?: { autoAdvance?: boolean },
    ) => {
      try {
        await logRobotActionMutation.mutateAsync({
          trialId: trial.id,
          pluginName,
          actionId,
          parameters,
          duration: 0,
          result: { skipped: true },
        });

        toast.info(`Action skipped: ${actionId}`);
        if (options?.autoAdvance) {
          handleNextStep();
        }
      } catch (error) {
        console.error("Failed to skip action:", error);
        toast.error("Failed to skip action");
      }
    },
    [logRobotActionMutation, trial.id],
  );

  return (
    <div className="flex h-full flex-col">
      {/* Compact Status Bar */}
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

          <div className="text-muted-foreground flex items-center gap-4 text-sm">
            <div>{trial.experiment.name}</div>
            <div>{trial.participant.participantCode}</div>
            <Badge
              variant={rosConnected ? "default" : "outline"}
              className="text-xs"
            >
              {rosConnected ? "ROS Connected" : "ROS Offline"}
            </Badge>
            <button
              onClick={() => startTour("wizard")}
              className="hover:bg-muted p-1 rounded-full transition-colors"
              title="Start Tour"
            >
              <HelpCircle className="h-4 w-4" />
            </button>
          </div>
        </div>
      </div>

      {/* Main Content with Vertical Resizable Split */}
      <div className="min-h-0 flex-1">
        <ResizablePanelGroup direction="vertical">
          <ResizablePanel defaultSize={75} minSize={30}>
            <PanelsContainer
              left={
                <div id="tour-wizard-controls" className="h-full">
                  <WizardControlPanel
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
                    onExecuteRobotAction={handleExecuteRobotAction}
                    studyId={trial.experiment.studyId}
                    _isConnected={rosConnected}
                    activeTab={controlPanelTab}
                    onTabChange={setControlPanelTab}
                    isStarting={startTrialMutation.isPending}
                    onSetAutonomousLife={setAutonomousLife}
                    readOnly={trial.status === 'completed' || _userRole === 'observer'}
                  />
                </div>
              }
              center={
                <div id="tour-wizard-timeline" className="h-full">
                  <WizardExecutionPanel
                    trial={trial}
                    currentStep={currentStep}
                    steps={steps}
                    currentStepIndex={currentStepIndex}
                    trialEvents={trialEvents}
                    onStepSelect={(index: number) => setCurrentStepIndex(index)}
                    onExecuteAction={handleExecuteAction}
                    onExecuteRobotAction={handleExecuteRobotAction}
                    activeTab={executionPanelTab}
                    onTabChange={setExecutionPanelTab}
                    onSkipAction={handleSkipAction}
                    isExecuting={isExecutingAction}
                    onNextStep={handleNextStep}
                    completedActionsCount={completedActionsCount}
                    onActionCompleted={() => setCompletedActionsCount(c => c + 1)}
                    onCompleteTrial={handleCompleteTrial}
                    readOnly={trial.status === 'completed' || _userRole === 'observer'}
                  />
                </div>
              }
              right={
                <div id="tour-wizard-robot-status" className="h-full">
                  <WizardMonitoringPanel
                    rosConnected={rosConnected}
                    rosConnecting={rosConnecting}
                    rosError={rosError ?? undefined}
                    robotStatus={robotStatus}
                    connectRos={connectRos}
                    disconnectRos={disconnectRos}
                    executeRosAction={executeRosAction}
                    readOnly={trial.status === 'completed' || _userRole === 'observer'}
                  />
                </div>
              }
              showDividers={true}
              className="h-full"
            />
          </ResizablePanel>

          <ResizableHandle />

          <ResizablePanel defaultSize={25} minSize={10}>
            <WizardObservationPane
              onAddAnnotation={handleAddAnnotation}
              isSubmitting={addAnnotationMutation.isPending}
              trialEvents={trialEvents}
              // Observation pane is where observers usually work, so not readOnly for them?
              // But maybe we want 'readOnly' for completed trials.
              readOnly={trial.status === 'completed'}
            />
          </ResizablePanel>
        </ResizablePanelGroup>
      </div>
    </div>
  );
});

export default WizardInterface;
