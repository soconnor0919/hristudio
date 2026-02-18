"use client";

import React, { useState, useEffect, useCallback, useMemo } from "react";
import {
  Play,
  CheckCircle,
  X,
  Clock,
  AlertCircle,
  PanelLeftClose,
  PanelLeftOpen,
  PanelRightClose,
  PanelRightOpen,
  ChevronDown,
  ChevronUp,
  Pause,
  SkipForward
} from "lucide-react";
import { useRouter } from "next/navigation";
import { cn } from "~/lib/utils";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { PageHeader } from "~/components/ui/page-header";
import Link from "next/link";
import { Progress } from "~/components/ui/progress";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { WizardControlPanel } from "./panels/WizardControlPanel";
import { WizardExecutionPanel } from "./panels/WizardExecutionPanel";
import { WizardMonitoringPanel } from "./panels/WizardMonitoringPanel";
import { WizardObservationPane } from "./panels/WizardObservationPane";
import { TrialStatusBar } from "./panels/TrialStatusBar";
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
  | "conditional";
  parameters: Record<string, unknown>;
  conditions?: {
    nextStepId?: string;
    options?: {
      label: string;
      value: string;
      nextStepId?: string;
      nextStepIndex?: number;
      variant?: "default" | "destructive" | "outline" | "secondary" | "ghost" | "link";
    }[];
  };
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

  // UI State
  const [executionPanelTab, setExecutionPanelTab] = useState<"current" | "timeline" | "events">("timeline");

  const [isExecutingAction, setIsExecutingAction] = useState(false);
  const [monitoringPanelTab, setMonitoringPanelTab] = useState<
    "status" | "robot" | "events"
  >("status");
  const [completedActionsCount, setCompletedActionsCount] = useState(0);

  // Collapse state for panels
  const [leftCollapsed, setLeftCollapsed] = useState(false);
  const [rightCollapsed, setRightCollapsed] = useState(false);
  const [obsCollapsed, setObsCollapsed] = useState(false);

  // Center tabs (Timeline | Actions)
  const [centerTab, setCenterTab] = useState<"timeline" | "actions">("timeline");

  // Reset completed actions when step changes
  useEffect(() => {
    setCompletedActionsCount(0);
  }, [currentStepIndex]);

  // Track the last response value from wizard_wait_for_response for branching
  const [lastResponse, setLastResponse] = useState<string | null>(null);

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
        return "conditional" as const;
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
    setAutonomousLife: setAutonomousLifeRaw,
  } = useWizardRos({
    autoConnect: true,
    onActionCompleted,
    onActionFailed,
  });

  // Wrap setAutonomousLife in a stable callback to prevent infinite re-renders
  // The raw function from useWizardRos is recreated when isConnected changes,
  // which would cause WizardControlPanel (wrapped in React.memo) to re-render infinitely
  const setAutonomousLife = useCallback(
    async (enabled: boolean) => {
      return setAutonomousLifeRaw(enabled);
    },
    [setAutonomousLifeRaw]
  );

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

        setTrial((prev) => {
          // Double check inside setter to be safe
          if (prev.status === pollingData.status &&
            prev.startedAt?.getTime() === pollingData.startedAt?.getTime() &&
            prev.completedAt?.getTime() === pollingData.completedAt?.getTime()) {
            return prev;
          }
          return {
            ...prev,
            status: pollingData.status,
            startedAt: pollingData.startedAt
              ? new Date(pollingData.startedAt)
              : prev.startedAt,
            completedAt: pollingData.completedAt
              ? new Date(pollingData.completedAt)
              : prev.completedAt,
          };
        });
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pollingData]);

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
  const steps: StepData[] = useMemo(() =>
    experimentSteps?.map((step, index) => ({
      id: step.id,
      name: step.name ?? `Step ${index + 1}`,
      description: step.description,
      type: mapStepType(step.type),
      // Fix: Conditions are at root level from API
      conditions: (step as any).conditions ?? (step as any).trigger?.conditions ?? undefined,
      parameters: step.parameters ?? {},
      order: step.order ?? index,
      actions: step.actions?.filter(a => a.type !== 'branch').map((action) => ({
        id: action.id,
        name: action.name,
        description: action.description,
        type: action.type,
        parameters: action.parameters ?? {},
        order: action.order,
        pluginId: action.pluginId,
      })) ?? [],
    })) ?? [], [experimentSteps]);


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

  const pauseTrialMutation = api.trials.pause.useMutation({
    onSuccess: () => {
      toast.success("Trial paused");
      // Optionally update local state if needed, though status might not change on backend strictly to "paused"
      // depending on enum. But we logged the event.
    },
    onError: (error) => {
      toast.error("Failed to pause trial", { description: error.message });
    },
  });

  const archiveTrialMutation = api.trials.archive.useMutation({
    onSuccess: () => {
      console.log("Trial archived successfully");
    },
    onError: (error) => {
      console.error("Failed to archive trial", error);
    },
  });

  const logEventMutation = api.trials.logEvent.useMutation({
    onSuccess: () => {
      // toast.success("Event logged"); // Too noisy
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
    try {
      await pauseTrialMutation.mutateAsync({ id: trial.id });
      logEventMutation.mutate({
        trialId: trial.id,
        type: "trial_paused",
        data: { timestamp: new Date() }
      });
    } catch (error) {
      console.error("Failed to pause trial:", error);
    }
  };

  const handleNextStep = (targetIndex?: number) => {
    // If explicit target provided (from branching choice), use it
    if (typeof targetIndex === 'number') {
      // Find step by index to ensure safety
      if (targetIndex >= 0 && targetIndex < steps.length) {
        console.log(`[WizardInterface] Manual jump to step ${targetIndex}`);

        // Log manual jump
        logEventMutation.mutate({
          trialId: trial.id,
          type: "step_jumped",
          data: {
            fromIndex: currentStepIndex,
            toIndex: targetIndex,
            fromStepId: steps[currentStepIndex]?.id,
            toStepId: steps[targetIndex]?.id,
            reason: "manual_choice"
          }
        });

        setCompletedActionsCount(0);
        setCurrentStepIndex(targetIndex);
        setLastResponse(null);
        return;
      }
    }

    // Dynamic Branching Logic
    const currentStep = steps[currentStepIndex];

    // Check if we have a stored response that dictates the next step
    if (currentStep?.type === 'conditional' && currentStep.conditions?.options && lastResponse) {
      const matchedOption = currentStep.conditions.options.find(opt => opt.value === lastResponse);
      if (matchedOption && matchedOption.nextStepId) {
        // Find index of the target step
        const targetIndex = steps.findIndex(s => s.id === matchedOption.nextStepId);
        if (targetIndex !== -1) {
          console.log(`[WizardInterface] Branching to step ${targetIndex} (${matchedOption.label})`);

          logEventMutation.mutate({
            trialId: trial.id,
            type: "step_branched",
            data: {
              fromIndex: currentStepIndex,
              toIndex: targetIndex,
              condition: matchedOption.label,
              value: lastResponse
            }
          });

          setCurrentStepIndex(targetIndex);
          setLastResponse(null); // Reset after consuming
          return;
        }
      }
    }

    // Check for explicit nextStepId in conditions (e.g. for end of branch)
    console.log("[WizardInterface] Checking for nextStepId condition:", currentStep?.conditions);
    if (currentStep?.conditions?.nextStepId) {
      const nextId = String(currentStep.conditions.nextStepId);
      const targetIndex = steps.findIndex(s => s.id === nextId);
      if (targetIndex !== -1) {
        console.log(`[WizardInterface] Condition-based jump to step ${targetIndex} (${nextId})`);

        logEventMutation.mutate({
          trialId: trial.id,
          type: "step_jumped",
          data: {
            fromIndex: currentStepIndex,
            toIndex: targetIndex,
            reason: "condition_next_step"
          }
        });

        setCurrentStepIndex(targetIndex);
        setCompletedActionsCount(0);
        return;
      } else {
        console.warn(`[WizardInterface] Targeted nextStepId ${nextId} not found in steps list.`);
      }
    } else {
      console.log("[WizardInterface] No nextStepId found in conditions, proceeding linearly.");
    }

    // Default: Linear progression
    const nextIndex = currentStepIndex + 1;
    if (nextIndex < steps.length) {
      // Log step change
      logEventMutation.mutate({
        trialId: trial.id,
        type: "step_changed",
        data: {
          fromIndex: currentStepIndex,
          toIndex: nextIndex,
          fromStepId: currentStep?.id,
          toStepId: steps[nextIndex]?.id,
          stepName: steps[nextIndex]?.name,
        }
      });

      setCurrentStepIndex(nextIndex);
    } else {
      handleCompleteTrial();
    }
  };

  const handleCompleteTrial = async () => {
    try {
      await completeTrialMutation.mutateAsync({ id: trial.id });



      // Trigger archive in background
      archiveTrialMutation.mutate({ id: trial.id });
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



  // Mutation for interventions
  const addInterventionMutation = api.trials.addIntervention.useMutation({
    onSuccess: () => toast.success("Intervention logged"),
  });

  const handleExecuteAction = async (
    actionId: string,
    parameters?: Record<string, unknown>,
  ) => {
    try {
      // Log action execution
      console.log("Executing action:", actionId, parameters);

      // Handle branching logic (wizard_wait_for_response)
      if (parameters?.value && parameters?.label) {
        setLastResponse(String(parameters.value));

        // If nextStepId is provided, jump immediately
        if (parameters.nextStepId) {
          const nextId = String(parameters.nextStepId);
          const targetIndex = steps.findIndex(s => s.id === nextId);
          if (targetIndex !== -1) {
            console.log(`[WizardInterface] Choice-based jump to step ${targetIndex} (${nextId})`);
            handleNextStep(targetIndex);
            return; // Exit after jump
          }
        }
      }

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
      } else {
        // Generic action logging
        await logEventMutation.mutateAsync({
          trialId: trial.id,
          type: "action_executed",
          data: {
            actionId,
            parameters
          }
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
        // If it's a robot action (indicated by pluginName), use the robot logger
        if (pluginName) {
          await logRobotActionMutation.mutateAsync({
            trialId: trial.id,
            pluginName,
            actionId,
            parameters,
            duration: 0,
            result: { skipped: true },
          });
        } else {
          // Generic skip logging
          await logEventMutation.mutateAsync({
            trialId: trial.id,
            type: "action_skipped",
            data: {
              actionId,
              parameters
            }
          });
        }

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
    <div className="flex h-[calc(100vh-5rem)] w-full flex-col overflow-hidden bg-background">
      <PageHeader
        title="Trial Execution"
        description={`Session ${trial.sessionNumber} • Participant ${trial.participant.participantCode}`}
        icon={Play}
        actions={
          <div className="flex items-center gap-2">
            {trial.status === "scheduled" && (
              <Button
                onClick={handleStartTrial}
                size="sm"
                className="gap-2"
              >
                <Play className="h-4 w-4" />
                Start Trial
              </Button>
            )}

            {trial.status === "in_progress" && (
              <>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={handlePauseTrial}
                  className="gap-2"
                >
                  <Pause className="h-4 w-4" />
                  Pause
                </Button>

                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleNextStep()}
                  className="gap-2"
                >
                  <SkipForward className="h-4 w-4" />
                  Next Step
                </Button>

                <Button
                  variant="destructive"
                  size="sm"
                  onClick={handleAbortTrial}
                  className="gap-2"
                >
                  <X className="h-4 w-4" />
                  Abort
                </Button>

                <Button
                  variant="default"
                  size="sm"
                  onClick={handleCompleteTrial}
                  className="gap-2 bg-green-600 hover:bg-green-700"
                >
                  <CheckCircle className="h-4 w-4" />
                  Complete
                </Button>
              </>
            )}

            {_userRole !== "participant" && (
              <Button asChild variant="ghost" size="sm">
                <Link href={`/studies/${trial.experiment.studyId}/trials`}>
                  Exit
                </Link>
              </Button>
            )}
          </div>
        }
        className="flex-none px-2 pb-2"
      />

      {/* Main Grid - 2 rows */}
      <div className="flex-1 min-h-0 flex flex-col gap-2 px-2 pb-2">
        {/* Top Row - 3 Column Layout */}
        <div className="flex-1 min-h-0 flex gap-2">
          {/* Left Sidebar - Control Panel (Collapsible) */}
          {!leftCollapsed && (
            <div className="flex flex-col overflow-hidden rounded-lg border bg-background shadow-sm w-80">
              <div className="flex items-center justify-between border-b px-3 py-2 bg-muted/30">
                <span className="text-sm font-medium">Control</span>
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-6 w-6"
                  onClick={() => setLeftCollapsed(true)}
                >
                  <PanelLeftClose className="h-4 w-4" />
                </Button>
              </div>
              <div className="flex-1 overflow-hidden min-h-0 bg-muted/10">
                <div id="tour-wizard-controls-wrapper" className="h-full">
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
                    isStarting={startTrialMutation.isPending}
                    readOnly={trial.status === 'completed' || _userRole === 'observer'}
                  />
                </div>
              </div>
            </div>
          )}

          {/* Center - Tabbed Workspace */}
          {/* Center - Execution Workspace */}
          <div className="flex-1 flex flex-col overflow-hidden rounded-lg border bg-background shadow-sm">
            <div className="flex items-center border-b px-3 py-2 bg-muted/30 min-h-[45px]">
              {leftCollapsed && (
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-6 w-6 mr-2"
                  onClick={() => setLeftCollapsed(false)}
                  title="Open Tools Panel"
                >
                  <PanelLeftOpen className="h-4 w-4" />
                </Button>
              )}

              <div className="flex items-center gap-2">
                <span className="text-sm font-medium">Trial Execution</span>
                {currentStep && (
                  <Badge variant="outline" className="text-xs font-normal">
                    {currentStep.name}
                  </Badge>
                )}
              </div>

              <div className="flex-1" />

              <div className="mr-2 text-xs text-muted-foreground font-medium">
                Step {currentStepIndex + 1} / {steps.length}
              </div>

              {rightCollapsed && (
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-6 w-6"
                  onClick={() => setRightCollapsed(false)}
                  title="Open Robot Status"
                >
                  <PanelRightOpen className="h-4 w-4" />
                </Button>
              )}
            </div>
            <div className="flex-1 overflow-auto min-h-0 bg-muted/10">
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
                  rosConnected={rosConnected}
                />
              </div>
            </div>
          </div>

          {/* Right Sidebar - Robot Status (Collapsible) */}
          {!rightCollapsed && (
            <div className="flex flex-col overflow-hidden rounded-lg border bg-background shadow-sm w-80">
              <div className="flex items-center justify-between border-b px-3 py-2 bg-muted/30">
                <span className="text-sm font-medium">Robot Control & Status</span>
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-6 w-6"
                  onClick={() => setRightCollapsed(true)}
                >
                  <PanelRightClose className="h-4 w-4" />
                </Button>
              </div>
              <div className="flex-1 overflow-auto min-h-0 bg-muted/10">
                <div id="tour-wizard-robot-status" className="h-full">
                  <WizardMonitoringPanel
                    rosConnected={rosConnected}
                    rosConnecting={rosConnecting}
                    rosError={rosError ?? undefined}
                    robotStatus={robotStatus}
                    connectRos={connectRos}
                    disconnectRos={disconnectRos}
                    executeRosAction={executeRosAction}
                    onSetAutonomousLife={setAutonomousLife}
                    onExecuteRobotAction={handleExecuteRobotAction}
                    studyId={trial.experiment.studyId}
                    trialId={trial.id}
                    readOnly={trial.status === 'completed' || _userRole === 'observer'}
                  />
                </div>
              </div>
            </div>
          )}
        </div>

        {/* Bottom Row - Observations (Full Width, Collapsible) */}
        {!obsCollapsed && (
          <div className="flex flex-col overflow-hidden rounded-lg border bg-background shadow-sm h-48 flex-none">
            <div className="flex items-center border-b px-3 py-2 bg-muted/30 gap-3">
              <span className="text-sm font-medium">Observations</span>
              <div className="flex-1" />
              <Button
                variant="ghost"
                size="icon"
                className="h-6 w-6"
                onClick={() => setObsCollapsed(true)}
              >
                <ChevronDown className="h-4 w-4" />
              </Button>
            </div>
            <div className="flex-1 overflow-auto min-h-0 bg-muted/10">
              <WizardObservationPane
                onAddAnnotation={handleAddAnnotation}
                isSubmitting={addAnnotationMutation.isPending}
                trialEvents={trialEvents}
                readOnly={trial.status === 'completed'}
              />
            </div>
          </div>
        )}
        {
          obsCollapsed && (
            <Button
              variant="outline"
              size="sm"
              onClick={() => setObsCollapsed(false)}
              className="w-full flex-none"
            >
              <ChevronUp className="h-4 w-4 mr-2" />
              Show Observations
            </Button>
          )
        }
      </div >
    </div >
  );
});

export default WizardInterface;
