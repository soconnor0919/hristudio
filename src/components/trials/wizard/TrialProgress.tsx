"use client";

import {
  Activity,
  Bot,
  CheckCircle,
  Circle,
  Clock,
  GitBranch,
  Play,
  Target,
  Users,
  SkipForward
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Progress } from "~/components/ui/progress";
import { Separator } from "~/components/ui/separator";

interface TrialProgressProps {
  steps: Array<{
    id: string;
    name: string;
    type:
    | "wizard_action"
    | "robot_action"
    | "parallel_steps"
    | "conditional_branch";
    description?: string;
    duration?: number;
    parameters?: Record<string, unknown>;
  }>;
  currentStepIndex: number;
  completedSteps: Set<number>;
  skippedSteps: Set<number>;
  trialStatus: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
}

const stepTypeConfig = {
  wizard_action: {
    label: "Wizard",
    icon: Play,
    color: "blue",
    bgColor: "bg-blue-100",
    textColor: "text-blue-600",
    borderColor: "border-blue-300",
  },
  robot_action: {
    label: "Robot",
    icon: Bot,
    color: "green",
    bgColor: "bg-green-100",
    textColor: "text-green-600",
    borderColor: "border-green-300",
  },
  parallel_steps: {
    label: "Parallel",
    icon: Users,
    color: "purple",
    bgColor: "bg-purple-100",
    textColor: "text-purple-600",
    borderColor: "border-purple-300",
  },
  conditional_branch: {
    label: "Branch",
    icon: GitBranch,
    color: "orange",
    bgColor: "bg-orange-100",
    textColor: "text-orange-600",
    borderColor: "border-orange-300",
  },
};

export function TrialProgress({
  steps,
  currentStepIndex,
  completedSteps,
  skippedSteps,
  trialStatus,
}: TrialProgressProps) {
  if (!steps || steps.length === 0) {
    return (
      <Card>
        <CardContent className="p-6 text-center">
          <div className="text-slate-500">
            <Target className="mx-auto mb-2 h-8 w-8 opacity-50" />
            <p className="text-sm">No experiment steps defined</p>
          </div>
        </CardContent>
      </Card>
    );
  }

  const progress =
    trialStatus === "completed"
      ? 100
      : trialStatus === "aborted"
        ? 0
        : ((currentStepIndex + 1) / steps.length) * 100;

  const completedCount =
    trialStatus === "completed"
      ? steps.length
      : trialStatus === "aborted" || trialStatus === "failed"
        ? 0
        : currentStepIndex;

  const getStepStatus = (index: number) => {
    if (trialStatus === "aborted" || trialStatus === "failed") return "aborted";
    if (trialStatus === "completed") return "completed";

    if (skippedSteps.has(index)) return "skipped";
    if (completedSteps.has(index)) return "completed";

    if (index === currentStepIndex && trialStatus === "in_progress")
      return "active";
    if (index === currentStepIndex && trialStatus === "scheduled")
      return "pending";

    // Default fallback if jumping around without explicitly adding to sets
    if (index < currentStepIndex && !skippedSteps.has(index)) return "completed";

    return "upcoming";
  };

  const getStepStatusConfig = (status: string) => {
    switch (status) {
      case "completed":
        return {
          icon: CheckCircle,
          iconColor: "text-green-600",
          bgColor: "bg-green-100",
          borderColor: "border-green-300",
          textColor: "text-green-800",
        };
      case "active":
        return {
          icon: Activity,
          iconColor: "text-blue-600",
          bgColor: "bg-blue-100",
          borderColor: "border-blue-300",
          textColor: "text-blue-800",
        };
      case "pending":
        return {
          icon: Clock,
          iconColor: "text-amber-600",
          bgColor: "bg-amber-100",
          borderColor: "border-amber-300",
          textColor: "text-amber-800",
        };
      case "aborted":
        return {
          icon: Circle,
          iconColor: "text-red-600",
          bgColor: "bg-red-100",
          borderColor: "border-red-300",
          textColor: "text-red-800",
        };
      case "skipped":
        return {
          icon: Circle,
          iconColor: "text-slate-400 opacity-50",
          bgColor: "bg-slate-50 opacity-50",
          borderColor: "border-slate-200 border-dashed",
          textColor: "text-slate-500",
        };
      default: // upcoming
        return {
          icon: Circle,
          iconColor: "text-slate-400",
          bgColor: "bg-slate-100",
          borderColor: "border-slate-300",
          textColor: "text-slate-600",
        };
    }
  };

  const totalDuration = steps.reduce(
    (sum, step) => sum + (step.duration ?? 0),
    0,
  );

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle className="flex items-center space-x-2">
            <Target className="h-5 w-5" />
            <span>Trial Progress</span>
          </CardTitle>
          <div className="flex items-center space-x-2">
            <Badge variant="outline" className="text-xs">
              {completedCount}/{steps.length} steps
            </Badge>
            {totalDuration > 0 && (
              <Badge variant="outline" className="text-xs">
                ~{Math.round(totalDuration / 60)}min
              </Badge>
            )}
          </div>
        </div>
      </CardHeader>

      <CardContent className="space-y-6">
        {/* Overall Progress Bar */}
        <div className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-slate-600">Overall Progress</span>
            <span className="font-medium">{Math.round(progress)}%</span>
          </div>
          <Progress
            value={progress}
            className={`h-2 ${trialStatus === "completed"
              ? "bg-green-100"
              : trialStatus === "aborted" || trialStatus === "failed"
                ? "bg-red-100"
                : "bg-blue-100"
              }`}
          />
          <div className="flex justify-between text-xs text-slate-500">
            <span>Start</span>
            <span>
              {trialStatus === "completed"
                ? "Completed"
                : trialStatus === "aborted"
                  ? "Aborted"
                  : trialStatus === "failed"
                    ? "Failed"
                    : trialStatus === "in_progress"
                      ? "In Progress"
                      : "Not Started"}
            </span>
          </div>
        </div>

        <Separator />

        {/* Steps Timeline */}
        <div className="space-y-4">
          <h4 className="text-sm font-medium text-slate-900">
            Experiment Steps
          </h4>

          <div className="space-y-3">
            {steps.map((step, index) => {
              const stepConfig = stepTypeConfig[step.type];
              const StepIcon = stepConfig.icon;
              const status = getStepStatus(index);
              const statusConfig = getStepStatusConfig(status);
              const StatusIcon = statusConfig.icon;

              return (
                <div key={step.id} className="relative">
                  {/* Connection Line */}
                  {index < steps.length - 1 && (
                    <div
                      className={`absolute top-12 left-6 h-6 w-0.5 ${getStepStatus(index + 1) === "completed" ||
                        (getStepStatus(index + 1) === "active" &&
                          status === "completed")
                        ? "bg-green-300"
                        : "bg-slate-300"
                        }`}
                    />
                  )}

                  {/* Step Card */}
                  <div
                    className={`flex items-start space-x-3 rounded-lg border p-3 transition-all ${status === "active"
                      ? `${statusConfig.bgColor} ${statusConfig.borderColor} shadow-md ring-2 ring-blue-200`
                      : status === "completed"
                        ? `${statusConfig.bgColor} ${statusConfig.borderColor}`
                        : status === "aborted"
                          ? `${statusConfig.bgColor} ${statusConfig.borderColor}`
                          : "border-slate-200 bg-slate-50"
                      }`}
                  >
                    {/* Step Number & Status */}
                    <div className="flex-shrink-0 space-y-1">
                      <div
                        className={`flex h-8 w-12 items-center justify-center rounded-lg ${status === "active"
                          ? statusConfig.bgColor
                          : status === "completed"
                            ? "bg-green-100"
                            : status === "aborted"
                              ? "bg-red-100"
                              : "bg-slate-100"
                          }`}
                      >
                        <span
                          className={`text-sm font-medium ${status === "active"
                            ? statusConfig.textColor
                            : status === "completed"
                              ? "text-green-700"
                              : status === "aborted"
                                ? "text-red-700"
                                : "text-slate-600"
                            }`}
                        >
                          {index + 1}
                        </span>
                      </div>
                      <div className="flex justify-center">
                        <StatusIcon
                          className={`h-4 w-4 ${statusConfig.iconColor}`}
                        />
                      </div>
                    </div>

                    {/* Step Content */}
                    <div className="min-w-0 flex-1">
                      <div className="flex items-start justify-between">
                        <div className="min-w-0 flex-1">
                          <h5
                            className={`truncate font-medium ${status === "active"
                              ? "text-slate-900"
                              : status === "completed"
                                ? "text-green-900"
                                : status === "aborted"
                                  ? "text-red-900"
                                  : "text-slate-700"
                              }`}
                          >
                            {step.name}
                          </h5>
                          {step.description && (
                            <p className="mt-1 line-clamp-2 text-sm text-slate-600">
                              {step.description}
                            </p>
                          )}
                        </div>

                        <div className="ml-3 flex-shrink-0 space-y-1">
                          <Badge
                            variant="outline"
                            className={`text-xs ${stepConfig.textColor} ${stepConfig.borderColor}`}
                          >
                            <StepIcon className="mr-1 h-3 w-3" />
                            {stepConfig.label}
                          </Badge>
                          {step.duration && (
                            <div className="flex items-center space-x-1 text-xs text-slate-500">
                              <Clock className="h-3 w-3" />
                              <span>{step.duration}s</span>
                            </div>
                          )}
                        </div>
                      </div>

                      {/* Step Status Message */}
                      {status === "active" && trialStatus === "in_progress" && (
                        <div className="mt-2 flex items-center space-x-1 text-sm text-blue-600">
                          <Activity className="h-3 w-3 animate-pulse" />
                          <span>Currently executing...</span>
                        </div>
                      )}
                      {status === "active" && trialStatus === "scheduled" && (
                        <div className="mt-2 flex items-center space-x-1 text-sm text-amber-600">
                          <Clock className="h-3 w-3" />
                          <span>Ready to start</span>
                        </div>
                      )}
                      {status === "completed" && (
                        <div className="mt-2 flex items-center space-x-1 text-sm text-green-600">
                          <CheckCircle className="h-3 w-3" />
                          <span>Completed</span>
                        </div>
                      )}
                      {status === "skipped" && (
                        <div className="mt-2 flex items-center space-x-1 text-sm text-slate-500 opacity-80">
                          <SkipForward className="h-3 w-3" />
                          <span>Skipped</span>
                        </div>
                      )}
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Summary Stats */}
        <Separator />
        <div className="grid grid-cols-3 gap-4 text-center">
          <div>
            <div className="text-2xl font-bold text-green-600">
              {completedCount}
            </div>
            <div className="text-xs text-slate-600">Completed</div>
          </div>
          <div>
            <div className="text-2xl font-bold text-blue-600">
              {trialStatus === "in_progress" ? 1 : 0}
            </div>
            <div className="text-xs text-slate-600">Active</div>
          </div>
          <div>
            <div className="text-2xl font-bold text-slate-600">
              {steps.length -
                completedCount -
                (trialStatus === "in_progress" ? 1 : 0)}
            </div>
            <div className="text-xs text-slate-600">Remaining</div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
