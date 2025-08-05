"use client";

import {
    Activity, ArrowRight, Bot, CheckCircle, GitBranch, MessageSquare, Play, Settings, Timer,
    User, Users
} from "lucide-react";
import { useState } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Progress } from "~/components/ui/progress";
import { Separator } from "~/components/ui/separator";

interface StepDisplayProps {
  step: {
    id: string;
    name: string;
    type: "wizard_action" | "robot_action" | "parallel_steps" | "conditional_branch";
    description?: string;
    parameters?: any;
    duration?: number;
    actions?: any[];
    conditions?: any;
    branches?: any[];
    substeps?: any[];
  };
  stepIndex: number;
  totalSteps: number;
  isActive: boolean;
  onExecuteAction: (actionType: string, actionData: any) => Promise<void>;
}

const stepTypeConfig = {
  wizard_action: {
    label: "Wizard Action",
    icon: User,
    color: "blue",
    description: "Action to be performed by the wizard operator",
  },
  robot_action: {
    label: "Robot Action",
    icon: Bot,
    color: "green",
    description: "Automated action performed by the robot",
  },
  parallel_steps: {
    label: "Parallel Steps",
    icon: Users,
    color: "purple",
    description: "Multiple actions happening simultaneously",
  },
  conditional_branch: {
    label: "Conditional Branch",
    icon: GitBranch,
    color: "orange",
    description: "Step with conditional logic and branching",
  },
};

export function StepDisplay({
  step,
  stepIndex,
  totalSteps,
  isActive,
  onExecuteAction
}: StepDisplayProps) {
  const [isExecuting, setIsExecuting] = useState(false);
  const [completedActions, setCompletedActions] = useState<Set<string>>(new Set());

  const stepConfig = stepTypeConfig[step.type];
  const StepIcon = stepConfig.icon;

  const handleActionExecution = async (actionId: string, actionData: any) => {
    setIsExecuting(true);
    try {
      await onExecuteAction(actionId, actionData);
      setCompletedActions(prev => new Set([...prev, actionId]));
    } catch (_error) {
      console.error("Failed to execute action:", _error);
    } finally {
      setIsExecuting(false);
    }
  };

  const renderStepContent = () => {
    switch (step.type) {
      case "wizard_action":
        return (
          <div className="space-y-4">
            {step.description && (
              <Alert>
                <MessageSquare className="h-4 w-4" />
                <AlertDescription>{step.description}</AlertDescription>
              </Alert>
            )}

            {step.actions && step.actions.length > 0 && (
              <div className="space-y-3">
                <h4 className="font-medium text-slate-900">Available Actions:</h4>
                <div className="grid gap-2">
                  {step.actions.map((action: any, index: number) => {
                    const isCompleted = completedActions.has(action.id);
                    return (
                      <div
                        key={action.id || index}
                        className={`flex items-center justify-between p-3 rounded-lg border ${
                          isCompleted
                            ? "bg-green-50 border-green-200"
                            : "bg-slate-50 border-slate-200"
                        }`}
                      >
                        <div className="flex items-center space-x-3">
                          {isCompleted ? (
                            <CheckCircle className="h-4 w-4 text-green-600" />
                          ) : (
                            <Play className="h-4 w-4 text-slate-400" />
                          )}
                          <div>
                            <p className="font-medium text-sm">{action.name}</p>
                            {action.description && (
                              <p className="text-xs text-slate-600">{action.description}</p>
                            )}
                          </div>
                        </div>
                        {isActive && !isCompleted && (
                          <Button
                            size="sm"
                            onClick={() => handleActionExecution(action.id, action)}
                            disabled={isExecuting}
                          >
                            Execute
                          </Button>
                        )}
                      </div>
                    );
                  })}
                </div>
              </div>
            )}
          </div>
        );

      case "robot_action":
        return (
          <div className="space-y-4">
            {step.description && (
              <Alert>
                <Bot className="h-4 w-4" />
                <AlertDescription>{step.description}</AlertDescription>
              </Alert>
            )}

            {step.parameters && (
              <div className="space-y-2">
                <h4 className="font-medium text-slate-900">Robot Parameters:</h4>
                <div className="bg-slate-50 rounded-lg p-3 text-sm font-mono">
                  <pre>{JSON.stringify(step.parameters, null, 2)}</pre>
                </div>
              </div>
            )}

            {isActive && (
              <div className="flex items-center space-x-2 text-sm text-slate-600">
                <Activity className="h-4 w-4 animate-pulse" />
                <span>Robot executing action...</span>
              </div>
            )}
          </div>
        );

      case "parallel_steps":
        return (
          <div className="space-y-4">
            {step.description && (
              <Alert>
                <Users className="h-4 w-4" />
                <AlertDescription>{step.description}</AlertDescription>
              </Alert>
            )}

            {step.substeps && step.substeps.length > 0 && (
              <div className="space-y-3">
                <h4 className="font-medium text-slate-900">Parallel Actions:</h4>
                <div className="grid gap-3">
                  {step.substeps.map((substep: any, index: number) => (
                    <div
                      key={substep.id || index}
                      className="flex items-center space-x-3 p-3 bg-slate-50 rounded-lg border"
                    >
                      <div className="flex-shrink-0">
                        <div className="w-6 h-6 rounded-full bg-purple-100 flex items-center justify-center text-xs font-medium text-purple-600">
                          {index + 1}
                        </div>
                      </div>
                      <div className="flex-1">
                        <p className="font-medium text-sm">{substep.name}</p>
                        {substep.description && (
                          <p className="text-xs text-slate-600">{substep.description}</p>
                        )}
                      </div>
                      <div className="flex-shrink-0">
                        <Badge variant="outline" className="text-xs">
                          {substep.type}
                        </Badge>
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </div>
        );

      case "conditional_branch":
        return (
          <div className="space-y-4">
            {step.description && (
              <Alert>
                <GitBranch className="h-4 w-4" />
                <AlertDescription>{step.description}</AlertDescription>
              </Alert>
            )}

            {step.conditions && (
              <div className="space-y-2">
                <h4 className="font-medium text-slate-900">Conditions:</h4>
                <div className="bg-slate-50 rounded-lg p-3 text-sm">
                  <pre>{JSON.stringify(step.conditions, null, 2)}</pre>
                </div>
              </div>
            )}

            {step.branches && step.branches.length > 0 && (
              <div className="space-y-3">
                <h4 className="font-medium text-slate-900">Possible Branches:</h4>
                <div className="grid gap-2">
                  {step.branches.map((branch: any, index: number) => (
                    <div
                      key={branch.id || index}
                      className="flex items-center justify-between p-3 bg-slate-50 rounded-lg border"
                    >
                      <div className="flex items-center space-x-3">
                        <ArrowRight className="h-4 w-4 text-orange-500" />
                        <div>
                          <p className="font-medium text-sm">{branch.name}</p>
                          {branch.condition && (
                            <p className="text-xs text-slate-600">If: {branch.condition}</p>
                          )}
                        </div>
                      </div>
                      {isActive && (
                        <Button
                          size="sm"
                          variant="outline"
                          onClick={() => handleActionExecution(`branch_${branch.id}`, branch)}
                          disabled={isExecuting}
                        >
                          Select
                        </Button>
                      )}
                    </div>
                  ))}
                </div>
              </div>
            )}
          </div>
        );

      default:
        return (
          <div className="text-center py-8 text-slate-500">
            <Settings className="h-8 w-8 mx-auto mb-2" />
            <p>Unknown step type: {step.type}</p>
          </div>
        );
    }
  };

  return (
    <Card className={`transition-all duration-200 ${
      isActive ? "ring-2 ring-blue-500 shadow-lg" : "border-slate-200"
    }`}>
      <CardHeader>
        <div className="flex items-start justify-between">
          <div className="flex items-start space-x-3">
            <div className={`flex-shrink-0 w-10 h-10 rounded-lg flex items-center justify-center ${
              stepConfig.color === "blue" ? "bg-blue-100" :
              stepConfig.color === "green" ? "bg-green-100" :
              stepConfig.color === "purple" ? "bg-purple-100" :
              stepConfig.color === "orange" ? "bg-orange-100" :
              "bg-slate-100"
            }`}>
              <StepIcon className={`h-5 w-5 ${
                stepConfig.color === "blue" ? "text-blue-600" :
                stepConfig.color === "green" ? "text-green-600" :
                stepConfig.color === "purple" ? "text-purple-600" :
                stepConfig.color === "orange" ? "text-orange-600" :
                "text-slate-600"
              }`} />
            </div>
            <div className="flex-1 min-w-0">
              <CardTitle className="text-lg font-semibold text-slate-900">
                {step.name}
              </CardTitle>
              <div className="flex items-center space-x-2 mt-1">
                <Badge variant="outline" className="text-xs">
                  {stepConfig.label}
                </Badge>
                <span className="text-xs text-slate-500">
                  Step {stepIndex + 1} of {totalSteps}
                </span>
              </div>
              <p className="text-sm text-slate-600 mt-1">
                {stepConfig.description}
              </p>
            </div>
          </div>

          <div className="flex flex-col items-end space-y-2">
            {isActive && (
              <Badge className="bg-green-100 text-green-800">
                <Activity className="mr-1 h-3 w-3 animate-pulse" />
                Active
              </Badge>
            )}
            {step.duration && (
              <div className="flex items-center space-x-1 text-xs text-slate-500">
                <Timer className="h-3 w-3" />
                <span>{step.duration}s</span>
              </div>
            )}
          </div>
        </div>
      </CardHeader>

      <CardContent>
        {renderStepContent()}

        {/* Step Progress Indicator */}
        <Separator className="my-4" />
        <div className="flex items-center justify-between text-xs text-slate-500">
          <span>Step Progress</span>
          <span>{stepIndex + 1}/{totalSteps}</span>
        </div>
        <Progress value={((stepIndex + 1) / totalSteps) * 100} className="h-1 mt-2" />
      </CardContent>
    </Card>
  );
}
