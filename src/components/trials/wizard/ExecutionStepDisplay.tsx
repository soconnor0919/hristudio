"use client";

import { CheckCircle, Clock, PlayCircle, AlertCircle, Eye } from "lucide-react";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Progress } from "~/components/ui/progress";
import { Alert, AlertDescription } from "~/components/ui/alert";

interface ActionDefinition {
  id: string;
  stepId: string;
  name: string;
  description?: string;
  type: string;
  orderIndex: number;
  parameters: Record<string, unknown>;
  timeout?: number;
  required: boolean;
  condition?: string;
}

interface StepDefinition {
  id: string;
  name: string;
  description?: string;
  type: string;
  orderIndex: number;
  condition?: string;
  actions: ActionDefinition[];
}

interface ExecutionContext {
  trialId: string;
  experimentId: string;
  participantId: string;
  wizardId?: string;
  currentStepIndex: number;
  startTime: Date;
  variables: Record<string, unknown>;
}

interface ExecutionStepDisplayProps {
  currentStep: StepDefinition | null;
  executionContext: ExecutionContext | null;
  totalSteps: number;
  onExecuteStep: () => void;
  onAdvanceStep: () => void;
  onCompleteWizardAction: (
    actionId: string,
    data?: Record<string, unknown>,
  ) => void;
  isExecuting: boolean;
}

export function ExecutionStepDisplay({
  currentStep,
  executionContext,
  totalSteps,
  onExecuteStep,
  onAdvanceStep,
  onCompleteWizardAction,
  isExecuting,
}: ExecutionStepDisplayProps) {
  if (!currentStep || !executionContext) {
    return (
      <Card className="shadow-sm">
        <CardContent className="p-6 text-center">
          <div className="text-muted-foreground">
            <Clock className="mx-auto mb-2 h-8 w-8 opacity-50" />
            <p className="text-sm">No active step</p>
            <p className="mt-1 text-xs">
              Trial may not be started or all steps completed
            </p>
          </div>
        </CardContent>
      </Card>
    );
  }

  const progress =
    totalSteps > 0
      ? ((executionContext.currentStepIndex + 1) / totalSteps) * 100
      : 0;

  const getActionConfig = (
    type: string,
  ): { icon: typeof PlayCircle; label: string } => {
    const configs: Record<string, { icon: typeof PlayCircle; label: string }> =
      {
        wizard_say: {
          icon: PlayCircle,
          label: "Wizard Speech",
        },
        wizard_gesture: {
          icon: PlayCircle,
          label: "Wizard Gesture",
        },
        wizard_show_object: {
          icon: Eye,
          label: "Show Object",
        },
        observe_behavior: {
          icon: Eye,
          label: "Observe Behavior",
        },
        wait: { icon: Clock, label: "Wait" },
      };

    return (
      configs[type] ?? {
        icon: PlayCircle,
        label: "Action",
      }
    );
  };

  const getWizardInstructions = (action: ActionDefinition): string => {
    switch (action.type) {
      case "wizard_say":
        return `Say: "${String(action.parameters.text) ?? "Please speak to the participant"}";`;
      case "wizard_gesture":
        return `Perform gesture: ${String(action.parameters.gesture) ?? "as specified in the protocol"}`;
      case "wizard_show_object":
        return `Show object: ${String(action.parameters.object) ?? "as specified in the protocol"}`;
      case "observe_behavior":
        return `Observe and record: ${String(action.parameters.behavior) ?? "participant behavior"}`;
      case "wait":
        return `Wait for ${String(action.parameters.duration) ?? "1000"}ms`;
      default:
        return `Execute: ${action.name ?? "Unknown Action"}`;
    }
  };

  const requiresWizardInput = (action: ActionDefinition): boolean => {
    return [
      "wizard_say",
      "wizard_gesture",
      "wizard_show_object",
      "observe_behavior",
    ].includes(action.type);
  };

  return (
    <div className="space-y-4">
      {/* Step Progress */}
      <Card className="shadow-sm">
        <CardHeader className="pb-3">
          <div className="flex items-center justify-between">
            <CardTitle className="text-lg font-semibold">
              Step {executionContext.currentStepIndex + 1} of {totalSteps}
            </CardTitle>
            <Badge variant="outline" className="text-xs">
              {Math.round(progress)}% Complete
            </Badge>
          </div>
          <Progress value={progress} className="h-2" />
        </CardHeader>
        <CardContent className="pt-0">
          <div className="space-y-2">
            <h3 className="font-medium">{currentStep.name}</h3>
            {currentStep.description && (
              <p className="text-muted-foreground text-sm">
                {currentStep.description}
              </p>
            )}
            <div className="flex items-center space-x-2">
              <Badge variant="secondary" className="text-xs">
                {currentStep.type
                  .replace(/_/g, " ")
                  .replace(/\b\w/g, (l) => l.toUpperCase())}
              </Badge>
              <span className="text-muted-foreground text-xs">
                {currentStep.actions.length} action
                {currentStep.actions.length !== 1 ? "s" : ""}
              </span>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Step Actions */}
      <Card className="shadow-sm">
        <CardHeader className="pb-3">
          <div className="flex items-center justify-between">
            <CardTitle className="text-base font-medium">
              Step Actions
            </CardTitle>
            <Button
              onClick={onExecuteStep}
              disabled={isExecuting}
              size="sm"
              className="h-8"
            >
              <PlayCircle className="mr-1 h-3 w-3" />
              {isExecuting ? "Executing..." : "Execute Step"}
            </Button>
          </div>
        </CardHeader>
        <CardContent className="pt-0">
          <div className="space-y-3">
            {currentStep.actions?.map((action, _index) => {
              const config = getActionConfig(action.type);
              const Icon = config.icon;
              const needsWizardInput = requiresWizardInput(action);

              return (
                <div key={action.id} className="rounded-lg border p-3">
                  <div className="flex items-start justify-between">
                    <div className="flex-1 space-y-2">
                      <div className="flex items-center space-x-2">
                        <Icon className="h-4 w-4" />
                        <span className="text-sm font-medium">
                          {action.name}
                        </span>
                        <Badge variant="outline" className="text-xs">
                          {config.label}
                        </Badge>
                        {action.required && (
                          <Badge variant="destructive" className="text-xs">
                            Required
                          </Badge>
                        )}
                      </div>

                      {action.description && (
                        <p className="text-muted-foreground ml-6 text-xs">
                          {action.description}
                        </p>
                      )}

                      {needsWizardInput && (
                        <Alert className="mt-2 ml-6">
                          <AlertCircle className="h-4 w-4" />
                          <AlertDescription className="text-xs">
                            {getWizardInstructions(action)}
                          </AlertDescription>
                        </Alert>
                      )}

                      {/* Action Parameters */}
                      {Object.keys(action.parameters).length > 0 && (
                        <div className="mt-2 ml-6">
                          <details className="text-xs">
                            <summary className="text-muted-foreground cursor-pointer">
                              Parameters (
                              {Object.keys(action.parameters).length})
                            </summary>
                            <div className="mt-1 space-y-1">
                              {Object.entries(action.parameters).map(
                                ([key, value]) => (
                                  <div
                                    key={key}
                                    className="flex justify-between text-xs"
                                  >
                                    <span className="text-muted-foreground font-mono">
                                      {key}:
                                    </span>
                                    <span className="font-mono">
                                      {typeof value === "string"
                                        ? `"${value}"`
                                        : String(value)}
                                    </span>
                                  </div>
                                ),
                              )}
                            </div>
                          </details>
                        </div>
                      )}
                    </div>

                    {needsWizardInput && (
                      <Button
                        onClick={() => onCompleteWizardAction(action.id, {})}
                        size="sm"
                        variant="outline"
                        className="h-7 text-xs"
                      >
                        <CheckCircle className="mr-1 h-3 w-3" />
                        Complete
                      </Button>
                    )}
                  </div>
                </div>
              );
            })}
          </div>

          {/* Step Controls */}
          <div className="mt-4 flex justify-end space-x-2">
            <Button
              onClick={onAdvanceStep}
              variant="outline"
              size="sm"
              disabled={isExecuting}
            >
              Next Step
            </Button>
          </div>
        </CardContent>
      </Card>

      {/* Execution Variables (if any) */}
      {Object.keys(executionContext.variables).length > 0 && (
        <Card className="shadow-sm">
          <CardHeader className="pb-3">
            <CardTitle className="text-base font-medium">
              Execution Variables
            </CardTitle>
          </CardHeader>
          <CardContent className="pt-0">
            <div className="space-y-1">
              {Object.entries(executionContext.variables).map(
                ([key, value]) => (
                  <div key={key} className="flex justify-between text-xs">
                    <span className="font-mono text-slate-600">{key}:</span>
                    <span className="font-mono text-slate-900">
                      {typeof value === "string" ? `"${value}"` : String(value)}
                    </span>
                  </div>
                ),
              )}
            </div>
          </CardContent>
        </Card>
      )}
    </div>
  );
}
