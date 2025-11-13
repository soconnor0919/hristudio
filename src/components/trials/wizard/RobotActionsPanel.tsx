"use client";

import React, { useState, useEffect } from "react";
import {
  Bot,
  Play,
  Settings,
  AlertCircle,
  CheckCircle,
  Loader2,
  Volume2,
  Move,
  Eye,
  Hand,
  Zap,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import { Slider } from "~/components/ui/slider";
import { Switch } from "~/components/ui/switch";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Separator } from "~/components/ui/separator";
import { Alert, AlertDescription } from "~/components/ui/alert";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  Collapsible,
  CollapsibleContent,
  CollapsibleTrigger,
} from "~/components/ui/collapsible";
import { api } from "~/trpc/react";
import { toast } from "sonner";

interface RobotAction {
  id: string;
  name: string;
  description: string;
  category: string;
  parameters?: Array<{
    name: string;
    type: "text" | "number" | "boolean" | "select";
    description: string;
    required: boolean;
    min?: number;
    max?: number;
    step?: number;
    default?: unknown;
    options?: Array<{ value: string; label: string }>;
    placeholder?: string;
    maxLength?: number;
  }>;
}

interface Plugin {
  plugin: {
    id: string;
    name: string;
    version: string;
    description: string;
    trustLevel: string;
    actionDefinitions: RobotAction[];
  };
  installation: {
    id: string;
    configuration: Record<string, unknown>;
    installedAt: Date;
  };
}

interface RobotActionsPanelProps {
  studyId: string;
  trialId: string;
  onExecuteAction: (
    pluginName: string,
    actionId: string,
    parameters: Record<string, unknown>,
  ) => Promise<void>;
}

export function RobotActionsPanel({
  studyId,
  trialId,
  onExecuteAction,
}: RobotActionsPanelProps) {
  const [selectedPlugin, setSelectedPlugin] = useState<string>("");
  const [selectedAction, setSelectedAction] = useState<RobotAction | null>(
    null,
  );
  const [actionParameters, setActionParameters] = useState<
    Record<string, unknown>
  >({});
  const [executingActions, setExecutingActions] = useState<Set<string>>(
    new Set(),
  );
  const [expandedCategories, setExpandedCategories] = useState<Set<string>>(
    new Set(["movement", "speech"]),
  );

  // Get installed plugins for the study
  const { data: plugins = [], isLoading } =
    api.robots.plugins.getStudyPlugins.useQuery({
      studyId,
    });

  // Get actions for selected plugin
  const selectedPluginData = plugins.find(
    (p) => p.plugin.id === selectedPlugin,
  );

  // Initialize parameters when action changes
  useEffect(() => {
    if (selectedAction) {
      const defaultParams: Record<string, unknown> = {};

      selectedAction.parameters?.forEach((param) => {
        if (param.default !== undefined) {
          defaultParams[param.name] = param.default;
        } else if (param.required) {
          // Set reasonable defaults for required params
          switch (param.type) {
            case "text":
              defaultParams[param.name] = "";
              break;
            case "number":
              defaultParams[param.name] = param.min ?? 0;
              break;
            case "boolean":
              defaultParams[param.name] = false;
              break;
            case "select":
              defaultParams[param.name] = param.options?.[0]?.value ?? "";
              break;
          }
        }
      });

      setActionParameters(defaultParams);
    } else {
      setActionParameters({});
    }
  }, [selectedAction]);

  const handleExecuteAction = async () => {
    if (!selectedAction || !selectedPluginData) return;

    const actionKey = `${selectedPluginData.plugin.name}.${selectedAction.id}`;
    setExecutingActions((prev) => new Set([...prev, actionKey]));

    try {
      await onExecuteAction(
        selectedPluginData.plugin.name,
        selectedAction.id,
        actionParameters,
      );

      toast.success(`Executed: ${selectedAction.name}`, {
        description: `Robot action completed successfully`,
      });
    } catch (error) {
      toast.error(`Failed to execute: ${selectedAction.name}`, {
        description: error instanceof Error ? error.message : "Unknown error",
      });
    } finally {
      setExecutingActions((prev) => {
        const next = new Set(prev);
        next.delete(actionKey);
        return next;
      });
    }
  };

  const handleParameterChange = (paramName: string, value: unknown) => {
    setActionParameters((prev) => ({
      ...prev,
      [paramName]: value,
    }));
  };

  const renderParameterInput = (
    param: NonNullable<RobotAction["parameters"]>[0],
    paramIndex: number,
  ) => {
    if (!param) return null;

    const value = actionParameters[param.name];

    switch (param.type) {
      case "text":
        return (
          <div key={param.name} className="space-y-2">
            <Label htmlFor={param.name}>
              {param.name} {param.required && "*"}
            </Label>
            {param.maxLength && param.maxLength > 100 ? (
              <Textarea
                id={param.name}
                value={(value as string) || ""}
                onChange={(e) =>
                  handleParameterChange(param.name, e.target.value)
                }
                placeholder={param.placeholder}
                maxLength={param.maxLength}
              />
            ) : (
              <Input
                id={param.name}
                value={(value as string) || ""}
                onChange={(e) =>
                  handleParameterChange(param.name, e.target.value)
                }
                placeholder={param.placeholder}
                maxLength={param.maxLength}
              />
            )}
            <p className="text-muted-foreground text-xs">{param.description}</p>
          </div>
        );

      case "number":
        return (
          <div key={param.name} className="space-y-2">
            <Label htmlFor={param.name}>
              {param.name} {param.required && "*"}
            </Label>
            {param.min !== undefined && param.max !== undefined ? (
              <div className="space-y-2">
                <Slider
                  value={[Number(value) || param.min]}
                  onValueChange={(newValue) =>
                    handleParameterChange(param.name, newValue[0])
                  }
                  min={param.min}
                  max={param.max}
                  step={param.step || 0.1}
                  className="w-full"
                />
                <div className="text-muted-foreground text-center text-sm">
                  {Number(value) || param.min}
                </div>
              </div>
            ) : (
              <Input
                id={param.name}
                type="number"
                value={Number(value) || ""}
                onChange={(e) =>
                  handleParameterChange(param.name, Number(e.target.value))
                }
                min={param.min}
                max={param.max}
                step={param.step}
              />
            )}
            <p className="text-muted-foreground text-xs">{param.description}</p>
          </div>
        );

      case "boolean":
        return (
          <div key={param.name} className="flex items-center space-x-2">
            <Switch
              id={param.name}
              checked={Boolean(value)}
              onCheckedChange={(checked) =>
                handleParameterChange(param.name, checked)
              }
            />
            <Label htmlFor={param.name}>
              {param.name} {param.required && "*"}
            </Label>
            <p className="text-muted-foreground ml-auto text-xs">
              {param.description}
            </p>
          </div>
        );

      case "select":
        return (
          <div key={param.name} className="space-y-2">
            <Label htmlFor={param.name}>
              {param.name} {param.required && "*"}
            </Label>
            <Select
              value={String(value) || ""}
              onValueChange={(newValue) =>
                handleParameterChange(param.name, newValue)
              }
            >
              <SelectTrigger>
                <SelectValue placeholder={`Select ${param.name}`} />
              </SelectTrigger>
              <SelectContent>
                {param.options?.map(
                  (option: { value: string; label: string }) => (
                    <SelectItem key={option.value} value={option.value}>
                      {option.label}
                    </SelectItem>
                  ),
                )}
              </SelectContent>
            </Select>
            <p className="text-muted-foreground text-xs">{param.description}</p>
          </div>
        );

      default:
        return null;
    }
  };

  const getCategoryIcon = (category: string) => {
    switch (category.toLowerCase()) {
      case "movement":
        return Move;
      case "speech":
        return Volume2;
      case "sensors":
        return Eye;
      case "interaction":
        return Hand;
      default:
        return Zap;
    }
  };

  const groupActionsByCategory = (actions: RobotAction[]) => {
    const grouped: Record<string, RobotAction[]> = {};

    actions.forEach((action) => {
      const category = action.category || "other";
      if (!grouped[category]) {
        grouped[category] = [];
      }
      grouped[category].push(action);
    });

    return grouped;
  };

  const toggleCategory = (category: string) => {
    setExpandedCategories((prev) => {
      const next = new Set(prev);
      if (next.has(category)) {
        next.delete(category);
      } else {
        next.add(category);
      }
      return next;
    });
  };

  if (isLoading) {
    return (
      <div className="flex items-center justify-center p-8">
        <Loader2 className="h-6 w-6 animate-spin" />
        <span className="ml-2">Loading robot plugins...</span>
      </div>
    );
  }

  if (plugins.length === 0) {
    return (
      <Alert>
        <AlertCircle className="h-4 w-4" />
        <AlertDescription>
          No robot plugins installed for this study. Install plugins from the
          study settings to enable robot control.
        </AlertDescription>
      </Alert>
    );
  }

  return (
    <div className="space-y-4">
      {/* Plugin Selection */}
      <div className="space-y-2">
        <Label>Select Robot Plugin</Label>
        <Select value={selectedPlugin} onValueChange={setSelectedPlugin}>
          <SelectTrigger>
            <SelectValue placeholder="Choose a robot plugin" />
          </SelectTrigger>
          <SelectContent>
            {plugins.map((plugin) => (
              <SelectItem key={plugin.plugin.id} value={plugin.plugin.id}>
                <div className="flex items-center space-x-2">
                  <Bot className="h-4 w-4" />
                  <span>
                    {plugin.plugin.name} v{plugin.plugin.version}
                  </span>
                  <Badge variant="outline" className="ml-auto">
                    {plugin.plugin.trustLevel}
                  </Badge>
                </div>
              </SelectItem>
            ))}
          </SelectContent>
        </Select>
      </div>

      {/* Action Selection */}
      {selectedPluginData && (
        <div className="space-y-2">
          <Label>Available Actions</Label>
          <ScrollArea className="h-64 rounded-md border">
            <div className="space-y-2 p-2">
              {Object.entries(
                groupActionsByCategory(
                  (selectedPluginData.plugin
                    .actionDefinitions as RobotAction[]) || [],
                ),
              ).map(([category, actions]) => {
                const CategoryIcon = getCategoryIcon(category);
                const isExpanded = expandedCategories.has(category);

                return (
                  <Collapsible
                    key={category}
                    open={isExpanded}
                    onOpenChange={() => toggleCategory(category)}
                  >
                    <CollapsibleTrigger asChild>
                      <Button
                        variant="ghost"
                        className="w-full justify-start p-2"
                      >
                        <CategoryIcon className="mr-2 h-4 w-4" />
                        {category.charAt(0).toUpperCase() + category.slice(1)}
                        <Badge variant="secondary" className="ml-auto">
                          {actions.length}
                        </Badge>
                      </Button>
                    </CollapsibleTrigger>
                    <CollapsibleContent className="ml-6 space-y-1">
                      {actions.map((action) => (
                        <Button
                          key={action.id}
                          variant={
                            selectedAction?.id === action.id
                              ? "default"
                              : "ghost"
                          }
                          className="w-full justify-start text-sm"
                          onClick={() => setSelectedAction(action)}
                        >
                          {action.name}
                        </Button>
                      ))}
                    </CollapsibleContent>
                  </Collapsible>
                );
              })}
            </div>
          </ScrollArea>
        </div>
      )}

      {/* Action Configuration */}
      {selectedAction && (
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center space-x-2">
              <Bot className="h-4 w-4" />
              <span>{selectedAction.name}</span>
            </CardTitle>
            <CardDescription>{selectedAction.description}</CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            {/* Parameters */}
            {selectedAction.parameters &&
            selectedAction.parameters.length > 0 ? (
              <div className="space-y-4">
                <Label className="text-base">Parameters</Label>
                {selectedAction.parameters.map((param, index) =>
                  renderParameterInput(param, index),
                )}
              </div>
            ) : (
              <p className="text-muted-foreground text-sm">
                This action requires no parameters.
              </p>
            )}

            <Separator />

            {/* Execute Button */}
            <Button
              onClick={handleExecuteAction}
              disabled={
                !selectedPluginData ||
                executingActions.has(
                  `${selectedPluginData.plugin.name}.${selectedAction.id}`,
                )
              }
              className="w-full"
            >
              {selectedPluginData &&
              executingActions.has(
                `${selectedPluginData.plugin.name}.${selectedAction.id}`,
              ) ? (
                <>
                  <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                  Executing...
                </>
              ) : (
                <>
                  <Play className="mr-2 h-4 w-4" />
                  Execute Action
                </>
              )}
            </Button>

            {/* Quick Actions for Common Robot Commands */}
            {selectedAction.category === "movement" && selectedPluginData && (
              <div className="grid grid-cols-2 gap-2 pt-2">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => {
                    if (!selectedPluginData) return;
                    const stopAction = (
                      selectedPluginData.plugin
                        .actionDefinitions as RobotAction[]
                    )?.find((a: RobotAction) => a.id === "stop_movement");
                    if (stopAction) {
                      onExecuteAction(
                        selectedPluginData.plugin.name,
                        stopAction.id,
                        {},
                      );
                    }
                  }}
                  disabled={
                    !selectedPluginData ||
                    !(
                      selectedPluginData.plugin
                        .actionDefinitions as RobotAction[]
                    )?.some((a: RobotAction) => a.id === "stop_movement")
                  }
                >
                  Emergency Stop
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => {
                    if (!selectedPluginData) return;
                    const wakeAction = (
                      selectedPluginData.plugin
                        .actionDefinitions as RobotAction[]
                    )?.find((a: RobotAction) => a.id === "wake_up");
                    if (wakeAction) {
                      onExecuteAction(
                        selectedPluginData.plugin.name,
                        wakeAction.id,
                        {},
                      );
                    }
                  }}
                  disabled={
                    !selectedPluginData ||
                    !(
                      selectedPluginData.plugin
                        .actionDefinitions as RobotAction[]
                    )?.some((a: RobotAction) => a.id === "wake_up")
                  }
                >
                  Wake Up
                </Button>
              </div>
            )}
          </CardContent>
        </Card>
      )}

      {/* Plugin Info */}
      {selectedPluginData && (
        <Alert>
          <CheckCircle className="h-4 w-4" />
          <AlertDescription>
            <strong>{selectedPluginData.plugin.name}</strong> -{" "}
            {selectedPluginData.plugin.description}
            <br />
            <span className="text-xs">
              Installed:{" "}
              {selectedPluginData.installation.installedAt.toLocaleDateString()}{" "}
              | Trust Level: {selectedPluginData.plugin.trustLevel} | Actions:{" "}
              {
                (
                  (selectedPluginData.plugin
                    .actionDefinitions as RobotAction[]) || []
                ).length
              }
            </span>
          </AlertDescription>
        </Alert>
      )}
    </div>
  );
}
