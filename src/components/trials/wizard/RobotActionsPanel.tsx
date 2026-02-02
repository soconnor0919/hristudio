"use client";

import React, { useState, useEffect, useMemo, useCallback } from "react";
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
  Wifi,
  WifiOff,
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
import { useWizardRos } from "~/hooks/useWizardRos";

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
  onExecuteAction?: (
    pluginName: string,
    actionId: string,
    parameters: Record<string, unknown>,
  ) => Promise<void>;
}

// Helper functions moved outside component to prevent re-renders
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
    const category = action.category ?? "other";
    if (!grouped[category]) {
      grouped[category] = [];
    }
    grouped[category]!.push(action);
  });

  return grouped;
};

export function RobotActionsPanel({
  studyId,
  trialId: _trialId,
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

  // WebSocket ROS integration
  const {
    isConnected: rosConnected,
    isConnecting: rosConnecting,
    connectionError: rosError,
    robotStatus,
    activeActions,
    connect: connectRos,
    disconnect: disconnectRos,
    executeRobotAction: executeRosAction,
  } = useWizardRos({
    autoConnect: true,
    onActionCompleted: (execution) => {
      toast.success(`Completed: ${execution.actionId}`, {
        description: `Action executed in ${execution.endTime ? execution.endTime.getTime() - execution.startTime.getTime() : 0}ms`,
      });
      // Remove from executing set
      setExecutingActions((prev) => {
        const next = new Set(prev);
        next.delete(`${execution.pluginName}.${execution.actionId}`);
        return next;
      });
    },
    onActionFailed: (execution) => {
      toast.error(`Failed: ${execution.actionId}`, {
        description: execution.error || "Unknown error",
      });
      // Remove from executing set
      setExecutingActions((prev) => {
        const next = new Set(prev);
        next.delete(`${execution.pluginName}.${execution.actionId}`);
        return next;
      });
    },
  });

  // Get installed plugins for the study
  const { data: plugins = [], isLoading } =
    api.robots.plugins.getStudyPlugins.useQuery({
      studyId,
    });

  // Get actions for selected plugin - memoized to prevent infinite re-renders
  const selectedPluginData = useMemo(
    () => plugins.find((p) => p.plugin.id === selectedPlugin),
    [plugins, selectedPlugin],
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

  const toggleCategory = useCallback((category: string) => {
    setExpandedCategories((prev) => {
      const next = new Set(prev);
      if (next.has(category)) {
        next.delete(category);
      } else {
        next.add(category);
      }
      return next;
    });
  }, []);

  const handleExecuteAction = useCallback(async () => {
    if (!selectedAction || !selectedPluginData) return;

    const actionKey = `${selectedPluginData.plugin.name}.${selectedAction.id}`;
    setExecutingActions((prev) => new Set([...prev, actionKey]));

    try {
      // Get action configuration from plugin
      const actionDef = (
        selectedPluginData.plugin.actionDefinitions as RobotAction[]
      )?.find((def: RobotAction) => def.id === selectedAction.id);

      // Try direct WebSocket execution first
      if (rosConnected && actionDef) {
        try {
          // Look for ROS2 configuration in the action definition
          const actionConfig = (actionDef as any).ros2
            ? {
              topic: (actionDef as any).ros2.topic,
              messageType: (actionDef as any).ros2.messageType,
              payloadMapping: (actionDef as any).ros2.payloadMapping,
            }
            : undefined;

          await executeRosAction(
            selectedPluginData.plugin.name,
            selectedAction.id,
            actionParameters,
            actionConfig,
          );

          toast.success(`Executed: ${selectedAction.name}`, {
            description: `Robot action completed via WebSocket`,
          });
        } catch (rosError) {
          console.warn(
            "WebSocket execution failed, falling back to tRPC:",
            rosError,
          );

          // Fallback to tRPC execution
          if (onExecuteAction) {
            await onExecuteAction(
              selectedPluginData.plugin.name,
              selectedAction.id,
              actionParameters,
            );

            toast.success(`Executed: ${selectedAction.name}`, {
              description: `Robot action completed via tRPC fallback`,
            });
          } else {
            throw rosError;
          }
        }
      } else if (onExecuteAction) {
        // Use tRPC execution if WebSocket not available
        await onExecuteAction(
          selectedPluginData.plugin.name,
          selectedAction.id,
          actionParameters,
        );

        toast.success(`Executed: ${selectedAction.name}`, {
          description: `Robot action completed via tRPC`,
        });
      } else {
        throw new Error("No execution method available");
      }
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
  }, [
    selectedAction,
    selectedPluginData,
    rosConnected,
    executeRosAction,
    onExecuteAction,
  ]);

  const handleParameterChange = useCallback(
    (paramName: string, value: unknown) => {
      setActionParameters((prev) => ({
        ...prev,
        [paramName]: value,
      }));
    },
    [],
  );

  const renderParameterInput = (
    param: NonNullable<RobotAction["parameters"]>[0],
    _paramIndex: number,
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
      <div className="space-y-3">
        <div className="flex items-center justify-between rounded-lg border p-3">
          <div className="flex items-center space-x-2">
            {rosConnected ? (
              <Wifi className="h-4 w-4 text-green-500" />
            ) : rosConnecting ? (
              <Loader2 className="h-4 w-4 animate-spin text-yellow-500" />
            ) : (
              <WifiOff className="h-4 w-4 text-red-500" />
            )}
            <span className="text-sm font-medium">ROS Bridge</span>
          </div>

          <div className="flex items-center space-x-2">
            <Badge
              variant={
                rosConnected
                  ? "default"
                  : rosConnecting
                    ? "secondary"
                    : "destructive"
              }
              className="text-xs"
            >
              {rosConnected
                ? "Connected"
                : rosConnecting
                  ? "Connecting"
                  : "Disconnected"}
            </Badge>

            {!rosConnected && !rosConnecting && (
              <Button size="sm" variant="outline" onClick={() => connectRos()}>
                Connect
              </Button>
            )}

            {rosConnected && (
              <Button
                size="sm"
                variant="outline"
                onClick={() => disconnectRos()}
              >
                Disconnect
              </Button>
            )}
          </div>
        </div>
        <Alert>
          <AlertCircle className="h-4 w-4" />
          <AlertDescription>
            No robot plugins are installed in this study. Install plugins to
            control robots during trials.
          </AlertDescription>
        </Alert>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      <ConnectionStatus />
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
          </CardContent>
        </Card>
      )}

      {/* Quick Actions */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center space-x-2">
            <Zap className="h-4 w-4" />
            <span>Quick Actions</span>
          </CardTitle>
          <CardDescription>
            Common robot actions for quick execution
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-2">
          <div className="grid grid-cols-2 gap-2">
            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "say_text", {
                    text: "Hello, I am ready!",
                  }).catch((error) => {
                    console.error("Quick action failed:", error);
                  });
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <Volume2 className="mr-1 h-3 w-3" />
              Say Hello
            </Button>

            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "emergency_stop", {}).catch(
                    (error) => {
                      console.error("Emergency stop failed:", error);
                    },
                  );
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <AlertCircle className="mr-1 h-3 w-3" />
              Stop
            </Button>

            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "turn_head", {
                    yaw: 0,
                    pitch: 0,
                    speed: 0.3,
                  }).catch((error) => {
                    console.error("Head center failed:", error);
                  });
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <Eye className="mr-1 h-3 w-3" />
              Center Head
            </Button>

            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "walk_forward", {
                    speed: 0.1,
                    duration: 2,
                  }).catch((error) => {
                    console.error("Walk forward failed:", error);
                  });
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <Move className="mr-1 h-3 w-3" />
              Walk Test
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  );

  function ConnectionStatus() {
    return (
      <div className="flex items-center justify-between rounded-lg border p-3">
        <div className="flex items-center space-x-2">
          {rosConnected ? (
            <Wifi className="h-4 w-4 text-green-500" />
          ) : rosConnecting ? (
            <Loader2 className="h-4 w-4 animate-spin text-yellow-500" />
          ) : (
            <WifiOff className="h-4 w-4 text-red-500" />
          )}
          <span className="text-sm font-medium">ROS Bridge</span>
        </div>

        <div className="flex items-center space-x-2">
          <Badge
            variant={
              rosConnected
                ? "default"
                : rosConnecting
                  ? "secondary"
                  : "destructive"
            }
            className="text-xs"
          >
            {rosConnected
              ? "Connected"
              : rosConnecting
                ? "Connecting"
                : "Disconnected"}
          </Badge>

          {!rosConnected && !rosConnecting && (
            <Button size="sm" variant="outline" onClick={() => connectRos()}>
              Connect
            </Button>
          )}

          {rosConnected && (
            <Button size="sm" variant="outline" onClick={() => disconnectRos()}>
              Disconnect
            </Button>
          )}
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      <div className="flex items-center justify-between rounded-lg border p-3">
        <div className="flex items-center space-x-2">
          {rosConnected ? (
            <Wifi className="h-4 w-4 text-green-500" />
          ) : rosConnecting ? (
            <Loader2 className="h-4 w-4 animate-spin text-yellow-500" />
          ) : (
            <WifiOff className="h-4 w-4 text-red-500" />
          )}
          <span className="text-sm font-medium">ROS Bridge</span>
        </div>

        <div className="flex items-center space-x-2">
          <Badge
            variant={
              rosConnected
                ? "default"
                : rosConnecting
                  ? "secondary"
                  : "destructive"
            }
            className="text-xs"
          >
            {rosConnected
              ? "Connected"
              : rosConnecting
                ? "Connecting"
                : "Disconnected"}
          </Badge>

          {!rosConnected && !rosConnecting && (
            <Button size="sm" variant="outline" onClick={() => connectRos()}>
              Connect
            </Button>
          )}

          {rosConnected && (
            <Button size="sm" variant="outline" onClick={() => disconnectRos()}>
              Disconnect
            </Button>
          )}
        </div>
      </div>
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
              {selectedPluginData &&
                Object.entries(
                  groupActionsByCategory(
                    (selectedPluginData?.plugin
                      .actionDefinitions as RobotAction[]) ?? [],
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
              <span>{selectedAction?.name}</span>
            </CardTitle>
            <CardDescription>{selectedAction?.description}</CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            {/* Parameters */}
            {selectedAction?.parameters &&
              (selectedAction?.parameters?.length ?? 0) > 0 ? (
              <div className="space-y-4">
                <Label className="text-base">Parameters</Label>
                {selectedAction?.parameters?.map((param, index) =>
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
                !selectedAction ||
                executingActions.has(
                  `${selectedPluginData?.plugin.name}.${selectedAction?.id}`,
                )
              }
              className="w-full"
            >
              {selectedPluginData &&
                selectedAction &&
                executingActions.has(
                  `${selectedPluginData?.plugin.name}.${selectedAction?.id}`,
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
          </CardContent>
        </Card>
      )}

      {/* Quick Actions */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center space-x-2">
            <Zap className="h-4 w-4" />
            <span>Quick Actions</span>
          </CardTitle>
          <CardDescription>
            Common robot actions for quick execution
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-2">
          <div className="grid grid-cols-2 gap-2">
            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "say_text", {
                    text: "Hello, I am ready!",
                  }).catch((error: unknown) => {
                    console.error("Quick action failed:", error);
                  });
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <Volume2 className="mr-1 h-3 w-3" />
              Say Hello
            </Button>

            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "emergency_stop", {}).catch(
                    (error: unknown) => {
                      console.error("Emergency stop failed:", error);
                    },
                  );
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <AlertCircle className="mr-1 h-3 w-3" />
              Stop
            </Button>

            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "turn_head", {
                    yaw: 0,
                    pitch: 0,
                    speed: 0.3,
                  }).catch((error: unknown) => {
                    console.error("Head center failed:", error);
                  });
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <Eye className="mr-1 h-3 w-3" />
              Center Head
            </Button>

            <Button
              size="sm"
              variant="outline"
              onClick={() => {
                if (rosConnected) {
                  executeRosAction("nao6-ros2", "walk_forward", {
                    speed: 0.1,
                    duration: 2,
                  }).catch((error: unknown) => {
                    console.error("Walk forward failed:", error);
                  });
                }
              }}
              disabled={!rosConnected || rosConnecting}
            >
              <Move className="mr-1 h-3 w-3" />
              Walk Test
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
