"use client";

import React, { useState, useRef } from "react";
import {
  Bot,
  User,
  Activity,
  Wifi,
  WifiOff,
  AlertCircle,
  CheckCircle,
  Clock,
  Power,
  PowerOff,
  Eye,
  Volume2,
  Move,
  Hand,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";
import { Progress } from "~/components/ui/progress";
import { Button } from "~/components/ui/button";

interface TrialData {
  id: string;
  status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
  scheduledAt: Date | null;
  startedAt: Date | null;
  completedAt: Date | null;
  duration: number | null;
  sessionNumber: number | null;
  notes: string | null;
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
}

interface TrialEvent {
  type: string;
  timestamp: Date;
  data?: unknown;
  message?: string;
}

interface WizardMonitoringPanelProps {
  trial: TrialData;
  trialEvents: TrialEvent[];
  isConnected: boolean;
  wsError?: string;
  activeTab: "status" | "robot" | "events";
  onTabChange: (tab: "status" | "robot" | "events") => void;
  // ROS connection props
  rosConnected: boolean;
  rosConnecting: boolean;
  rosError?: string;
  robotStatus: {
    connected: boolean;
    battery: number;
    position: { x: number; y: number; theta: number };
    joints: Record<string, unknown>;
    sensors: Record<string, unknown>;
    lastUpdate: Date;
  };
  connectRos: () => Promise<void>;
  disconnectRos: () => void;
  executeRosAction: (
    pluginName: string,
    actionId: string,
    parameters: Record<string, unknown>,
  ) => Promise<unknown>;
}

const WizardMonitoringPanel = React.memo(function WizardMonitoringPanel({
  trial,
  trialEvents,
  isConnected,
  wsError,
  activeTab,
  onTabChange,
  rosConnected,
  rosConnecting,
  rosError,
  robotStatus,
  connectRos,
  disconnectRos,
  executeRosAction,
}: WizardMonitoringPanelProps) {
  // ROS connection is now passed as props, no need for separate hook

  // Don't close connection on unmount to prevent disconnection issues
  // Connection will persist across component re-renders

  // Removed auto-reconnect to prevent interference with manual connections

  const formatTimestamp = React.useCallback((timestamp: Date) => {
    return new Date(timestamp).toLocaleTimeString();
  }, []);

  const getEventIcon = (eventType: string) => {
    switch (eventType.toLowerCase()) {
      case "trial_started":
      case "trial_resumed":
        return CheckCircle;
      case "trial_paused":
      case "trial_stopped":
        return AlertCircle;
      case "step_completed":
      case "action_completed":
        return CheckCircle;
      case "robot_action":
      case "robot_status":
        return Bot;
      case "wizard_action":
      case "wizard_intervention":
        return User;
      case "system_error":
      case "connection_error":
        return AlertCircle;
      default:
        return Activity;
    }
  };

  const getEventColor = (eventType: string) => {
    switch (eventType.toLowerCase()) {
      case "trial_started":
      case "trial_resumed":
      case "step_completed":
      case "action_completed":
        return "text-green-600";
      case "trial_paused":
      case "trial_stopped":
        return "text-yellow-600";
      case "system_error":
      case "connection_error":
      case "trial_failed":
        return "text-red-600";
      case "robot_action":
      case "robot_status":
        return "text-blue-600";
      case "wizard_action":
      case "wizard_intervention":
        return "text-purple-600";
      default:
        return "text-muted-foreground";
    }
  };

  return (
    <div className="flex h-full flex-col">
      {/* Header */}
      <div className="border-b p-3">
        <div className="flex items-center justify-between">
          <h3 className="text-sm font-medium">Monitoring</h3>
          <div className="flex items-center gap-2">
            {isConnected ? (
              <Wifi className="h-4 w-4 text-green-600" />
            ) : (
              <WifiOff className="h-4 w-4 text-orange-600" />
            )}
            <Badge
              variant={isConnected ? "default" : "secondary"}
              className="text-xs"
            >
              {isConnected ? "Live" : "Offline"}
            </Badge>
          </div>
        </div>
        {wsError && (
          <Alert variant="destructive" className="mt-2">
            <AlertCircle className="h-4 w-4" />
            <AlertDescription className="text-xs">{wsError}</AlertDescription>
          </Alert>
        )}
      </div>

      {/* Tabbed Content */}
      <Tabs
        value={activeTab}
        onValueChange={(value: string) => {
          if (value === "status" || value === "robot" || value === "events") {
            onTabChange(value);
          }
        }}
        className="flex min-h-0 flex-1 flex-col"
      >
        <div className="border-b px-2 py-1">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="status" className="text-xs">
              <Eye className="mr-1 h-3 w-3" />
              Status
            </TabsTrigger>
            <TabsTrigger value="robot" className="text-xs">
              <Bot className="mr-1 h-3 w-3" />
              Robot
            </TabsTrigger>
            <TabsTrigger value="events" className="text-xs">
              <Activity className="mr-1 h-3 w-3" />
              Events
              {trialEvents.length > 0 && (
                <Badge variant="secondary" className="ml-1 text-xs">
                  {trialEvents.length}
                </Badge>
              )}
            </TabsTrigger>
          </TabsList>
        </div>

        <div className="min-h-0 flex-1">
          {/* Status Tab */}
          <TabsContent value="status" className="m-0 h-full">
            <ScrollArea className="h-full">
              <div className="space-y-4 p-3">
                {/* Connection Status */}
                <div className="space-y-2">
                  <div className="text-sm font-medium">Connection</div>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        ROS Bridge
                      </span>
                      <Badge
                        variant={isConnected ? "default" : "secondary"}
                        className="text-xs"
                      >
                        {isConnected ? "Connected" : "Offline"}
                      </Badge>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Data Mode
                      </span>
                      <span className="text-xs">
                        {isConnected ? "Real-time" : "Polling"}
                      </span>
                    </div>
                  </div>
                </div>

                <Separator />

                {/* Trial Information */}
                <div className="space-y-2">
                  <div className="text-sm font-medium">Trial Info</div>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">ID</span>
                      <span className="font-mono text-xs">
                        {trial.id.slice(-8)}
                      </span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Session
                      </span>
                      <span className="text-xs">#{trial.sessionNumber}</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Status
                      </span>
                      <Badge variant="outline" className="text-xs">
                        {trial.status.replace("_", " ")}
                      </Badge>
                    </div>
                    {trial.startedAt && (
                      <div className="flex items-center justify-between">
                        <span className="text-muted-foreground text-xs">
                          Started
                        </span>
                        <span className="text-xs">
                          {formatTimestamp(new Date(trial.startedAt))}
                        </span>
                      </div>
                    )}
                  </div>
                </div>

                <Separator />

                {/* Participant Information */}
                <div className="space-y-2">
                  <div className="text-sm font-medium">Participant</div>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Code
                      </span>
                      <span className="font-mono text-xs">
                        {trial.participant.participantCode}
                      </span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Session
                      </span>
                      <span className="text-xs">#{trial.sessionNumber}</span>
                    </div>
                    {trial.participant.demographics && (
                      <div className="flex items-center justify-between">
                        <span className="text-muted-foreground text-xs">
                          Demographics
                        </span>
                        <span className="text-xs">
                          {Object.keys(trial.participant.demographics).length}{" "}
                          fields
                        </span>
                      </div>
                    )}
                  </div>
                </div>

                <Separator />

                {/* System Information */}
                <div className="space-y-2">
                  <div className="text-sm font-medium">System</div>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Experiment
                      </span>
                      <span
                        className="max-w-24 truncate text-xs"
                        title={trial.experiment.name}
                      >
                        {trial.experiment.name}
                      </span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Study
                      </span>
                      <span className="font-mono text-xs">
                        {trial.experiment.studyId.slice(-8)}
                      </span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Platform
                      </span>
                      <span className="text-xs">HRIStudio</span>
                    </div>
                  </div>
                </div>
              </div>
            </ScrollArea>
          </TabsContent>

          {/* Robot Tab */}
          <TabsContent value="robot" className="m-0 h-full">
            <ScrollArea className="h-full">
              <div className="space-y-4 p-3">
                {/* Robot Status */}
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <div className="text-sm font-medium">Robot Status</div>
                    <div className="flex items-center gap-1">
                      {rosConnected ? (
                        <Power className="h-3 w-3 text-green-600" />
                      ) : (
                        <PowerOff className="h-3 w-3 text-gray-400" />
                      )}
                    </div>
                  </div>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        ROS Bridge
                      </span>
                      <div className="flex items-center gap-1">
                        <Badge
                          variant={
                            rosConnected
                              ? "default"
                              : rosError
                                ? "destructive"
                                : "outline"
                          }
                          className="text-xs"
                        >
                          {rosConnecting
                            ? "Connecting..."
                            : rosConnected
                              ? "Ready"
                              : rosError
                                ? "Failed"
                                : "Offline"}
                        </Badge>
                        {rosConnected && (
                          <span className="animate-pulse text-xs text-green-600">
                            ●
                          </span>
                        )}
                        {rosConnecting && (
                          <span className="animate-spin text-xs text-blue-600">
                            ⟳
                          </span>
                        )}
                      </div>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Battery
                      </span>
                      <div className="flex items-center gap-1">
                        <span className="text-xs">
                          {robotStatus && robotStatus.battery > 0
                            ? `${Math.round(robotStatus.battery)}%`
                            : rosConnected
                              ? "Reading..."
                              : "No data"}
                        </span>
                        <Progress
                          value={
                            robotStatus && robotStatus.battery > 0
                              ? robotStatus.battery
                              : 0
                          }
                          className="h-1 w-8"
                        />
                      </div>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Position
                      </span>
                      <span className="text-xs">
                        {robotStatus
                          ? `(${robotStatus.position.x.toFixed(1)}, ${robotStatus.position.y.toFixed(1)})`
                          : "--"}
                      </span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Last Update
                      </span>
                      <span className="text-xs">
                        {robotStatus
                          ? robotStatus.lastUpdate.toLocaleTimeString()
                          : "--"}
                      </span>
                    </div>
                  </div>

                  {/* ROS Connection Controls */}
                  <div className="pt-2">
                    {!rosConnected ? (
                      <Button
                        size="sm"
                        variant="outline"
                        className="w-full text-xs"
                        onClick={() => connectRos()}
                        disabled={rosConnecting || rosConnected}
                      >
                        <Bot className="mr-1 h-3 w-3" />
                        {rosConnecting
                          ? "Connecting..."
                          : rosConnected
                            ? "Connected ✓"
                            : "Connect to NAO6"}
                      </Button>
                    ) : (
                      <Button
                        size="sm"
                        variant="outline"
                        className="w-full text-xs"
                        onClick={() => disconnectRos()}
                      >
                        <PowerOff className="mr-1 h-3 w-3" />
                        Disconnect
                      </Button>
                    )}
                  </div>

                  {rosError && (
                    <Alert variant="destructive" className="mt-2">
                      <AlertCircle className="h-4 w-4" />
                      <AlertDescription className="text-xs">
                        {rosError}
                      </AlertDescription>
                    </Alert>
                  )}

                  {/* Connection Help */}
                  {!rosConnected && !rosConnecting && (
                    <Alert className="mt-2">
                      <AlertCircle className="h-4 w-4" />
                      <AlertDescription className="text-xs">
                        <div className="space-y-1">
                          <div className="font-medium">Troubleshooting:</div>
                          <div>
                            1. Check ROS Bridge:{" "}
                            <code className="bg-muted rounded px-1 text-xs">
                              telnet localhost 9090
                            </code>
                          </div>
                          <div>2. NAO6 must be awake and connected</div>
                          <div>
                            3. Try: Click Connect → Wait 2s → Test Speech
                          </div>
                        </div>
                      </AlertDescription>
                    </Alert>
                  )}
                </div>

                <Separator />

                {/* Robot Actions */}
                <div className="space-y-2">
                  <div className="text-sm font-medium">Active Actions</div>
                  <div className="space-y-1">
                    <div className="text-muted-foreground text-center text-xs">
                      No active actions
                    </div>
                  </div>
                </div>

                <Separator />

                {/* Recent Trial Events */}
                <div className="space-y-2">
                  <div className="text-sm font-medium">Recent Events</div>
                  <div className="space-y-1">
                    {trialEvents
                      .filter((e) => e.type.includes("robot"))
                      .slice(-2)
                      .map((event, index) => (
                        <div
                          key={index}
                          className="border-border/50 flex items-center justify-between rounded border p-2"
                        >
                          <span className="text-xs font-medium">
                            {event.type.replace(/_/g, " ")}
                          </span>
                          <span className="text-muted-foreground text-xs">
                            {formatTimestamp(event.timestamp)}
                          </span>
                        </div>
                      ))}
                    {trialEvents.filter((e) => e.type.includes("robot"))
                      .length === 0 && (
                        <div className="text-muted-foreground py-2 text-center text-xs">
                          No robot events yet
                        </div>
                      )}
                  </div>
                </div>

                <Separator />

                {/* Robot Configuration */}
                <div className="space-y-2">
                  <div className="text-sm font-medium">Configuration</div>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Type
                      </span>
                      <span className="text-xs">NAO6</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        ROS Bridge
                      </span>
                      <span className="font-mono text-xs">localhost:9090</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Platform
                      </span>
                      <span className="font-mono text-xs">NAOqi</span>
                    </div>
                    {robotStatus &&
                      Object.keys(robotStatus.joints).length > 0 && (
                        <div className="flex items-center justify-between">
                          <span className="text-muted-foreground text-xs">
                            Joints
                          </span>
                          <span className="text-xs">
                            {Object.keys(robotStatus.joints).length} active
                          </span>
                        </div>
                      )}
                  </div>
                </div>

                {/* Manual Subscription Controls */}
                {rosConnected && (
                  <div className="space-y-2">
                    <div className="text-sm font-medium">Manual Controls</div>

                    {/* Connection Test */}
                    <div className="grid grid-cols-1 gap-1">
                      <Button
                        size="sm"
                        variant="outline"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "say_text", {
                              text: "Connection test - can you hear me?",
                            }).catch(console.error);
                          }
                        }}
                        disabled={!rosConnected}
                      >
                        <Volume2 className="mr-1 h-3 w-3" />
                        Test Speech
                      </Button>
                    </div>

                    {/* Topic Subscriptions */}
                    <div className="space-y-1">
                      <div className="text-muted-foreground text-xs font-medium">
                        Subscribe to Topics:
                      </div>
                      <div className="grid grid-cols-1 gap-1">
                        <div className="text-muted-foreground text-xs">
                          Subscriptions managed automatically
                        </div>
                      </div>
                    </div>
                  </div>
                )}

                {/* Quick Robot Actions */}
                {rosConnected && (
                  <div className="space-y-2">
                    <div className="text-sm font-medium">Robot Actions</div>

                    {/* Movement Controls */}
                    <div className="grid grid-cols-3 gap-1">
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "walk_forward", {
                              speed: 0.05,
                              duration: 2,
                            }).catch(console.error);
                          }
                        }}
                      >
                        Forward
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "turn_left", {
                              speed: 0.3,
                              duration: 2,
                            }).catch(console.error);
                          }
                        }}
                      >
                        Turn Left
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "turn_right", {
                              speed: 0.3,
                              duration: 2,
                            }).catch(console.error);
                          }
                        }}
                      >
                        Turn Right
                      </Button>
                    </div>

                    {/* Head Controls */}
                    <div className="grid grid-cols-3 gap-1">
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "turn_head", {
                              yaw: 0,
                              pitch: 0,
                              speed: 0.3,
                            }).catch(console.error);
                          }
                        }}
                      >
                        Center Head
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "turn_head", {
                              yaw: 0.5,
                              pitch: 0,
                              speed: 0.3,
                            }).catch(console.error);
                          }
                        }}
                      >
                        Look Left
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "turn_head", {
                              yaw: -0.5,
                              pitch: 0,
                              speed: 0.3,
                            }).catch(console.error);
                          }
                        }}
                      >
                        Look Right
                      </Button>
                    </div>

                    {/* Animation & LED Controls */}
                    <div className="grid grid-cols-2 gap-1">
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "say_text", {
                              text: "Hello! I am NAO!",
                            }).catch(console.error);
                          }
                        }}
                      >
                        Say Hello
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction("nao6-ros2", "say_text", {
                              text: "Experiment ready!",
                            }).catch(console.error);
                          }
                        }}
                      >
                        Say Ready
                      </Button>
                    </div>

                    {/* Emergency Controls */}
                    <div className="grid grid-cols-1 gap-1">
                      <Button
                        size="sm"
                        variant="destructive"
                        className="text-xs"
                        onClick={() => {
                          if (rosConnected) {
                            executeRosAction(
                              "nao6-ros2",
                              "emergency_stop",
                              {},
                            ).catch(console.error);
                          }
                        }}
                      >
                        🛑 Emergency Stop
                      </Button>
                    </div>
                  </div>
                )}

                {!rosConnected && !rosConnecting && (
                  <div className="mt-4">
                    <Alert>
                      <AlertCircle className="h-4 w-4" />
                      <AlertDescription className="text-xs">
                        Connect to ROS bridge for live robot monitoring and
                        control
                      </AlertDescription>
                    </Alert>
                  </div>
                )}
              </div>
            </ScrollArea>
          </TabsContent>

          {/* Events Tab */}
          <TabsContent value="events" className="m-0 h-full">
            <ScrollArea className="h-full">
              <div className="p-3">
                {trialEvents.length === 0 ? (
                  <div className="text-muted-foreground flex h-32 items-center justify-center text-center text-sm">
                    No events recorded yet
                  </div>
                ) : (
                  <div className="space-y-2">
                    <div className="mb-3 flex items-center justify-between">
                      <span className="text-sm font-medium">Live Events</span>
                      <Badge variant="secondary" className="text-xs">
                        {trialEvents.length}
                      </Badge>
                    </div>

                    {trialEvents
                      .slice()
                      .reverse()
                      .map((event, index) => {
                        const EventIcon = getEventIcon(event.type);
                        const eventColor = getEventColor(event.type);

                        return (
                          <div
                            key={`${event.timestamp.getTime()}-${index}`}
                            className="border-border/50 flex items-start gap-2 rounded-lg border p-2"
                          >
                            <div className={`mt-0.5 ${eventColor}`}>
                              <EventIcon className="h-3 w-3" />
                            </div>
                            <div className="min-w-0 flex-1">
                              <div className="text-sm font-medium capitalize">
                                {event.type.replace(/_/g, " ")}
                              </div>
                              {event.message && (
                                <div className="text-muted-foreground mt-1 text-xs">
                                  {event.message}
                                </div>
                              )}
                              <div className="text-muted-foreground mt-1 flex items-center gap-1 text-xs">
                                <Clock className="h-3 w-3" />
                                {formatTimestamp(event.timestamp)}
                              </div>
                            </div>
                          </div>
                        );
                      })}
                  </div>
                )}
              </div>
            </ScrollArea>
          </TabsContent>
        </div>
      </Tabs>
    </div>
  );
});

export { WizardMonitoringPanel };
