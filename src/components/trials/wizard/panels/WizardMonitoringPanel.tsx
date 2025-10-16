"use client";

import React from "react";
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
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";
import { Progress } from "~/components/ui/progress";
import { Button } from "~/components/ui/button";
// import { useRosBridge } from "~/hooks/useRosBridge"; // Removed ROS dependency

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
}

export function WizardMonitoringPanel({
  trial,
  trialEvents,
  isConnected,
  wsError,
  activeTab,
  onTabChange,
}: WizardMonitoringPanelProps) {
  // Mock robot status for development (ROS bridge removed for now)
  const mockRobotStatus = {
    connected: false,
    battery: 85,
    position: { x: 0, y: 0, theta: 0 },
    joints: {},
    sensors: {},
    lastUpdate: new Date(),
  };

  const rosConnected = false;
  const rosConnecting = false;
  const rosError = null;
  const robotStatus = mockRobotStatus;
  // const connectRos = () => console.log("ROS connection not implemented yet");
  const disconnectRos = () =>
    console.log("ROS disconnection not implemented yet");
  const executeRobotAction = (
    action: string,
    parameters?: Record<string, unknown>,
  ) => console.log("Robot action:", action, parameters);

  const formatTimestamp = (timestamp: Date) => {
    return new Date(timestamp).toLocaleTimeString();
  };

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
                        WebSocket
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
                      <Badge
                        variant={rosConnected ? "default" : "outline"}
                        className="text-xs"
                      >
                        {rosConnecting
                          ? "Connecting..."
                          : rosConnected
                            ? "Connected"
                            : "Offline"}
                      </Badge>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground text-xs">
                        Battery
                      </span>
                      <div className="flex items-center gap-1">
                        <span className="text-xs">
                          {robotStatus
                            ? `${Math.round(robotStatus.battery * 100)}%`
                            : "--"}
                        </span>
                        <Progress
                          value={robotStatus ? robotStatus.battery * 100 : 0}
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
                        onClick={() =>
                          console.log("Connect robot (not implemented)")
                        }
                        disabled={true}
                      >
                        <Bot className="mr-1 h-3 w-3" />
                        Connect Robot (Coming Soon)
                      </Button>
                    ) : (
                      <Button
                        size="sm"
                        variant="outline"
                        className="w-full text-xs"
                        onClick={disconnectRos}
                      >
                        <PowerOff className="mr-1 h-3 w-3" />
                        Disconnect Robot
                      </Button>
                    )}
                  </div>

                  {rosError && (
                    <Alert variant="destructive" className="mt-2">
                      <AlertCircle className="h-4 w-4" />
                      <AlertDescription className="text-xs">
                        ROS Error: {rosError}
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

                {/* Quick Robot Actions */}
                {rosConnected && (
                  <div className="space-y-2">
                    <div className="text-sm font-medium">Quick Actions</div>
                    <div className="grid grid-cols-2 gap-1">
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("say_text", {
                            text: "Hello from wizard!",
                          })
                        }
                      >
                        Say Hello
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("play_animation", {
                            animation: "Hello",
                          })
                        }
                      >
                        Wave
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("set_led_color", {
                            color: "blue",
                            intensity: 1.0,
                          })
                        }
                      >
                        Blue LEDs
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("turn_head", {
                            yaw: 0,
                            pitch: 0,
                            speed: 0.3,
                          })
                        }
                      >
                        Center Head
                      </Button>
                    </div>
                  </div>
                )}

                {!rosConnected && !rosConnecting && (
                  <Alert className="mt-4">
                    <AlertCircle className="h-4 w-4" />
                    <AlertDescription className="text-xs">
                      Connect to ROS bridge for live robot monitoring and
                      control
                    </AlertDescription>
                  </Alert>
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
}
