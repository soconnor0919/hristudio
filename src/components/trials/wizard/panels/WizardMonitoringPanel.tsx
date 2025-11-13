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
}

const WizardMonitoringPanel = React.memo(function WizardMonitoringPanel({
  trial,
  trialEvents,
  isConnected,
  wsError,
  activeTab,
  onTabChange,
}: WizardMonitoringPanelProps) {
  // ROS Bridge connection state
  const [rosConnected, setRosConnected] = useState(false);
  const [rosConnecting, setRosConnecting] = useState(false);
  const [rosError, setRosError] = useState<string | null>(null);
  const [rosSocket, setRosSocket] = useState<WebSocket | null>(null);
  const [robotStatus, setRobotStatus] = useState({
    connected: false,
    battery: 0,
    position: { x: 0, y: 0, theta: 0 },
    joints: {},
    sensors: {},
    lastUpdate: new Date(),
  });

  const ROS_BRIDGE_URL = "ws://134.82.159.25:9090";

  // Use refs to persist connection state across re-renders
  const connectionAttemptRef = useRef(false);
  const socketRef = useRef<WebSocket | null>(null);

  const connectRos = () => {
    // Prevent multiple connection attempts
    if (connectionAttemptRef.current) {
      console.log("Connection already in progress, skipping");
      return;
    }

    if (
      rosSocket?.readyState === WebSocket.OPEN ||
      socketRef.current?.readyState === WebSocket.OPEN
    ) {
      console.log("Already connected, skipping");
      return;
    }

    // Prevent rapid reconnection attempts
    if (rosConnecting) {
      console.log("Connection in progress, please wait");
      return;
    }

    connectionAttemptRef.current = true;
    setRosConnecting(true);
    setRosError(null);

    console.log("🔌 Connecting to ROS Bridge:", ROS_BRIDGE_URL);
    const socket = new WebSocket(ROS_BRIDGE_URL);
    socketRef.current = socket;

    // Add connection timeout
    const connectionTimeout = setTimeout(() => {
      if (socket.readyState === WebSocket.CONNECTING) {
        socket.close();
        connectionAttemptRef.current = false;
        setRosConnecting(false);
        setRosError("Connection timeout (10s) - ROS Bridge not responding");
      }
    }, 10000);

    socket.onopen = () => {
      clearTimeout(connectionTimeout);
      connectionAttemptRef.current = false;
      console.log("Connected to ROS Bridge successfully");
      setRosConnected(true);
      setRosConnecting(false);
      setRosSocket(socket);
      setRosError(null);

      // Just log connection success - no auto actions
      console.log("WebSocket connected successfully to ROS Bridge");

      setRobotStatus((prev) => ({
        ...prev,
        connected: true,
        lastUpdate: new Date(),
      }));
    };

    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data as string) as {
          topic?: string;
          msg?: Record<string, unknown>;
          op?: string;
          level?: string;
        };

        // Handle status messages
        if (data.op === "status") {
          console.log("ROS Bridge status:", data.msg, "Level:", data.level);
          return;
        }

        // Handle topic messages
        if (data.topic === "/joint_states" && data.msg) {
          setRobotStatus((prev) => ({
            ...prev,
            joints: data.msg ?? {},
            lastUpdate: new Date(),
          }));
        } else if (data.topic === "/naoqi_driver/battery" && data.msg) {
          const batteryPercent = (data.msg.percentage as number) || 0;
          setRobotStatus((prev) => ({
            ...prev,
            battery: Math.round(batteryPercent),
            lastUpdate: new Date(),
          }));
        } else if (data.topic === "/diagnostics" && data.msg) {
          // Handle diagnostic messages for battery
          console.log("Diagnostics received:", data.msg);
        }
      } catch (error) {
        console.error("Error parsing ROS message:", error);
      }
    };

    socket.onclose = (event) => {
      clearTimeout(connectionTimeout);
      connectionAttemptRef.current = false;
      setRosConnected(false);
      setRosConnecting(false);
      setRosSocket(null);
      socketRef.current = null;
      setRobotStatus((prev) => ({
        ...prev,
        connected: false,
        battery: 0,
        joints: {},
        sensors: {},
      }));

      // Only show error if it wasn't a normal closure (code 1000)
      if (event.code !== 1000) {
        let errorMsg = "Connection lost";
        if (event.code === 1006) {
          errorMsg =
            "ROS Bridge not responding - check if rosbridge_server is running";
        } else if (event.code === 1011) {
          errorMsg = "Server error in ROS Bridge";
        } else if (event.code === 1002) {
          errorMsg = "Protocol error - check ROS Bridge version";
        } else if (event.code === 1001) {
          errorMsg = "Server going away - ROS Bridge may have restarted";
        }
        console.log(
          `🔌 Connection closed - Code: ${event.code}, Reason: ${event.reason}`,
        );
        setRosError(`${errorMsg} (${event.code})`);
      }
    };

    socket.onerror = (error) => {
      clearTimeout(connectionTimeout);
      connectionAttemptRef.current = false;
      console.error("ROS Bridge WebSocket error:", error);
      setRosConnected(false);
      setRosConnecting(false);
      setRosError(
        "Failed to connect to ROS bridge - check if rosbridge_server is running",
      );
      setRobotStatus((prev) => ({ ...prev, connected: false }));
    };
  };

  const disconnectRos = () => {
    console.log("Manually disconnecting from ROS Bridge");
    connectionAttemptRef.current = false;
    if (rosSocket) {
      // Close with normal closure code to avoid error messages
      rosSocket.close(1000, "User disconnected");
    }
    if (socketRef.current) {
      socketRef.current.close(1000, "User disconnected");
    }
    // Clear all state
    setRosSocket(null);
    socketRef.current = null;
    setRosConnected(false);
    setRosConnecting(false);
    setRosError(null);
    setRobotStatus({
      connected: false,
      battery: 0,
      position: { x: 0, y: 0, theta: 0 },
      joints: {},
      sensors: {},
      lastUpdate: new Date(),
    });
  };

  const executeRobotAction = (
    action: string,
    parameters?: Record<string, unknown>,
  ) => {
    if (!rosSocket || !rosConnected) {
      setRosError("Robot not connected");
      return;
    }

    let message: {
      op: string;
      topic: string;
      type: string;
      msg: Record<string, unknown>;
    };

    switch (action) {
      case "say_text":
        const speechText = parameters?.text ?? "Hello from wizard interface!";
        console.log("🔊 Preparing speech command:", speechText);
        message = {
          op: "publish",
          topic: "/speech",
          type: "std_msgs/String",
          msg: { data: speechText },
        };
        console.log(
          "📤 Speech message constructed:",
          JSON.stringify(message, null, 2),
        );
        break;

      case "move_forward":
      case "move_backward":
      case "turn_left":
      case "turn_right":
        const speed = (parameters?.speed as number) || 0.1;
        const linear = action.includes("forward")
          ? speed
          : action.includes("backward")
            ? -speed
            : 0;
        const angular = action.includes("left")
          ? speed
          : action.includes("right")
            ? -speed
            : 0;

        message = {
          op: "publish",
          topic: "/cmd_vel",
          type: "geometry_msgs/Twist",
          msg: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
          },
        };
        break;

      case "stop_movement":
        message = {
          op: "publish",
          topic: "/cmd_vel",
          type: "geometry_msgs/Twist",
          msg: {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 },
          },
        };
        break;

      case "head_movement":
      case "turn_head":
        const yaw = (parameters?.yaw as number) || 0;
        const pitch = (parameters?.pitch as number) || 0;
        const headSpeed = (parameters?.speed as number) || 0.3;

        message = {
          op: "publish",
          topic: "/joint_angles",
          type: "naoqi_bridge_msgs/JointAnglesWithSpeed",
          msg: {
            joint_names: ["HeadYaw", "HeadPitch"],
            joint_angles: [yaw, pitch],
            speed: headSpeed,
          },
        };
        break;

      case "play_animation":
        const animation = (parameters?.animation as string) ?? "Hello";

        message = {
          op: "publish",
          topic: "/naoqi_driver/animation",
          type: "std_msgs/String",
          msg: { data: animation },
        };
        break;

      default:
        setRosError(`Unknown action: ${String(action)}`);
        return;
    }

    try {
      const messageStr = JSON.stringify(message);
      console.log("📡 Sending to ROS Bridge:", messageStr);
      rosSocket.send(messageStr);
      console.log(`✅ Sent robot action: ${action}`, parameters);
    } catch (error) {
      console.error("❌ Failed to send command:", error);
      setRosError(`Failed to send command: ${String(error)}`);
    }
  };

  const subscribeToTopic = (topic: string, messageType: string) => {
    if (!rosSocket || !rosConnected) {
      setRosError("Cannot subscribe - not connected");
      return;
    }

    try {
      const subscribeMsg = {
        op: "subscribe",
        topic: topic,
        type: messageType,
      };
      rosSocket.send(JSON.stringify(subscribeMsg));
      console.log(`Manually subscribed to ${topic}`);
    } catch (error) {
      setRosError(`Failed to subscribe to ${topic}: ${String(error)}`);
    }
  };

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
                        onClick={connectRos}
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
                        onClick={disconnectRos}
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
                              telnet 134.82.159.25 9090
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
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("say_text", {
                            text: "Connection test - can you hear me?",
                          })
                        }
                        disabled={!rosConnected}
                      >
                        {rosConnected ? "🔊 Test Speech" : "🔊 Not Ready"}
                      </Button>
                    </div>

                    {/* Topic Subscriptions */}
                    <div className="space-y-1">
                      <div className="text-muted-foreground text-xs font-medium">
                        Subscribe to Topics:
                      </div>
                      <div className="grid grid-cols-1 gap-1">
                        <Button
                          size="sm"
                          variant="ghost"
                          className="justify-start text-xs"
                          onClick={() =>
                            subscribeToTopic(
                              "/naoqi_driver/battery",
                              "naoqi_bridge_msgs/Battery",
                            )
                          }
                        >
                          🔋 Battery Status
                        </Button>
                        <Button
                          size="sm"
                          variant="ghost"
                          className="justify-start text-xs"
                          onClick={() =>
                            subscribeToTopic(
                              "/naoqi_driver/joint_states",
                              "sensor_msgs/JointState",
                            )
                          }
                        >
                          🤖 Joint States
                        </Button>
                        <Button
                          size="sm"
                          variant="ghost"
                          className="justify-start text-xs"
                          onClick={() =>
                            subscribeToTopic(
                              "/naoqi_driver/bumper",
                              "naoqi_bridge_msgs/Bumper",
                            )
                          }
                        >
                          👟 Bumper Sensors
                        </Button>
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
                        onClick={() =>
                          executeRobotAction("move_forward", { speed: 0.05 })
                        }
                      >
                        Forward
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("turn_left", { speed: 0.3 })
                        }
                      >
                        Turn Left
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("turn_right", { speed: 0.3 })
                        }
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
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("turn_head", {
                            yaw: 0.5,
                            pitch: 0,
                            speed: 0.3,
                          })
                        }
                      >
                        Look Left
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("turn_head", {
                            yaw: -0.5,
                            pitch: 0,
                            speed: 0.3,
                          })
                        }
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
                        onClick={() =>
                          executeRobotAction("play_animation", {
                            animation: "Hello",
                          })
                        }
                      >
                        Wave Hello
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        className="text-xs"
                        onClick={() =>
                          executeRobotAction("say_text", {
                            text: "Experiment ready!",
                          })
                        }
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
                        onClick={() => executeRobotAction("stop_movement", {})}
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
