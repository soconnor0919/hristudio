"use client";

import { useState, useEffect, useRef } from "react";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { PageHeader } from "~/components/ui/page-header";
import { PageLayout } from "~/components/ui/page-layout";
import { ScrollArea } from "~/components/ui/scroll-area";
import {
  Wifi,
  WifiOff,
  AlertTriangle,
  CheckCircle,
  Play,
  Square,
  Trash2,
  Copy,
} from "lucide-react";

export default function DebugPage() {
  const [connectionStatus, setConnectionStatus] = useState<
    "disconnected" | "connecting" | "connected" | "error"
  >("disconnected");
  const [rosSocket, setRosSocket] = useState<WebSocket | null>(null);
  const [logs, setLogs] = useState<string[]>([]);
  const [messages, setMessages] = useState<any[]>([]);
  const [testMessage, setTestMessage] = useState("");
  const [selectedTopic, setSelectedTopic] = useState("/speech");
  const [messageType, setMessageType] = useState("std_msgs/String");
  const [lastError, setLastError] = useState<string | null>(null);
  const [connectionAttempts, setConnectionAttempts] = useState(0);
  const logsEndRef = useRef<HTMLDivElement>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const ROS_BRIDGE_URL = "ws://134.82.159.25:9090";

  const addLog = (message: string, type: "info" | "error" | "success" = "info") => {
    const timestamp = new Date().toLocaleTimeString();
    const logEntry = `[${timestamp}] [${type.toUpperCase()}] ${message}`;
    setLogs((prev) => [...prev.slice(-99), logEntry]);
    console.log(logEntry);
  };

  const addMessage = (message: any, direction: "sent" | "received") => {
    const timestamp = new Date().toLocaleTimeString();
    setMessages((prev) => [
      ...prev.slice(-49),
      {
        timestamp,
        direction,
        data: message,
      },
    ]);
  };

  useEffect(() => {
    logsEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [logs]);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  const connectToRos = () => {
    if (rosSocket?.readyState === WebSocket.OPEN) return;

    setConnectionStatus("connecting");
    setConnectionAttempts((prev) => prev + 1);
    setLastError(null);
    addLog(`Attempting connection #${connectionAttempts + 1} to ${ROS_BRIDGE_URL}`);

    const socket = new WebSocket(ROS_BRIDGE_URL);

    // Connection timeout
    const timeout = setTimeout(() => {
      if (socket.readyState === WebSocket.CONNECTING) {
        addLog("Connection timeout (10s) - closing socket", "error");
        socket.close();
      }
    }, 10000);

    socket.onopen = () => {
      clearTimeout(timeout);
      setConnectionStatus("connected");
      setRosSocket(socket);
      setLastError(null);
      addLog("✅ WebSocket connection established successfully", "success");

      // Test basic functionality by advertising
      const advertiseMsg = {
        op: "advertise",
        topic: "/hristudio_debug",
        type: "std_msgs/String",
      };
      socket.send(JSON.stringify(advertiseMsg));
      addMessage(advertiseMsg, "sent");
    };

    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        addMessage(data, "received");

        if (data.level === "error") {
          addLog(`ROS Error: ${data.msg}`, "error");
        } else if (data.op === "status") {
          addLog(`Status: ${data.msg} (Level: ${data.level})`);
        } else {
          addLog(`Received: ${data.op || "unknown"} operation`);
        }
      } catch (error) {
        addLog(`Failed to parse message: ${error}`, "error");
        addMessage({ raw: event.data, error: String(error) }, "received");
      }
    };

    socket.onclose = (event) => {
      clearTimeout(timeout);
      const wasConnected = connectionStatus === "connected";
      setConnectionStatus("disconnected");
      setRosSocket(null);

      let reason = "Unknown reason";
      if (event.code === 1000) {
        reason = "Normal closure";
        addLog(`Connection closed normally: ${event.reason || reason}`);
      } else if (event.code === 1006) {
        reason = "Connection lost/refused";
        setLastError("ROS Bridge server not responding - check if rosbridge_server is running");
        addLog(`❌ Connection failed: ${reason} (${event.code})`, "error");
      } else if (event.code === 1011) {
        reason = "Server error";
        setLastError("ROS Bridge server encountered an error");
        addLog(`❌ Server error: ${reason} (${event.code})`, "error");
      } else {
        reason = `Code ${event.code}`;
        setLastError(`Connection closed with code ${event.code}: ${event.reason || "No reason given"}`);
        addLog(`❌ Connection closed: ${reason}`, "error");
      }

      if (wasConnected) {
        addLog("Connection was working but lost - check network/server");
      }
    };

    socket.onerror = (error) => {
      clearTimeout(timeout);
      setConnectionStatus("error");
      const errorMsg = "WebSocket error occurred";
      setLastError(errorMsg);
      addLog(`❌ ${errorMsg}`, "error");
      console.error("WebSocket error details:", error);
    };
  };

  const disconnectFromRos = () => {
    if (rosSocket) {
      addLog("Manually closing connection");
      rosSocket.close(1000, "Manual disconnect");
    }
  };

  const sendTestMessage = () => {
    if (!rosSocket || connectionStatus !== "connected") {
      addLog("Cannot send message - not connected", "error");
      return;
    }

    try {
      let message: any;

      if (selectedTopic === "/speech" && messageType === "std_msgs/String") {
        message = {
          op: "publish",
          topic: "/speech",
          type: "std_msgs/String",
          msg: { data: testMessage || "Hello from debug page" },
        };
      } else if (selectedTopic === "/cmd_vel") {
        message = {
          op: "publish",
          topic: "/cmd_vel",
          type: "geometry_msgs/Twist",
          msg: {
            linear: { x: 0.1, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 },
          },
        };
      } else {
        // Generic message
        message = {
          op: "publish",
          topic: selectedTopic,
          type: messageType,
          msg: { data: testMessage || "test" },
        };
      }

      rosSocket.send(JSON.stringify(message));
      addMessage(message, "sent");
      addLog(`Sent message to ${selectedTopic}`, "success");
    } catch (error) {
      addLog(`Failed to send message: ${error}`, "error");
    }
  };

  const subscribeToTopic = () => {
    if (!rosSocket || connectionStatus !== "connected") {
      addLog("Cannot subscribe - not connected", "error");
      return;
    }

    const subscribeMsg = {
      op: "subscribe",
      topic: selectedTopic,
      type: messageType,
    };

    rosSocket.send(JSON.stringify(subscribeMsg));
    addMessage(subscribeMsg, "sent");
    addLog(`Subscribed to ${selectedTopic}`, "success");
  };

  const clearLogs = () => {
    setLogs([]);
    setMessages([]);
    addLog("Logs cleared");
  };

  const copyLogs = () => {
    const logText = logs.join("\n");
    navigator.clipboard.writeText(logText);
    addLog("Logs copied to clipboard", "success");
  };

  const getStatusIcon = () => {
    switch (connectionStatus) {
      case "connected":
        return <CheckCircle className="h-4 w-4 text-green-600" />;
      case "connecting":
        return <Wifi className="h-4 w-4 animate-pulse text-blue-600" />;
      case "error":
        return <AlertTriangle className="h-4 w-4 text-red-600" />;
      default:
        return <WifiOff className="h-4 w-4 text-gray-400" />;
    }
  };

  const commonTopics = [
    { topic: "/speech", type: "std_msgs/String" },
    { topic: "/cmd_vel", type: "geometry_msgs/Twist" },
    { topic: "/joint_angles", type: "naoqi_bridge_msgs/JointAnglesWithSpeed" },
    { topic: "/naoqi_driver/joint_states", type: "sensor_msgs/JointState" },
    { topic: "/naoqi_driver/bumper", type: "naoqi_bridge_msgs/Bumper" },
  ];

  return (
    <PageLayout>
      <PageHeader
        title="ROS Bridge WebSocket Debug"
        description="Debug and test WebSocket connection to ROS Bridge server"
      />

      <div className="grid gap-6 md:grid-cols-2">
        {/* Connection Control */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              {getStatusIcon()}
              Connection Control
            </CardTitle>
            <CardDescription>
              Connect to ROS Bridge at {ROS_BRIDGE_URL}
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="flex items-center gap-2">
              <Badge
                variant={
                  connectionStatus === "connected"
                    ? "default"
                    : connectionStatus === "error"
                      ? "destructive"
                      : "outline"
                }
              >
                {connectionStatus.toUpperCase()}
              </Badge>
              <span className="text-sm text-muted-foreground">
                Attempts: {connectionAttempts}
              </span>
            </div>

            {lastError && (
              <Alert variant="destructive">
                <AlertTriangle className="h-4 w-4" />
                <AlertDescription className="text-sm">{lastError}</AlertDescription>
              </Alert>
            )}

            <div className="flex gap-2">
              {connectionStatus !== "connected" ? (
                <Button
                  onClick={connectToRos}
                  disabled={connectionStatus === "connecting"}
                  className="flex-1"
                >
                  <Play className="mr-2 h-4 w-4" />
                  {connectionStatus === "connecting" ? "Connecting..." : "Connect"}
                </Button>
              ) : (
                <Button
                  onClick={disconnectFromRos}
                  variant="outline"
                  className="flex-1"
                >
                  <Square className="mr-2 h-4 w-4" />
                  Disconnect
                </Button>
              )}
            </div>

            <Separator />

            {/* Message Testing */}
            <div className="space-y-3">
              <Label>Test Messages</Label>

              <div className="grid grid-cols-2 gap-2">
                <div>
                  <Label htmlFor="topic" className="text-xs">
                    Topic
                  </Label>
                  <Input
                    id="topic"
                    value={selectedTopic}
                    onChange={(e) => setSelectedTopic(e.target.value)}
                    placeholder="/speech"
                  />
                </div>
                <div>
                  <Label htmlFor="msgType" className="text-xs">
                    Message Type
                  </Label>
                  <Input
                    id="msgType"
                    value={messageType}
                    onChange={(e) => setMessageType(e.target.value)}
                    placeholder="std_msgs/String"
                  />
                </div>
              </div>

              <div>
                <Label htmlFor="testMsg" className="text-xs">
                  Test Message
                </Label>
                <Input
                  id="testMsg"
                  value={testMessage}
                  onChange={(e) => setTestMessage(e.target.value)}
                  placeholder="Hello from debug page"
                />
              </div>

              <div className="flex gap-2">
                <Button
                  onClick={sendTestMessage}
                  disabled={connectionStatus !== "connected"}
                  size="sm"
                  className="flex-1"
                >
                  Publish
                </Button>
                <Button
                  onClick={subscribeToTopic}
                  disabled={connectionStatus !== "connected"}
                  size="sm"
                  variant="outline"
                  className="flex-1"
                >
                  Subscribe
                </Button>
              </div>

              {/* Quick Topic Buttons */}
              <div className="space-y-1">
                <Label className="text-xs">Quick Topics</Label>
                <div className="grid grid-cols-1 gap-1">
                  {commonTopics.map((item) => (
                    <Button
                      key={item.topic}
                      onClick={() => {
                        setSelectedTopic(item.topic);
                        setMessageType(item.type);
                      }}
                      variant="ghost"
                      size="sm"
                      className="justify-start text-xs"
                    >
                      {item.topic}
                    </Button>
                  ))}
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        {/* Connection Logs */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center justify-between">
              Connection Logs
              <div className="flex gap-1">
                <Button onClick={copyLogs} size="sm" variant="ghost">
                  <Copy className="h-4 w-4" />
                </Button>
                <Button onClick={clearLogs} size="sm" variant="ghost">
                  <Trash2 className="h-4 w-4" />
                </Button>
              </div>
            </CardTitle>
            <CardDescription>
              Real-time connection and message logs ({logs.length}/100)
            </CardDescription>
          </CardHeader>
          <CardContent>
            <ScrollArea className="h-64 w-full rounded border p-2">
              <div className="space-y-1 font-mono text-xs">
                {logs.map((log, index) => (
                  <div
                    key={index}
                    className={`${
                      log.includes("ERROR")
                        ? "text-red-600"
                        : log.includes("SUCCESS")
                          ? "text-green-600"
                          : "text-slate-600"
                    }`}
                  >
                    {log}
                  </div>
                ))}
                {logs.length === 0 && (
                  <div className="text-muted-foreground">No logs yet...</div>
                )}
                <div ref={logsEndRef} />
              </div>
            </ScrollArea>
          </CardContent>
        </Card>

        {/* Message Inspector */}
        <Card className="md:col-span-2">
          <CardHeader>
            <CardTitle>Message Inspector</CardTitle>
            <CardDescription>
              Raw WebSocket messages sent and received ({messages.length}/50)
            </CardDescription>
          </CardHeader>
          <CardContent>
            <ScrollArea className="h-64 w-full rounded border p-2">
              <div className="space-y-2">
                {messages.map((msg, index) => (
                  <div
                    key={index}
                    className={`rounded p-2 text-xs ${
                      msg.direction === "sent"
                        ? "bg-blue-50 border-l-2 border-blue-400"
                        : "bg-green-50 border-l-2 border-green-400"
                    }`}
                  >
                    <div className="flex items-center justify-between mb-1">
                      <Badge
                        variant={msg.direction === "sent" ? "default" : "secondary"}
                        className="text-xs"
                      >
                        {msg.direction === "sent" ? "SENT" : "RECEIVED"}
                      </Badge>
                      <span className="text-muted-foreground">{msg.timestamp}</span>
                    </div>
                    <pre className="whitespace-pre-wrap text-xs">
                      {JSON.stringify(msg.data, null, 2)}
                    </pre>
                  </div>
                ))}
                {messages.length === 0 && (
                  <div className="text-center text-muted-foreground py-8">
                    No messages yet. Connect and send a test message to see data here.
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>
            </ScrollArea>
          </CardContent>
        </Card>
      </div>
    </PageLayout>
  );
}
