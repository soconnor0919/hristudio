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
import { Slider } from "~/components/ui/slider";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { PageHeader } from "~/components/ui/page-header";
import { PageLayout } from "~/components/ui/page-layout";
import {
  Play,
  Square,
  Volume2,
  Camera,
  Zap,
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  RotateCcw,
  RotateCw,
  Wifi,
  WifiOff,
  AlertTriangle,
  CheckCircle,
  Activity,
  Battery,
  Eye,
  Hand,
  Footprints,
} from "lucide-react";

interface RosMessage {
  topic: string;
  msg: any;
  type: string;
}

export default function NaoTestPage() {
  const [connectionStatus, setConnectionStatus] = useState<
    "disconnected" | "connecting" | "connected" | "error"
  >("disconnected");
  const [rosSocket, setRosSocket] = useState<WebSocket | null>(null);
  const [robotStatus, setRobotStatus] = useState<any>(null);
  const [jointStates, setJointStates] = useState<any>(null);
  const [speechText, setSpeechText] = useState("");
  const [walkSpeed, setWalkSpeed] = useState([0.1]);
  const [turnSpeed, setTurnSpeed] = useState([0.3]);
  const [headYaw, setHeadYaw] = useState([0]);
  const [headPitch, setHeadPitch] = useState([0]);
  const [logs, setLogs] = useState<string[]>([]);
  const [sensorData, setSensorData] = useState<any>({});
  const logsEndRef = useRef<HTMLDivElement>(null);

  const ROS_BRIDGE_URL =
    process.env.NEXT_PUBLIC_ROS_BRIDGE_URL || "ws://localhost:9090";

  const addLog = (message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    setLogs((prev) => [...prev.slice(-49), `[${timestamp}] ${message}`]);
  };

  useEffect(() => {
    logsEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [logs]);

  const connectToRos = () => {
    if (rosSocket?.readyState === WebSocket.OPEN) return;

    setConnectionStatus("connecting");
    addLog("Connecting to ROS bridge...");

    const socket = new WebSocket(ROS_BRIDGE_URL);

    socket.onopen = () => {
      setConnectionStatus("connected");
      setRosSocket(socket);
      addLog("Connected to ROS bridge successfully");

      // Subscribe to robot topics
      subscribeToTopics(socket);
    };

    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        handleRosMessage(data);
      } catch (error) {
        console.error("Error parsing ROS message:", error);
      }
    };

    socket.onclose = () => {
      setConnectionStatus("disconnected");
      setRosSocket(null);
      addLog("Disconnected from ROS bridge");
    };

    socket.onerror = () => {
      setConnectionStatus("error");
      addLog("Error connecting to ROS bridge");
    };
  };

  const disconnectFromRos = () => {
    if (rosSocket) {
      rosSocket.close();
      setRosSocket(null);
      setConnectionStatus("disconnected");
      addLog("Manually disconnected from ROS bridge");
    }
  };

  const subscribeToTopics = (socket: WebSocket) => {
    const topics = [
      { topic: "/naoqi_driver/joint_states", type: "sensor_msgs/JointState" },
      { topic: "/naoqi_driver/info", type: "naoqi_bridge_msgs/StringStamped" },
      { topic: "/naoqi_driver/bumper", type: "naoqi_bridge_msgs/Bumper" },
      {
        topic: "/naoqi_driver/hand_touch",
        type: "naoqi_bridge_msgs/HandTouch",
      },
      {
        topic: "/naoqi_driver/head_touch",
        type: "naoqi_bridge_msgs/HeadTouch",
      },
      { topic: "/naoqi_driver/sonar/left", type: "sensor_msgs/Range" },
      { topic: "/naoqi_driver/sonar/right", type: "sensor_msgs/Range" },
    ];

    topics.forEach(({ topic, type }) => {
      const subscribeMsg = {
        op: "subscribe",
        topic,
        type,
      };
      socket.send(JSON.stringify(subscribeMsg));
      addLog(`Subscribed to ${topic}`);
    });
  };

  const handleRosMessage = (data: any) => {
    if (data.topic === "/naoqi_driver/joint_states") {
      setJointStates(data.msg);
    } else if (data.topic === "/naoqi_driver/info") {
      setRobotStatus(data.msg);
    } else if (
      data.topic?.includes("bumper") ||
      data.topic?.includes("touch") ||
      data.topic?.includes("sonar")
    ) {
      setSensorData((prev: any) => ({
        ...prev,
        [data.topic]: data.msg,
      }));
    }
  };

  const publishMessage = (topic: string, type: string, msg: any) => {
    if (!rosSocket || rosSocket.readyState !== WebSocket.OPEN) {
      addLog("Error: Not connected to ROS bridge");
      return;
    }

    const rosMsg = {
      op: "publish",
      topic,
      type,
      msg,
    };

    rosSocket.send(JSON.stringify(rosMsg));
    addLog(`Published to ${topic}: ${JSON.stringify(msg)}`);
  };

  const sayText = () => {
    if (!speechText.trim()) return;

    publishMessage("/speech", "std_msgs/String", {
      data: speechText,
    });
    setSpeechText("");
  };

  const walkForward = () => {
    publishMessage("/cmd_vel", "geometry_msgs/Twist", {
      linear: { x: walkSpeed[0] ?? 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
  };

  const walkBackward = () => {
    publishMessage("/cmd_vel", "geometry_msgs/Twist", {
      linear: { x: -(walkSpeed[0] ?? 0), y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
  };

  const turnLeft = () => {
    publishMessage("/cmd_vel", "geometry_msgs/Twist", {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: turnSpeed[0] ?? 0 },
    });
  };

  const turnRight = () => {
    publishMessage("/cmd_vel", "geometry_msgs/Twist", {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: -(turnSpeed[0] ?? 0) },
    });
  };

  const stopMovement = () => {
    publishMessage("/cmd_vel", "geometry_msgs/Twist", {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
  };

  const moveHead = () => {
    publishMessage("/joint_angles", "naoqi_bridge_msgs/JointAnglesWithSpeed", {
      joint_names: ["HeadYaw", "HeadPitch"],
      joint_angles: [headYaw[0] ?? 0, headPitch[0] ?? 0],
      speed: 0.3,
    });
  };

  const getConnectionStatusIcon = () => {
    switch (connectionStatus) {
      case "connected":
        return <Wifi className="h-4 w-4 text-green-500" />;
      case "connecting":
        return <Activity className="h-4 w-4 animate-spin text-yellow-500" />;
      case "error":
        return <AlertTriangle className="h-4 w-4 text-red-500" />;
      default:
        return <WifiOff className="h-4 w-4 text-gray-500" />;
    }
  };

  const getConnectionStatusBadge = () => {
    const variants = {
      connected: "default",
      connecting: "secondary",
      error: "destructive",
      disconnected: "outline",
    } as const;

    return (
      <Badge
        variant={variants[connectionStatus]}
        className="flex items-center gap-1"
      >
        {getConnectionStatusIcon()}
        {connectionStatus.charAt(0).toUpperCase() + connectionStatus.slice(1)}
      </Badge>
    );
  };

  return (
    <PageLayout>
      <PageHeader
        title="NAO Robot Test Console"
        description="Test and control your NAO6 robot through ROS bridge"
      />

      <div className="space-y-6">
        {/* Connection Status */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center justify-between">
              ROS Bridge Connection
              {getConnectionStatusBadge()}
            </CardTitle>
            <CardDescription>
              Connect to ROS bridge at {ROS_BRIDGE_URL}
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="flex gap-2">
              {connectionStatus === "connected" ? (
                <Button onClick={disconnectFromRos} variant="destructive">
                  <WifiOff className="mr-2 h-4 w-4" />
                  Disconnect
                </Button>
              ) : (
                <Button
                  onClick={connectToRos}
                  disabled={connectionStatus === "connecting"}
                >
                  <Wifi className="mr-2 h-4 w-4" />
                  {connectionStatus === "connecting"
                    ? "Connecting..."
                    : "Connect"}
                </Button>
              )}
            </div>
          </CardContent>
        </Card>

        {connectionStatus === "connected" && (
          <Tabs defaultValue="control" className="space-y-4">
            <TabsList>
              <TabsTrigger value="control">Robot Control</TabsTrigger>
              <TabsTrigger value="sensors">Sensor Data</TabsTrigger>
              <TabsTrigger value="status">Robot Status</TabsTrigger>
              <TabsTrigger value="logs">Logs</TabsTrigger>
            </TabsList>

            <TabsContent value="control" className="space-y-4">
              <div className="grid grid-cols-1 gap-4 md:grid-cols-2">
                {/* Speech Control */}
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Volume2 className="h-4 w-4" />
                      Speech
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="space-y-2">
                      <Label htmlFor="speech">Text to Speech</Label>
                      <Textarea
                        id="speech"
                        placeholder="Enter text for NAO to say..."
                        value={speechText}
                        onChange={(e) => setSpeechText(e.target.value)}
                        onKeyDown={(e) =>
                          e.key === "Enter" &&
                          !e.shiftKey &&
                          (e.preventDefault(), sayText())
                        }
                      />
                    </div>
                    <Button
                      onClick={sayText}
                      disabled={!speechText.trim()}
                      className="w-full"
                    >
                      <Play className="mr-2 h-4 w-4" />
                      Say Text
                    </Button>
                  </CardContent>
                </Card>

                {/* Movement Control */}
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Footprints className="h-4 w-4" />
                      Movement
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="space-y-2">
                      <Label>Walk Speed: {(walkSpeed[0] ?? 0).toFixed(2)} m/s</Label>
                      <Slider
                        value={walkSpeed}
                        onValueChange={setWalkSpeed}
                        max={0.5}
                        min={0.05}
                        step={0.05}
                      />
                    </div>
                    <div className="space-y-2">
                      <Label>Turn Speed: {(turnSpeed[0] ?? 0).toFixed(2)} rad/s</Label>
                      <Slider
                        value={turnSpeed}
                        onValueChange={setTurnSpeed}
                        max={1.0}
                        min={0.1}
                        step={0.1}
                      />
                    </div>
                    <div className="grid grid-cols-3 gap-2">
                      <Button variant="outline" onClick={walkForward}>
                        <ArrowUp className="h-4 w-4" />
                      </Button>
                      <Button variant="destructive" onClick={stopMovement}>
                        <Square className="h-4 w-4" />
                      </Button>
                      <Button variant="outline" onClick={walkBackward}>
                        <ArrowDown className="h-4 w-4" />
                      </Button>
                      <Button variant="outline" onClick={turnLeft}>
                        <RotateCcw className="h-4 w-4" />
                      </Button>
                      <div></div>
                      <Button variant="outline" onClick={turnRight}>
                        <RotateCw className="h-4 w-4" />
                      </Button>
                    </div>
                  </CardContent>
                </Card>

                {/* Head Control */}
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Eye className="h-4 w-4" />
                      Head Control
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="space-y-2">
                      <Label>Head Yaw: {(headYaw[0] ?? 0).toFixed(2)} rad</Label>
                      <Slider
                        value={headYaw}
                        onValueChange={setHeadYaw}
                        max={2.09}
                        min={-2.09}
                        step={0.1}
                      />
                    </div>
                    <div className="space-y-2">
                      <Label>Head Pitch: {(headPitch[0] ?? 0).toFixed(2)} rad</Label>
                      <Slider
                        value={headPitch}
                        onValueChange={setHeadPitch}
                        max={0.51}
                        min={-0.67}
                        step={0.1}
                      />
                    </div>
                    <Button onClick={moveHead} className="w-full">
                      Move Head
                    </Button>
                  </CardContent>
                </Card>

                {/* Emergency Stop */}
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2 text-red-600">
                      <AlertTriangle className="h-4 w-4" />
                      Emergency
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <Button
                      onClick={stopMovement}
                      variant="destructive"
                      size="lg"
                      className="w-full"
                    >
                      <Square className="mr-2 h-4 w-4" />
                      EMERGENCY STOP
                    </Button>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            <TabsContent value="sensors" className="space-y-4">
              <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3">
                {Object.entries(sensorData).map(([topic, data]) => (
                  <Card key={topic}>
                    <CardHeader>
                      <CardTitle className="text-sm">
                        {topic
                          .split("/")
                          .pop()
                          ?.replace(/_/g, " ")
                          .toUpperCase()}
                      </CardTitle>
                    </CardHeader>
                    <CardContent>
                      <pre className="max-h-32 overflow-auto rounded bg-gray-100 p-2 text-xs">
                        {JSON.stringify(data, null, 2)}
                      </pre>
                    </CardContent>
                  </Card>
                ))}
                {Object.keys(sensorData).length === 0 && (
                  <Alert>
                    <AlertTriangle className="h-4 w-4" />
                    <AlertDescription>
                      No sensor data received yet. Make sure the robot is
                      connected and publishing data.
                    </AlertDescription>
                  </Alert>
                )}
              </div>
            </TabsContent>

            <TabsContent value="status" className="space-y-4">
              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center gap-2">
                    <Activity className="h-4 w-4" />
                    Robot Status
                  </CardTitle>
                </CardHeader>
                <CardContent>
                  {robotStatus ? (
                    <div className="space-y-4">
                      <div className="grid grid-cols-2 gap-4">
                        <div>
                          <Label>Robot Info</Label>
                          <pre className="mt-1 rounded bg-gray-100 p-2 text-xs">
                            {JSON.stringify(robotStatus, null, 2)}
                          </pre>
                        </div>
                        {jointStates && (
                          <div>
                            <Label>Joint States</Label>
                            <div className="mt-1 max-h-64 overflow-auto rounded bg-gray-100 p-2 text-xs">
                              <div>Joints: {jointStates.name?.length || 0}</div>
                              <div>
                                Last Update: {new Date().toLocaleTimeString()}
                              </div>
                              {jointStates.name
                                ?.slice(0, 10)
                                .map((name: string, i: number) => (
                                  <div
                                    key={name}
                                    className="flex justify-between"
                                  >
                                    <span>{name}:</span>
                                    <span>
                                      {jointStates.position?.[i]?.toFixed(3) ||
                                        "N/A"}
                                    </span>
                                  </div>
                                ))}
                              {(jointStates.name?.length || 0) > 10 && (
                                <div className="text-gray-500">
                                  ... and {(jointStates.name?.length || 0) - 10}{" "}
                                  more
                                </div>
                              )}
                            </div>
                          </div>
                        )}
                      </div>
                    </div>
                  ) : (
                    <Alert>
                      <AlertTriangle className="h-4 w-4" />
                      <AlertDescription>
                        No robot status data received. Check that the NAO robot
                        is connected and the naoqi_driver is running.
                      </AlertDescription>
                    </Alert>
                  )}
                </CardContent>
              </Card>
            </TabsContent>

            <TabsContent value="logs" className="space-y-4">
              <Card>
                <CardHeader>
                  <CardTitle>Communication Logs</CardTitle>
                  <CardDescription>
                    Real-time log of ROS bridge communication
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="h-64 overflow-auto rounded bg-black p-4 font-mono text-xs text-green-400">
                    {logs.map((log, index) => (
                      <div key={index}>{log}</div>
                    ))}
                    <div ref={logsEndRef} />
                  </div>
                  <Button
                    onClick={() => setLogs([])}
                    variant="outline"
                    className="mt-2"
                  >
                    Clear Logs
                  </Button>
                </CardContent>
              </Card>
            </TabsContent>
          </Tabs>
        )}

        {connectionStatus !== "connected" && (
          <Alert>
            <AlertTriangle className="h-4 w-4" />
            <AlertDescription>
              Connect to ROS bridge to start controlling the robot. Make sure
              the NAO integration is running:
              <br />
              <code className="mt-2 block rounded bg-gray-100 p-2">
                ros2 launch nao6_hristudio.launch.py nao_ip:=nao.local
                password:=robolab
              </code>
            </AlertDescription>
          </Alert>
        )}
      </div>
    </PageLayout>
  );
}
