"use client";

import {
  Activity,
  AlertTriangle,
  Battery,
  BatteryLow,
  Bot,
  CheckCircle,
  Clock,
  RefreshCw,
  Signal,
  SignalHigh,
  SignalLow,
  SignalMedium,
  WifiOff,
} from "lucide-react";
import { useEffect, useState } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";

import { Progress } from "~/components/ui/progress";

interface RobotStatusProps {
  trialId: string;
}

interface RobotStatus {
  id: string;
  name: string;
  connectionStatus: "connected" | "disconnected" | "connecting" | "error";
  batteryLevel?: number;
  signalStrength?: number;
  currentMode: string;
  lastHeartbeat?: Date;
  errorMessage?: string;
  capabilities: string[];
  communicationProtocol: string;
  isMoving: boolean;
  position?: {
    x: number;
    y: number;
    z?: number;
    orientation?: number;
  };
  sensors?: Record<string, string>;
}

export function RobotStatus({ trialId: _trialId }: RobotStatusProps) {
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date());
  const [refreshing, setRefreshing] = useState(false);

  // Mock robot status - in real implementation, this would come from API/WebSocket
  useEffect(() => {
    // Simulate robot status updates
    const mockStatus: RobotStatus = {
      id: "robot_001",
      name: "TurtleBot3 Burger",
      connectionStatus: "connected",
      batteryLevel: 85,
      signalStrength: 75,
      currentMode: "autonomous_navigation",
      lastHeartbeat: new Date(),
      capabilities: ["navigation", "manipulation", "speech", "vision"],
      communicationProtocol: "ROS2",
      isMoving: false,
      position: {
        x: 1.2,
        y: 0.8,
        orientation: 45,
      },
      sensors: {
        lidar: "operational",
        camera: "operational",
        imu: "operational",
        odometry: "operational",
      },
    };

    setRobotStatus(mockStatus);

    // Simulate periodic updates
    const interval = setInterval(() => {
      setRobotStatus((prev) => {
        if (!prev) return prev;
        return {
          ...prev,
          batteryLevel: Math.max(
            0,
            (prev.batteryLevel ?? 0) - Math.random() * 0.5,
          ),
          signalStrength: Math.max(
            0,
            Math.min(
              100,
              (prev.signalStrength ?? 0) + (Math.random() - 0.5) * 10,
            ),
          ),
          lastHeartbeat: new Date(),
          position: prev.position
            ? {
                ...prev.position,
                x: prev.position.x + (Math.random() - 0.5) * 0.1,
                y: prev.position.y + (Math.random() - 0.5) * 0.1,
              }
            : undefined,
        };
      });
      setLastUpdate(new Date());
    }, 3000);

    return () => clearInterval(interval);
  }, []);

  const getConnectionStatusConfig = (status: string) => {
    switch (status) {
      case "connected":
        return {
          icon: CheckCircle,
          color: "text-green-600",
          bgColor: "bg-green-100",
          label: "Connected",
        };
      case "connecting":
        return {
          icon: RefreshCw,
          color: "text-blue-600",
          bgColor: "bg-blue-100",
          label: "Connecting",
        };
      case "disconnected":
        return {
          icon: WifiOff,
          color: "text-gray-600",
          bgColor: "bg-gray-100",
          label: "Disconnected",
        };
      case "error":
        return {
          icon: AlertTriangle,
          color: "text-red-600",
          bgColor: "bg-red-100",
          label: "Error",
        };
      default:
        return {
          icon: WifiOff,
          color: "text-gray-600",
          bgColor: "bg-gray-100",
          label: "Unknown",
        };
    }
  };

  const getSignalIcon = (strength: number) => {
    if (strength >= 75) return SignalHigh;
    if (strength >= 50) return SignalMedium;
    if (strength >= 25) return SignalLow;
    return Signal;
  };

  const getBatteryIcon = (level: number) => {
    return level <= 20 ? BatteryLow : Battery;
  };

  const handleRefreshStatus = async () => {
    setRefreshing(true);
    // Simulate API call
    setTimeout(() => {
      setRefreshing(false);
      setLastUpdate(new Date());
    }, 1000);
  };

  if (!robotStatus) {
    return (
      <div className="space-y-4">
        <div className="rounded-lg border p-4 text-center">
          <div className="text-slate-500">
            <Bot className="mx-auto mb-2 h-8 w-8 opacity-50" />
            <p className="text-sm">No robot connected</p>
          </div>
        </div>
      </div>
    );
  }

  const statusConfig = getConnectionStatusConfig(robotStatus.connectionStatus);
  const StatusIcon = statusConfig.icon;
  const SignalIcon = getSignalIcon(robotStatus.signalStrength ?? 0);
  const BatteryIcon = getBatteryIcon(robotStatus.batteryLevel ?? 0);

  return (
    <div className="space-y-4">
      <div className="flex items-center justify-end">
        <Button
          variant="ghost"
          size="sm"
          onClick={handleRefreshStatus}
          disabled={refreshing}
        >
          <RefreshCw
            className={`h-3 w-3 ${refreshing ? "animate-spin" : ""}`}
          />
        </Button>
      </div>

      {/* Main Status Card */}
      <div className="rounded-lg border p-4">
        <div className="space-y-3">
          {/* Robot Info */}
          <div className="flex items-center justify-between">
            <div className="font-medium text-slate-900">{robotStatus.name}</div>
            <Badge
              className={`${statusConfig.bgColor} ${statusConfig.color}`}
              variant="secondary"
            >
              <StatusIcon className="mr-1 h-3 w-3" />
              {statusConfig.label}
            </Badge>
          </div>

          {/* Connection Details */}
          <div className="text-sm text-slate-600">
            Protocol: {robotStatus.communicationProtocol}
          </div>

          {/* Status Indicators */}
          <div className="grid grid-cols-2 gap-3">
            {/* Battery */}
            {robotStatus.batteryLevel !== undefined && (
              <div className="space-y-1">
                <div className="flex items-center space-x-1 text-xs text-slate-600">
                  <BatteryIcon className="h-3 w-3" />
                  <span>Battery</span>
                </div>
                <div className="flex items-center space-x-2">
                  <Progress
                    value={robotStatus.batteryLevel}
                    className="h-1.5 flex-1"
                  />
                  <span className="w-8 text-xs font-medium">
                    {Math.round(robotStatus.batteryLevel)}%
                  </span>
                </div>
              </div>
            )}

            {/* Signal Strength */}
            {robotStatus.signalStrength !== undefined && (
              <div className="space-y-1">
                <div className="flex items-center space-x-1 text-xs text-slate-600">
                  <SignalIcon className="h-3 w-3" />
                  <span>Signal</span>
                </div>
                <div className="flex items-center space-x-2">
                  <Progress
                    value={robotStatus.signalStrength}
                    className="h-1.5 flex-1"
                  />
                  <span className="w-8 text-xs font-medium">
                    {Math.round(robotStatus.signalStrength)}%
                  </span>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Current Mode */}
      <div className="rounded-lg border p-3">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <Activity className="h-3 w-3 text-slate-600" />
            <span className="text-sm text-slate-600">Mode:</span>
          </div>
          <Badge variant="outline" className="text-xs">
            {robotStatus.currentMode
              .replace(/_/g, " ")
              .replace(/\b\w/g, (l) => l.toUpperCase())}
          </Badge>
        </div>
        {robotStatus.isMoving && (
          <div className="mt-2 flex items-center space-x-1 text-xs">
            <div className="h-1.5 w-1.5 animate-pulse rounded-full"></div>
            <span>Robot is moving</span>
          </div>
        )}
      </div>

      {/* Position Info */}
      {robotStatus.position && (
        <div className="rounded-lg border p-4">
          <div className="mb-3 text-sm font-medium text-slate-700">
            Position
          </div>
          <div>
            <div className="grid grid-cols-2 gap-2 text-xs">
              <div className="flex justify-between">
                <span className="text-slate-600">X:</span>
                <span className="font-mono">
                  {robotStatus.position.x.toFixed(2)}m
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-600">Y:</span>
                <span className="font-mono">
                  {robotStatus.position.y.toFixed(2)}m
                </span>
              </div>
              {robotStatus.position.orientation !== undefined && (
                <div className="col-span-2 flex justify-between">
                  <span className="text-slate-600">Orientation:</span>
                  <span className="font-mono">
                    {Math.round(robotStatus.position.orientation)}°
                  </span>
                </div>
              )}
            </div>
          </div>
        </div>
      )}

      {/* Sensors Status */}
      {robotStatus.sensors && (
        <div className="rounded-lg border p-4">
          <div className="mb-3 text-sm font-medium text-slate-700">Sensors</div>
          <div>
            <div className="space-y-1">
              {Object.entries(robotStatus.sensors).map(([sensor, status]) => (
                <div
                  key={sensor}
                  className="flex items-center justify-between text-xs"
                >
                  <span className="text-slate-600 capitalize">{sensor}:</span>
                  <Badge variant="outline" className="text-xs">
                    {status}
                  </Badge>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Error Alert */}
      {robotStatus.errorMessage && (
        <Alert variant="destructive">
          <AlertTriangle className="h-4 w-4" />
          <AlertDescription className="text-sm">
            {robotStatus.errorMessage}
          </AlertDescription>
        </Alert>
      )}

      {/* Last Update */}
      <div className="flex items-center space-x-1 text-xs text-slate-500">
        <Clock className="h-3 w-3" />
        <span>Last update: {lastUpdate.toLocaleTimeString()}</span>
      </div>
    </div>
  );
}
