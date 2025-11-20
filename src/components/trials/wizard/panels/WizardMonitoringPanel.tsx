"use client";

import React from "react";
import {
  Bot,
  Power,
  PowerOff,
  AlertCircle,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Progress } from "~/components/ui/progress";
import { Button } from "~/components/ui/button";

interface WizardMonitoringPanelProps {
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

const WizardMonitoringPanel = function WizardMonitoringPanel({
  rosConnected,
  rosConnecting,
  rosError,
  robotStatus,
  connectRos,
  disconnectRos,
  executeRosAction,
}: WizardMonitoringPanelProps) {
  return (
    <div className="flex h-full flex-col">
      {/* Header */}
      <div className="flex items-center justify-between border-b p-3">
        <h2 className="text-sm font-semibold">Robot Control</h2>
      </div>

      {/* Robot Status and Controls */}
      <ScrollArea className="flex-1">
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

            {!rosConnected && !rosConnecting && (
              <div className="mt-4">
                <Alert>
                  <AlertCircle className="h-4 w-4" />
                  <AlertDescription className="text-xs">
                    Connect to ROS bridge for live robot monitoring and
                    control.
                  </AlertDescription>
                </Alert>
              </div>
            )}
          </div>

          <Separator />

          {/* Movement Controls */}
          {rosConnected && (
            <div className="space-y-2">
              <div className="text-sm font-medium">Movement</div>
              <div className="grid grid-cols-3 gap-2">
                {/* Row 1: Turn Left, Forward, Turn Right */}
                <Button
                  size="sm"
                  variant="outline"
                  className="text-xs"
                  onClick={() => {
                    executeRosAction("nao6-ros2", "turn_left", {
                      speed: 0.3,
                    }).catch(console.error);
                  }}
                >
                  ↺ Turn L
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  className="text-xs"
                  onClick={() => {
                    executeRosAction("nao6-ros2", "walk_forward", {
                      speed: 0.5,
                    }).catch(console.error);
                  }}
                >
                  ↑ Forward
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  className="text-xs"
                  onClick={() => {
                    executeRosAction("nao6-ros2", "turn_right", {
                      speed: 0.3,
                    }).catch(console.error);
                  }}
                >
                  Turn R ↻
                </Button>

                {/* Row 2: Left, Stop, Right */}
                <Button
                  size="sm"
                  variant="outline"
                  className="text-xs"
                  onClick={() => {
                    executeRosAction("nao6-ros2", "strafe_left", {
                      speed: 0.3,
                    }).catch(console.error);
                  }}
                >
                  ← Left
                </Button>
                <Button
                  size="sm"
                  variant="destructive"
                  className="text-xs"
                  onClick={() => {
                    executeRosAction("nao6-ros2", "emergency_stop", {}).catch(
                      console.error,
                    );
                  }}
                >
                  ■ Stop
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  className="text-xs"
                  onClick={() => {
                    executeRosAction("nao6-ros2", "strafe_right", {
                      speed: 0.3,
                    }).catch(console.error);
                  }}
                >
                  Right →
                </Button>

                {/* Row 3: Empty, Back, Empty */}
                <div></div>
                <Button
                  size="sm"
                  variant="outline"
                  className="text-xs"
                  onClick={() => {
                    executeRosAction("nao6-ros2", "walk_backward", {
                      speed: 0.3,
                    }).catch(console.error);
                  }}
                >
                  ↓ Back
                </Button>
                <div></div>
              </div>
            </div>
          )}

          <Separator />

          {/* Quick Actions */}
          {rosConnected && (
            <div className="space-y-2">
              <div className="text-sm font-medium">Quick Actions</div>

              {/* TTS Input */}
              <div className="flex gap-2">
                <input
                  type="text"
                  placeholder="Type text to speak..."
                  className="flex-1 rounded-md border border-input bg-background px-2 py-1 text-xs ring-offset-background placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring"
                  onKeyDown={(e) => {
                    if (e.key === "Enter" && e.currentTarget.value.trim()) {
                      executeRosAction("nao6-ros2", "say_text", {
                        text: e.currentTarget.value.trim(),
                      }).catch(console.error);
                      e.currentTarget.value = "";
                    }
                  }}
                />
                <Button
                  size="sm"
                  variant="outline"
                  className="text-xs"
                  onClick={(e) => {
                    const input = e.currentTarget.previousElementSibling as HTMLInputElement;
                    if (input?.value.trim()) {
                      executeRosAction("nao6-ros2", "say_text", {
                        text: input.value.trim(),
                      }).catch(console.error);
                      input.value = "";
                    }
                  }}
                >
                  Say
                </Button>
              </div>

              {/* Preset Actions */}
              <div className="grid grid-cols-2 gap-2">
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
                        text: "I am ready!",
                      }).catch(console.error);
                    }
                  }}
                >
                  Say Ready
                </Button>
              </div>
            </div>
          )}
        </div>
      </ScrollArea>
    </div>
  );
};

export { WizardMonitoringPanel };
