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
import { Button } from "~/components/ui/button";
import { WebcamPanel } from "./WebcamPanel";

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
  readOnly?: boolean;
}

const WizardMonitoringPanel = function WizardMonitoringPanel({
  rosConnected,
  rosConnecting,
  rosError,
  robotStatus,
  connectRos,
  disconnectRos,
  executeRosAction,
  readOnly = false,
}: WizardMonitoringPanelProps) {
  return (
    <div className="flex h-full flex-col gap-2 p-2">
      {/* Camera View - Always Visible */}
      <div className="shrink-0 bg-muted/30 rounded-lg overflow-hidden border shadow-sm h-48 sm:h-56 relative group">
        <WebcamPanel readOnly={readOnly} />
      </div>

      {/* Robot Controls - Scrollable */}
      <div className="flex-1 min-h-0 bg-background rounded-lg border shadow-sm overflow-hidden flex flex-col">
        <div className="px-3 py-2 border-b bg-muted/30 flex items-center gap-2">
          <Bot className="h-4 w-4 text-muted-foreground" />
          <span className="text-xs font-semibold text-muted-foreground uppercase tracking-wider">Robot Control</span>
        </div>
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
                    <Badge variant="outline" className="text-gray-500 border-gray-300 text-xs text-muted-foreground w-auto px-1.5 py-0">Offline</Badge>
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
                    disabled={rosConnecting || rosConnected || readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    className="flex-1 rounded-md border border-input bg-background px-2 py-1 text-xs ring-offset-background placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring disabled:opacity-50"
                    disabled={readOnly}
                    onKeyDown={(e) => {
                      if (e.key === "Enter" && e.currentTarget.value.trim() && !readOnly) {
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
                    disabled={readOnly}
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
                    disabled={readOnly}
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
                    disabled={readOnly}
                  >
                    Say Ready
                  </Button>
                </div>
              </div>
            )}
          </div>
        </ScrollArea>
      </div>
    </div>
  );
};

export { WizardMonitoringPanel };
