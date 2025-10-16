"use client";

import React from "react";
import {
  Bot,
  User,
  Activity,
  Settings,
  Wifi,
  WifiOff,
  AlertCircle,
  CheckCircle,
  Zap,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Alert, AlertDescription } from "~/components/ui/alert";

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

interface MonitoringPanelProps {
  trial: TrialData;
  trialEvents: TrialEvent[];
  isConnected: boolean;
  wsError?: string;
}

export function MonitoringPanel({
  trial,
  trialEvents,
  isConnected,
  wsError,
}: MonitoringPanelProps) {
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
    <div className="flex h-full flex-col space-y-4 p-4">
      {/* Connection Status */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Settings className="h-4 w-4" />
            Connection Status
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              {isConnected ? (
                <Wifi className="h-4 w-4 text-green-600" />
              ) : (
                <WifiOff className="h-4 w-4 text-orange-600" />
              )}
              <span className="text-sm">WebSocket</span>
            </div>
            <Badge
              variant={isConnected ? "default" : "secondary"}
              className="text-xs"
            >
              {isConnected ? "Connected" : "Offline"}
            </Badge>
          </div>

          {wsError && (
            <Alert variant="destructive" className="mt-2">
              <AlertCircle className="h-4 w-4" />
              <AlertDescription className="text-xs">{wsError}</AlertDescription>
            </Alert>
          )}

          <Separator />

          <div className="text-muted-foreground space-y-2 text-xs">
            <div className="flex justify-between">
              <span>Trial ID</span>
              <span className="font-mono">{trial.id.slice(-8)}</span>
            </div>
            <div className="flex justify-between">
              <span>Session</span>
              <span>#{trial.sessionNumber}</span>
            </div>
            {trial.startedAt && (
              <div className="flex justify-between">
                <span>Started</span>
                <span>{formatTimestamp(new Date(trial.startedAt))}</span>
              </div>
            )}
          </div>
        </CardContent>
      </Card>

      {/* Robot Status */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Bot className="h-4 w-4" />
            Robot Status
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="flex items-center justify-between">
            <span className="text-sm">Status</span>
            <Badge variant="outline" className="text-xs">
              {isConnected ? "Ready" : "Unknown"}
            </Badge>
          </div>

          <div className="flex items-center justify-between">
            <span className="text-sm">Battery</span>
            <span className="text-muted-foreground text-sm">--</span>
          </div>

          <div className="flex items-center justify-between">
            <span className="text-sm">Position</span>
            <span className="text-muted-foreground text-sm">--</span>
          </div>

          <Separator />

          <div className="bg-muted/50 text-muted-foreground rounded-lg p-2 text-center text-xs">
            Robot monitoring requires WebSocket connection
          </div>
        </CardContent>
      </Card>

      {/* Participant Info */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <User className="h-4 w-4" />
            Participant
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-2">
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Code</span>
              <span className="font-mono">
                {trial.participant.participantCode}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Session</span>
              <span>#{trial.sessionNumber}</span>
            </div>
            {trial.participant.demographics && (
              <div className="flex justify-between">
                <span className="text-muted-foreground">Demographics</span>
                <span className="text-xs">
                  {Object.keys(trial.participant.demographics).length} fields
                </span>
              </div>
            )}
          </div>
        </CardContent>
      </Card>

      {/* Live Events */}
      <Card className="min-h-0 flex-1">
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Activity className="h-4 w-4" />
            Live Events
            {trialEvents.length > 0 && (
              <Badge variant="secondary" className="ml-auto text-xs">
                {trialEvents.length}
              </Badge>
            )}
          </CardTitle>
        </CardHeader>
        <CardContent className="h-full min-h-0 pb-2">
          <ScrollArea className="h-full">
            {trialEvents.length === 0 ? (
              <div className="text-muted-foreground flex h-32 items-center justify-center text-sm">
                No events yet
              </div>
            ) : (
              <div className="space-y-3">
                {trialEvents
                  .slice()
                  .reverse()
                  .map((event, index) => {
                    const EventIcon = getEventIcon(event.type);
                    const eventColor = getEventColor(event.type);

                    return (
                      <div
                        key={`${event.timestamp.getTime()}-${index}`}
                        className="border-border/50 flex items-start gap-2 rounded-lg border p-2 text-xs"
                      >
                        <div className={`mt-0.5 ${eventColor}`}>
                          <EventIcon className="h-3 w-3" />
                        </div>
                        <div className="min-w-0 flex-1">
                          <div className="font-medium capitalize">
                            {event.type.replace(/_/g, " ")}
                          </div>
                          {event.message && (
                            <div className="text-muted-foreground mt-1">
                              {event.message}
                            </div>
                          )}
                          <div className="text-muted-foreground mt-1">
                            {formatTimestamp(event.timestamp)}
                          </div>
                        </div>
                      </div>
                    );
                  })}
              </div>
            )}
          </ScrollArea>
        </CardContent>
      </Card>

      {/* System Info */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Zap className="h-4 w-4" />
            System
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="text-muted-foreground space-y-2 text-xs">
            <div className="flex justify-between">
              <span>Experiment</span>
              <span
                className="ml-2 max-w-24 truncate"
                title={trial.experiment.name}
              >
                {trial.experiment.name}
              </span>
            </div>
            <div className="flex justify-between">
              <span>Study ID</span>
              <span className="font-mono">
                {trial.experiment.studyId.slice(-8)}
              </span>
            </div>
            <div className="flex justify-between">
              <span>Platform</span>
              <span>HRIStudio</span>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
