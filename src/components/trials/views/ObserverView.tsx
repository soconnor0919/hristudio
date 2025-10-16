"use client";

import React from "react";
import {
  Eye,
  Clock,
  Play,
  CheckCircle,
  AlertCircle,
  User,
  Bot,
  Activity,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";

import { ScrollArea } from "~/components/ui/scroll-area";
import { PanelsContainer } from "~/components/experiments/designer/layout/PanelsContainer";

interface TrialData {
  id: string;
  status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
  scheduledAt: Date | null;
  startedAt: Date | null;
  completedAt: Date | null;
  duration: number | null;
  sessionNumber: number | null;
  notes: string | null;
  metadata: Record<string, unknown> | null;
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

interface ObserverViewProps {
  trial: TrialData;
}

export function ObserverView({ trial }: ObserverViewProps) {
  const getStatusConfig = (status: string) => {
    switch (status) {
      case "scheduled":
        return { variant: "outline" as const, color: "blue", icon: Clock };
      case "in_progress":
        return { variant: "default" as const, color: "green", icon: Play };
      case "completed":
        return {
          variant: "secondary" as const,
          color: "gray",
          icon: CheckCircle,
        };
      case "aborted":
        return {
          variant: "destructive" as const,
          color: "orange",
          icon: AlertCircle,
        };
      case "failed":
        return {
          variant: "destructive" as const,
          color: "red",
          icon: AlertCircle,
        };
      default:
        return { variant: "outline" as const, color: "gray", icon: Clock };
    }
  };

  const statusConfig = getStatusConfig(trial.status);
  const StatusIcon = statusConfig.icon;

  const formatElapsedTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  const leftPanel = (
    <div className="flex h-full flex-col space-y-4 p-4">
      {/* Trial Overview */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Eye className="h-4 w-4" />
            Trial Overview
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="flex items-center justify-between">
            <span className="text-muted-foreground text-sm">Status</span>
            <Badge
              variant={statusConfig.variant}
              className="flex items-center gap-1"
            >
              <StatusIcon className="h-3 w-3" />
              {trial.status.replace("_", " ")}
            </Badge>
          </div>

          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Session</span>
              <span>#{trial.sessionNumber}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Participant</span>
              <span className="font-mono">
                {trial.participant.participantCode}
              </span>
            </div>
            {trial.startedAt && (
              <div className="flex justify-between">
                <span className="text-muted-foreground">Started</span>
                <span>{new Date(trial.startedAt).toLocaleTimeString()}</span>
              </div>
            )}
            {trial.completedAt && (
              <div className="flex justify-between">
                <span className="text-muted-foreground">Completed</span>
                <span>{new Date(trial.completedAt).toLocaleTimeString()}</span>
              </div>
            )}
          </div>
        </CardContent>
      </Card>

      {/* Experiment Info */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="text-sm">Experiment</CardTitle>
        </CardHeader>
        <CardContent className="space-y-2">
          <div>
            <div className="text-sm font-medium">{trial.experiment.name}</div>
            {trial.experiment.description && (
              <div className="text-muted-foreground mt-1 text-xs">
                {trial.experiment.description}
              </div>
            )}
          </div>
        </CardContent>
      </Card>

      {/* Participant Info */}
      <Card className="flex-1">
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <User className="h-4 w-4" />
            Participant
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-2">
          <div className="text-sm">
            <div className="font-medium">
              {trial.participant.participantCode}
            </div>
            {trial.participant.demographics && (
              <div className="text-muted-foreground mt-1 text-xs">
                {Object.keys(trial.participant.demographics).length} demographic
                fields
              </div>
            )}
          </div>
        </CardContent>
      </Card>
    </div>
  );

  const centerPanel = (
    <div className="flex h-full flex-col p-6">
      {trial.status === "scheduled" ? (
        <div className="flex h-full items-center justify-center">
          <Card className="w-full max-w-md">
            <CardContent className="pt-6 text-center">
              <Clock className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
              <h3 className="mb-2 text-lg font-semibold">Trial Scheduled</h3>
              <p className="text-muted-foreground mb-4">
                This trial is scheduled but has not yet started. You will be
                able to observe the execution once it begins.
              </p>
              <div className="text-muted-foreground text-sm">
                Waiting for wizard to start the trial...
              </div>
            </CardContent>
          </Card>
        </div>
      ) : trial.status === "in_progress" ? (
        <div className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Activity className="h-5 w-5" />
                Trial in Progress
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <div className="text-muted-foreground text-sm">
                  The trial is currently running. You can observe the progress
                  and events as they happen.
                </div>

                {trial.startedAt && (
                  <div className="grid grid-cols-2 gap-4 text-sm">
                    <div>
                      <span className="text-muted-foreground">Started at:</span>
                      <div className="font-mono">
                        {new Date(trial.startedAt).toLocaleString()}
                      </div>
                    </div>
                    <div>
                      <span className="text-muted-foreground">Duration:</span>
                      <div className="font-mono">
                        {formatElapsedTime(
                          Math.floor(
                            (Date.now() - new Date(trial.startedAt).getTime()) /
                              1000,
                          ),
                        )}
                      </div>
                    </div>
                  </div>
                )}
              </div>
            </CardContent>
          </Card>

          <Card className="flex-1">
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Bot className="h-5 w-5" />
                Live Observation
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="bg-muted/50 rounded-lg p-8 text-center">
                <div className="text-muted-foreground">
                  Live trial observation interface
                </div>
                <div className="text-muted-foreground mt-2 text-xs">
                  Real-time trial events and robot status would appear here
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      ) : (
        <div className="flex h-full items-center justify-center">
          <Card className="w-full max-w-md">
            <CardContent className="pt-6 text-center">
              <CheckCircle className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
              <h3 className="mb-2 text-lg font-semibold">
                Trial {trial.status === "completed" ? "Completed" : "Ended"}
              </h3>
              <p className="text-muted-foreground mb-4">
                The trial execution has finished. Review the results and data
                collected during the session.
              </p>
              {trial.completedAt && (
                <div className="text-muted-foreground text-sm">
                  Ended at {new Date(trial.completedAt).toLocaleString()}
                </div>
              )}
            </CardContent>
          </Card>
        </div>
      )}
    </div>
  );

  const rightPanel = (
    <div className="flex h-full flex-col space-y-4 p-4">
      {/* System Status */}
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="text-sm">System Status</CardTitle>
        </CardHeader>
        <CardContent className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">Connection</span>
            <Badge variant="outline" className="text-xs">
              Observer Mode
            </Badge>
          </div>
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">View Only</span>
            <Badge variant="secondary" className="text-xs">
              Read Only
            </Badge>
          </div>
        </CardContent>
      </Card>

      {/* Recent Activity */}
      <Card className="flex-1">
        <CardHeader className="pb-3">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Activity className="h-4 w-4" />
            Recent Activity
          </CardTitle>
        </CardHeader>
        <CardContent>
          <ScrollArea className="h-full">
            <div className="space-y-3">
              <div className="text-muted-foreground py-8 text-center text-sm">
                No recent activity
              </div>
            </div>
          </ScrollArea>
        </CardContent>
      </Card>
    </div>
  );

  return (
    <div className="h-full">
      {/* Status Bar */}
      <div className="bg-background border-b px-4 py-2">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <Badge variant="outline" className="flex items-center gap-1">
              <Eye className="h-3 w-3" />
              Observer Mode
            </Badge>

            <Badge
              variant={statusConfig.variant}
              className="flex items-center gap-1"
            >
              <StatusIcon className="h-3 w-3" />
              {trial.status.replace("_", " ")}
            </Badge>
          </div>

          <div className="text-muted-foreground text-sm">
            {trial.experiment.name} • {trial.participant.participantCode}
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="min-h-0 flex-1">
        <PanelsContainer
          left={leftPanel}
          center={centerPanel}
          right={rightPanel}
          showDividers={true}
          className="h-full"
        />
      </div>
    </div>
  );
}
