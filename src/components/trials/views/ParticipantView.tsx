"use client";

import React from "react";
import {
  User,
  Clock,
  Play,
  CheckCircle,
  AlertCircle,
  Info,
  Heart,
  MessageCircle,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";

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

interface ParticipantViewProps {
  trial: TrialData;
}

export function ParticipantView({ trial }: ParticipantViewProps) {
  const getStatusConfig = (status: string) => {
    switch (status) {
      case "scheduled":
        return {
          variant: "outline" as const,
          color: "blue",
          icon: Clock,
          message: "Session scheduled",
        };
      case "in_progress":
        return {
          variant: "default" as const,
          color: "green",
          icon: Play,
          message: "Session in progress",
        };
      case "completed":
        return {
          variant: "secondary" as const,
          color: "gray",
          icon: CheckCircle,
          message: "Session completed",
        };
      case "aborted":
        return {
          variant: "destructive" as const,
          color: "orange",
          icon: AlertCircle,
          message: "Session ended early",
        };
      case "failed":
        return {
          variant: "destructive" as const,
          color: "red",
          icon: AlertCircle,
          message: "Session encountered an issue",
        };
      default:
        return {
          variant: "outline" as const,
          color: "gray",
          icon: Clock,
          message: "Session status unknown",
        };
    }
  };

  const statusConfig = getStatusConfig(trial.status);
  const StatusIcon = statusConfig.icon;

  const formatElapsedTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  const currentTime = new Date();
  const elapsedSeconds = trial.startedAt
    ? Math.floor(
        (currentTime.getTime() - new Date(trial.startedAt).getTime()) / 1000,
      )
    : 0;

  return (
    <div className="h-full bg-gradient-to-br from-blue-50 to-indigo-50">
      {/* Header */}
      <div className="border-b bg-white px-6 py-4 shadow-sm">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="flex h-10 w-10 items-center justify-center rounded-full bg-blue-100">
              <User className="h-5 w-5 text-blue-600" />
            </div>
            <div>
              <h1 className="text-lg font-semibold">Research Session</h1>
              <p className="text-muted-foreground text-sm">
                Participant {trial.participant.participantCode}
              </p>
            </div>
          </div>

          <Badge
            variant={statusConfig.variant}
            className="flex items-center gap-1 px-3 py-1"
          >
            <StatusIcon className="h-3 w-3" />
            {statusConfig.message}
          </Badge>
        </div>
      </div>

      <div className="flex h-full flex-col p-6">
        {trial.status === "scheduled" ? (
          // Pre-session view
          <div className="flex flex-1 items-center justify-center">
            <Card className="w-full max-w-lg shadow-lg">
              <CardContent className="pt-8 pb-8 text-center">
                <div className="mx-auto mb-6 flex h-16 w-16 items-center justify-center rounded-full bg-blue-100">
                  <Clock className="h-8 w-8 text-blue-600" />
                </div>
                <h2 className="mb-3 text-xl font-semibold">
                  Welcome to Your Session
                </h2>
                <p className="text-muted-foreground mb-6 leading-relaxed">
                  Your research session is scheduled and ready to begin. Please
                  wait for the researcher to start the session.
                </p>

                <div className="space-y-3 text-left">
                  <div className="bg-muted/50 flex items-center justify-between rounded-lg p-3">
                    <span className="text-sm font-medium">Experiment:</span>
                    <span className="text-sm">{trial.experiment.name}</span>
                  </div>
                  <div className="bg-muted/50 flex items-center justify-between rounded-lg p-3">
                    <span className="text-sm font-medium">Session Number:</span>
                    <span className="text-sm">#{trial.sessionNumber}</span>
                  </div>
                  {trial.scheduledAt && (
                    <div className="bg-muted/50 flex items-center justify-between rounded-lg p-3">
                      <span className="text-sm font-medium">
                        Scheduled Time:
                      </span>
                      <span className="text-sm">
                        {new Date(trial.scheduledAt).toLocaleString()}
                      </span>
                    </div>
                  )}
                </div>

                <Alert className="mt-6">
                  <Info className="h-4 w-4" />
                  <AlertDescription>
                    Please remain comfortable and ready. The session will begin
                    shortly.
                  </AlertDescription>
                </Alert>
              </CardContent>
            </Card>
          </div>
        ) : trial.status === "in_progress" ? (
          // Active session view
          <div className="flex flex-1 flex-col space-y-6">
            <Card className="shadow-sm">
              <CardContent className="pt-6">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-3">
                    <div className="h-3 w-3 animate-pulse rounded-full bg-green-500" />
                    <span className="font-medium">Session Active</span>
                  </div>
                  {trial.startedAt && (
                    <div className="text-right">
                      <div className="text-muted-foreground text-sm">
                        Duration
                      </div>
                      <div className="font-mono text-lg">
                        {formatElapsedTime(elapsedSeconds)}
                      </div>
                    </div>
                  )}
                </div>
              </CardContent>
            </Card>

            <div className="flex flex-1 items-center justify-center">
              <Card className="w-full max-w-2xl shadow-lg">
                <CardHeader className="text-center">
                  <CardTitle className="flex items-center justify-center gap-2">
                    <Heart className="h-5 w-5 text-pink-500" />
                    Session in Progress
                  </CardTitle>
                </CardHeader>
                <CardContent className="space-y-6 pb-8">
                  <div className="text-center">
                    <p className="text-muted-foreground text-lg leading-relaxed">
                      Thank you for participating! Please follow the
                      researcher&apos;s instructions and interact naturally with
                      the robot.
                    </p>
                  </div>

                  <div className="space-y-4">
                    <h3 className="font-semibold">Session Information</h3>
                    <div className="grid grid-cols-2 gap-4">
                      <div className="bg-muted/50 rounded-lg p-4 text-center">
                        <div className="text-muted-foreground text-sm">
                          Experiment
                        </div>
                        <div className="font-medium">
                          {trial.experiment.name}
                        </div>
                      </div>
                      <div className="bg-muted/50 rounded-lg p-4 text-center">
                        <div className="text-muted-foreground text-sm">
                          Session
                        </div>
                        <div className="font-medium">
                          #{trial.sessionNumber}
                        </div>
                      </div>
                    </div>
                  </div>

                  <Alert className="border-blue-200 bg-blue-50">
                    <MessageCircle className="h-4 w-4" />
                    <AlertDescription className="text-blue-800">
                      Feel free to ask questions at any time. Your comfort and
                      safety are our priority.
                    </AlertDescription>
                  </Alert>

                  <div className="text-center">
                    <Button variant="outline" size="lg" className="gap-2">
                      <AlertCircle className="h-4 w-4" />
                      Need Help?
                    </Button>
                  </div>
                </CardContent>
              </Card>
            </div>
          </div>
        ) : (
          // Post-session view
          <div className="flex flex-1 items-center justify-center">
            <Card className="w-full max-w-lg shadow-lg">
              <CardContent className="pt-8 pb-8 text-center">
                <div className="mx-auto mb-6 flex h-16 w-16 items-center justify-center rounded-full bg-green-100">
                  <CheckCircle className="h-8 w-8 text-green-600" />
                </div>
                <h2 className="mb-3 text-xl font-semibold">
                  {trial.status === "completed"
                    ? "Session Complete!"
                    : "Session Ended"}
                </h2>
                <p className="text-muted-foreground mb-6 leading-relaxed">
                  {trial.status === "completed"
                    ? "Thank you for your participation! Your session has been completed successfully."
                    : "Your session has ended. Thank you for your time and participation."}
                </p>

                <div className="space-y-3">
                  {trial.startedAt && trial.completedAt && (
                    <div className="bg-muted/50 flex items-center justify-between rounded-lg p-3">
                      <span className="text-sm font-medium">
                        Session Duration:
                      </span>
                      <span className="text-sm">
                        {formatElapsedTime(
                          Math.floor(
                            (new Date(trial.completedAt).getTime() -
                              new Date(trial.startedAt).getTime()) /
                              1000,
                          ),
                        )}
                      </span>
                    </div>
                  )}
                  {trial.completedAt && (
                    <div className="bg-muted/50 flex items-center justify-between rounded-lg p-3">
                      <span className="text-sm font-medium">Completed At:</span>
                      <span className="text-sm">
                        {new Date(trial.completedAt).toLocaleString()}
                      </span>
                    </div>
                  )}
                </div>

                {trial.status === "completed" && (
                  <Alert className="mt-6 border-green-200 bg-green-50">
                    <CheckCircle className="h-4 w-4" />
                    <AlertDescription className="text-green-800">
                      Your data has been recorded successfully. Thank you for
                      contributing to research!
                    </AlertDescription>
                  </Alert>
                )}

                <div className="mt-6">
                  <Button className="w-full" size="lg">
                    Continue
                  </Button>
                </div>
              </CardContent>
            </Card>
          </div>
        )}
      </div>
    </div>
  );
}
