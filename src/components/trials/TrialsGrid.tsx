"use client";

import { useState } from "react";
import { Plus, Play, Pause, Square, Clock, Users, Eye, Settings } from "lucide-react";
import { formatDistanceToNow, format } from "date-fns";
import Link from "next/link";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import { api } from "~/trpc/react";

type TrialWithRelations = {
  id: string;
  experimentId: string;
  participantId: string;
  scheduledAt: Date;
  startedAt: Date | null;
  completedAt: Date | null;
  status: "scheduled" | "in_progress" | "completed" | "cancelled";
  duration: number | null;
  notes: string | null;
  wizardId: string | null;
  createdAt: Date;
  experiment: {
    id: string;
    name: string;
    study: {
      id: string;
      name: string;
    };
  };
  participant: {
    id: string;
    participantCode: string;
    email: string | null;
    name: string | null;
  };
  wizard: {
    id: string;
    name: string | null;
    email: string;
  } | null;
  _count?: {
    events: number;
    mediaCaptures: number;
  };
};

const statusConfig = {
  scheduled: {
    label: "Scheduled",
    className: "bg-blue-100 text-blue-800 hover:bg-blue-200",
    icon: Clock,
    action: "Start Trial",
    actionIcon: Play,
  },
  in_progress: {
    label: "In Progress",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    icon: Play,
    action: "Monitor",
    actionIcon: Eye,
  },
  completed: {
    label: "Completed",
    className: "bg-gray-100 text-gray-800 hover:bg-gray-200",
    icon: Square,
    action: "Review",
    actionIcon: Eye,
  },
  cancelled: {
    label: "Cancelled",
    className: "bg-red-100 text-red-800 hover:bg-red-200",
    icon: Square,
    action: "View",
    actionIcon: Eye,
  },
};

interface TrialCardProps {
  trial: TrialWithRelations;
  userRole: string;
  onTrialAction: (trialId: string, action: string) => void;
}

function TrialCard({ trial, userRole, onTrialAction }: TrialCardProps) {
  const statusInfo = statusConfig[trial.status];
  const StatusIcon = statusInfo.icon;
  const ActionIcon = statusInfo.actionIcon;

  const isScheduledSoon = trial.status === "scheduled" &&
    new Date(trial.scheduledAt).getTime() - Date.now() < 60 * 60 * 1000; // Within 1 hour

  const canControl = userRole === "wizard" || userRole === "researcher" || userRole === "administrator";

  return (
    <Card className={`group transition-all duration-200 hover:border-slate-300 hover:shadow-md ${
      trial.status === "in_progress" ? "ring-2 ring-green-500 shadow-md" : ""
    }`}>
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between">
          <div className="min-w-0 flex-1">
            <CardTitle className="truncate text-lg font-semibold text-slate-900 transition-colors group-hover:text-blue-600">
              <Link
                href={`/trials/${trial.id}`}
                className="hover:underline"
              >
                {trial.experiment.name}
              </Link>
            </CardTitle>
            <CardDescription className="mt-1 text-sm text-slate-600">
              Participant: {trial.participant.participantCode}
            </CardDescription>
            <div className="mt-2 flex items-center space-x-4 text-xs text-slate-500">
              <Link
                href={`/studies/${trial.experiment.study.id}`}
                className="font-medium text-blue-600 hover:text-blue-800"
              >
                {trial.experiment.study.name}
              </Link>
              {trial.wizard && (
                <span>Wizard: {trial.wizard.name || trial.wizard.email}</span>
              )}
            </div>
          </div>
          <div className="flex flex-col items-end space-y-2">
            <Badge className={statusInfo.className} variant="secondary">
              <StatusIcon className="mr-1 h-3 w-3" />
              {statusInfo.label}
            </Badge>
            {isScheduledSoon && (
              <Badge variant="outline" className="text-orange-600 border-orange-600">
                Starting Soon
              </Badge>
            )}
          </div>
        </div>
      </CardHeader>

      <CardContent className="space-y-4">
        {/* Schedule Information */}
        <div className="space-y-2">
          <div className="flex items-center justify-between text-sm">
            <span className="text-slate-600">Scheduled:</span>
            <span className="font-medium">
              {format(trial.scheduledAt, "MMM d, yyyy 'at' h:mm a")}
            </span>
          </div>
          {trial.startedAt && (
            <div className="flex items-center justify-between text-sm">
              <span className="text-slate-600">Started:</span>
              <span className="font-medium">
                {formatDistanceToNow(trial.startedAt, { addSuffix: true })}
              </span>
            </div>
          )}
          {trial.completedAt && (
            <div className="flex items-center justify-between text-sm">
              <span className="text-slate-600">Completed:</span>
              <span className="font-medium">
                {formatDistanceToNow(trial.completedAt, { addSuffix: true })}
              </span>
            </div>
          )}
          {trial.duration && (
            <div className="flex items-center justify-between text-sm">
              <span className="text-slate-600">Duration:</span>
              <span className="font-medium">{Math.round(trial.duration / 60)} minutes</span>
            </div>
          )}
        </div>

        {/* Statistics */}
        {trial._count && (
          <>
            <Separator />
            <div className="grid grid-cols-2 gap-4 text-sm">
              <div className="flex justify-between">
                <span className="text-slate-600">Events:</span>
                <span className="font-medium">{trial._count.events}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-600">Media:</span>
                <span className="font-medium">{trial._count.mediaCaptures}</span>
              </div>
            </div>
          </>
        )}

        {/* Notes Preview */}
        {trial.notes && (
          <>
            <Separator />
            <div className="text-sm">
              <span className="text-slate-600">Notes: </span>
              <span className="text-slate-900">{trial.notes.substring(0, 100)}...</span>
            </div>
          </>
        )}

        {/* Actions */}
        <div className="flex gap-2 pt-2">
          {trial.status === "scheduled" && canControl && (
            <Button
              size="sm"
              className="flex-1"
              onClick={() => onTrialAction(trial.id, "start")}
            >
              <ActionIcon className="mr-1 h-3 w-3" />
              {statusInfo.action}
            </Button>
          )}
          {trial.status === "in_progress" && (
            <Button asChild size="sm" className="flex-1">
              <Link href={`/trials/${trial.id}/wizard`}>
                <Eye className="mr-1 h-3 w-3" />
                Wizard Interface
              </Link>
            </Button>
          )}
          {trial.status === "completed" && (
            <Button asChild size="sm" variant="outline" className="flex-1">
              <Link href={`/trials/${trial.id}/analysis`}>
                <Eye className="mr-1 h-3 w-3" />
                View Analysis
              </Link>
            </Button>
          )}
          <Button asChild size="sm" variant="outline">
            <Link href={`/trials/${trial.id}`}>
              <Settings className="mr-1 h-3 w-3" />
              Details
            </Link>
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}

export function TrialsGrid() {
  const [refreshKey, setRefreshKey] = useState(0);
  const [statusFilter, setStatusFilter] = useState<string>("all");

  const { data: userSession } = api.auth.me.useQuery();

  const {
    data: trialsData,
    isLoading,
    error,
    refetch,
  } = api.trials.getUserTrials.useQuery(
    {
      page: 1,
      limit: 50,
      status: statusFilter === "all" ? undefined : statusFilter as any,
    },
    {
      refetchOnWindowFocus: false,
      refetchInterval: 30000, // Refetch every 30 seconds for real-time updates
    },
  );

  const startTrialMutation = api.trials.start.useMutation({
    onSuccess: () => {
      void refetch();
    },
  });

  const trials = trialsData?.trials ?? [];
  const userRole = userSession?.roles?.[0]?.role || "observer";

  const handleTrialAction = async (trialId: string, action: string) => {
    if (action === "start") {
      try {
        await startTrialMutation.mutateAsync({ id: trialId });
      } catch (error) {
        console.error("Failed to start trial:", error);
      }
    }
  };

  const handleTrialCreated = () => {
    setRefreshKey((prev) => prev + 1);
    void refetch();
  };

  // Group trials by status for better organization
  const upcomingTrials = trials.filter(t => t.status === "scheduled");
  const activeTrials = trials.filter(t => t.status === "in_progress");
  const completedTrials = trials.filter(t => t.status === "completed");
  const cancelledTrials = trials.filter(t => t.status === "cancelled");

  if (isLoading) {
    return (
      <div className="space-y-6">
        {/* Status Filter Skeleton */}
        <div className="flex items-center space-x-2">
          {Array.from({ length: 4 }).map((_, i) => (
            <div key={i} className="h-8 w-20 rounded bg-slate-200 animate-pulse"></div>
          ))}
        </div>

        {/* Grid Skeleton */}
        <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
          {Array.from({ length: 6 }).map((_, i) => (
            <Card key={i} className="animate-pulse">
              <CardHeader>
                <div className="space-y-2">
                  <div className="h-5 w-3/4 rounded bg-slate-200"></div>
                  <div className="h-4 w-1/2 rounded bg-slate-200"></div>
                  <div className="h-3 w-2/3 rounded bg-slate-200"></div>
                </div>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="space-y-2">
                  <div className="h-4 w-full rounded bg-slate-200"></div>
                  <div className="h-4 w-full rounded bg-slate-200"></div>
                </div>
                <div className="flex gap-2">
                  <div className="h-8 flex-1 rounded bg-slate-200"></div>
                  <div className="h-8 w-16 rounded bg-slate-200"></div>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="text-center py-12">
        <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-red-100">
          <svg
            className="h-8 w-8 text-red-600"
            fill="none"
            stroke="currentColor"
            viewBox="0 0 24 24"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
            />
          </svg>
        </div>
        <h3 className="mb-2 text-lg font-semibold text-slate-900">
          Failed to Load Trials
        </h3>
        <p className="mb-4 text-slate-600">
          {error.message || "An error occurred while loading your trials."}
        </p>
        <Button onClick={() => refetch()} variant="outline">
          Try Again
        </Button>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      {/* Quick Actions Bar */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-2">
          <Button
            variant={statusFilter === "all" ? "default" : "outline"}
            size="sm"
            onClick={() => setStatusFilter("all")}
          >
            All ({trials.length})
          </Button>
          <Button
            variant={statusFilter === "scheduled" ? "default" : "outline"}
            size="sm"
            onClick={() => setStatusFilter("scheduled")}
          >
            Scheduled ({upcomingTrials.length})
          </Button>
          <Button
            variant={statusFilter === "in_progress" ? "default" : "outline"}
            size="sm"
            onClick={() => setStatusFilter("in_progress")}
          >
            Active ({activeTrials.length})
          </Button>
          <Button
            variant={statusFilter === "completed" ? "default" : "outline"}
            size="sm"
            onClick={() => setStatusFilter("completed")}
          >
            Completed ({completedTrials.length})
          </Button>
        </div>

        <Button asChild>
          <Link href="/trials/new">
            <Plus className="h-4 w-4 mr-2" />
            Schedule Trial
          </Link>
        </Button>
      </div>

      {/* Active Trials Section (Priority) */}
      {activeTrials.length > 0 && (statusFilter === "all" || statusFilter === "in_progress") && (
        <div className="space-y-4">
          <div className="flex items-center space-x-2">
            <div className="h-2 w-2 rounded-full bg-green-500 animate-pulse"></div>
            <h2 className="text-xl font-semibold text-slate-900">Active Trials</h2>
            <Badge className="bg-green-100 text-green-800">
              {activeTrials.length} running
            </Badge>
          </div>
          <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3">
            {activeTrials.map((trial) => (
              <TrialCard
                key={trial.id}
                trial={trial}
                userRole={userRole}
                onTrialAction={handleTrialAction}
              />
            ))}
          </div>
        </div>
      )}

      {/* Main Trials Grid */}
      <div className="space-y-4">
        {statusFilter !== "in_progress" && (
          <h2 className="text-xl font-semibold text-slate-900">
            {statusFilter === "all" ? "All Trials" :
             statusFilter === "scheduled" ? "Scheduled Trials" :
             statusFilter === "completed" ? "Completed Trials" :
             "Cancelled Trials"}
          </h2>
        )}

        {trials.length === 0 ? (
          <Card className="text-center py-12">
            <CardContent>
              <div className="mx-auto mb-4 flex h-24 w-24 items-center justify-center rounded-lg bg-slate-100">
                <Play className="h-12 w-12 text-slate-400" />
              </div>
              <h3 className="mb-2 text-lg font-semibold text-slate-900">
                No Trials Yet
              </h3>
              <p className="mb-4 text-slate-600">
                Schedule your first trial to start collecting data with real participants.
                Trials let you execute your designed experiments with wizard control.
              </p>
              <Button asChild>
                <Link href="/trials/new">Schedule Your First Trial</Link>
              </Button>
            </CardContent>
          </Card>
        ) : (
          <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3">
            {trials
              .filter(trial =>
                statusFilter === "all" ||
                trial.status === statusFilter ||
                (statusFilter === "in_progress" && trial.status === "in_progress")
              )
              .map((trial) => (
                <TrialCard
                  key={trial.id}
                  trial={trial}
                  userRole={userRole}
                  onTrialAction={handleTrialAction}
                />
              ))}
          </div>
        )}
      </div>
    </div>
  );
}
