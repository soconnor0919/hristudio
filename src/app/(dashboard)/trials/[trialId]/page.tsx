"use client";

import { format, formatDistanceToNow } from "date-fns";
import {
  Activity,
  AlertTriangle,
  ArrowLeft,
  BarChart3,
  Bot,
  CheckCircle,
  Clock,
  Download,
  Edit,
  Eye,
  FileText,
  Play,
  Settings,
  Share,
  Target,
  Timer,
  User,
  Users,
  XCircle,
} from "lucide-react";
import Link from "next/link";
import { notFound, redirect } from "next/navigation";
import { useEffect, useState } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
  EntityView,
  EntityViewHeader,
  EntityViewSection,
  EntityViewSidebar,
  EmptyState,
  InfoGrid,
  QuickActions,
  StatsGrid,
} from "~/components/ui/entity-view";
import { Progress } from "~/components/ui/progress";
import { Separator } from "~/components/ui/separator";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useSession } from "next-auth/react";
import { api } from "~/trpc/react";

interface TrialDetailPageProps {
  params: Promise<{
    trialId: string;
  }>;
  searchParams: Promise<{
    error?: string;
  }>;
}

const statusConfig = {
  scheduled: {
    label: "Scheduled",
    variant: "outline" as const,
    icon: "Clock" as const,
  },
  in_progress: {
    label: "In Progress",
    variant: "secondary" as const,
    icon: "Play" as const,
  },
  completed: {
    label: "Completed",
    variant: "default" as const,
    icon: "CheckCircle" as const,
  },
  failed: {
    label: "Failed",
    variant: "destructive" as const,
    icon: "XCircle" as const,
  },
  cancelled: {
    label: "Cancelled",
    variant: "destructive" as const,
    icon: "XCircle" as const,
  },
};

export default function TrialDetailPage({
  params,
  searchParams,
}: TrialDetailPageProps) {
  const { data: session } = useSession();
  const [trial, setTrial] = useState<any>(null);
  const [events, setEvents] = useState<any[]>([]);
  const [loading, setLoading] = useState(true);
  const [resolvedParams, setResolvedParams] = useState<{
    trialId: string;
  } | null>(null);
  const [resolvedSearchParams, setResolvedSearchParams] = useState<{
    error?: string;
  } | null>(null);

  useEffect(() => {
    async function resolveParams() {
      const resolvedP = await params;
      const resolvedSP = await searchParams;
      setResolvedParams(resolvedP);
      setResolvedSearchParams(resolvedSP);
    }
    resolveParams();
  }, [params, searchParams]);

  const { data: trialData } = api.trials.get.useQuery(
    { id: resolvedParams?.trialId ?? "" },
    { enabled: !!resolvedParams?.trialId },
  );

  const { data: eventsData } = api.trials.getEvents.useQuery(
    { trialId: resolvedParams?.trialId ?? "" },
    { enabled: !!resolvedParams?.trialId },
  );

  useEffect(() => {
    if (trialData) {
      setTrial(trialData);
    }
    if (eventsData) {
      setEvents(eventsData);
    }
    if (trialData !== undefined) {
      setLoading(false);
    }
  }, [trialData, eventsData]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Trials", href: "/trials" },
    { label: trial ? `Trial #${trial.id.slice(-6)}` : "Trial" },
  ]);

  if (!session?.user) {
    redirect("/auth/signin");
  }

  if (loading || !trial) {
    return <div>Loading...</div>;
  }

  const userRole = session.user.roles?.[0]?.role ?? "observer";
  const canEdit = ["administrator", "researcher"].includes(userRole);
  const canControl = ["administrator", "researcher", "wizard"].includes(
    userRole,
  );

  const statusInfo = statusConfig[trial.status];

  // Calculate trial stats
  const totalEvents = events.length;
  const errorEvents = events.filter((e) => e.eventType === "error").length;
  const completedSteps = events.filter(
    (e) => e.eventType === "step_completed",
  ).length;
  const progress = trial.experiment
    ? (completedSteps / (trial.experiment._count?.steps || 1)) * 100
    : 0;

  return (
    <EntityView>
      {/* Error Alert */}
      {resolvedSearchParams.error && (
        <Alert variant="destructive">
          <AlertTriangle className="h-4 w-4" />
          <AlertDescription>{resolvedSearchParams.error}</AlertDescription>
        </Alert>
      )}

      {/* Header */}
      <EntityViewHeader
        title={`Trial #${trial.id.slice(-6)}`}
        subtitle={trial.experiment?.name || "No experiment assigned"}
        icon="Target"
        status={{
          label: statusInfo.label,
          variant: statusInfo.variant,
          icon: statusInfo.icon,
        }}
        actions={
          <>
            {canControl && trial.status === "scheduled" && (
              <Button asChild>
                <Link href={`/trials/${trial.id}/wizard`}>
                  <Play className="mr-2 h-4 w-4" />
                  Start Trial
                </Link>
              </Button>
            )}
            {canControl && trial.status === "in_progress" && (
              <Button asChild variant="secondary">
                <Link href={`/trials/${trial.id}/wizard`}>
                  <Eye className="mr-2 h-4 w-4" />
                  Monitor
                </Link>
              </Button>
            )}
            {canEdit && (
              <Button asChild variant="outline">
                <Link href={`/trials/${trial.id}/edit`}>
                  <Edit className="mr-2 h-4 w-4" />
                  Edit
                </Link>
              </Button>
            )}
            {trial.status === "completed" && (
              <Button asChild variant="outline">
                <Link href={`/trials/${trial.id}/analysis`}>
                  <BarChart3 className="mr-2 h-4 w-4" />
                  Analysis
                </Link>
              </Button>
            )}
          </>
        }
      />

      <div className="grid grid-cols-1 gap-8 lg:grid-cols-3">
        {/* Main Content */}
        <div className="space-y-8 lg:col-span-2">
          {/* Trial Information */}
          <EntityViewSection title="Trial Information" icon="FileText">
            <InfoGrid
              items={[
                {
                  label: "Experiment",
                  value: trial.experiment ? (
                    <Link
                      href={`/experiments/${trial.experiment.id}`}
                      className="text-primary hover:underline"
                    >
                      {trial.experiment.name}
                    </Link>
                  ) : (
                    "No experiment assigned"
                  ),
                },
                {
                  label: "Participant",
                  value: trial.participant ? (
                    <Link
                      href={`/participants/${trial.participant.id}`}
                      className="text-primary hover:underline"
                    >
                      {trial.participant.name ||
                        trial.participant.participantCode}
                    </Link>
                  ) : (
                    "No participant assigned"
                  ),
                },
                {
                  label: "Study",
                  value: trial.study ? (
                    <Link
                      href={`/studies/${trial.study.id}`}
                      className="text-primary hover:underline"
                    >
                      {trial.study.name}
                    </Link>
                  ) : (
                    "No study assigned"
                  ),
                },
                {
                  label: "Robot Platform",
                  value: trial.experiment?.robot?.name || "Not specified",
                },
                {
                  label: "Scheduled",
                  value: trial.scheduledAt
                    ? format(trial.scheduledAt, "PPp")
                    : "Not scheduled",
                },
                {
                  label: "Duration",
                  value: trial.duration
                    ? `${Math.round(trial.duration / 60)} minutes`
                    : trial.status === "in_progress"
                      ? "In progress..."
                      : "Not started",
                },
              ]}
            />

            {/* Progress Bar for In-Progress Trials */}
            {trial.status === "in_progress" && trial.experiment && (
              <div className="border-t pt-4">
                <div className="mb-2 flex items-center justify-between">
                  <span className="text-sm font-medium">Progress</span>
                  <span className="text-muted-foreground text-sm">
                    {completedSteps} of {trial.experiment._count?.steps || 0}{" "}
                    steps
                  </span>
                </div>
                <Progress value={progress} className="h-2" />
              </div>
            )}

            {/* Trial Notes */}
            {trial.notes && (
              <div className="border-t pt-4">
                <h4 className="text-muted-foreground mb-2 text-sm font-medium">
                  Notes
                </h4>
                <div className="bg-muted rounded p-3 text-sm whitespace-pre-wrap">
                  {trial.notes}
                </div>
              </div>
            )}
          </EntityViewSection>

          {/* Trial Timeline */}
          <EntityViewSection
            title="Trial Timeline"
            icon="Activity"
            description="Real-time events and interactions"
            actions={
              <Button variant="outline" size="sm" asChild>
                <Link href={`/trials/${trial.id}/events`}>
                  <Eye className="mr-2 h-4 w-4" />
                  View All Events
                </Link>
              </Button>
            }
          >
            {events.length > 0 ? (
              <div className="space-y-3">
                {events.slice(-10).map((event, index) => (
                  <div
                    key={event.id}
                    className="flex items-start gap-3 rounded-lg border p-3"
                  >
                    <div className="flex-shrink-0">
                      {event.eventType === "error" ? (
                        <div className="rounded-full bg-red-100 p-1">
                          <XCircle className="h-4 w-4 text-red-600" />
                        </div>
                      ) : event.eventType === "step_completed" ? (
                        <div className="rounded-full bg-green-100 p-1">
                          <CheckCircle className="h-4 w-4 text-green-600" />
                        </div>
                      ) : (
                        <div className="rounded-full bg-blue-100 p-1">
                          <Activity className="h-4 w-4 text-blue-600" />
                        </div>
                      )}
                    </div>
                    <div className="min-w-0 flex-1">
                      <div className="flex items-center justify-between">
                        <p className="text-sm font-medium">
                          {event.eventType.replace("_", " ")}
                        </p>
                        <time className="text-muted-foreground text-xs">
                          {format(event.timestamp, "HH:mm:ss")}
                        </time>
                      </div>
                      {event.eventData && (
                        <p className="text-muted-foreground text-xs">
                          {typeof event.eventData === "string"
                            ? event.eventData
                            : JSON.stringify(event.eventData)}
                        </p>
                      )}
                    </div>
                  </div>
                ))}
                {events.length > 10 && (
                  <div className="pt-2 text-center">
                    <Button variant="outline" size="sm" asChild>
                      <Link href={`/trials/${trial.id}/events`}>
                        View All {events.length} Events
                      </Link>
                    </Button>
                  </div>
                )}
              </div>
            ) : (
              <EmptyState
                icon="Activity"
                title="No Events Yet"
                description="Trial events will appear here once the trial begins"
              />
            )}
          </EntityViewSection>
        </div>

        {/* Sidebar */}
        <EntityViewSidebar>
          {/* Trial Stats */}
          <EntityViewSection title="Statistics" icon="BarChart3">
            <StatsGrid
              stats={[
                {
                  label: "Total Events",
                  value: totalEvents,
                },
                {
                  label: "Completed Steps",
                  value: completedSteps,
                  color: "success",
                },
                {
                  label: "Error Events",
                  value: errorEvents,
                  color: errorEvents > 0 ? "error" : "default",
                },
                {
                  label: "Progress",
                  value: `${Math.round(progress)}%`,
                  color: progress === 100 ? "success" : "default",
                },
              ]}
            />
          </EntityViewSection>

          {/* Session Details */}
          <EntityViewSection title="Session Details" icon="Clock">
            <InfoGrid
              columns={1}
              items={[
                {
                  label: "Created",
                  value: formatDistanceToNow(trial.createdAt, {
                    addSuffix: true,
                  }),
                },
                {
                  label: "Started",
                  value: trial.startedAt
                    ? format(trial.startedAt, "PPp")
                    : "Not started",
                },
                {
                  label: "Completed",
                  value: trial.completedAt
                    ? format(trial.completedAt, "PPp")
                    : "Not completed",
                },
                {
                  label: "Created By",
                  value:
                    trial.createdBy?.name ||
                    trial.createdBy?.email ||
                    "Unknown",
                },
              ]}
            />
          </EntityViewSection>

          {/* Quick Actions */}
          <EntityViewSection title="Quick Actions" icon="Settings">
            <QuickActions
              actions={[
                ...(canControl && trial.status === "scheduled"
                  ? [
                      {
                        label: "Start Trial",
                        icon: "Play",
                        href: `/trials/${trial.id}/wizard`,
                        variant: "default" as const,
                      },
                    ]
                  : []),
                ...(canControl && trial.status === "in_progress"
                  ? [
                      {
                        label: "Monitor Trial",
                        icon: "Eye",
                        href: `/trials/${trial.id}/wizard`,
                      },
                    ]
                  : []),
                ...(trial.status === "completed"
                  ? [
                      {
                        label: "View Analysis",
                        icon: "BarChart3",
                        href: `/trials/${trial.id}/analysis`,
                      },
                      {
                        label: "Export Data",
                        icon: "Download",
                        href: `/trials/${trial.id}/export`,
                      },
                    ]
                  : []),
                ...(canEdit
                  ? [
                      {
                        label: "Edit Trial",
                        icon: "Edit",
                        href: `/trials/${trial.id}/edit`,
                      },
                    ]
                  : []),
                {
                  label: "Share Results",
                  icon: "Share",
                  href: `/trials/${trial.id}/share`,
                },
              ]}
            />
          </EntityViewSection>

          {/* Participant Info */}
          {trial.participant && (
            <EntityViewSection title="Participant" icon="User">
              <div className="space-y-3">
                <div className="flex items-center gap-3">
                  <div className="flex h-10 w-10 items-center justify-center rounded-full bg-blue-100">
                    <User className="h-5 w-5 text-blue-600" />
                  </div>
                  <div>
                    <p className="font-medium">
                      {trial.participant.name ||
                        trial.participant.participantCode}
                    </p>
                    <p className="text-muted-foreground text-xs">
                      {trial.participant.name
                        ? trial.participant.participantCode
                        : "Participant"}
                    </p>
                  </div>
                </div>
                <Button variant="outline" size="sm" className="w-full" asChild>
                  <Link href={`/participants/${trial.participant.id}`}>
                    View Profile
                  </Link>
                </Button>
              </div>
            </EntityViewSection>
          )}
        </EntityViewSidebar>
      </div>
    </EntityView>
  );
}
