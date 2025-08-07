"use client";

import { formatDistanceToNow } from "date-fns";
import {
  AlertCircle,
  Calendar,
  CheckCircle,
  Eye,
  Info,
  Play,
  Zap,
} from "lucide-react";
import Link from "next/link";
import { useEffect, useState } from "react";
import { useSession } from "next-auth/react";

import { Alert, AlertDescription, AlertTitle } from "~/components/ui/alert";
import { Button } from "~/components/ui/button";
import {
  EntityView,
  EntityViewHeader,
  EntityViewSection,
  EmptyState,
  InfoGrid,
  QuickActions,
  StatsGrid,
} from "~/components/ui/entity-view";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { api } from "~/trpc/react";

interface TrialDetailPageProps {
  params: Promise<{ trialId: string }>;
  searchParams: Promise<{ error?: string }>;
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
    icon: "AlertCircle" as const,
  },
  cancelled: {
    label: "Cancelled",
    variant: "outline" as const,
    icon: "AlertCircle" as const,
  },
};

type Trial = {
  id: string;
  participantId: string | null;
  experimentId: string;
  wizardId?: string | null;
  sessionNumber?: number;
  status: string;
  startedAt: Date | null;
  completedAt: Date | null;
  duration: number | null;
  notes: string | null;
  createdAt: Date;
  updatedAt: Date;
  experiment: {
    id: string;
    name: string;
    studyId: string;
  } | null;
  participant: {
    id: string;
    participantCode: string;
    name?: string | null;
  } | null;
};

type TrialEvent = {
  id: string;
  trialId: string;
  eventType: string;
  actionId: string | null;
  timestamp: Date;
  data: unknown;
  createdBy: string | null;
};

export default function TrialDetailPage({
  params,
  searchParams,
}: TrialDetailPageProps) {
  const { data: session } = useSession();
  const [trial, setTrial] = useState<Trial | null>(null);
  const [events, setEvents] = useState<TrialEvent[]>([]);
  const [loading, setLoading] = useState(true);
  const [resolvedParams, setResolvedParams] = useState<{
    trialId: string;
  } | null>(null);
  const [resolvedSearchParams, setResolvedSearchParams] = useState<{
    error?: string;
  } | null>(null);

  useEffect(() => {
    const resolveParams = async () => {
      const resolved = await params;
      setResolvedParams(resolved);
    };
    void resolveParams();
  }, [params]);

  useEffect(() => {
    const resolveSearchParams = async () => {
      const resolved = await searchParams;
      setResolvedSearchParams(resolved);
    };
    void resolveSearchParams();
  }, [searchParams]);

  const trialQuery = api.trials.get.useQuery(
    { id: resolvedParams?.trialId ?? "" },
    { enabled: !!resolvedParams?.trialId },
  );

  const eventsQuery = api.trials.getEvents.useQuery(
    { trialId: resolvedParams?.trialId ?? "" },
    { enabled: !!resolvedParams?.trialId },
  );

  useEffect(() => {
    if (trialQuery.data) {
      setTrial(trialQuery.data as Trial);
    }
  }, [trialQuery.data]);

  useEffect(() => {
    if (eventsQuery.data) {
      setEvents(eventsQuery.data as TrialEvent[]);
    }
  }, [eventsQuery.data]);

  useEffect(() => {
    if (trialQuery.isLoading || eventsQuery.isLoading) {
      setLoading(true);
    } else {
      setLoading(false);
    }
  }, [trialQuery.isLoading, eventsQuery.isLoading]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    {
      label: "Dashboard",
      href: "/",
    },
    {
      label: "Studies",
      href: "/studies",
    },
    {
      label: "Study",
      href: trial?.experiment?.studyId
        ? `/studies/${trial.experiment.studyId}`
        : "/studies",
    },
    {
      label: "Trials",
      href: trial?.experiment?.studyId
        ? `/studies/${trial.experiment.studyId}/trials`
        : "/trials",
    },
    {
      label: `Trial #${resolvedParams?.trialId?.slice(-6) ?? "Unknown"}`,
    },
  ]);

  if (loading) return <div>Loading...</div>;
  if (trialQuery.error || !trial) return <div>Trial not found</div>;

  const statusInfo = statusConfig[trial.status as keyof typeof statusConfig];
  const userRoles = session?.user?.roles?.map((r) => r.role) ?? [];
  const canControl =
    userRoles.includes("wizard") || userRoles.includes("researcher");

  const displayName = `Trial #${trial.id.slice(-6)}`;
  const experimentName = trial.experiment?.name ?? "Unknown Experiment";

  return (
    <EntityView>
      {resolvedSearchParams?.error && (
        <Alert variant="destructive" className="mb-6">
          <AlertCircle className="h-4 w-4" />
          <AlertTitle>Error</AlertTitle>
          <AlertDescription>{resolvedSearchParams.error}</AlertDescription>
        </Alert>
      )}

      <EntityViewHeader
        title={displayName}
        subtitle={`${experimentName} - ${trial.participant?.participantCode ?? "Unknown Participant"}`}
        icon="Play"
        status={
          statusInfo && {
            label: statusInfo.label,
            variant: statusInfo.variant,
            icon: statusInfo.icon,
          }
        }
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
            {trial.status === "completed" && (
              <Button asChild variant="outline">
                <Link href={`/trials/${trial.id}/analysis`}>
                  <Info className="mr-2 h-4 w-4" />
                  View Analysis
                </Link>
              </Button>
            )}
          </>
        }
      />

      <div className="grid gap-6 lg:grid-cols-3">
        <div className="space-y-6 lg:col-span-2">
          {/* Trial Information */}
          <EntityViewSection title="Trial Information" icon="Info">
            <InfoGrid
              columns={2}
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
                    "Unknown"
                  ),
                },
                {
                  label: "Participant",
                  value: trial.participant ? (
                    <Link
                      href={`/participants/${trial.participant.id}`}
                      className="text-primary hover:underline"
                    >
                      {trial.participant.name ??
                        trial.participant.participantCode}
                    </Link>
                  ) : (
                    "Unknown"
                  ),
                },
                {
                  label: "Study",
                  value: trial.experiment?.studyId ? (
                    <Link
                      href={`/studies/${trial.experiment.studyId}`}
                      className="text-primary hover:underline"
                    >
                      Study
                    </Link>
                  ) : (
                    "Unknown"
                  ),
                },
                {
                  label: "Status",
                  value: statusInfo?.label ?? trial.status,
                },
                {
                  label: "Scheduled",
                  value: trial.createdAt
                    ? formatDistanceToNow(trial.createdAt, { addSuffix: true })
                    : "Not scheduled",
                },
                {
                  label: "Duration",
                  value: trial.duration
                    ? `${Math.round(trial.duration / 60)} minutes`
                    : trial.status === "in_progress"
                      ? "Ongoing"
                      : "Not available",
                },
              ]}
            />
          </EntityViewSection>

          {/* Trial Notes */}
          {trial.notes && (
            <EntityViewSection title="Notes" icon="FileText">
              <div className="prose prose-sm max-w-none">
                <p className="text-muted-foreground">{trial.notes}</p>
              </div>
            </EntityViewSection>
          )}

          {/* Event Timeline */}
          <EntityViewSection
            title="Event Timeline"
            icon="Activity"
            description={`${events.length} events recorded`}
          >
            {events.length > 0 ? (
              <div className="space-y-4">
                {events.slice(0, 10).map((event) => (
                  <div key={event.id} className="rounded-lg border p-4">
                    <div className="mb-2 flex items-center justify-between">
                      <span className="font-medium">
                        {event.eventType
                          .replace(/_/g, " ")
                          .replace(/\b\w/g, (l) => l.toUpperCase())}
                      </span>
                      <span className="text-muted-foreground text-sm">
                        {formatDistanceToNow(event.timestamp, {
                          addSuffix: true,
                        })}
                      </span>
                    </div>
                    {event.data ? (
                      <div className="text-muted-foreground text-sm">
                        <pre className="text-xs">
                          {typeof event.data === "object" && event.data !== null
                            ? JSON.stringify(event.data, null, 2)
                            : String(event.data as string | number | boolean)}
                        </pre>
                      </div>
                    ) : null}
                  </div>
                ))}
                {events.length > 10 && (
                  <div className="text-center">
                    <Button variant="outline" size="sm">
                      View All Events ({events.length})
                    </Button>
                  </div>
                )}
              </div>
            ) : (
              <EmptyState
                icon="Activity"
                title="No events recorded"
                description="Events will appear here as the trial progresses"
              />
            )}
          </EntityViewSection>
        </div>

        <div className="space-y-6">
          {/* Statistics */}
          <EntityViewSection title="Statistics" icon="BarChart">
            <StatsGrid
              stats={[
                {
                  label: "Events",
                  value: events.length,
                },
                {
                  label: "Created",
                  value: formatDistanceToNow(trial.createdAt, {
                    addSuffix: true,
                  }),
                },
                {
                  label: "Started",
                  value: trial.startedAt
                    ? formatDistanceToNow(trial.startedAt, { addSuffix: true })
                    : "Not started",
                },
                {
                  label: "Completed",
                  value: trial.completedAt
                    ? formatDistanceToNow(trial.completedAt, {
                        addSuffix: true,
                      })
                    : "Not completed",
                },
                {
                  label: "Created By",
                  value: "System",
                },
              ]}
            />
          </EntityViewSection>

          {/* Quick Actions */}
          <EntityViewSection title="Quick Actions" icon="Zap">
            <QuickActions
              actions={[
                ...(canControl && trial.status === "scheduled"
                  ? [
                      {
                        label: "Start Trial",
                        icon: "Play" as const,
                        href: `/trials/${trial.id}/wizard`,
                        variant: "default" as const,
                      },
                    ]
                  : []),
                ...(canControl && trial.status === "in_progress"
                  ? [
                      {
                        label: "Monitor Trial",
                        icon: "Eye" as const,
                        href: `/trials/${trial.id}/wizard`,
                      },
                    ]
                  : []),
                ...(trial.status === "completed"
                  ? [
                      {
                        label: "View Analysis",
                        icon: "BarChart" as const,
                        href: `/trials/${trial.id}/analysis`,
                      },
                      {
                        label: "Export Data",
                        icon: "Download" as const,
                        href: `/trials/${trial.id}/export`,
                      },
                    ]
                  : []),
                {
                  label: "View Events",
                  icon: "Activity" as const,
                  href: `/trials/${trial.id}/events`,
                },
                {
                  label: "Export Report",
                  icon: "FileText" as const,
                  href: `/trials/${trial.id}/report`,
                },
              ]}
            />
          </EntityViewSection>

          {/* Participant Info */}
          {trial.participant && (
            <EntityViewSection title="Participant" icon="User">
              <InfoGrid
                columns={1}
                items={[
                  {
                    label: "Code",
                    value: trial.participant.participantCode,
                  },
                  {
                    label: "Name",
                    value: trial.participant.name ?? "Not provided",
                  },
                ]}
              />
            </EntityViewSection>
          )}
        </div>
      </div>
    </EntityView>
  );
}
