"use client";

import { formatDistanceToNow } from "date-fns";
import {
  ArrowLeft,
  BarChart3,
  Bot,
  Calendar,
  CheckCircle,
  Edit,
  FileText,
  FlaskConical,
  Play,
  Settings,
  Share,
  Target,
  Users,
  AlertTriangle,
  XCircle,
} from "lucide-react";
import Link from "next/link";
import { notFound } from "next/navigation";
import { useEffect, useState } from "react";
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
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useSession } from "next-auth/react";
import { api } from "~/trpc/react";

interface ExperimentDetailPageProps {
  params: Promise<{
    id: string;
  }>;
}

const statusConfig = {
  draft: {
    label: "Draft",
    variant: "secondary" as const,
    icon: "FileText" as const,
  },
  testing: {
    label: "Testing",
    variant: "outline" as const,
    icon: "FlaskConical" as const,
  },
  ready: {
    label: "Ready",
    variant: "default" as const,
    icon: "CheckCircle" as const,
  },
  deprecated: {
    label: "Deprecated",
    variant: "destructive" as const,
    icon: "AlertTriangle" as const,
  },
};

export default function ExperimentDetailPage({
  params,
}: ExperimentDetailPageProps) {
  const { data: session } = useSession();
  const [experiment, setExperiment] = useState<any>(null);
  const [trials, setTrials] = useState<any[]>([]);
  const [loading, setLoading] = useState(true);
  const [resolvedParams, setResolvedParams] = useState<{ id: string } | null>(
    null,
  );

  useEffect(() => {
    async function resolveParams() {
      const resolved = await params;
      setResolvedParams(resolved);
    }
    resolveParams();
  }, [params]);

  const { data: experimentData } = api.experiments.get.useQuery(
    { id: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: trialsData } = api.trials.list.useQuery(
    { experimentId: resolvedParams?.id ?? "", limit: 10 },
    { enabled: !!resolvedParams?.id },
  );

  useEffect(() => {
    if (experimentData) {
      setExperiment(experimentData);
    }
    if (trialsData) {
      setTrials(trialsData);
    }
    if (experimentData !== undefined) {
      setLoading(false);
    }
  }, [experimentData, trialsData]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Experiments", href: "/experiments" },
    { label: experiment?.name || "Experiment" },
  ]);

  if (!session?.user) {
    return notFound();
  }

  if (loading || !experiment) {
    return <div>Loading...</div>;
  }

  const userRole = session.user.roles?.[0]?.role ?? "observer";
  const canEdit = ["administrator", "researcher"].includes(userRole);

  const statusInfo = statusConfig[experiment.status];

  // TODO: Get actual stats from API
  const mockStats = {
    totalTrials: trials.length,
    completedTrials: trials.filter((t) => t.status === "completed").length,
    averageDuration: "—",
    successRate:
      trials.length > 0
        ? `${Math.round((trials.filter((t) => t.status === "completed").length / trials.length) * 100)}%`
        : "—",
  };

  return (
    <EntityView>
      {/* Header */}
      <EntityViewHeader
        title={experiment.name}
        subtitle={experiment.description}
        icon="FlaskConical"
        status={{
          label: statusInfo.label,
          variant: statusInfo.variant,
          icon: statusInfo.icon,
        }}
        actions={
          canEdit ? (
            <>
              <Button asChild variant="outline">
                <Link href={`/experiments/${experiment.id}/edit`}>
                  <Edit className="mr-2 h-4 w-4" />
                  Edit
                </Link>
              </Button>
              <Button asChild variant="outline">
                <Link href={`/experiments/${experiment.id}/designer`}>
                  <Settings className="mr-2 h-4 w-4" />
                  Designer
                </Link>
              </Button>
              <Button asChild>
                <Link href={`/trials/new?experimentId=${experiment.id}`}>
                  <Play className="mr-2 h-4 w-4" />
                  Start Trial
                </Link>
              </Button>
            </>
          ) : (
            <Button asChild>
              <Link href={`/trials/new?experimentId=${experiment.id}`}>
                <Play className="mr-2 h-4 w-4" />
                Start Trial
              </Link>
            </Button>
          )
        }
      />

      <div className="grid grid-cols-1 gap-8 lg:grid-cols-3">
        {/* Main Content */}
        <div className="space-y-8 lg:col-span-2">
          {/* Experiment Information */}
          <EntityViewSection title="Experiment Information" icon="FlaskConical">
            <InfoGrid
              items={[
                {
                  label: "Study",
                  value: experiment.study ? (
                    <Link
                      href={`/studies/${experiment.study.id}`}
                      className="text-primary hover:underline"
                    >
                      {experiment.study.name}
                    </Link>
                  ) : (
                    "No study assigned"
                  ),
                },
                {
                  label: "Robot Platform",
                  value: experiment.robot?.name || "Not specified",
                },
                {
                  label: "Created",
                  value: formatDistanceToNow(experiment.createdAt, {
                    addSuffix: true,
                  }),
                },
                {
                  label: "Last Updated",
                  value: formatDistanceToNow(experiment.updatedAt, {
                    addSuffix: true,
                  }),
                },
              ]}
            />
          </EntityViewSection>

          {/* Protocol Overview */}
          <EntityViewSection
            title="Protocol Overview"
            icon="Target"
            actions={
              canEdit && (
                <Button asChild variant="outline" size="sm">
                  <Link href={`/experiments/${experiment.id}/designer`}>
                    <Edit className="mr-2 h-4 w-4" />
                    Edit Protocol
                  </Link>
                </Button>
              )
            }
          >
            {experiment.protocol &&
            typeof experiment.protocol === "object" &&
            experiment.protocol !== null ? (
              <div className="space-y-3">
                <div className="bg-muted rounded-lg p-4">
                  <h4 className="mb-2 font-medium">Protocol Structure</h4>
                  <p className="text-muted-foreground text-sm">
                    Visual protocol designed with{" "}
                    {Array.isArray((experiment.protocol as any).blocks)
                      ? (experiment.protocol as any).blocks.length
                      : 0}{" "}
                    blocks
                  </p>
                </div>
              </div>
            ) : (
              <EmptyState
                icon="Target"
                title="No Protocol Defined"
                description="Use the experiment designer to create your protocol"
                action={
                  canEdit && (
                    <Button asChild>
                      <Link href={`/experiments/${experiment.id}/designer`}>
                        Open Designer
                      </Link>
                    </Button>
                  )
                }
              />
            )}
          </EntityViewSection>

          {/* Recent Trials */}
          <EntityViewSection
            title="Recent Trials"
            icon="Play"
            description="Latest experimental sessions"
            actions={
              <Button asChild variant="outline" size="sm">
                <Link href={`/trials/new?experimentId=${experiment.id}`}>
                  <Play className="mr-2 h-4 w-4" />
                  Start Trial
                </Link>
              </Button>
            }
          >
            {trials.length > 0 ? (
              <div className="space-y-3">
                {trials.slice(0, 5).map((trial) => (
                  <div
                    key={trial.id}
                    className="hover:bg-muted/50 rounded-lg border p-4 transition-colors"
                  >
                    <div className="mb-2 flex items-center justify-between">
                      <Link
                        href={`/trials/${trial.id}`}
                        className="font-medium hover:underline"
                      >
                        Trial #{trial.id.slice(-6)}
                      </Link>
                      <Badge
                        variant={
                          trial.status === "completed"
                            ? "default"
                            : trial.status === "in_progress"
                              ? "secondary"
                              : trial.status === "failed"
                                ? "destructive"
                                : "outline"
                        }
                      >
                        {trial.status.replace("_", " ")}
                      </Badge>
                    </div>
                    <div className="text-muted-foreground flex items-center gap-4 text-sm">
                      <span className="flex items-center gap-1">
                        <Calendar className="h-4 w-4" />
                        {trial.createdAt
                          ? formatDistanceToNow(new Date(trial.createdAt), {
                              addSuffix: true,
                            })
                          : "Not scheduled"}
                      </span>
                      {trial.participant && (
                        <span className="flex items-center gap-1">
                          <Users className="h-4 w-4" />
                          {trial.participant.name ||
                            trial.participant.participantCode}
                        </span>
                      )}
                    </div>
                  </div>
                ))}
                {trials.length > 5 && (
                  <div className="pt-2 text-center">
                    <Button variant="outline" size="sm" asChild>
                      <Link href={`/trials?experimentId=${experiment.id}`}>
                        View All Trials ({trials.length})
                      </Link>
                    </Button>
                  </div>
                )}
              </div>
            ) : (
              <EmptyState
                icon="Play"
                title="No Trials Yet"
                description="Start your first trial to begin collecting data"
                action={
                  <Button asChild>
                    <Link href={`/trials/new?experimentId=${experiment.id}`}>
                      Start First Trial
                    </Link>
                  </Button>
                }
              />
            )}
          </EntityViewSection>
        </div>

        {/* Sidebar */}
        <EntityViewSidebar>
          {/* Quick Stats */}
          <EntityViewSection title="Statistics" icon="BarChart3">
            <StatsGrid
              stats={[
                {
                  label: "Total Trials",
                  value: mockStats.totalTrials,
                },
                {
                  label: "Completed",
                  value: mockStats.completedTrials,
                  color: "success",
                },
                {
                  label: "Success Rate",
                  value: mockStats.successRate,
                  color: "success",
                },
                {
                  label: "Avg. Duration",
                  value: mockStats.averageDuration,
                },
              ]}
            />
          </EntityViewSection>

          {/* Robot Information */}
          {experiment.robot && (
            <EntityViewSection title="Robot Platform" icon="Bot">
              <InfoGrid
                columns={1}
                items={[
                  {
                    label: "Platform",
                    value: experiment.robot.name,
                  },
                  {
                    label: "Type",
                    value: experiment.robot.type || "Not specified",
                  },
                  {
                    label: "Connection",
                    value: experiment.robot.connectionType || "Not configured",
                  },
                ]}
              />
            </EntityViewSection>
          )}

          {/* Quick Actions */}
          <EntityViewSection title="Quick Actions" icon="Settings">
            <QuickActions
              actions={[
                {
                  label: "View All Trials",
                  icon: "Play",
                  href: `/trials?experimentId=${experiment.id}`,
                },
                {
                  label: "Export Data",
                  icon: "Share",
                  href: `/experiments/${experiment.id}/export`,
                },
                ...(canEdit
                  ? [
                      {
                        label: "Edit Experiment",
                        icon: "Edit",
                        href: `/experiments/${experiment.id}/edit`,
                      },
                      {
                        label: "Protocol Designer",
                        icon: "Settings",
                        href: `/experiments/${experiment.id}/designer`,
                      },
                    ]
                  : []),
              ]}
            />
          </EntityViewSection>
        </EntityViewSidebar>
      </div>
    </EntityView>
  );
}
