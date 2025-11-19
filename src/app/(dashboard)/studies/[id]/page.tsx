"use client";

import { formatDistanceToNow } from "date-fns";
import { Plus, Settings, Shield } from "lucide-react";
import Link from "next/link";
import { notFound } from "next/navigation";
import { useEffect, useState } from "react";
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

interface StudyDetailPageProps {
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
  active: {
    label: "Active",
    variant: "default" as const,
    icon: "CheckCircle" as const,
  },
  completed: {
    label: "Completed",
    variant: "outline" as const,
    icon: "CheckCircle" as const,
  },
  archived: {
    label: "Archived",
    variant: "destructive" as const,
    icon: "XCircle" as const,
  },
};

type Study = {
  id: string;
  name: string;
  description: string | null;
  status: string;
  institution: string | null;
  irbProtocol: string | null;
  createdAt: Date;
  updatedAt: Date;
};

type Member = {
  role: string;
  user: {
    name: string | null;
    email: string;
  };
};

export default function StudyDetailPage({ params }: StudyDetailPageProps) {
  const { data: session } = useSession();
  const [study, setStudy] = useState<Study | null>(null);
  const [members, setMembers] = useState<Member[]>([]);
  const [loading, setLoading] = useState(true);
  const [resolvedParams, setResolvedParams] = useState<{ id: string } | null>(
    null,
  );

  useEffect(() => {
    const resolveParams = async () => {
      const resolved = await params;
      setResolvedParams(resolved);
    };
    void resolveParams();
  }, [params]);

  const { data: studyData } = api.studies.get.useQuery(
    { id: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: membersData } = api.studies.getMembers.useQuery(
    { studyId: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: experimentsData } = api.experiments.list.useQuery(
    { studyId: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: participantsData } = api.participants.list.useQuery(
    { studyId: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: trialsData } = api.trials.list.useQuery(
    { studyId: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: activityData } = api.studies.getActivity.useQuery(
    { studyId: resolvedParams?.id ?? "", limit: 5 },
    { enabled: !!resolvedParams?.id },
  );

  useEffect(() => {
    if (studyData) {
      setStudy(studyData);
    }
    if (membersData) {
      setMembers(membersData);
    }
    if (studyData !== undefined) {
      setLoading(false);
    }
  }, [studyData, membersData]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study" },
  ]);

  if (!session?.user) {
    return notFound();
  }

  if (loading || !study) {
    return <div>Loading...</div>;
  }

  const statusInfo = statusConfig[study.status as keyof typeof statusConfig];

  const experiments = experimentsData ?? [];
  const participants = participantsData?.participants ?? [];
  const trials = trialsData ?? [];
  const activities = activityData?.activities ?? [];

  const completedTrials = trials.filter(
    (trial: { status: string }) => trial.status === "completed",
  ).length;
  const totalTrials = trials.length;

  const stats = {
    experiments: experiments.length,
    totalTrials: totalTrials,
    participants: participants.length,
    completionRate:
      totalTrials > 0
        ? `${Math.round((completedTrials / totalTrials) * 100)}%`
        : "—",
  };

  return (
    <EntityView>
      {/* Header */}
      <EntityViewHeader
        title={study.name}
        subtitle={study.description ?? undefined}
        icon="Building"
        status={{
          label: statusInfo?.label ?? "Unknown",
          variant: statusInfo?.variant ?? "secondary",
          icon: statusInfo?.icon ?? "FileText",
        }}
        actions={
          <>
            <Button asChild variant="outline">
              <Link href={`/studies/${study.id}/edit`}>
                <Settings className="mr-2 h-4 w-4" />
                Edit Study
              </Link>
            </Button>
            <Button asChild>
              <Link href={`/studies/${study.id}/experiments/new`}>
                <Plus className="mr-2 h-4 w-4" />
                New Experiment
              </Link>
            </Button>
          </>
        }
      />

      <div className="grid grid-cols-1 gap-8 lg:grid-cols-3">
        {/* Main Content */}
        <div className="space-y-8 lg:col-span-2">
          {/* Study Information */}
          <EntityViewSection title="Study Information" icon="Building">
            <InfoGrid
              items={[
                {
                  label: "Institution",
                  value: study.institution ?? "Not specified",
                },
                {
                  label: "IRB Protocol",
                  value: study.irbProtocol ?? "Not required",
                },
                {
                  label: "Created",
                  value: formatDistanceToNow(study.createdAt, {
                    addSuffix: true,
                  }),
                },
                {
                  label: "Last Updated",
                  value: formatDistanceToNow(study.updatedAt, {
                    addSuffix: true,
                  }),
                },
              ]}
            />
          </EntityViewSection>

          {/* Experiments */}
          <EntityViewSection
            title="Experiments"
            icon="FlaskConical"
            description="Design and manage experimental protocols for this study"
            actions={
              <Button asChild variant="outline" size="sm">
                <Link href={`/studies/${study.id}/experiments/new`}>
                  <Plus className="mr-2 h-4 w-4" />
                  Add Experiment
                </Link>
              </Button>
            }
          >
            {experiments.length === 0 ? (
              <EmptyState
                icon="FlaskConical"
                title="No Experiments Yet"
                description="Create your first experiment to start designing research protocols"
                action={
                  <Button asChild>
                    <Link href={`/studies/${study.id}/experiments/new`}>
                      Create First Experiment
                    </Link>
                  </Button>
                }
              />
            ) : (
              <div className="space-y-4">
                {experiments.map((experiment) => (
                  <div
                    key={experiment.id}
                    className="flex items-center justify-between rounded-lg border p-4"
                  >
                    <div className="flex-1">
                      <div className="flex items-center space-x-3">
                        <h4 className="font-medium">
                          <Link
                            href={`/studies/${study.id}/experiments/${experiment.id}`}
                            className="hover:underline"
                          >
                            {experiment.name}
                          </Link>
                        </h4>
                        <span
                          className={`inline-flex items-center rounded-full px-2 py-1 text-xs font-medium ${experiment.status === "draft"
                              ? "bg-gray-100 text-gray-800"
                              : experiment.status === "ready"
                                ? "bg-green-100 text-green-800"
                                : "bg-blue-100 text-blue-800"
                            }`}
                        >
                          {experiment.status}
                        </span>
                      </div>
                      {experiment.description && (
                        <p className="text-muted-foreground mt-1 text-sm">
                          {experiment.description}
                        </p>
                      )}
                      <div className="text-muted-foreground mt-2 flex items-center space-x-4 text-xs">
                        <span>
                          Created{" "}
                          {formatDistanceToNow(experiment.createdAt, {
                            addSuffix: true,
                          })}
                        </span>
                        {experiment.estimatedDuration && (
                          <span>Est. {experiment.estimatedDuration} min</span>
                        )}
                      </div>
                    </div>
                    <div className="flex items-center space-x-2">
                      <Button asChild variant="outline" size="sm">
                        <Link href={`/studies/${study.id}/experiments/${experiment.id}/designer`}>
                          Design
                        </Link>
                      </Button>
                      <Button asChild variant="outline" size="sm">
                        <Link href={`/studies/${study.id}/experiments/${experiment.id}`}>View</Link>
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </EntityViewSection>

          {/* Recent Activity */}
          <EntityViewSection title="Recent Activity" icon="BarChart3">
            {activities.length === 0 ? (
              <EmptyState
                icon="Calendar"
                title="No Recent Activity"
                description="Activity will appear here once you start working on this study"
              />
            ) : (
              <div className="space-y-3">
                {activities.map((activity) => (
                  <div
                    key={activity.id}
                    className="flex items-start space-x-3 rounded-lg border p-3"
                  >
                    <div className="flex h-8 w-8 items-center justify-center rounded-full bg-blue-100">
                      <span className="text-sm font-medium text-blue-600">
                        {activity.user?.name?.charAt(0) ??
                          activity.user?.email?.charAt(0) ??
                          "?"}
                      </span>
                    </div>
                    <div className="min-w-0 flex-1">
                      <div className="flex items-center space-x-2">
                        <p className="text-sm font-medium">
                          {activity.user?.name ??
                            activity.user?.email ??
                            "Unknown User"}
                        </p>
                        <span className="text-muted-foreground text-xs">
                          {formatDistanceToNow(activity.createdAt, {
                            addSuffix: true,
                          })}
                        </span>
                      </div>
                      <p className="text-muted-foreground mt-1 text-sm">
                        {activity.description}
                      </p>
                    </div>
                  </div>
                ))}
                {activityData && activityData.pagination.total > 5 && (
                  <div className="pt-2">
                    <Button
                      asChild
                      variant="outline"
                      size="sm"
                      className="w-full"
                    >
                      <Link href={`/studies/${study.id}/activity`}>
                        View All Activity ({activityData.pagination.total})
                      </Link>
                    </Button>
                  </div>
                )}
              </div>
            )}
          </EntityViewSection>
        </div>

        {/* Sidebar */}
        <EntityViewSidebar>
          {/* Team Members */}
          <EntityViewSection
            title="Team"
            icon="Users"
            description={`${members.length} team member${members.length !== 1 ? "s" : ""}`}
            actions={
              <Button variant="outline" size="sm">
                <Plus className="mr-2 h-4 w-4" />
                Invite
              </Button>
            }
          >
            <div className="space-y-3">
              {members.map((member, index) => (
                <div
                  key={`${member.user.email}-${index}`}
                  className="flex items-center space-x-3"
                >
                  <div className="flex h-8 w-8 items-center justify-center rounded-full bg-blue-100">
                    <span className="text-sm font-medium text-blue-600">
                      {(member.user.name ?? member.user.email)
                        .charAt(0)
                        .toUpperCase()}
                    </span>
                  </div>
                  <div className="min-w-0 flex-1">
                    <p className="truncate text-sm font-medium">
                      {member.user.name ?? member.user.email}
                    </p>
                    <p className="text-muted-foreground text-xs capitalize">
                      {member.role}
                    </p>
                  </div>
                  {member.role === "owner" && (
                    <Shield className="h-4 w-4 text-amber-600" />
                  )}
                </div>
              ))}
            </div>
          </EntityViewSection>

          {/* Quick Stats */}
          <EntityViewSection title="Quick Stats" icon="BarChart3">
            <StatsGrid
              stats={[
                {
                  label: "Experiments",
                  value: stats.experiments,
                },
                {
                  label: "Total Trials",
                  value: stats.totalTrials,
                },
                {
                  label: "Participants",
                  value: stats.participants,
                },
                {
                  label: "Completion Rate",
                  value: stats.completionRate,
                  color: "success",
                },
              ]}
            />
          </EntityViewSection>

          {/* Quick Actions */}
          <EntityViewSection title="Quick Actions" icon="Settings">
            <QuickActions
              actions={[
                {
                  label: "Manage Participants",
                  icon: "Users",
                  href: `/studies/${study.id}/participants`,
                },
                {
                  label: "Schedule Trials",
                  icon: "Calendar",
                  href: `/studies/${study.id}/trials`,
                },
                {
                  label: "View Analytics",
                  icon: "BarChart3",
                  href: `/studies/${study.id}/analytics`,
                },
              ]}
            />
          </EntityViewSection>
        </EntityViewSidebar>
      </div>
    </EntityView>
  );
}
