"use client";

import { formatDistanceToNow } from "date-fns";
import {
  ArrowLeft,
  BarChart3,
  Building,
  Calendar,
  CheckCircle,
  Clock,
  Edit,
  FileText,
  FlaskConical,
  Plus,
  Settings,
  Shield,
  Users,
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
import { Separator } from "~/components/ui/separator";
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

export default function StudyDetailPage({ params }: StudyDetailPageProps) {
  const { data: session } = useSession();
  const [study, setStudy] = useState<any>(null);
  const [members, setMembers] = useState<any[]>([]);
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

  const { data: studyData } = api.studies.get.useQuery(
    { id: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: membersData } = api.studies.getMembers.useQuery(
    { studyId: resolvedParams?.id ?? "" },
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
    { label: study?.name || "Study" },
  ]);

  if (!session?.user) {
    return notFound();
  }

  if (loading || !study) {
    return <div>Loading...</div>;
  }

  const statusInfo = statusConfig[study.status];

  // TODO: Get actual stats from API
  const mockStats = {
    experiments: 0,
    totalTrials: 0,
    participants: 0,
    completionRate: "—",
  };

  return (
    <EntityView>
      {/* Header */}
      <EntityViewHeader
        title={study.name}
        subtitle={study.description}
        icon="Building"
        status={{
          label: statusInfo.label,
          variant: statusInfo.variant,
          icon: statusInfo.icon,
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
              <Link href={`/experiments/new?studyId=${study.id}`}>
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
                  value: study.institution,
                },
                {
                  label: "IRB Protocol",
                  value: study.irbProtocol || "Not specified",
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
                <Link href={`/experiments/new?studyId=${study.id}`}>
                  <Plus className="mr-2 h-4 w-4" />
                  Add Experiment
                </Link>
              </Button>
            }
          >
            <EmptyState
              icon="FlaskConical"
              title="No Experiments Yet"
              description="Create your first experiment to start designing research protocols"
              action={
                <Button asChild>
                  <Link href={`/experiments/new?studyId=${study.id}`}>
                    Create First Experiment
                  </Link>
                </Button>
              }
            />
          </EntityViewSection>

          {/* Recent Activity */}
          <EntityViewSection title="Recent Activity" icon="BarChart3">
            <EmptyState
              icon="Calendar"
              title="No Recent Activity"
              description="Activity will appear here once you start working on this study"
            />
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
              {members.map((member) => (
                <div
                  key={member.user.id}
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
                  value: mockStats.experiments,
                },
                {
                  label: "Total Trials",
                  value: mockStats.totalTrials,
                },
                {
                  label: "Participants",
                  value: mockStats.participants,
                },
                {
                  label: "Completion Rate",
                  value: mockStats.completionRate,
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
                  href: `/participants?studyId=${study.id}`,
                },
                {
                  label: "Schedule Trials",
                  icon: "Calendar",
                  href: `/trials?studyId=${study.id}`,
                },
                {
                  label: "View Analytics",
                  icon: "BarChart3",
                  href: `/analytics?studyId=${study.id}`,
                },
              ]}
            />
          </EntityViewSection>
        </EntityViewSidebar>
      </div>
    </EntityView>
  );
}
