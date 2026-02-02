"use client";

import React from "react";
import Link from "next/link";
import { Plus, FlaskConical } from "lucide-react";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { PageHeader, ActionButton } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyManagement } from "~/hooks/useStudyManagement";
import { StudyCard } from "./StudyCard";
import { api } from "~/trpc/react";

type StudyWithRelations = {
  id: string;
  name: string;
  description: string | null;
  status: "draft" | "active" | "completed" | "archived";
  createdAt: Date;
  updatedAt: Date;
  institution: string | null;
  irbProtocol: string | null;
  createdBy: string;
  members?: Array<{
    id: string;
    role: "owner" | "researcher" | "wizard" | "observer";
    user: {
      id: string;
      name: string | null;
      email: string;
    };
  }>;
  experiments?: Array<{
    id: string;
    name: string;
  }>;
  trials?: Array<{
    id: string;
    name: string;
  }>;
  participants?: Array<{
    id: string;
    name: string;
  }>;
  _count?: {
    experiments: number;
    trials: number;
    studyMembers: number;
    participants: number;
  };
};

type ProcessedStudy = {
  id: string;
  name: string;
  description: string | null;
  status: "draft" | "active" | "completed" | "archived";
  createdAt: Date;
  updatedAt: Date;
  institution: string | null;
  irbProtocolNumber?: string;
  ownerId?: string;
  owner: {
    name: string | null;
    email: string;
  };
  _count?: {
    experiments: number;
    trials: number;
    studyMembers: number;
    participants: number;
  };
  userRole?: "owner" | "researcher" | "wizard" | "observer";
  isOwner?: boolean;
};

// Process studies helper function
const processStudies = (
  rawStudies: StudyWithRelations[],
  currentUserId?: string,
): ProcessedStudy[] => {
  return rawStudies.map((study) => {
    // Find current user's membership
    const userMembership = study.members?.find(
      (member) => member.user.id === currentUserId,
    );

    // Find owner from members
    const owner = study.members?.find((member) => member.role === "owner");

    return {
      id: study.id,
      name: study.name,
      description: study.description,
      status: study.status,
      createdAt: study.createdAt,
      updatedAt: study.updatedAt,
      institution: study.institution,
      irbProtocolNumber: study.irbProtocol ?? undefined,
      ownerId: owner?.user.id,
      owner: {
        name: owner?.user.name ?? null,
        email: owner?.user.email ?? "",
      },
      _count: {
        experiments:
          study._count?.experiments ?? study.experiments?.length ?? 0,
        trials: study._count?.trials ?? study.trials?.length ?? 0,
        studyMembers: study._count?.studyMembers ?? study.members?.length ?? 0,
        participants:
          study._count?.participants ?? study.participants?.length ?? 0,
      },
      userRole: userMembership?.role,
      isOwner: userMembership?.role === "owner",
    };
  });
};

export function StudiesGrid() {
  const { data: session } = api.auth.me.useQuery();
  const { userStudies, isLoadingUserStudies, refreshStudyData } =
    useStudyManagement();

  // Auto-refresh studies when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void refreshStudyData();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refreshStudyData]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies" },
  ]);

  // Process studies data
  const studies = userStudies ? processStudies(userStudies, session?.id) : [];
  const isLoading = isLoadingUserStudies;

  if (isLoading) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Studies"
          description="Manage your Human-Robot Interaction research studies"
          icon={FlaskConical}
          actions={
            <ActionButton href="/studies/new">
              <Plus className="mr-2 h-4 w-4" />
              New Study
            </ActionButton>
          }
        />
        <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
          {Array.from({ length: 6 }).map((_, i) => (
            <Card key={i} className="animate-pulse">
              <CardHeader>
                <div className="h-4 w-3/4 rounded bg-slate-200"></div>
                <div className="h-3 w-1/2 rounded bg-slate-200"></div>
              </CardHeader>
              <CardContent>
                <div className="space-y-2">
                  <div className="h-3 w-full rounded bg-slate-200"></div>
                  <div className="h-3 w-2/3 rounded bg-slate-200"></div>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <PageHeader
        title="Studies"
        description="Manage your Human-Robot Interaction research studies"
        icon={FlaskConical}
        actions={
          <ActionButton href="/studies/new">
            <Plus className="mr-2 h-4 w-4" />
            New Study
          </ActionButton>
        }
      />

      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
        {/* Create Study Card */}
        <Card className="border-2 border-dashed border-slate-200 transition-colors hover:border-slate-300">
          <CardHeader>
            <CardTitle className="text-slate-600">Create New Study</CardTitle>
            <CardDescription>Start a new HRI research study</CardDescription>
          </CardHeader>
          <CardContent>
            <Button className="w-full" asChild>
              <Link href="/studies/new">Create Study</Link>
            </Button>
          </CardContent>
        </Card>

        {/* Study Cards */}
        {studies.map((study) => (
          <StudyCard
            key={study.id}
            study={study}
            userRole={study.userRole}
            isOwner={study.isOwner}
          />
        ))}

        {/* Add more create study cards for empty slots */}
        {studies.length > 0 && studies.length < 3 && (
          <Card className="border-2 border-dashed border-slate-200 transition-colors hover:border-slate-300">
            <CardHeader>
              <CardTitle className="text-slate-600">Create New Study</CardTitle>
              <CardDescription>Start a new HRI research study</CardDescription>
            </CardHeader>
            <CardContent>
              <Button className="w-full" asChild>
                <Link href="/studies/new">Create Study</Link>
              </Button>
            </CardContent>
          </Card>
        )}

        {studies.length > 3 && studies.length < 6 && (
          <Card className="border-2 border-dashed border-slate-200 transition-colors hover:border-slate-300">
            <CardHeader>
              <CardTitle className="text-slate-600">Create New Study</CardTitle>
              <CardDescription>Start a new HRI research study</CardDescription>
            </CardHeader>
            <CardContent>
              <Button className="w-full" asChild>
                <Link href="/studies/new">Create Study</Link>
              </Button>
            </CardContent>
          </Card>
        )}

        {/* Empty State */}
        {studies.length === 0 && (
          <Card className="col-span-full">
            <CardContent className="flex flex-col items-center justify-center py-16">
              <div className="mx-auto max-w-sm text-center">
                <FlaskConical className="mx-auto h-12 w-12 text-slate-400" />
                <h3 className="mb-2 text-lg font-semibold text-slate-900">
                  No Studies Yet
                </h3>
                <p className="mb-4 text-slate-600">
                  Get started by creating your first Human-Robot Interaction
                  research study. Studies help you organize experiments, manage
                  participants, and collaborate with your team.
                </p>
                <Button asChild>
                  <Link href="/studies/new">Create Your First Study</Link>
                </Button>
              </div>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
}
