"use client";

import { useState } from "react";
import { Plus } from "lucide-react";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { CreateStudyDialog } from "./CreateStudyDialog";
import { StudyCard } from "./StudyCard";
import { api } from "~/trpc/react";

type StudyWithRelations = {
  id: string;
  name: string;
  description: string;
  status: "draft" | "active" | "completed" | "archived";
  institution: string;
  irbProtocolNumber: string | null;
  createdAt: Date;
  updatedAt: Date;
  ownerId: string;
  createdBy: {
    id: string;
    name: string | null;
    email: string;
  };
  members: Array<{
    role: "owner" | "researcher" | "wizard" | "observer";
    user: {
      id: string;
      name: string | null;
      email: string;
    };
  }>;
  experiments?: Array<{ id: string }>;
  participants?: Array<{ id: string }>;
};

type ProcessedStudy = {
  id: string;
  name: string;
  description: string;
  status: "draft" | "active" | "completed" | "archived";
  institution: string;
  irbProtocolNumber?: string;
  createdAt: Date;
  updatedAt: Date;
  ownerId: string;
  owner: {
    name: string | null;
    email: string;
  };
  userRole?: "owner" | "researcher" | "wizard" | "observer";
  isOwner?: boolean;
  _count?: {
    experiments: number;
    trials: number;
    studyMembers: number;
    participants: number;
  };
};

export function StudiesGrid() {
  const [refreshKey, setRefreshKey] = useState(0);
  const { data: session } = api.auth.me.useQuery();

  const {
    data: studiesData,
    isLoading,
    error,
    refetch,
  } = api.studies.list.useQuery(
    { memberOnly: true },
    {
      refetchOnWindowFocus: false,
    },
  );

  const processStudies = (
    rawStudies: StudyWithRelations[],
  ): ProcessedStudy[] => {
    const currentUserId = session?.id;

    return rawStudies.map((study) => {
      // Find current user's membership
      const userMembership = study.members?.find(
        (member) => member.user.id === currentUserId,
      );

      return {
        id: study.id,
        name: study.name,
        description: study.description,
        status: study.status,
        institution: study.institution,
        irbProtocolNumber: study.irbProtocolNumber ?? undefined,
        createdAt: study.createdAt,
        updatedAt: study.updatedAt,
        ownerId: study.ownerId,
        owner: {
          name: study.createdBy.name,
          email: study.createdBy.email,
        },
        userRole: userMembership?.role,
        isOwner: study.ownerId === currentUserId,
        _count: {
          experiments: study.experiments?.length ?? 0,
          trials: 0, // Will be populated when trials relation is added
          studyMembers: study.members?.length ?? 0,
          participants: study.participants?.length ?? 0,
        },
      };
    });
  };

  const studies = studiesData?.studies
    ? processStudies(studiesData.studies)
    : [];

  const handleStudyCreated = () => {
    setRefreshKey((prev) => prev + 1);
    void refetch();
  };

  if (isLoading) {
    return (
      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
        {/* Create Study Card Skeleton */}
        <Card className="border-2 border-dashed border-slate-300">
          <CardHeader className="text-center">
            <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-blue-100">
              <Plus className="h-8 w-8 text-blue-600" />
            </div>
            <CardTitle>Create New Study</CardTitle>
            <CardDescription>Start a new HRI research study</CardDescription>
          </CardHeader>
          <CardContent>
            <CreateStudyDialog onSuccess={handleStudyCreated}>
              <Button className="w-full">Create Study</Button>
            </CreateStudyDialog>
          </CardContent>
        </Card>

        {/* Loading Skeletons */}
        {Array.from({ length: 5 }).map((_, i) => (
          <Card key={i} className="animate-pulse">
            <CardHeader>
              <div className="flex items-start justify-between">
                <div className="flex-1 space-y-2">
                  <div className="h-5 w-3/4 rounded bg-slate-200"></div>
                  <div className="h-4 w-full rounded bg-slate-200"></div>
                  <div className="h-4 w-2/3 rounded bg-slate-200"></div>
                </div>
                <div className="h-6 w-16 rounded bg-slate-200"></div>
              </div>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="space-y-2">
                <div className="h-4 w-3/4 rounded bg-slate-200"></div>
                <div className="h-4 w-1/2 rounded bg-slate-200"></div>
              </div>
              <div className="h-px bg-slate-200"></div>
              <div className="grid grid-cols-2 gap-4">
                <div className="space-y-2">
                  <div className="h-3 rounded bg-slate-200"></div>
                  <div className="h-3 rounded bg-slate-200"></div>
                </div>
                <div className="space-y-2">
                  <div className="h-3 rounded bg-slate-200"></div>
                  <div className="h-3 rounded bg-slate-200"></div>
                </div>
              </div>
              <div className="h-px bg-slate-200"></div>
              <div className="flex gap-2">
                <div className="h-8 flex-1 rounded bg-slate-200"></div>
                <div className="h-8 flex-1 rounded bg-slate-200"></div>
              </div>
            </CardContent>
          </Card>
        ))}
      </div>
    );
  }

  if (error) {
    return (
      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
        {/* Create Study Card */}
        <Card className="border-2 border-dashed border-slate-300 transition-colors hover:border-slate-400">
          <CardHeader className="text-center">
            <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-blue-100">
              <Plus className="h-8 w-8 text-blue-600" />
            </div>
            <CardTitle>Create New Study</CardTitle>
            <CardDescription>Start a new HRI research study</CardDescription>
          </CardHeader>
          <CardContent>
            <CreateStudyDialog onSuccess={handleStudyCreated}>
              <Button className="w-full">Create Study</Button>
            </CreateStudyDialog>
          </CardContent>
        </Card>

        {/* Error State */}
        <Card className="md:col-span-2">
          <CardContent className="pt-6">
            <div className="text-center">
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
                Failed to Load Studies
              </h3>
              <p className="mb-4 text-slate-600">
                {error.message ||
                  "An error occurred while loading your studies."}
              </p>
              <Button onClick={() => refetch()} variant="outline">
                Try Again
              </Button>
            </div>
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
      {/* Create New Study Card */}
      <Card className="border-2 border-dashed border-slate-300 transition-colors hover:border-slate-400">
        <CardHeader className="text-center">
          <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-blue-100">
            <Plus className="h-8 w-8 text-blue-600" />
          </div>
          <CardTitle>Create New Study</CardTitle>
          <CardDescription>Start a new HRI research study</CardDescription>
        </CardHeader>
        <CardContent>
          <CreateStudyDialog onSuccess={handleStudyCreated}>
            <Button className="w-full">Create Study</Button>
          </CreateStudyDialog>
        </CardContent>
      </Card>

      {/* Studies */}
      {studies.map((study) => (
        <StudyCard
          key={study.id}
          study={study}
          userRole={study.userRole}
          isOwner={study.isOwner}
        />
      ))}

      {/* Empty State */}
      {studies.length === 0 && (
        <Card className="md:col-span-2 lg:col-span-2">
          <CardContent className="pt-6">
            <div className="text-center">
              <div className="mx-auto mb-4 flex h-24 w-24 items-center justify-center rounded-lg bg-slate-100">
                <svg
                  className="h-12 w-12 text-slate-400"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z"
                  />
                </svg>
              </div>
              <h3 className="mb-2 text-lg font-semibold text-slate-900">
                No Studies Yet
              </h3>
              <p className="mb-4 text-slate-600">
                Get started by creating your first Human-Robot Interaction
                research study. Studies help you organize experiments, manage
                participants, and collaborate with your team.
              </p>
              <CreateStudyDialog onSuccess={handleStudyCreated}>
                <Button>Create Your First Study</Button>
              </CreateStudyDialog>
            </div>
          </CardContent>
        </Card>
      )}
    </div>
  );
}
