"use client";

import { formatDistanceToNow } from "date-fns";
import { Calendar, FlaskConical, Plus, Settings, Users } from "lucide-react";
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

type ExperimentWithRelations = {
  id: string;
  name: string;
  description: string | null;
  status: "draft" | "testing" | "ready" | "deprecated";
  estimatedDuration: number | null;
  createdAt: Date;
  updatedAt: Date;
  studyId: string;
  createdById?: string;
  study: {
    id: string;
    name: string;
  };
  createdBy: {
    id: string;
    name: string | null;
    email: string;
  };
  _count?: {
    steps: number;
    trials: number;
  };
};

const statusConfig = {
  draft: {
    label: "Draft",
    className: "bg-gray-100 text-gray-800 hover:bg-gray-200",
    icon: "📝",
  },
  testing: {
    label: "Testing",
    className: "bg-yellow-100 text-yellow-800 hover:bg-yellow-200",
    icon: "🧪",
  },
  ready: {
    label: "Ready",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    icon: "✅",
  },
  deprecated: {
    label: "Deprecated",
    className: "bg-red-100 text-red-800 hover:bg-red-200",
    icon: "🗑️",
  },
};

interface ExperimentCardProps {
  experiment: ExperimentWithRelations;
}

function ExperimentCard({ experiment }: ExperimentCardProps) {
  const statusInfo = statusConfig[experiment.status];

  return (
    <Card className="group transition-all duration-200 hover:border-slate-300 hover:shadow-md">
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between">
          <div className="min-w-0 flex-1">
            <CardTitle className="truncate text-lg font-semibold text-slate-900 transition-colors group-hover:text-blue-600">
              <Link
                href={`/studies/${experiment.studyId}/experiments/${experiment.id}`}
                className="hover:underline"
              >
                {experiment.name}
              </Link>
            </CardTitle>
            <CardDescription className="mt-1 line-clamp-2 text-sm text-slate-600">
              {experiment.description}
            </CardDescription>
            <div className="mt-2 flex items-center text-xs text-slate-500">
              <span>Study: </span>
              <Link
                href={`/studies/${experiment.study.id}`}
                className="ml-1 font-medium text-blue-600 hover:text-blue-800"
              >
                {experiment.study.name}
              </Link>
            </div>
          </div>
          <Badge className={statusInfo.className} variant="secondary">
            <span className="mr-1">{statusInfo.icon}</span>
            {statusInfo.label}
          </Badge>
        </div>
      </CardHeader>

      <CardContent className="space-y-4">
        {/* Experiment Metadata */}
        <div className="space-y-1">
          {experiment.estimatedDuration && (
            <div className="flex items-center text-sm text-slate-600">
              <Calendar className="mr-2 h-4 w-4" />
              Estimated duration: {experiment.estimatedDuration} minutes
            </div>
          )}
          <div className="flex items-center text-sm text-slate-600">
            <Users className="mr-2 h-4 w-4" />
            Created by {experiment.createdBy.name ?? experiment.createdBy.email}
          </div>
        </div>

        {/* Statistics */}
        {experiment._count && (
          <>
            <Separator />
            <div className="grid grid-cols-2 gap-4 text-sm">
              <div className="flex justify-between">
                <span className="text-slate-600">Steps:</span>
                <span className="font-medium">{experiment._count.steps}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-600">Trials:</span>
                <span className="font-medium">{experiment._count.trials}</span>
              </div>
            </div>
          </>
        )}

        {/* Metadata */}
        <Separator />
        <div className="space-y-1 text-xs text-slate-500">
          <div className="flex justify-between">
            <span>Created:</span>
            <span>
              {formatDistanceToNow(experiment.createdAt, { addSuffix: true })}
            </span>
          </div>
          {experiment.updatedAt !== experiment.createdAt && (
            <div className="flex justify-between">
              <span>Updated:</span>
              <span>
                {formatDistanceToNow(experiment.updatedAt, { addSuffix: true })}
              </span>
            </div>
          )}
        </div>

        {/* Actions */}
        <div className="flex gap-2 pt-2">
          <Button asChild size="sm" className="flex-1">
            <Link href={`/studies/${experiment.studyId}/experiments/${experiment.id}`}>View Details</Link>
          </Button>
          <Button asChild size="sm" variant="outline" className="flex-1">
            <Link href={`/studies/${experiment.studyId}/experiments/${experiment.id}/designer`}>
              <Settings className="mr-1 h-3 w-3" />
              Design
            </Link>
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}

export function ExperimentsGrid() {
  const {
    data: experimentsData,
    isLoading,
    error,
    refetch,
  } = api.experiments.getUserExperiments.useQuery(
    { page: 1, limit: 50 },
    {
      refetchOnWindowFocus: false,
    },
  );

  const experiments = experimentsData?.experiments ?? [];

  if (isLoading) {
    return (
      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
        {/* Create Experiment Card */}
        <Card className="border-2 border-dashed border-slate-300">
          <CardHeader className="text-center">
            <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-blue-100">
              <Plus className="h-8 w-8 text-blue-600" />
            </div>
            <CardTitle>Create New Experiment</CardTitle>
            <CardDescription>
              Design a new experimental protocol
            </CardDescription>
          </CardHeader>
          <CardContent>
            <Button asChild className="w-full">
              <Link href="/experiments/new">Create Experiment</Link>
            </Button>
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
                  <div className="h-3 w-1/2 rounded bg-slate-200"></div>
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
                <div className="h-3 rounded bg-slate-200"></div>
                <div className="h-3 rounded bg-slate-200"></div>
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
        {/* Create Experiment Card */}
        <Card className="border-2 border-dashed border-slate-300 transition-colors hover:border-slate-400">
          <CardHeader className="text-center">
            <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-blue-100">
              <Plus className="h-8 w-8 text-blue-600" />
            </div>
            <CardTitle>Create New Experiment</CardTitle>
            <CardDescription>
              Design a new experimental protocol
            </CardDescription>
          </CardHeader>
          <CardContent>
            <Button asChild className="w-full">
              <Link href="/experiments/new">Create Experiment</Link>
            </Button>
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
                Failed to Load Experiments
              </h3>
              <p className="mb-4 text-slate-600">
                {error?.message ??
                  "An error occurred while loading your experiments."}
              </p>
              <Button onClick={() => void refetch()} variant="outline">
                Try Again
              </Button>
            </div>
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      {/* Header */}
      <div>
        <h1 className="text-3xl font-bold tracking-tight">Experiments</h1>
        <p className="text-muted-foreground">
          Design and manage experimental protocols for your HRI studies
        </p>
      </div>

      {/* Grid */}
      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
        {/* Create New Experiment Card */}
        <Card className="border-2 border-dashed border-slate-300 transition-colors hover:border-slate-400">
          <CardHeader className="text-center">
            <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-blue-100">
              <Plus className="h-8 w-8 text-blue-600" />
            </div>
            <CardTitle>Create New Experiment</CardTitle>
            <CardDescription>
              Design a new experimental protocol
            </CardDescription>
          </CardHeader>
          <CardContent>
            <Button asChild className="w-full">
              <Link href="/experiments/new">Create Experiment</Link>
            </Button>
          </CardContent>
        </Card>

        {/* Experiments */}
        {experiments.map((experiment) => (
          <ExperimentCard key={experiment.id} experiment={experiment} />
        ))}

        {/* Empty State */}
        {experiments.length === 0 && (
          <Card className="md:col-span-2 lg:col-span-2">
            <CardContent className="pt-6">
              <div className="text-center">
                <div className="mx-auto mb-4 flex h-24 w-24 items-center justify-center rounded-lg bg-slate-100">
                  <FlaskConical className="h-12 w-12 text-slate-400" />
                </div>
                <h3 className="mb-2 text-lg font-semibold text-slate-900">
                  No Experiments Yet
                </h3>
                <p className="mb-4 text-slate-600">
                  Create your first experiment to start designing HRI protocols.
                  Experiments define the structure and flow of your research
                  trials.
                </p>
                <Button asChild>
                  <Link href="/experiments/new">
                    Create Your First Experiment
                  </Link>
                </Button>
              </div>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
}
