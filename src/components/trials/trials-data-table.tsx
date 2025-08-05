"use client";

import React from "react";
import { Plus, TestTube, Eye } from "lucide-react";

import { Button } from "~/components/ui/button";
import { DataTable } from "~/components/ui/data-table";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { PageHeader, ActionButton } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";

import { trialsColumns, type Trial } from "./trials-columns";
import { api } from "~/trpc/react";

export function TrialsDataTable() {
  const [statusFilter, setStatusFilter] = React.useState("all");
  const { selectedStudyId } = useStudyContext();

  const {
    data: trialsData,
    isLoading,
    error,
    refetch,
  } = api.trials.getUserTrials.useQuery(
    {
      page: 1,
      limit: 50,
      studyId: selectedStudyId ?? undefined,
      status:
        statusFilter === "all"
          ? undefined
          : (statusFilter as
              | "scheduled"
              | "in_progress"
              | "completed"
              | "aborted"
              | "failed"),
    },
    {
      refetchOnWindowFocus: false,
      refetchInterval: 30000, // Refetch every 30 seconds for real-time updates
      enabled: !!selectedStudyId, // Only fetch when a study is selected
    },
  );

  // Auto-refresh trials when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void refetch();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refetch]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Trials" },
  ]);

  // Transform trials data to match the Trial type expected by columns
  const trials: Trial[] = React.useMemo(() => {
    if (!trialsData?.trials) return [];

    return trialsData.trials.map((trial) => ({
      id: trial.id,
      name: trial.notes
        ? `Trial: ${trial.notes}`
        : `Trial ${trial.sessionNumber || trial.id.slice(-8)}`,
      description: trial.notes,
      status: trial.status,
      scheduledAt: trial.scheduledAt ? new Date(trial.scheduledAt) : null,
      startedAt: trial.startedAt ? new Date(trial.startedAt) : null,
      completedAt: trial.completedAt ? new Date(trial.completedAt) : null,
      createdAt: trial.createdAt,
      updatedAt: trial.updatedAt,
      studyId: trial.experiment?.studyId ?? "",
      experimentId: trial.experimentId,
      participantId: trial.participantId ?? "",
      wizardId: trial.wizardId,
      study: {
        id: trial.experiment?.studyId ?? "",
        name: trial.experiment?.study?.name ?? "",
      },
      experiment: {
        id: trial.experimentId,
        name: trial.experiment?.name ?? "",
      },
      participant: {
        id: trial.participantId ?? "",
        name:
          trial.participant?.name ?? trial.participant?.participantCode ?? "",
        email: trial.participant?.email ?? "",
      },
      wizard: trial.wizard
        ? {
            id: trial.wizard.id,
            name: trial.wizard.name,
            email: trial.wizard.email,
          }
        : null,
      duration: trial.duration ? Math.round(trial.duration / 60) : undefined,
      _count: {
        actions: trial._count?.events ?? 0,
        logs: trial._count?.mediaCaptures ?? 0,
      },
      userRole: trial.userRole,
      canAccess: trial.canAccess ?? false,
      canEdit:
        trial.canAccess &&
        (trial.status === "scheduled" || trial.status === "aborted"),
      canDelete:
        trial.canAccess &&
        (trial.status === "scheduled" ||
          trial.status === "aborted" ||
          trial.status === "failed"),
      canExecute:
        trial.canAccess &&
        (trial.status === "scheduled" || trial.status === "in_progress"),
    }));
  }, [trialsData]);

  // Status filter options
  const statusOptions = [
    { label: "All Statuses", value: "all" },
    { label: "Scheduled", value: "scheduled" },
    { label: "In Progress", value: "in_progress" },
    { label: "Completed", value: "completed" },
    { label: "Aborted", value: "aborted" },
    { label: "Failed", value: "failed" },
  ];

  // Filter trials based on selected filters
  const filteredTrials = React.useMemo(() => {
    return trials.filter((trial) => {
      const statusMatch =
        statusFilter === "all" || trial.status === statusFilter;
      return statusMatch;
    });
  }, [trials, statusFilter]);

  const filters = (
    <div className="flex items-center space-x-2">
      <Select value={statusFilter} onValueChange={setStatusFilter}>
        <SelectTrigger className="w-[140px]">
          <SelectValue placeholder="Status" />
        </SelectTrigger>
        <SelectContent>
          {statusOptions.map((option) => (
            <SelectItem key={option.value} value={option.value}>
              {option.label}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
    </div>
  );

  if (error) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Trials"
          description="Monitor and manage trial execution for your HRI experiments"
          icon={TestTube}
          actions={
            <ActionButton href="/trials/new">
              <Plus className="mr-2 h-4 w-4" />
              New Trial
            </ActionButton>
          }
        />
        <div className="rounded-lg border border-red-200 bg-red-50 p-6 text-center">
          <div className="text-red-800">
            <h3 className="mb-2 text-lg font-semibold">
              Failed to Load Trials
            </h3>
            <p className="mb-4">
              {error.message || "An error occurred while loading your trials."}
            </p>
            <Button onClick={() => refetch()} variant="outline">
              Try Again
            </Button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <PageHeader
        title="Trials"
        description="Monitor and manage trial execution for your HRI experiments"
        icon={TestTube}
        actions={
          <ActionButton href="/trials/new">
            <Plus className="mr-2 h-4 w-4" />
            New Trial
          </ActionButton>
        }
      />

      <div className="space-y-4">
        {filteredTrials.some((trial) => !trial.canAccess) && (
          <div className="rounded-lg border border-amber-200 bg-amber-50 p-4">
            <div className="flex items-start gap-3">
              <div className="mt-0.5 flex-shrink-0">
                <div className="rounded-full bg-amber-100 p-1">
                  <Eye className="h-4 w-4 text-amber-600" />
                </div>
              </div>
              <div>
                <h3 className="text-sm font-medium text-amber-800">
                  Limited Trial Access
                </h3>
                <p className="mt-1 text-sm text-amber-700">
                  Some trials are marked as "View Only" or "Restricted" because
                  you have observer-level access to their studies. Only
                  researchers, wizards, and study owners can view detailed trial
                  information.
                </p>
              </div>
            </div>
          </div>
        )}

        <DataTable
          columns={trialsColumns}
          data={filteredTrials}
          searchKey="name"
          searchPlaceholder="Search trials..."
          isLoading={isLoading}
          loadingRowCount={5}
          filters={filters}
        />
      </div>
    </div>
  );
}
