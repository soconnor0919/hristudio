"use client";

import React from "react";
import { Plus, FlaskConical } from "lucide-react";

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
import { useActiveStudy } from "~/hooks/useActiveStudy";
import { experimentsColumns, type Experiment } from "./experiments-columns";
import { api } from "~/trpc/react";

export function ExperimentsDataTable() {
  const { activeStudy } = useActiveStudy();
  const [statusFilter, setStatusFilter] = React.useState("all");

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

  // Auto-refresh experiments when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void refetch();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refetch]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    ...(activeStudy
      ? [{ label: activeStudy.title, href: `/studies/${activeStudy.id}` }]
      : []),
    { label: "Experiments" },
  ]);

  // Transform experiments data to match the Experiment type expected by columns
  const experiments: Experiment[] = React.useMemo(() => {
    if (!experimentsData?.experiments) return [];

    return experimentsData.experiments.map((experiment) => ({
      id: experiment.id,
      name: experiment.name,
      description: experiment.description,
      status: experiment.status,
      createdAt: experiment.createdAt,
      updatedAt: experiment.updatedAt,
      studyId: experiment.studyId,
      study: experiment.study,
      createdBy: experiment.createdBy ?? "",
      owner: {
        name: experiment.createdBy?.name ?? null,
        email: experiment.createdBy?.email ?? "",
      },
      _count: {
        steps: experiment._count?.steps ?? 0,
        trials: experiment._count?.trials ?? 0,
      },
      userRole: undefined,
      canEdit: true,
      canDelete: true,
    }));
  }, [experimentsData]);

  // Status filter options
  const statusOptions = [
    { label: "All Statuses", value: "all" },
    { label: "Draft", value: "draft" },
    { label: "Testing", value: "testing" },
    { label: "Ready", value: "ready" },
    { label: "Deprecated", value: "deprecated" },
  ];

  // Filter experiments based on selected filters
  const filteredExperiments = React.useMemo(() => {
    return experiments.filter((experiment) => {
      const statusMatch =
        statusFilter === "all" || experiment.status === statusFilter;
      return statusMatch;
    });
  }, [experiments, statusFilter]);

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
          title="Experiments"
          description="Design and manage experimental protocols for your HRI studies"
          icon={FlaskConical}
          actions={
            <ActionButton href="/experiments/new">
              <Plus className="mr-2 h-4 w-4" />
              New Experiment
            </ActionButton>
          }
        />
        <div className="rounded-lg border border-red-200 bg-red-50 p-6 text-center">
          <div className="text-red-800">
            <h3 className="mb-2 text-lg font-semibold">
              Failed to Load Experiments
            </h3>
            <p className="mb-4">
              {error.message ||
                "An error occurred while loading your experiments."}
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
        title="Experiments"
        description="Design and manage experimental protocols for your HRI studies"
        icon={FlaskConical}
        actions={
          <ActionButton href="/experiments/new">
            <Plus className="mr-2 h-4 w-4" />
            New Experiment
          </ActionButton>
        }
      />

      <div className="space-y-4">
        <DataTable
          columns={experimentsColumns}
          data={filteredExperiments}
          searchKey="name"
          searchPlaceholder="Search experiments..."
          isLoading={isLoading}
          loadingRowCount={5}
          filters={filters}
        />
      </div>
    </div>
  );
}
