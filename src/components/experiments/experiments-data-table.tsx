"use client";

import React from "react";

import { Button } from "~/components/ui/button";
import { DataTable } from "~/components/ui/data-table";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { useStudyContext } from "~/lib/study-context";
import { api } from "~/trpc/react";
import { experimentsColumns, type Experiment } from "./experiments-columns";

export function ExperimentsDataTable() {
  const { selectedStudyId } = useStudyContext();
  const [statusFilter, setStatusFilter] = React.useState("all");

  const columns = React.useMemo(() => {
    return experimentsColumns.filter(
      (col) => !("accessorKey" in col) || col.accessorKey !== "study",
    );
  }, []);

  const {
    data: experimentsData,
    isLoading,
    error,
    refetch,
  } = api.experiments.list.useQuery(
    { studyId: selectedStudyId ?? "" },
    {
      refetchOnWindowFocus: false,
      enabled: !!selectedStudyId,
    },
  );

  // Auto-refresh experiments when component mounts to catch external changes
  React.useEffect(() => {
    if (!selectedStudyId) return;
    const interval = setInterval(() => {
      void refetch();
    }, 30000);
    return () => clearInterval(interval);
  }, [refetch, selectedStudyId]);

  // Transform experiments data (already filtered by studyId) to match columns
  const experiments: Experiment[] = React.useMemo(() => {
    if (!experimentsData) return [];
    if (!selectedStudyId) return [];

    interface ListExperiment {
      id: string;
      name: string;
      description: string | null;
      status: Experiment["status"];
      createdAt: string | Date;
      updatedAt: string | Date;
      studyId: string;
      createdBy?: { name?: string | null; email?: string | null } | null;
      steps?: unknown[];
      trials?: unknown[];
    }

    return (experimentsData as ListExperiment[]).map((exp) => ({
      id: exp.id,
      name: exp.name,
      description: exp.description,
      status: exp.status,
      createdAt:
        exp.createdAt instanceof Date ? exp.createdAt : new Date(exp.createdAt),
      updatedAt:
        exp.updatedAt instanceof Date ? exp.updatedAt : new Date(exp.updatedAt),
      studyId: exp.studyId,
      study: {
        id: exp.studyId,
        name: "Active Study",
      },
      createdBy: exp.createdBy?.name ?? exp.createdBy?.email ?? "",
      owner: {
        name: exp.createdBy?.name ?? null,
        email: exp.createdBy?.email ?? "",
      },
      _count: {
        steps: Array.isArray(exp.steps) ? exp.steps.length : 0,
        trials: Array.isArray(exp.trials) ? exp.trials.length : 0,
      },
      userRole: undefined,
      canEdit: true,
      canDelete: true,
    }));
  }, [experimentsData, selectedStudyId]);

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
        <SelectTrigger className="h-8 w-[140px]">
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
    );
  }

  return (
    <div className="space-y-4">
      <DataTable
        columns={columns}
        data={filteredExperiments}
        searchKey="name"
        searchPlaceholder="Search experiments..."
        isLoading={isLoading}
        loadingRowCount={5}
        filters={filters}
      />
    </div>
  );
}
