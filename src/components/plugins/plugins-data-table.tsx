"use client";

import Link from "next/link";
import React from "react";

import { Button } from "~/components/ui/button";
import { DataTable } from "~/components/ui/data-table";
import { EmptyState } from "~/components/ui/entity-view";

import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { useStudyContext } from "~/lib/study-context";
import { api } from "~/trpc/react";
import { pluginsColumns, type Plugin } from "./plugins-columns";

export function PluginsDataTable() {
  const [statusFilter, setStatusFilter] = React.useState("all");
  const [trustLevelFilter, setTrustLevelFilter] = React.useState("all");
  const { selectedStudyId } = useStudyContext();

  const {
    data: pluginsData,
    isLoading,
    error,
    refetch,
  } = api.robots.plugins.getStudyPlugins.useQuery(
    {
      studyId: selectedStudyId!,
    },
    {
      enabled: !!selectedStudyId,
      refetchOnWindowFocus: false,
    },
  );

  // Auto-refresh plugins when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void refetch();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refetch]);

  // Get study data for breadcrumbs

  // Transform plugins data to match the Plugin type expected by columns
  const plugins: Plugin[] = React.useMemo(() => {
    if (!pluginsData) return [];
    return pluginsData as Plugin[];
  }, [pluginsData]);

  // Status filter options
  const statusOptions = [
    { label: "All Statuses", value: "all" },
    { label: "Active", value: "active" },
    { label: "Deprecated", value: "deprecated" },
    { label: "Disabled", value: "disabled" },
  ];

  // Trust level filter options
  const trustLevelOptions = [
    { label: "All Trust Levels", value: "all" },
    { label: "Official", value: "official" },
    { label: "Verified", value: "verified" },
    { label: "Community", value: "community" },
  ];

  // Filter plugins based on selected filters
  const filteredPlugins = React.useMemo(() => {
    return plugins.filter((plugin) => {
      const statusMatch =
        statusFilter === "all" || plugin.plugin.status === statusFilter;
      const trustLevelMatch =
        trustLevelFilter === "all" ||
        plugin.plugin.trustLevel === trustLevelFilter;
      return statusMatch && trustLevelMatch;
    });
  }, [plugins, statusFilter, trustLevelFilter]);

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

      <Select value={trustLevelFilter} onValueChange={setTrustLevelFilter}>
        <SelectTrigger className="h-8 w-[160px]">
          <SelectValue placeholder="Trust Level" />
        </SelectTrigger>
        <SelectContent>
          {trustLevelOptions.map((option) => (
            <SelectItem key={option.value} value={option.value}>
              {option.label}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
    </div>
  );

  // Show message if no study is selected
  if (!selectedStudyId) {
    return (
      <EmptyState
        icon="Building"
        title="No Study Selected"
        description="Please select a study from the sidebar to view and manage plugins."
        action={
          <Button asChild>
            <Link href="/studies">Select Study</Link>
          </Button>
        }
      />
    );
  }

  // Show error state
  if (error) {
    return (
      <div className="rounded-lg border border-red-200 bg-red-50 p-6 text-center">
        <div className="text-red-800">
          <h3 className="mb-2 text-lg font-semibold">Failed to Load Plugins</h3>
          <p className="mb-4">
            {error.message || "An error occurred while loading plugins."}
          </p>
          <Button onClick={() => refetch()} variant="outline">
            Try Again
          </Button>
        </div>
      </div>
    );
  }

  // Show empty state if no plugins
  if (!isLoading && plugins.length === 0) {
    return (
      <EmptyState
        icon="Puzzle"
        title="No plugins installed"
        description="Browse and install plugins to extend your robot's capabilities for this study."
        action={
          <Button asChild>
            <Link href="/plugins/browse">Browse Plugins</Link>
          </Button>
        }
      />
    );
  }

  return (
    <div className="space-y-4">
      <DataTable
        columns={pluginsColumns}
        data={filteredPlugins}
        searchKey="name"
        searchPlaceholder="Search plugins..."
        isLoading={isLoading}
        loadingRowCount={5}
        filters={filters}
      />
    </div>
  );
}
