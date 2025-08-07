"use client";

import { Plus, Database } from "lucide-react";
import React from "react";

import { Button } from "~/components/ui/button";
import { DataTable } from "~/components/ui/data-table";
import { EmptyState } from "~/components/ui/entity-view";

import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { ActionButton, PageHeader } from "~/components/ui/page-header";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { api } from "~/trpc/react";
import {
  repositoriesColumns,
  type Repository,
} from "~/components/admin/repositories-columns";

export function RepositoriesDataTable() {
  const [trustLevelFilter, setTrustLevelFilter] = React.useState("all");
  const [enabledFilter, setEnabledFilter] = React.useState("all");

  const {
    data: repositoriesData,
    isLoading,
    error,
    refetch,
  } = api.admin.repositories.list.useQuery(
    {
      trustLevel:
        trustLevelFilter === "all"
          ? undefined
          : (trustLevelFilter as "official" | "verified" | "community"),
      isEnabled:
        enabledFilter === "all" ? undefined : enabledFilter === "enabled",
      limit: 50,
    },
    {
      refetchOnWindowFocus: false,
    },
  );

  // Auto-refresh repositories when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void refetch();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refetch]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Administration", href: "/admin" },
    { label: "Plugin Repositories" },
  ]);

  // Transform repositories data to match the Repository type expected by columns
  const repositories: Repository[] = React.useMemo(() => {
    if (!repositoriesData) return [];
    return repositoriesData as Repository[];
  }, [repositoriesData]);

  // Trust level filter options
  const trustLevelOptions = [
    { label: "All Trust Levels", value: "all" },
    { label: "Official", value: "official" },
    { label: "Verified", value: "verified" },
    { label: "Community", value: "community" },
  ];

  // Enabled filter options
  const enabledOptions = [
    { label: "All Repositories", value: "all" },
    { label: "Enabled", value: "enabled" },
    { label: "Disabled", value: "disabled" },
  ];

  // Filter repositories based on selected filters
  const filteredRepositories = React.useMemo(() => {
    return repositories.filter((repository) => {
      const trustLevelMatch =
        trustLevelFilter === "all" ||
        repository.trustLevel === trustLevelFilter;
      const enabledMatch =
        enabledFilter === "all" ||
        (enabledFilter === "enabled" && repository.isEnabled) ||
        (enabledFilter === "disabled" && !repository.isEnabled);
      return trustLevelMatch && enabledMatch;
    });
  }, [repositories, trustLevelFilter, enabledFilter]);

  const filters = (
    <div className="flex items-center space-x-2">
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

      <Select value={enabledFilter} onValueChange={setEnabledFilter}>
        <SelectTrigger className="h-8 w-[140px]">
          <SelectValue placeholder="Status" />
        </SelectTrigger>
        <SelectContent>
          {enabledOptions.map((option) => (
            <SelectItem key={option.value} value={option.value}>
              {option.label}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
    </div>
  );

  // Show error state
  if (error) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Plugin Repositories"
          description="Manage plugin repositories for the HRIStudio platform"
          icon={Database}
          actions={
            <ActionButton href="/admin/repositories/new">
              <Plus className="mr-2 h-4 w-4" />
              Add Repository
            </ActionButton>
          }
        />
        <div className="rounded-lg border border-red-200 bg-red-50 p-6 text-center">
          <div className="text-red-800">
            <h3 className="mb-2 text-lg font-semibold">
              Failed to Load Repositories
            </h3>
            <p className="mb-4">
              {(error as unknown as Error)?.message ??
                "An error occurred while loading repositories."}
            </p>
            <Button onClick={() => void refetch()} variant="outline">
              Try Again
            </Button>
          </div>
        </div>
      </div>
    );
  }

  // Show empty state if no repositories
  if (!isLoading && repositories.length === 0) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Plugin Repositories"
          description="Manage plugin repositories for the HRIStudio platform"
          icon={Database}
          actions={
            <ActionButton href="/admin/repositories/new">
              <Plus className="mr-2 h-4 w-4" />
              Add Repository
            </ActionButton>
          }
        />
        <EmptyState
          icon="Database"
          title="No Plugin Repositories"
          description="Add plugin repositories to enable users to browse and install plugins."
          action={
            <Button asChild>
              <a href="/admin/repositories/new">Add First Repository</a>
            </Button>
          }
        />
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <PageHeader
        title="Plugin Repositories"
        description="Manage plugin repositories for the HRIStudio platform"
        icon={Database}
        actions={
          <ActionButton href="/admin/repositories/new">
            <Plus className="mr-2 h-4 w-4" />
            Add Repository
          </ActionButton>
        }
      />

      <div className="space-y-4">
        {/* Data Table */}
        <DataTable
          columns={repositoriesColumns}
          data={filteredRepositories}
          searchKey="name"
          searchPlaceholder="Search repositories..."
          isLoading={isLoading}
          loadingRowCount={5}
          filters={filters}
        />
      </div>
    </div>
  );
}
