"use client";

import {
  Puzzle,
  Search,
  Filter,
  ExternalLink,
  Download,
  Shield,
  User,
  Calendar,
  Database,
} from "lucide-react";
import React from "react";
import { formatDistanceToNow } from "date-fns";
import { toast } from "sonner";

import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { PageHeader } from "~/components/ui/page-header";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { useStudyContext } from "~/lib/study-context";
import { api } from "~/trpc/react";

interface PluginStoreItem {
  id: string;
  robotId: string | null;
  name: string;
  version: string;
  description: string | null;
  author: string | null;
  repositoryUrl: string | null;
  trustLevel: "official" | "verified" | "community" | null;
  status: "active" | "deprecated" | "disabled";
  createdAt: Date;
  updatedAt: Date;
  metadata: unknown;
}

const trustLevelConfig = {
  official: {
    label: "Official",
    className: "bg-blue-100 text-blue-800",
    icon: Shield,
    description: "Official HRIStudio plugin",
  },
  verified: {
    label: "Verified",
    className: "bg-green-100 text-green-800",
    icon: Shield,
    description: "Verified by the community",
  },
  community: {
    label: "Community",
    className: "bg-yellow-100 text-yellow-800",
    icon: User,
    description: "Community contributed",
  },
};

function PluginCard({
  plugin,
  onInstall,
  repositoryName,
  isInstalled,
}: {
  plugin: PluginStoreItem;
  onInstall: (pluginId: string) => void;
  repositoryName?: string;
  isInstalled?: boolean;
}) {
  const trustLevel = plugin.trustLevel;
  const trustConfig = trustLevel ? trustLevelConfig[trustLevel] : null;
  const TrustIcon = trustConfig?.icon ?? User;

  return (
    <Card className="flex h-full flex-col">
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between">
          <div className="flex min-w-0 flex-1 items-center space-x-2">
            <Puzzle className="text-muted-foreground h-5 w-5 flex-shrink-0" />
            <div className="min-w-0">
              <CardTitle className="truncate text-base">
                {plugin.name}
              </CardTitle>
              <div className="mt-1 flex items-center space-x-2">
                <Badge variant="outline" className="font-mono text-xs">
                  v{plugin.version}
                </Badge>
                {trustConfig && (
                  <Badge
                    variant="secondary"
                    className={`${trustConfig.className} text-xs`}
                  >
                    <TrustIcon className="mr-1 h-3 w-3" />
                    {trustConfig.label}
                  </Badge>
                )}
              </div>
            </div>
          </div>
        </div>
        {plugin.description && (
          <CardDescription className="line-clamp-2 text-sm">
            {plugin.description}
          </CardDescription>
        )}
      </CardHeader>

      <CardContent className="flex-1 pb-3">
        <div className="text-muted-foreground space-y-2 text-sm">
          {plugin.author && (
            <div className="flex items-center space-x-2">
              <User className="h-3 w-3" />
              <span className="truncate">{plugin.author}</span>
            </div>
          )}
          <div className="flex items-center space-x-2">
            <Calendar className="h-3 w-3" />
            <span>
              Updated{" "}
              {formatDistanceToNow(plugin.updatedAt, { addSuffix: true })}
            </span>
          </div>
          {repositoryName && (
            <div className="flex items-center space-x-2">
              <Database className="h-3 w-3" />
              <span className="truncate text-xs">{repositoryName}</span>
            </div>
          )}
        </div>
      </CardContent>

      <CardFooter className="flex items-center justify-between pt-3">
        <div className="flex space-x-2">
          <Button
            size="sm"
            onClick={() => onInstall(plugin.id)}
            disabled={plugin.status !== "active" || isInstalled}
          >
            <Download className="mr-2 h-3 w-3" />
            {isInstalled ? "Installed" : "Install"}
          </Button>
          {plugin.repositoryUrl && (
            <Button variant="outline" size="sm" asChild>
              <a
                href={plugin.repositoryUrl}
                target="_blank"
                rel="noopener noreferrer"
              >
                <ExternalLink className="h-3 w-3" />
              </a>
            </Button>
          )}
        </div>
        {plugin.status !== "active" && (
          <Badge variant="secondary" className="text-xs">
            {plugin.status}
          </Badge>
        )}
      </CardFooter>
    </Card>
  );
}

export function PluginStoreBrowse() {
  const [searchTerm, setSearchTerm] = React.useState("");
  const [statusFilter, setStatusFilter] = React.useState("all");
  const [trustLevelFilter, setTrustLevelFilter] = React.useState("all");
  const { selectedStudyId } = useStudyContext();

  // Get enabled repositories first
  const { data: repositories } = api.admin.repositories.list.useQuery(
    {
      isEnabled: true,
      limit: 100,
    },
    {
      refetchOnWindowFocus: false,
    },
  ) as { data: Array<{ id: string; url: string; name: string }> | undefined };

  // Get installed plugins for current study
  const { data: installedPlugins } =
    api.robots.plugins.getStudyPlugins.useQuery(
      {
        studyId: selectedStudyId!,
      },
      {
        enabled: !!selectedStudyId,
        refetchOnWindowFocus: false,
      },
    );

  const {
    data: availablePlugins,
    isLoading,
    error,
    refetch,
  } = api.robots.plugins.list.useQuery(
    {
      status:
        statusFilter === "all"
          ? undefined
          : (statusFilter as "active" | "deprecated" | "disabled"),
      limit: 50,
    },
    {
      refetchOnWindowFocus: false,
      enabled: Boolean(repositories?.length),
    },
  );

  const installPluginMutation = api.robots.plugins.install.useMutation({
    onSuccess: () => {
      toast.success("Plugin installed successfully!");
      void refetch();
    },
    onError: (error) => {
      toast.error(error.message || "Failed to install plugin");
    },
  });

  // Get study data for breadcrumbs
  const { data: studyData } = api.studies.get.useQuery(
    { id: selectedStudyId! },
    { enabled: !!selectedStudyId },
  );

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    ...(selectedStudyId && studyData
      ? [
          { label: studyData.name, href: `/studies/${selectedStudyId}` },
          { label: "Plugins", href: "/plugins" },
          { label: "Browse" },
        ]
      : [{ label: "Plugins", href: "/plugins" }, { label: "Browse" }]),
  ]);

  const handleInstall = React.useCallback(
    (pluginId: string) => {
      if (!selectedStudyId) {
        toast.error("Please select a study first");
        return;
      }

      installPluginMutation.mutate({
        studyId: selectedStudyId,
        pluginId,
      });
    },
    [selectedStudyId, installPluginMutation],
  );

  // Transform and filter plugins
  const filteredPlugins = React.useMemo(() => {
    if (!availablePlugins) return [];

    return availablePlugins.filter((plugin) => {
      const matchesSearch =
        searchTerm === "" ||
        plugin.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
        (plugin.description?.toLowerCase().includes(searchTerm.toLowerCase()) ??
          false) ||
        (plugin.author?.toLowerCase().includes(searchTerm.toLowerCase()) ??
          false);

      const matchesStatus =
        statusFilter === "all" || plugin.status === statusFilter;

      const matchesTrustLevel =
        trustLevelFilter === "all" || plugin.trustLevel === trustLevelFilter;

      return matchesSearch && matchesStatus && matchesTrustLevel;
    });
  }, [availablePlugins, searchTerm, statusFilter, trustLevelFilter]);

  // Create a set of installed plugin IDs for quick lookup
  const installedPluginIds = React.useMemo(() => {
    if (!installedPlugins) return new Set<string>();
    return new Set(installedPlugins.map((p) => p.plugin.id));
  }, [installedPlugins]);

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

  return (
    <div className="space-y-6">
      <PageHeader
        title="Plugin Store"
        description="Browse and install robot plugins for your study"
        icon={Puzzle}
      />

      {!selectedStudyId && (
        <div className="rounded-lg border border-amber-200 bg-amber-50 p-4">
          <div className="flex items-center space-x-2 text-amber-800">
            <Shield className="h-5 w-5" />
            <p className="text-sm font-medium">
              Select a study from the sidebar to install plugins
            </p>
          </div>
        </div>
      )}

      {repositories?.length === 0 && (
        <div className="rounded-lg border border-blue-200 bg-blue-50 p-4">
          <div className="flex items-center space-x-2 text-blue-800">
            <Database className="h-5 w-5" />
            <div>
              <p className="text-sm font-medium">
                No Plugin Repositories Configured
              </p>
              <p className="mt-1 text-xs">
                Contact your administrator to add plugin repositories.
              </p>
            </div>
          </div>
        </div>
      )}

      {/* Search and Filters */}
      <div className="flex flex-col space-y-4 sm:flex-row sm:items-center sm:justify-between sm:space-y-0 sm:space-x-4">
        <div className="flex flex-1 items-center space-x-2">
          <div className="relative max-w-sm flex-1">
            <Search className="text-muted-foreground absolute top-1/2 left-3 h-4 w-4 -translate-y-1/2" />
            <Input
              placeholder="Search plugins..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="pl-9"
            />
          </div>
        </div>

        <div className="flex items-center space-x-2">
          <Filter className="text-muted-foreground h-4 w-4" />
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

          <Select value={trustLevelFilter} onValueChange={setTrustLevelFilter}>
            <SelectTrigger className="w-[160px]">
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
      </div>

      {/* Loading State */}
      {isLoading && (
        <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
          {Array.from({ length: 6 }).map((_, i) => (
            <Card key={i} className="h-48">
              <CardHeader>
                <div className="bg-muted h-4 animate-pulse rounded" />
                <div className="bg-muted h-3 w-2/3 animate-pulse rounded" />
              </CardHeader>
              <CardContent>
                <div className="space-y-2">
                  <div className="bg-muted h-3 animate-pulse rounded" />
                  <div className="bg-muted h-3 w-1/2 animate-pulse rounded" />
                </div>
              </CardContent>
            </Card>
          ))}
        </div>
      )}

      {/* Error State */}
      {error && (
        <div className="rounded-lg border border-red-200 bg-red-50 p-6 text-center">
          <div className="text-red-800">
            <h3 className="mb-2 text-lg font-semibold">
              Failed to Load Plugins
            </h3>
            <p className="mb-4">
              {error.message ||
                "An error occurred while loading the plugin store."}
            </p>
            <Button onClick={() => refetch()} variant="outline">
              Try Again
            </Button>
          </div>
        </div>
      )}

      {/* Plugin Grid */}
      {!isLoading && !error && (
        <>
          {filteredPlugins.length === 0 ? (
            <div className="py-12 text-center">
              <Puzzle className="text-muted-foreground mx-auto h-12 w-12" />
              <h3 className="mt-4 text-lg font-semibold">No Plugins Found</h3>
              <p className="text-muted-foreground mt-2">
                {searchTerm ||
                statusFilter !== "all" ||
                trustLevelFilter !== "all"
                  ? "Try adjusting your search or filters"
                  : "No plugins are currently available"}
              </p>
            </div>
          ) : (
            <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
              {filteredPlugins.map((plugin) => {
                // Find repository for this plugin by checking metadata
                const repository = repositories?.find((repo) => {
                  // First try to match by URL
                  if (plugin.repositoryUrl?.includes(repo.url)) {
                    return true;
                  }
                  // Then try to match by repository ID in metadata if available
                  const metadata = plugin.metadata as {
                    repositoryId?: string;
                  } | null;
                  return metadata?.repositoryId === repo.id;
                });

                return (
                  <PluginCard
                    key={plugin.id}
                    plugin={plugin}
                    onInstall={handleInstall}
                    repositoryName={repository?.name}
                    isInstalled={installedPluginIds.has(plugin.id)}
                  />
                );
              })}
            </div>
          )}

          {/* Results Count */}
          {filteredPlugins.length > 0 && (
            <div className="text-muted-foreground text-center text-sm">
              Showing {filteredPlugins.length} plugin
              {filteredPlugins.length !== 1 ? "s" : ""}
              {availablePlugins &&
                filteredPlugins.length < availablePlugins.length &&
                ` of ${availablePlugins.length} total`}
            </div>
          )}
        </>
      )}
    </div>
  );
}
