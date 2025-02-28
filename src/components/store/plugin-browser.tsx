"use client";

import { useState } from "react";
import { type RepositoryMetadata, type RobotPlugin } from "~/lib/plugin-store/types";
import { Button } from "~/components/ui/button";
import { Bot, Search, Filter } from "lucide-react";
import { RepositorySection } from "./repository-section";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { RobotGrid } from "./robot-grid";
import { RobotDetails } from "./robot-details";
import { api } from "~/trpc/react";
import { Input } from "~/components/ui/input";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuCheckboxItem,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import { Skeleton } from "~/components/ui/skeleton";

interface PluginBrowserProps {
  repositories: RepositoryMetadata[];
  initialPlugins: RobotPlugin[];
}

function RobotSkeleton() {
  return (
    <div className="flex gap-3 rounded-lg border p-4">
      <div className="relative aspect-square h-20 shrink-0 overflow-hidden rounded-md border bg-muted">
        <Skeleton className="h-full w-full" />
      </div>
      <div className="flex flex-1 flex-col justify-between">
        <div className="space-y-2">
          <div className="flex items-center justify-between gap-2">
            <Skeleton className="h-6 w-32" />
            <Skeleton className="h-5 w-20" />
          </div>
          <div className="space-y-1">
            <Skeleton className="h-4 w-full" />
            <Skeleton className="h-4 w-3/4" />
          </div>
        </div>
        <div className="mt-2 flex items-center justify-between">
          <div className="flex items-center gap-4">
            <Skeleton className="h-4 w-16" />
            <Skeleton className="h-4 w-16" />
          </div>
          <Skeleton className="h-8 w-24" />
        </div>
      </div>
    </div>
  );
}

export function PluginBrowser({ repositories, initialPlugins }: PluginBrowserProps) {
  // State
  const [searchQuery, setSearchQuery] = useState("");
  const [selectedRepository, setSelectedRepository] = useState<string>("all");
  const [showInstalled, setShowInstalled] = useState<boolean>(true);
  const [showAvailable, setShowAvailable] = useState<boolean>(true);
  const [selectedRobot, setSelectedRobot] = useState<RobotPlugin | null>(
    initialPlugins[0] ?? null
  );

  // Queries
  const { data: installedPlugins, isLoading: isLoadingInstalled } = api.pluginStore.getInstalledPlugins.useQuery(undefined, {
    refetchOnMount: true,
    refetchOnWindowFocus: true,
  });
  const { data: plugins, isLoading: isLoadingPlugins } = api.pluginStore.getPlugins.useQuery(undefined, {
    initialData: initialPlugins,
    refetchOnMount: true,
    refetchOnWindowFocus: true,
  });
  const installedPluginIds = installedPlugins?.map(p => p.robotId) ?? [];

  // Loading state
  const isLoading = isLoadingInstalled || isLoadingPlugins;

  // Filter plugins
  const filteredPlugins = plugins.filter(plugin => {
    // Repository filter
    if (selectedRepository !== "all" && plugin.repositoryId !== selectedRepository) {
      return false;
    }

    // Installation status filter
    const isInstalled = installedPluginIds.includes(plugin.robotId);
    if (!showInstalled && isInstalled) return false;
    if (!showAvailable && !isInstalled) return false;

    // Search query filter
    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      return (
        plugin.name.toLowerCase().includes(query) ||
        plugin.description?.toLowerCase().includes(query) ||
        plugin.platform.toLowerCase().includes(query) ||
        plugin.manufacturer.name.toLowerCase().includes(query)
      );
    }

    return true;
  });

  return (
    <Tabs defaultValue="plugins" className="space-y-6">
      <TabsList>
        <TabsTrigger value="plugins">Robots</TabsTrigger>
        <TabsTrigger value="repositories">Repositories</TabsTrigger>
      </TabsList>

      <TabsContent value="plugins">
        <Card>
          <CardHeader>
            <CardTitle>Robot Plugins</CardTitle>
            <CardDescription>
              Browse and manage robot plugins from your configured repositories
            </CardDescription>
            <div className="mt-4 flex flex-col gap-4 md:flex-row md:items-center">
              <div className="relative flex-1">
                <Search className="absolute left-2 top-2.5 h-4 w-4 text-muted-foreground" />
                <Input
                  placeholder="Search robots..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  className="pl-8"
                />
              </div>
              <div className="flex items-center gap-2">
                <Select
                  value={selectedRepository}
                  onValueChange={setSelectedRepository}
                >
                  <SelectTrigger className="w-[200px]">
                    <SelectValue placeholder="Select Repository" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="all">All Repositories</SelectItem>
                    {repositories.map((repo) => (
                      <SelectItem key={repo.id} value={repo.id}>
                        {repo.name}
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
                <DropdownMenu>
                  <DropdownMenuTrigger asChild>
                    <Button variant="outline" size="icon">
                      <Filter className="h-4 w-4" />
                    </Button>
                  </DropdownMenuTrigger>
                  <DropdownMenuContent align="end">
                    <DropdownMenuLabel>Show</DropdownMenuLabel>
                    <DropdownMenuSeparator />
                    <DropdownMenuCheckboxItem
                      checked={showInstalled}
                      onCheckedChange={setShowInstalled}
                    >
                      Installed
                    </DropdownMenuCheckboxItem>
                    <DropdownMenuCheckboxItem
                      checked={showAvailable}
                      onCheckedChange={setShowAvailable}
                    >
                      Available
                    </DropdownMenuCheckboxItem>
                  </DropdownMenuContent>
                </DropdownMenu>
              </div>
            </div>
          </CardHeader>
          <CardContent>
            <div className="grid h-[calc(100vh-24rem)] grid-cols-[400px_1fr] gap-8">
              {/* Left Pane - Robot List */}
              <div className="overflow-y-auto rounded-lg pr-4">
                {isLoading ? (
                  <div className="space-y-4">
                    {Array.from({ length: 3 }).map((_, i) => (
                      <RobotSkeleton key={i} />
                    ))}
                  </div>
                ) : (
                  <RobotGrid
                    plugins={filteredPlugins}
                    installedPluginIds={installedPluginIds}
                    selectedRobotId={selectedRobot?.robotId}
                    onSelectRobot={setSelectedRobot}
                  />
                )}
              </div>

              {/* Right Pane - Robot Details */}
              <div className="overflow-y-auto rounded-lg border bg-card">
                {isLoading ? (
                  <div className="p-6">
                    <div className="space-y-4">
                      <Skeleton className="h-8 w-64" />
                      <Skeleton className="h-4 w-full" />
                      <div className="flex gap-4">
                        <Skeleton className="h-4 w-24" />
                        <Skeleton className="h-4 w-24" />
                        <Skeleton className="h-4 w-24" />
                      </div>
                    </div>
                  </div>
                ) : selectedRobot && (
                  <RobotDetails
                    robot={selectedRobot}
                    isInstalled={installedPluginIds.includes(selectedRobot.robotId)}
                  />
                )}
              </div>
            </div>
          </CardContent>
        </Card>
      </TabsContent>

      <TabsContent value="repositories">
        <Card>
          <CardHeader>
            <CardTitle>Plugin Repositories</CardTitle>
            <CardDescription>
              Manage your robot plugin sources
            </CardDescription>
          </CardHeader>
          <CardContent>
            {repositories.length === 0 ? (
              <div className="flex flex-col items-center justify-center min-h-[400px] text-center">
                <Bot className="h-16 w-16 text-muted-foreground/50 mb-4" />
                <h3 className="text-lg font-medium">No Repositories Added</h3>
                <p className="text-sm text-muted-foreground">
                  Add a repository using the button above
                </p>
              </div>
            ) : (
              <RepositorySection repositories={repositories} />
            )}
          </CardContent>
        </Card>
      </TabsContent>
    </Tabs>
  );
} 