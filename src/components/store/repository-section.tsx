"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useToast } from "~/hooks/use-toast";
import { type RepositoryMetadata } from "~/lib/plugin-store/types";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Bot, Star, Download, Package, Calendar } from "lucide-react";
import Image from "next/image";
import { cn } from "~/lib/utils";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { api } from "~/trpc/react";
import { formatDistanceToNow } from "date-fns";

interface RepositorySectionProps {
  repositories: RepositoryMetadata[];
}

function RepositoryListItem({
  repository,
  isSelected,
  onSelect,
  onRemove,
}: {
  repository: RepositoryMetadata;
  isSelected: boolean;
  onSelect: () => void;
  onRemove?: (id: string) => void;
}) {
  return (
    <div
      className={cn(
        "group relative flex cursor-pointer gap-3 rounded-lg border p-4 transition-all",
        isSelected
          ? "border-primary bg-card ring-2 ring-primary/10"
          : "hover:border-primary/50 hover:bg-accent/50"
      )}
      onClick={onSelect}
    >
      <div className="relative aspect-square h-20 shrink-0 overflow-hidden rounded-md border bg-muted">
        {repository.assets?.logo ? (
          <Image
            src={repository.assets.logo}
            alt={repository.name}
            fill
            className="object-contain p-2"
          />
        ) : repository.assets?.icon ? (
          <Image
            src={repository.assets.icon}
            alt={repository.name}
            fill
            className="object-cover"
          />
        ) : (
          <div className="flex h-full items-center justify-center">
            <Bot className="h-10 w-10 text-muted-foreground/50" />
          </div>
        )}
      </div>
      <div className="flex-1 space-y-2">
        <div className="flex items-center justify-between gap-2">
          <h3 className="line-clamp-1 font-semibold tracking-tight">{repository.name}</h3>
          {repository.official && (
            <Badge variant="default" className="shrink-0">Official</Badge>
          )}
        </div>
        <p className="line-clamp-2 text-sm text-muted-foreground">
          {repository.description}
        </p>
        <div className="flex items-center gap-4 text-xs text-muted-foreground">
          <div className="flex items-center gap-1">
            <Star className="h-3 w-3" />
            <span>{repository.stats?.stars ?? 0}</span>
          </div>
          <div className="flex items-center gap-1">
            <Download className="h-3 w-3" />
            <span>{repository.stats?.downloads ?? 0}</span>
          </div>
          <div className="flex items-center gap-1">
            <Package className="h-3 w-3" />
            <span>{repository.stats?.plugins ?? 0} plugins</span>
          </div>
        </div>
      </div>
    </div>
  );
}

function RepositoryDetails({ repository, onRemove }: { repository: RepositoryMetadata; onRemove?: (id: string) => void }) {
  return (
    <div className="overflow-y-auto rounded-lg border bg-card">
      <div className="border-b p-6">
        <div className="mb-4 flex items-start justify-between">
          <div>
            <h2 className="text-2xl font-semibold">{repository.name}</h2>
            <p className="mt-1 text-muted-foreground">
              {repository.description}
            </p>
          </div>
          <div className="flex items-center gap-2">
            <Button variant="outline" asChild>
              <a
                href={repository.urls.repository}
                target="_blank"
                rel="noopener noreferrer"
              >
                View Repository
              </a>
            </Button>
            {repository.urls.git && (
              <Button variant="outline" asChild>
                <a
                  href={repository.urls.git}
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  View on GitHub
                </a>
              </Button>
            )}
            {onRemove && !repository.official && (
              <Button
                variant="ghost"
                className="text-destructive hover:text-destructive hover:bg-destructive/10"
                onClick={() => onRemove(repository.id)}
              >
                Remove Repository
              </Button>
            )}
          </div>
        </div>
        <div className="flex items-center gap-4 text-sm">
          <div className="flex items-center gap-1.5">
            <Package className="h-4 w-4 text-muted-foreground" />
            <span>{repository.stats?.plugins ?? 0} plugins</span>
          </div>
          <div className="flex items-center gap-1.5">
            <Calendar className="h-4 w-4 text-muted-foreground" />
            <span>Updated {formatDistanceToNow(new Date(repository.lastUpdated), { addSuffix: true })}</span>
          </div>
        </div>
      </div>

      <div className="p-6">
        <Tabs defaultValue="overview" className="w-full">
          <TabsList className="w-full">
            <TabsTrigger value="overview">Overview</TabsTrigger>
            <TabsTrigger value="plugins">Plugins</TabsTrigger>
            <TabsTrigger value="compatibility">Compatibility</TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="space-y-6 mt-6">
            {repository.assets?.banner && (
              <div className="relative h-[200px] w-full overflow-hidden rounded-lg border">
                <Image
                  src={repository.assets.banner}
                  alt={repository.name}
                  fill
                  className="object-cover"
                />
              </div>
            )}
            <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
              <h4 className="mb-4 font-medium">Author</h4>
              <div className="grid gap-2 text-sm">
                <div>
                  <span className="text-muted-foreground">Name: </span>
                  <span>{repository.author.name}</span>
                </div>
                {repository.author.organization && (
                  <div>
                    <span className="text-muted-foreground">Organization: </span>
                    <span>{repository.author.organization}</span>
                  </div>
                )}
                {repository.author.url && (
                  <a
                    href={repository.author.url}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="text-primary hover:underline"
                  >
                    View Profile
                  </a>
                )}
              </div>
            </div>
            <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
              <h4 className="mb-4 font-medium">Tags</h4>
              <div className="flex flex-wrap gap-2">
                {repository.tags.map((tag) => (
                  <Badge key={tag} variant="secondary">
                    {tag}
                  </Badge>
                ))}
              </div>
            </div>
          </TabsContent>

          <TabsContent value="plugins" className="mt-6">
            <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
              <h4 className="mb-4 font-medium">Available Plugins</h4>
              <p className="text-sm text-muted-foreground">
                This repository contains {repository.stats?.plugins ?? 0} robot plugins.
              </p>
            </div>
          </TabsContent>

          <TabsContent value="compatibility" className="mt-6">
            <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
              <h4 className="mb-4 font-medium">HRIStudio Compatibility</h4>
              <div className="grid gap-2 text-sm">
                <div>
                  <span className="text-muted-foreground">Minimum Version: </span>
                  <code className="rounded bg-muted px-1.5 py-0.5">
                    {repository.compatibility.hristudio.min}
                  </code>
                </div>
                {repository.compatibility.hristudio.recommended && (
                  <div>
                    <span className="text-muted-foreground">Recommended Version: </span>
                    <code className="rounded bg-muted px-1.5 py-0.5">
                      {repository.compatibility.hristudio.recommended}
                    </code>
                  </div>
                )}
              </div>
            </div>
            {repository.compatibility.ros2 && (
              <div className="mt-4 rounded-lg border bg-card/50 p-4 shadow-sm">
                <h4 className="mb-4 font-medium">ROS 2 Compatibility</h4>
                <div className="grid gap-2 text-sm">
                  <div>
                    <span className="text-muted-foreground">Supported Distributions: </span>
                    <div className="flex flex-wrap gap-2 mt-1">
                      {repository.compatibility.ros2.distributions.map((dist) => (
                        <Badge key={dist} variant="secondary">
                          {dist}
                        </Badge>
                      ))}
                    </div>
                  </div>
                  {repository.compatibility.ros2.recommended && (
                    <div>
                      <span className="text-muted-foreground">Recommended Distribution: </span>
                      <code className="rounded bg-muted px-1.5 py-0.5">
                        {repository.compatibility.ros2.recommended}
                      </code>
                    </div>
                  )}
                </div>
              </div>
            )}
          </TabsContent>
        </Tabs>
      </div>
    </div>
  );
}

export function RepositorySection({ repositories }: RepositorySectionProps) {
  const router = useRouter();
  const { toast } = useToast();
  const [isRemoving, setIsRemoving] = useState(false);
  const [selectedRepository, setSelectedRepository] = useState<RepositoryMetadata | null>(
    repositories[0] ?? null
  );

  const utils = api.useUtils();

  const removeRepository = api.pluginStore.removeRepository.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Repository removed successfully",
      });
      // Invalidate all plugin store queries
      utils.pluginStore.getRepositories.invalidate();
      utils.pluginStore.getPlugins.invalidate();
      utils.pluginStore.getInstalledPlugins.invalidate();
    },
    onError: (error) => {
      console.error("Failed to remove repository:", error);
      toast({
        title: "Error",
        description: error.message || "Failed to remove repository",
        variant: "destructive",
      });
    },
  });

  const handleRemoveRepository = async (id: string) => {
    if (isRemoving) return;

    try {
      setIsRemoving(true);
      await removeRepository.mutateAsync({ id });
    } finally {
      setIsRemoving(false);
    }
  };

  if (!repositories.length) {
    return (
      <div className="flex h-[calc(100vh-24rem)] items-center justify-center">
        <p className="text-muted-foreground">No repositories added</p>
      </div>
    );
  }

  return (
    <div className="grid h-[calc(100vh-24rem)] grid-cols-[400px_1fr] gap-8">
      {/* Left Pane - Repository List */}
      <div className="overflow-y-auto rounded-lg pr-4">
        <div className="space-y-3">
          {repositories.map((repository) => (
            <RepositoryListItem
              key={repository.id}
              repository={repository}
              isSelected={selectedRepository?.id === repository.id}
              onSelect={() => setSelectedRepository(repository)}
              onRemove={handleRemoveRepository}
            />
          ))}
        </div>
      </div>

      {/* Right Pane - Repository Details */}
      {selectedRepository && (
        <RepositoryDetails
          repository={selectedRepository}
          onRemove={handleRemoveRepository}
        />
      )}
    </div>
  );
}