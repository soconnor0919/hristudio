"use client";

import { type RepositoryMetadata } from "~/lib/plugin-store/types";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Bot, Star, Download, Package, Calendar } from "lucide-react";
import { formatDistanceToNow } from "date-fns";
import Image from "next/image";

interface RepositoryCardProps {
  repository: RepositoryMetadata;
  onRemove?: (id: string) => void;
}

export function RepositoryCard({ repository, onRemove }: RepositoryCardProps) {
  const lastUpdated = new Date(repository.lastUpdated);

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center gap-3">
          <div className="relative aspect-square h-12 shrink-0 overflow-hidden rounded-md border bg-muted">
            {repository.assets?.logo ? (
              <Image
                src={repository.assets.logo}
                alt={repository.name}
                fill
                className="object-contain p-1.5"
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
                <Bot className="h-6 w-6 text-muted-foreground/50" />
              </div>
            )}
          </div>
          <div className="flex-1 min-w-0">
            <CardTitle className="flex items-center gap-2">
              <span className="truncate">{repository.name}</span>
              {repository.official && (
                <Badge variant="default" className="shrink-0 text-xs">Official</Badge>
              )}
            </CardTitle>
            <CardDescription className="line-clamp-2">{repository.description}</CardDescription>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="grid gap-4">
          <div className="flex flex-wrap items-center gap-4 text-sm">
            <div className="flex items-center gap-1.5">
              <Star className="h-4 w-4 text-muted-foreground" />
              <span>{repository.stats?.stars ?? 0}</span>
            </div>
            <div className="flex items-center gap-1.5">
              <Download className="h-4 w-4 text-muted-foreground" />
              <span>{repository.stats?.downloads ?? 0}</span>
            </div>
            <div className="flex items-center gap-1.5">
              <Package className="h-4 w-4 text-muted-foreground" />
              <span>{repository.stats?.plugins ?? 0} plugins</span>
            </div>
            <div className="flex items-center gap-1.5">
              <Calendar className="h-4 w-4 text-muted-foreground" />
              <span>Updated {formatDistanceToNow(lastUpdated, { addSuffix: true })}</span>
            </div>
          </div>
          <div className="flex flex-wrap gap-1.5">
            {repository.tags.map((tag) => (
              <Badge key={tag} variant="secondary" className="text-xs">
                {tag}
              </Badge>
            ))}
          </div>
        </div>
      </CardContent>
      {onRemove && !repository.official && (
        <CardFooter>
          <Button
            variant="ghost"
            size="sm"
            className="text-destructive hover:text-destructive hover:bg-destructive/10"
            onClick={() => onRemove(repository.id)}
          >
            Remove Repository
          </Button>
        </CardFooter>
      )}
    </Card>
  );
}