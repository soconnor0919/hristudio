"use client";

import { useState } from "react";
import { type RobotPlugin } from "~/lib/plugin-store/types";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Bot, Download, Info, Zap, Battery, Scale, Ruler, Check, Trash2 } from "lucide-react";
import Image from "next/image";
import { cn } from "~/lib/utils";
import { api } from "~/trpc/react";
import { useToast } from "~/hooks/use-toast";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "~/components/ui/alert-dialog";

interface RobotGridProps {
  plugins: RobotPlugin[];
  installedPluginIds?: string[];
  selectedRobotId?: string;
  onSelectRobot: (robot: RobotPlugin) => void;
}

function RobotCard({
  plugin,
  isInstalled,
  isSelected,
  onSelect,
}: {
  plugin: RobotPlugin;
  isInstalled: boolean;
  isSelected: boolean;
  onSelect: () => void;
}) {
  const { toast } = useToast();
  const utils = api.useUtils();
  const [isProcessing, setIsProcessing] = useState(false);
  const [showUninstallDialog, setShowUninstallDialog] = useState(false);

  const installPlugin = api.pluginStore.installPlugin.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: `${plugin.name} installed successfully`,
      });
      utils.pluginStore.getInstalledPlugins.invalidate();
      utils.pluginStore.getPlugins.invalidate();
    },
    onError: (error) => {
      console.error("Failed to install plugin:", error);
      toast({
        title: "Error",
        description: error.message || "Failed to install plugin",
        variant: "destructive",
      });
    },
  });

  const uninstallPlugin = api.pluginStore.uninstallPlugin.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: `${plugin.name} uninstalled successfully`,
      });
      utils.pluginStore.getInstalledPlugins.invalidate();
      utils.pluginStore.getPlugins.invalidate();
    },
    onError: (error) => {
      console.error("Failed to uninstall plugin:", error);
      toast({
        title: "Error",
        description: error.message || "Failed to uninstall plugin",
        variant: "destructive",
      });
    },
  });

  const handleInstall = async (e: React.MouseEvent) => {
    e.stopPropagation();
    if (isProcessing) return;
    try {
      setIsProcessing(true);
      await installPlugin.mutateAsync({
        robotId: plugin.robotId,
        repositoryId: "hristudio-official", // TODO: Get from context
      });
    } finally {
      setIsProcessing(false);
    }
  };

  const handleUninstall = async () => {
    if (isProcessing) return;
    try {
      setIsProcessing(true);
      await uninstallPlugin.mutateAsync({ robotId: plugin.robotId });
    } finally {
      setIsProcessing(false);
      setShowUninstallDialog(false);
    }
  };

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
        {plugin.assets.logo ? (
          <Image
            src={plugin.assets.logo}
            alt={plugin.name}
            fill
            className="object-contain p-2"
          />
        ) : plugin.assets.thumbnailUrl ? (
          <Image
            src={plugin.assets.thumbnailUrl}
            alt={plugin.name}
            fill
            className="object-cover transition-transform group-hover:scale-105"
          />
        ) : (
          <div className="flex h-full items-center justify-center">
            <Bot className="h-10 w-10 text-muted-foreground/50" />
          </div>
        )}
      </div>
      <div className="flex flex-1 flex-col justify-between">
        <div className="space-y-2">
          <div className="flex items-center justify-between gap-2">
            <h3 className="line-clamp-1 font-semibold tracking-tight">{plugin.name}</h3>
            <div className="flex items-center gap-2">
              <Badge variant="secondary" className="shrink-0">
                {plugin.platform}
              </Badge>
              {isInstalled && (
                <Badge variant="default" className="shrink-0 bg-primary">
                  Installed
                </Badge>
              )}
            </div>
          </div>
          <p className="line-clamp-2 text-sm text-muted-foreground">
            {plugin.description}
          </p>
        </div>
        <div className="mt-2 flex items-center justify-between">
          <div className="flex items-center gap-4 text-xs text-muted-foreground">
            <div className="flex items-center gap-1">
              <Zap className="h-3 w-3" />
              <span>{plugin.specs.maxSpeed}m/s</span>
            </div>
            <div className="flex items-center gap-1">
              <Battery className="h-3 w-3" />
              <span>{plugin.specs.batteryLife}h</span>
            </div>
          </div>
          <div className="flex items-center gap-2">
            {isInstalled ? (
              <AlertDialog open={showUninstallDialog} onOpenChange={setShowUninstallDialog}>
                <AlertDialogTrigger asChild>
                  <Button
                    variant="ghost"
                    size="sm"
                    className="text-destructive hover:bg-destructive/10 hover:text-destructive"
                    onClick={(e) => {
                      e.stopPropagation();
                      setShowUninstallDialog(true);
                    }}
                  >
                    <Trash2 className="h-4 w-4" />
                    <span className="ml-2">Uninstall</span>
                  </Button>
                </AlertDialogTrigger>
                <AlertDialogContent>
                  <AlertDialogHeader>
                    <AlertDialogTitle>Uninstall Robot</AlertDialogTitle>
                    <AlertDialogDescription>
                      Are you sure you want to uninstall {plugin.name}? This action cannot be undone.
                    </AlertDialogDescription>
                  </AlertDialogHeader>
                  <AlertDialogFooter>
                    <AlertDialogCancel>Cancel</AlertDialogCancel>
                    <AlertDialogAction
                      onClick={handleUninstall}
                      className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                    >
                      {isProcessing ? "Uninstalling..." : "Uninstall"}
                    </AlertDialogAction>
                  </AlertDialogFooter>
                </AlertDialogContent>
              </AlertDialog>
            ) : (
              <Button
                size="sm"
                onClick={handleInstall}
                disabled={isProcessing}
              >
                {isProcessing ? (
                  "Installing..."
                ) : (
                  <>
                    <Download className="mr-2 h-4 w-4" />
                    Install
                  </>
                )}
              </Button>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}

export function RobotGrid({ plugins, installedPluginIds = [], selectedRobotId, onSelectRobot }: RobotGridProps) {
  if (!plugins.length) {
    return (
      <div className="flex h-[400px] items-center justify-center">
        <div className="text-center">
          <Bot className="mx-auto h-16 w-16 text-muted-foreground/50" />
          <h3 className="mt-4 text-lg font-medium">No Robots Found</h3>
          <p className="mt-2 text-sm text-muted-foreground">
            Try adjusting your filters or adding more repositories.
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      {plugins.map((plugin) => (
        <RobotCard
          key={plugin.robotId}
          plugin={plugin}
          isInstalled={installedPluginIds.includes(plugin.robotId)}
          isSelected={plugin.robotId === selectedRobotId}
          onSelect={() => onSelectRobot(plugin)}
        />
      ))}
    </div>
  );
} 