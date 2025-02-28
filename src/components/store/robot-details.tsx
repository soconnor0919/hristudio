"use client";

import { useState, useRef, useEffect } from "react";
import { type RobotPlugin } from "~/lib/plugin-store/types";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Bot, Download, Info, Zap, Battery, Scale, Ruler, Trash2 } from "lucide-react";
import Image from "next/image";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
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

interface RobotDetailsProps {
  robot: RobotPlugin;
  isInstalled: boolean;
}

function RobotHeader({ robot, isInstalled }: RobotDetailsProps) {
  const { toast } = useToast();
  const utils = api.useUtils();
  const [isProcessing, setIsProcessing] = useState(false);
  const [showUninstallDialog, setShowUninstallDialog] = useState(false);

  const installPlugin = api.pluginStore.installPlugin.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: `${robot.name} installed successfully`,
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
        description: `${robot.name} uninstalled successfully`,
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

  const handleInstall = async () => {
    if (isProcessing) return;
    try {
      setIsProcessing(true);
      await installPlugin.mutateAsync({
        robotId: robot.robotId,
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
      await uninstallPlugin.mutateAsync({ robotId: robot.robotId });
    } finally {
      setIsProcessing(false);
      setShowUninstallDialog(false);
    }
  };

  return (
    <div className="border-b p-6">
      <div className="mb-4 flex items-start justify-between">
        <div>
          <h2 className="text-2xl font-semibold">{robot.name}</h2>
          <p className="mt-1 text-muted-foreground">
            {robot.description}
          </p>
        </div>
        <div className="flex items-center gap-2">
          <Button variant="outline" asChild>
            <a
              href={robot.documentation.mainUrl}
              target="_blank"
              rel="noopener noreferrer"
            >
              <Info className="mr-2 h-4 w-4" />
              Documentation
            </a>
          </Button>
          {isInstalled ? (
            <AlertDialog open={showUninstallDialog} onOpenChange={setShowUninstallDialog}>
              <AlertDialogTrigger asChild>
                <Button
                  variant="ghost"
                  className="text-destructive hover:bg-destructive/10 hover:text-destructive"
                >
                  <Trash2 className="mr-2 h-4 w-4" />
                  Uninstall
                </Button>
              </AlertDialogTrigger>
              <AlertDialogContent>
                <AlertDialogHeader>
                  <AlertDialogTitle>Uninstall Robot</AlertDialogTitle>
                  <AlertDialogDescription>
                    Are you sure you want to uninstall {robot.name}? This action cannot be undone.
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
              onClick={handleInstall}
              disabled={isProcessing}
            >
              <Download className="mr-2 h-4 w-4" />
              {isProcessing ? "Installing..." : "Install"}
            </Button>
          )}
        </div>
      </div>
      <div className="flex items-center gap-4 text-sm">
        <div className="flex items-center gap-1.5">
          <Zap className="h-4 w-4 text-muted-foreground" />
          <span>{robot.specs.maxSpeed}m/s</span>
        </div>
        <div className="flex items-center gap-1.5">
          <Battery className="h-4 w-4 text-muted-foreground" />
          <span>{robot.specs.batteryLife}h</span>
        </div>
        <div className="flex items-center gap-1.5">
          <Scale className="h-4 w-4 text-muted-foreground" />
          <span>{robot.specs.dimensions.weight}kg</span>
        </div>
      </div>
    </div>
  );
}

function RobotImages({ robot }: { robot: RobotPlugin }) {
  const [showLeftFade, setShowLeftFade] = useState(false);
  const [showRightFade, setShowRightFade] = useState(false);
  const scrollRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const el = scrollRef.current;
    if (!el) return;

    const checkScroll = () => {
      const hasLeftScroll = el.scrollLeft > 0;
      const hasRightScroll = el.scrollLeft < (el.scrollWidth - el.clientWidth);

      setShowLeftFade(hasLeftScroll);
      setShowRightFade(hasRightScroll);
    };

    // Check initial scroll
    checkScroll();

    // Add scroll listener
    el.addEventListener('scroll', checkScroll);
    // Add resize listener to handle window changes
    window.addEventListener('resize', checkScroll);

    return () => {
      el.removeEventListener('scroll', checkScroll);
      window.removeEventListener('resize', checkScroll);
    };
  }, []);

  return (
    <div className="relative">
      <div ref={scrollRef} className="overflow-x-auto pb-4">
        <div className="flex gap-4">
          {/* Main Image */}
          <div className="relative h-[300px] aspect-video shrink-0 overflow-hidden rounded-lg border bg-muted">
            <Image
              src={robot.assets.images.main}
              alt={robot.name}
              fill
              className="object-cover"
            />
          </div>

          {/* Angle Images */}
          {robot.assets.images.angles && (
            <div className="flex gap-4">
              {Object.entries(robot.assets.images.angles).map(([angle, url]) => url && (
                <div
                  key={angle}
                  className="relative h-[300px] aspect-square shrink-0 overflow-hidden rounded-lg border bg-muted"
                >
                  <Image
                    src={url}
                    alt={`${robot.name} - ${angle} view`}
                    fill
                    className="object-cover"
                  />
                  <div className="absolute inset-x-0 bottom-0 bg-gradient-to-t from-black/50 to-transparent p-4">
                    <span className="text-xs font-medium text-white capitalize">
                      {angle} View
                    </span>
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>

      {/* Fade indicators */}
      {showLeftFade && (
        <div className="pointer-events-none absolute inset-y-0 left-0 w-8 bg-gradient-to-r from-background to-transparent" />
      )}
      {showRightFade && (
        <div className="pointer-events-none absolute inset-y-0 right-0 w-8 bg-gradient-to-l from-background to-transparent" />
      )}
    </div>
  );
}

function RobotSpecs({ robot }: { robot: RobotPlugin }) {
  return (
    <div className="space-y-6">
      <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
        <h4 className="mb-4 font-medium">Physical Specifications</h4>
        <div className="grid gap-4 md:grid-cols-2">
          <div className="flex items-center gap-2">
            <Ruler className="h-4 w-4 text-muted-foreground" />
            <span className="text-sm">
              {robot.specs.dimensions.length}m × {robot.specs.dimensions.width}m × {robot.specs.dimensions.height}m
            </span>
          </div>
          <div className="flex items-center gap-2">
            <Scale className="h-4 w-4 text-muted-foreground" />
            <span className="text-sm">{robot.specs.dimensions.weight}kg</span>
          </div>
          <div className="flex items-center gap-2">
            <Zap className="h-4 w-4 text-muted-foreground" />
            <span className="text-sm">{robot.specs.maxSpeed}m/s</span>
          </div>
          <div className="flex items-center gap-2">
            <Battery className="h-4 w-4 text-muted-foreground" />
            <span className="text-sm">{robot.specs.batteryLife}h</span>
          </div>
        </div>
      </div>

      <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
        <h4 className="mb-4 font-medium">Capabilities</h4>
        <div className="flex flex-wrap gap-2">
          {robot.specs.capabilities.map((capability) => (
            <Badge key={capability} variant="secondary">
              {capability}
            </Badge>
          ))}
        </div>
      </div>

      <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
        <h4 className="mb-4 font-medium">ROS 2 Configuration</h4>
        <div className="grid gap-3 text-sm">
          <div>
            <span className="text-muted-foreground">Namespace: </span>
            <code className="rounded bg-muted px-1.5 py-0.5">
              {robot.ros2Config.namespace}
            </code>
          </div>
          <div>
            <span className="text-muted-foreground">Node Prefix: </span>
            <code className="rounded bg-muted px-1.5 py-0.5">
              {robot.ros2Config.nodePrefix}
            </code>
          </div>
          <div className="grid gap-2">
            <span className="text-muted-foreground">Default Topics:</span>
            <div className="grid gap-1.5 pl-4">
              {Object.entries(robot.ros2Config.defaultTopics).map(([name, topic]) => (
                <div key={name}>
                  <span className="text-muted-foreground">{name}: </span>
                  <code className="rounded bg-muted px-1.5 py-0.5">{topic}</code>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

function RobotActions({ robot }: { robot: RobotPlugin }) {
  return (
    <div className="space-y-4">
      {robot.actions.map((action) => (
        <div key={action.actionId} className="rounded-lg border bg-card/50 p-4 shadow-sm">
          <div className="mb-3 flex items-center justify-between">
            <h4 className="font-medium">{action.title}</h4>
            <Badge variant="secondary">{action.type}</Badge>
          </div>
          <p className="mb-4 text-sm text-muted-foreground">
            {action.description}
          </p>
          <div className="grid gap-2">
            <h5 className="text-sm font-medium text-muted-foreground">Parameters:</h5>
            <div className="grid gap-2 pl-4">
              {Object.entries(action.parameters.properties).map(([name, prop]) => (
                <div key={name} className="text-sm">
                  <span className="font-medium">{prop.title}</span>
                  {prop.unit && (
                    <span className="text-muted-foreground"> ({prop.unit})</span>
                  )}
                  {prop.description && (
                    <p className="mt-0.5 text-muted-foreground">{prop.description}</p>
                  )}
                </div>
              ))}
            </div>
          </div>
        </div>
      ))}
    </div>
  );
}

export function RobotDetails({ robot, isInstalled }: RobotDetailsProps) {
  return (
    <>
      <RobotHeader robot={robot} isInstalled={isInstalled} />

      <div className="p-6">
        <Tabs defaultValue="overview" className="w-full">
          <TabsList className="w-full">
            <TabsTrigger value="overview">Overview</TabsTrigger>
            <TabsTrigger value="specs">Specifications</TabsTrigger>
            <TabsTrigger value="actions">Actions</TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="space-y-6 mt-6">
            <RobotImages robot={robot} />
            <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
              <h4 className="mb-4 font-medium">Documentation</h4>
              <div className="grid gap-2 text-sm">
                <a
                  href={robot.documentation.mainUrl}
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-primary hover:underline"
                >
                  User Manual
                </a>
                {robot.documentation.apiReference && (
                  <a
                    href={robot.documentation.apiReference}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="text-primary hover:underline"
                  >
                    API Reference
                  </a>
                )}
              </div>
            </div>
          </TabsContent>

          <TabsContent value="specs" className="mt-6">
            <RobotSpecs robot={robot} />
          </TabsContent>

          <TabsContent value="actions" className="mt-6">
            <RobotActions robot={robot} />
          </TabsContent>
        </Tabs>
      </div>
    </>
  );
} 