"use client";

import { type RobotPlugin } from "~/lib/plugin-store/types";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Bot, Download, Info, Zap, Battery, Scale, Ruler } from "lucide-react";
import Image from "next/image";
import { cn } from "~/lib/utils";
import { useState, useRef, useEffect } from "react";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { api } from "~/trpc/react";
import { useToast } from "~/hooks/use-toast";
import { useRouter } from "next/navigation";

interface RobotListProps {
  plugins: RobotPlugin[];
}

function RobotListItem({
  plugin,
  isSelected,
  onSelect
}: {
  plugin: RobotPlugin;
  isSelected: boolean;
  onSelect: () => void;
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
      <div className="flex-1 space-y-2">
        <div className="flex items-center justify-between gap-2">
          <h3 className="line-clamp-1 font-semibold tracking-tight">{plugin.name}</h3>
          <Badge variant="secondary" className="shrink-0">
            {plugin.platform}
          </Badge>
        </div>
        <p className="line-clamp-2 text-sm text-muted-foreground">
          {plugin.description}
        </p>
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
      </div>
    </div>
  );
}

function RobotHeader({ robot }: { robot: RobotPlugin }) {
  const router = useRouter();
  const { toast } = useToast();
  const [isInstalling, setIsInstalling] = useState(false);

  const utils = api.useUtils();
  const installPlugin = api.pluginStore.installPlugin.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: `${robot.name} installed successfully`,
      });
      // Invalidate both queries to refresh the data
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

  const handleInstall = async () => {
    if (isInstalling) return;

    try {
      setIsInstalling(true);
      await installPlugin.mutateAsync({
        robotId: robot.robotId,
        repositoryId: "hristudio-official", // TODO: Get from context
      });
    } finally {
      setIsInstalling(false);
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
          <Button
            onClick={handleInstall}
            disabled={isInstalling || installPlugin.isLoading}
          >
            <Download className="mr-2 h-4 w-4" />
            {isInstalling || installPlugin.isLoading ? "Installing..." : "Install"}
          </Button>
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

export function RobotList({ plugins }: RobotListProps) {
  const [selectedRobot, setSelectedRobot] = useState<RobotPlugin | null>(plugins[0] ?? null);

  if (!plugins.length) {
    return (
      <div className="flex h-[calc(100vh-24rem)] items-center justify-center">
        <p className="text-muted-foreground">No robots available</p>
      </div>
    );
  }

  return (
    <div className="grid h-[calc(100vh-24rem)] grid-cols-[400px_1fr] gap-8">
      {/* Left Pane - Robot List */}
      <div className="overflow-y-auto rounded-lg pr-4">
        <div className="space-y-3">
          {plugins.map((plugin) => (
            <RobotListItem
              key={plugin.robotId}
              plugin={plugin}
              isSelected={selectedRobot?.robotId === plugin.robotId}
              onSelect={() => setSelectedRobot(plugin)}
            />
          ))}
        </div>
      </div>

      {/* Right Pane - Robot Details */}
      {selectedRobot && (
        <div className="overflow-y-auto rounded-lg border bg-card">
          <RobotHeader robot={selectedRobot} />

          <div className="p-6">
            <Tabs defaultValue="overview" className="w-full">
              <TabsList className="w-full">
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="specs">Specifications</TabsTrigger>
                <TabsTrigger value="actions">Actions</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-6 mt-6">
                <RobotImages robot={selectedRobot} />
                <div className="rounded-lg border bg-card/50 p-4 shadow-sm">
                  <h4 className="mb-4 font-medium">Documentation</h4>
                  <div className="grid gap-2 text-sm">
                    <a
                      href={selectedRobot.documentation.mainUrl}
                      target="_blank"
                      rel="noopener noreferrer"
                      className="text-primary hover:underline"
                    >
                      User Manual
                    </a>
                    {selectedRobot.documentation.apiReference && (
                      <a
                        href={selectedRobot.documentation.apiReference}
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
                <RobotSpecs robot={selectedRobot} />
              </TabsContent>

              <TabsContent value="actions" className="mt-6">
                <RobotActions robot={selectedRobot} />
              </TabsContent>
            </Tabs>
          </div>
        </div>
      )}
    </div>
  );
}