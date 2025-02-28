"use client";

import { memo, useState } from "react";
import { Handle, Position, type NodeProps } from "reactflow";
import { motion } from "framer-motion";
import { BUILT_IN_ACTIONS, getPluginActions } from "~/lib/experiments/plugin-actions";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { Settings, ArrowDown, ArrowUp } from "lucide-react";
import { cn } from "~/lib/utils";
import { ActionConfigDialog } from "../action-config-dialog";
import { Tooltip, TooltipContent, TooltipTrigger } from "~/components/ui/tooltip";
import { api } from "~/trpc/react";

interface ActionNodeData {
  type: string;
  parameters: Record<string, any>;
  onChange?: (parameters: Record<string, any>) => void;
}

export const ActionNode = memo(({ data, selected }: NodeProps<ActionNodeData>) => {
  const [configOpen, setConfigOpen] = useState(false);
  const [isHovered, setIsHovered] = useState(false);

  // Get available plugins
  const { data: plugins } = api.pluginStore.getPlugins.useQuery();
  const { data: installedPlugins } = api.pluginStore.getInstalledPlugins.useQuery();
  const installedPluginIds = installedPlugins?.map(p => p.robotId) ?? [];

  // Get available actions from installed plugins
  const installedPluginActions = plugins
    ? getPluginActions(plugins.filter(p => installedPluginIds.includes(p.robotId)))
    : [];

  // Combine built-in actions with plugin actions
  const availableActions = [...BUILT_IN_ACTIONS, ...installedPluginActions];
  const actionConfig = availableActions.find((a) => a.type === data.type);

  if (!actionConfig) return null;

  return (
    <>
      <Handle
        type="target"
        position={Position.Top}
        className="!bg-primary !border-primary-foreground"
      />
      <motion.div
        initial={{ scale: 0.8, opacity: 0 }}
        animate={{
          scale: 1,
          opacity: 1,
        }}
        transition={{ duration: 0.2 }}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        className={cn(
          "relative",
          "before:absolute before:inset-[-2px] before:rounded-xl before:bg-gradient-to-br before:from-border before:to-border/50 before:opacity-100",
          "after:absolute after:inset-[-1px] after:rounded-xl after:bg-gradient-to-br after:from-background after:to-background",
          selected && "before:from-primary/50 before:to-primary/20",
          isHovered && "before:from-border/80 before:to-border/30",
        )}
      >
        <Card className="relative z-10 min-w-[240px] overflow-hidden">
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <div className="flex items-center gap-2">
              <div className="flex h-8 w-8 items-center justify-center rounded-md bg-gradient-to-br from-primary/20 to-primary/10">
                {actionConfig.icon}
              </div>
              <CardTitle className="text-base">{actionConfig.title}</CardTitle>
            </div>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant="ghost"
                  size="icon"
                  className="h-8 w-8 shrink-0"
                  onClick={() => setConfigOpen(true)}
                >
                  <Settings className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>Configure Action</TooltipContent>
            </Tooltip>
          </CardHeader>
          <CardContent>
            <CardDescription className="line-clamp-2">
              {actionConfig.description}
            </CardDescription>
          </CardContent>
        </Card>
      </motion.div>
      <Handle
        type="source"
        position={Position.Bottom}
        className="!bg-primary !border-primary-foreground"
      />
      <ActionConfigDialog
        open={configOpen}
        onOpenChange={setConfigOpen}
        type={data.type}
        parameters={data.parameters}
        onSubmit={data.onChange ?? (() => { })}
        actionConfig={actionConfig}
      />
    </>
  );
}); 