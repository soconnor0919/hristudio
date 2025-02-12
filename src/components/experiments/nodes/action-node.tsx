"use client";

import { memo, useState } from "react";
import { Handle, Position, type NodeProps } from "reactflow";
import { motion } from "framer-motion";
import { AVAILABLE_ACTIONS } from "~/lib/experiments/actions";
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

interface ActionNodeData {
  type: string;
  parameters: Record<string, any>;
  onChange?: (parameters: Record<string, any>) => void;
}

export const ActionNode = memo(({ data, selected }: NodeProps<ActionNodeData>) => {
  const [configOpen, setConfigOpen] = useState(false);
  const [isHovered, setIsHovered] = useState(false);
  const actionConfig = AVAILABLE_ACTIONS.find((a) => a.type === data.type);
  if (!actionConfig) return null;

  return (
    <>
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
        <Card className="relative z-10 w-[250px] bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/80 border-none">
          <CardHeader className="flex flex-row items-center justify-between space-y-0 p-4">
            <div className="flex items-center gap-2">
              <div className="flex h-8 w-8 items-center justify-center rounded-md bg-gradient-to-br from-primary/20 to-primary/10 text-primary">
                {actionConfig.icon}
              </div>
              <CardTitle className="text-sm font-medium leading-none">
                {actionConfig.title}
              </CardTitle>
            </div>
            <Button
              variant="ghost"
              size="icon"
              className="h-8 w-8 shrink-0"
              onClick={() => setConfigOpen(true)}
            >
              <Settings className="h-4 w-4" />
            </Button>
          </CardHeader>
          <CardContent className="p-4 pt-0">
            <CardDescription className="text-xs">
              {actionConfig.description}
            </CardDescription>
          </CardContent>

          <Tooltip>
            <TooltipTrigger asChild>
              <Handle
                type="target"
                position={Position.Top}
                className={cn(
                  "!h-3 !w-3 !border-2 !bg-background",
                  "!border-border transition-colors duration-200",
                  "data-[connecting=true]:!border-primary data-[connecting=true]:!bg-primary",
                  "before:absolute before:inset-[-4px] before:rounded-full before:border-2 before:border-background",
                  "after:absolute after:inset-[-8px] after:rounded-full after:border-2 after:border-border/50"
                )}
              />
            </TooltipTrigger>
            <TooltipContent side="top" className="flex items-center gap-2">
              <ArrowDown className="h-3 w-3" />
              Input Connection
            </TooltipContent>
          </Tooltip>

          <Tooltip>
            <TooltipTrigger asChild>
              <Handle
                type="source"
                position={Position.Bottom}
                className={cn(
                  "!h-3 !w-3 !border-2 !bg-background",
                  "!border-border transition-colors duration-200",
                  "data-[connecting=true]:!border-primary data-[connecting=true]:!bg-primary",
                  "before:absolute before:inset-[-4px] before:rounded-full before:border-2 before:border-background",
                  "after:absolute after:inset-[-8px] after:rounded-full after:border-2 after:border-border/50"
                )}
              />
            </TooltipTrigger>
            <TooltipContent side="bottom" className="flex items-center gap-2">
              <ArrowUp className="h-3 w-3" />
              Output Connection
            </TooltipContent>
          </Tooltip>
        </Card>
      </motion.div>

      <ActionConfigDialog
        open={configOpen}
        onOpenChange={setConfigOpen}
        type={data.type as any}
        parameters={data.parameters}
        onSubmit={data.onChange ?? (() => {})}
      />
    </>
  );
}); 