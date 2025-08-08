"use client";

import React, { useState } from "react";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { cn } from "~/lib/utils";
import { actionRegistry } from "./ActionRegistry";
import type { ActionDefinition } from "~/lib/experiment-designer/types";
import {
  Plus,
  User,
  Bot,
  GitBranch,
  Eye,
  GripVertical,
  Zap,
  MessageSquare,
  Hand,
  Navigation,
  Volume2,
  Clock,
  Timer,
  MousePointer,
  Mic,
  Activity,
  Play,
} from "lucide-react";
import { useDraggable } from "@dnd-kit/core";

// Local icon map (duplicated minimal map for isolation to avoid circular imports)
const iconMap: Record<string, React.ComponentType<{ className?: string }>> = {
  MessageSquare,
  Hand,
  Navigation,
  Volume2,
  Clock,
  Eye,
  Bot,
  User,
  Zap,
  Timer,
  MousePointer,
  Mic,
  Activity,
  Play,
};

interface DraggableActionProps {
  action: ActionDefinition;
}

function DraggableAction({ action }: DraggableActionProps) {
  const [showTooltip, setShowTooltip] = useState(false);
  const { attributes, listeners, setNodeRef, transform, isDragging } =
    useDraggable({
      id: `action-${action.id}`,
      data: { action },
    });

  const style = {
    transform: transform
      ? `translate3d(${transform.x}px, ${transform.y}px, 0)`
      : undefined,
  };

  const IconComponent = iconMap[action.icon] ?? Zap;

  const categoryColors: Record<ActionDefinition["category"], string> = {
    wizard: "bg-blue-500",
    robot: "bg-emerald-500",
    control: "bg-amber-500",
    observation: "bg-purple-500",
  };

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...listeners}
      {...attributes}
      className={cn(
        "group hover:bg-accent/50 relative flex cursor-grab items-center gap-2 rounded-md border p-2 text-xs transition-colors",
        isDragging && "opacity-50",
      )}
      onMouseEnter={() => setShowTooltip(true)}
      onMouseLeave={() => setShowTooltip(false)}
      draggable={false}
    >
      <div
        className={cn(
          "flex h-5 w-5 flex-shrink-0 items-center justify-center rounded text-white",
          categoryColors[action.category],
        )}
      >
        <IconComponent className="h-3 w-3" />
      </div>
      <div className="min-w-0 flex-1">
        <div className="flex items-center gap-1 truncate font-medium">
          {action.source.kind === "plugin" ? (
            <span className="inline-flex h-3 w-3 items-center justify-center rounded-full bg-emerald-600 text-[8px] font-bold text-white">
              P
            </span>
          ) : (
            <span className="inline-flex h-3 w-3 items-center justify-center rounded-full bg-slate-500 text-[8px] font-bold text-white">
              C
            </span>
          )}
          {action.name}
        </div>
        <div className="text-muted-foreground truncate text-xs">
          {action.description ?? ""}
        </div>
      </div>
      <div className="text-muted-foreground opacity-0 transition-opacity group-hover:opacity-100">
        <GripVertical className="h-3 w-3" />
      </div>

      {showTooltip && (
        <div className="bg-popover absolute top-0 left-full z-50 ml-2 max-w-xs rounded-md border p-2 text-xs shadow-md">
          <div className="font-medium">{action.name}</div>
            <div className="text-muted-foreground">{action.description}</div>
          <div className="mt-1 text-xs opacity-75">
            Category: {action.category} • ID: {action.id}
          </div>
          {action.parameters.length > 0 && (
            <div className="mt-1 text-xs opacity-75">
              Parameters: {action.parameters.map((p) => p.name).join(", ")}
            </div>
          )}
        </div>
      )}
    </div>
  );
}

export interface ActionLibraryProps {
  className?: string;
}

export function ActionLibrary({ className }: ActionLibraryProps) {
  const registry = actionRegistry;
  const [activeCategory, setActiveCategory] =
    useState<ActionDefinition["category"]>("wizard");

  const categories: Array<{
    key: ActionDefinition["category"];
    label: string;
    icon: React.ComponentType<{ className?: string }>;
    color: string;
  }> = [
    {
      key: "wizard",
      label: "Wizard",
      icon: User,
      color: "bg-blue-500",
    },
    {
      key: "robot",
      label: "Robot",
      icon: Bot,
      color: "bg-emerald-500",
    },
    {
      key: "control",
      label: "Control",
      icon: GitBranch,
      color: "bg-amber-500",
    },
    {
      key: "observation",
      label: "Observe",
      icon: Eye,
      color: "bg-purple-500",
    },
  ];

  return (
    <div className={cn("flex h-full flex-col", className)}>
      {/* Category tabs */}
      <div className="border-b p-2">
        <div className="grid grid-cols-2 gap-1">
          {categories.map((category) => {
            const IconComponent = category.icon;
            const isActive = activeCategory === category.key;
            return (
              <Button
                key={category.key}
                variant={isActive ? "default" : "ghost"}
                size="sm"
                className={cn(
                  "h-7 justify-start text-xs",
                  isActive && `${category.color} text-white hover:opacity-90`,
                )}
                onClick={() => setActiveCategory(category.key)}
              >
                <IconComponent className="mr-1 h-3 w-3" />
                {category.label}
              </Button>
            );
          })}
        </div>
      </div>

      {/* Actions list */}
      <ScrollArea className="flex-1">
        <div className="space-y-1 p-2">
          {registry.getActionsByCategory(activeCategory).length === 0 ? (
            <div className="text-muted-foreground py-8 text-center">
              <div className="bg-muted mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full">
                <Plus className="h-4 w-4" />
              </div>
              <p className="text-sm">No actions available</p>
              <p className="text-xs">Check plugin configuration</p>
            </div>
          ) : (
            registry
              .getActionsByCategory(activeCategory)
              .map((action) => <DraggableAction key={action.id} action={action} />)
          )}
        </div>
      </ScrollArea>

      <div className="border-t p-2">
        <div className="flex items-center justify-between">
          <Badge variant="secondary" className="text-[10px]">
            {registry.getAllActions().length} total
          </Badge>
          <Badge variant="outline" className="text-[10px]">
            {registry.getActionsByCategory(activeCategory).length} in view
          </Badge>
        </div>
      </div>
    </div>
  );
}
