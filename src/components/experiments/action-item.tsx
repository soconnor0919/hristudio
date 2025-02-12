"use client";

import { type ActionType } from "~/lib/experiments/types";
import { cn } from "~/lib/utils";

interface ActionItemProps {
  type: ActionType;
  title: string;
  description?: string;
  icon: React.ReactNode;
  draggable?: boolean;
  onDragStart?: (event: React.DragEvent) => void;
}

export function ActionItem({
  type,
  title,
  description,
  icon,
  draggable,
  onDragStart,
}: ActionItemProps) {
  return (
    <div
      draggable={draggable}
      onDragStart={onDragStart}
      className={cn(
        "flex cursor-grab items-center gap-3 rounded-lg border bg-card p-3 text-left",
        "hover:bg-accent hover:text-accent-foreground",
        "focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring",
        "active:cursor-grabbing"
      )}
    >
      <div className="flex h-9 w-9 shrink-0 items-center justify-center rounded-md border bg-background">
        {icon}
      </div>
      <div className="flex-1 space-y-1">
        <p className="text-sm font-medium leading-none">{title}</p>
        {description && (
          <p className="text-xs text-muted-foreground">{description}</p>
        )}
      </div>
    </div>
  );
} 