"use client";

import React, { useMemo } from "react";
import { useSortable } from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import { useDroppable } from "@dnd-kit/core";
import {
  ChevronRight,
  Trash2,
  Clock,
  GitBranch,
  Repeat,
  Layers,
  List,
  AlertCircle,
  Play,
  HelpCircle,
} from "lucide-react";
import { cn } from "~/lib/utils";
import { type ExperimentAction } from "~/lib/experiment-designer/types";
import { actionRegistry } from "../ActionRegistry";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { useDesignerStore } from "../state/store";

export interface ActionChipProps {
  stepId: string;
  action: ExperimentAction;
  parentId: string | null;
  selectedActionId: string | null | undefined;
  onSelectAction: (stepId: string, actionId: string | undefined) => void;
  onDeleteAction: (stepId: string, actionId: string) => void;
  onReorderAction?: (
    stepId: string,
    actionId: string,
    direction: "up" | "down",
  ) => void;
  dragHandle?: boolean;
  isFirst?: boolean;
  isLast?: boolean;
}

export interface ActionChipVisualsProps {
  action: ExperimentAction;
  isSelected?: boolean;
  isDragging?: boolean;
  isOverNested?: boolean;
  onSelect?: (e: React.MouseEvent) => void;
  onDelete?: (e: React.MouseEvent) => void;
  onReorder?: (direction: "up" | "down") => void;
  dragHandleProps?: React.HTMLAttributes<HTMLElement>;
  children?: React.ReactNode;
  isFirst?: boolean;
  isLast?: boolean;
  validationStatus?: "error" | "warning" | "info";
}

/**
 * Helper to determine visual style based on action type/category
 */
function getActionVisualStyle(action: ExperimentAction) {
  const def = actionRegistry.getAction(action.type);
  const category = def?.category || "other";

  // Specific Control Types
  if (action.type === "hristudio-core.wait" || action.type === "wait") {
    return {
      variant: "wait",
      icon: Clock,
      bg: "bg-amber-500/10 hover:bg-amber-500/20",
      border: "border-amber-200 dark:border-amber-800",
      text: "text-amber-700 dark:text-amber-400",
      accent: "bg-amber-500",
    };
  }

  if (action.type === "hristudio-core.branch" || action.type === "branch") {
    return {
      variant: "branch",
      icon: GitBranch,
      bg: "bg-orange-500/10 hover:bg-orange-500/20",
      border: "border-orange-200 dark:border-orange-800",
      text: "text-orange-700 dark:text-orange-400",
      accent: "bg-orange-500",
    };
  }

  if (action.type === "hristudio-core.loop" || action.type === "loop") {
    return {
      variant: "loop",
      icon: Repeat,
      bg: "bg-purple-500/10 hover:bg-purple-500/20",
      border: "border-purple-200 dark:border-purple-800",
      text: "text-purple-700 dark:text-purple-400",
      accent: "bg-purple-500",
    };
  }

  if (action.type === "hristudio-core.parallel" || action.type === "parallel") {
    return {
      variant: "parallel",
      icon: Layers,
      bg: "bg-emerald-500/10 hover:bg-emerald-500/20",
      border: "border-emerald-200 dark:border-emerald-800",
      text: "text-emerald-700 dark:text-emerald-400",
      accent: "bg-emerald-500",
    };
  }

  // General Categories
  if (category === "wizard") {
    return {
      variant: "wizard",
      icon: HelpCircle,
      bg: "bg-indigo-500/5 hover:bg-indigo-500/10",
      border: "border-indigo-200 dark:border-indigo-800",
      text: "text-indigo-700 dark:text-indigo-300",
      accent: "bg-indigo-500",
    };
  }

  if (
    (category as string) === "robot" ||
    (category as string) === "movement" ||
    (category as string) === "speech"
  ) {
    return {
      variant: "robot",
      icon: Play, // Or specific robot icon if available
      bg: "bg-slate-100 hover:bg-slate-200 dark:bg-slate-800 dark:hover:bg-slate-700",
      border: "border-slate-200 dark:border-slate-700",
      text: "text-slate-700 dark:text-slate-300",
      accent: "bg-slate-500",
    };
  }

  // Default
  return {
    variant: "default",
    icon: undefined,
    bg: "bg-muted/40 hover:bg-accent/40",
    border: "border-border",
    text: "text-foreground",
    accent: "bg-muted-foreground",
  };
}

export function ActionChipVisuals({
  action,
  isSelected,
  isDragging,
  isOverNested,
  onSelect,
  onDelete,
  onReorder,
  dragHandleProps,
  children,
  isFirst,
  isLast,
  validationStatus,
}: ActionChipVisualsProps) {
  const def = actionRegistry.getAction(action.type);
  const style = getActionVisualStyle(action);
  const Icon = style.icon;

  return (
    <div
      className={cn(
        "group relative flex w-full flex-col items-start gap-1 rounded border px-3 py-2 text-[11px] transition-all duration-200",
        style.bg,
        style.border,
        isSelected && "ring-primary border-primary bg-accent/50 ring-2",
        isDragging && "scale-95 opacity-70 shadow-lg",
        isOverNested &&
          !isDragging &&
          "bg-blue-50/50 ring-2 ring-blue-400 ring-offset-1 dark:bg-blue-900/20",
      )}
      onClick={onSelect}
      role="button"
      aria-pressed={isSelected}
      tabIndex={0}
    >
      {/* Accent Bar logic for control flow */}
      {style.variant !== "default" && style.variant !== "robot" && (
        <div
          className={cn(
            "absolute top-0 bottom-0 left-0 w-1 rounded-l",
            style.accent,
          )}
        />
      )}

      <div
        className={cn(
          "flex w-full items-center gap-2",
          style.variant !== "default" && style.variant !== "robot" && "pl-2",
        )}
      >
        <div className="flex min-w-0 flex-1 items-center gap-2">
          {Icon && (
            <Icon className={cn("h-3.5 w-3.5 flex-shrink-0", style.text)} />
          )}
          <span
            className={cn(
              "truncate leading-snug font-medium break-words",
              style.text,
            )}
          >
            {action.name}
          </span>

          {/* Inline Info for Control Actions */}
          {style.variant === "wait" && !!action.parameters.duration && (
            <span className="bg-background/50 text-muted-foreground ml-1 rounded px-1.5 py-0.5 font-mono text-[10px]">
              {String(action.parameters.duration ?? "")}s
            </span>
          )}
          {style.variant === "loop" && (
            <span className="bg-background/50 text-muted-foreground ml-1 rounded px-1.5 py-0.5 font-mono text-[10px]">
              {String(action.parameters.iterations || 1)}x
            </span>
          )}
          {style.variant === "loop" &&
            action.parameters.requireApproval !== false && (
              <span
                className="ml-1 flex items-center gap-0.5 rounded bg-purple-500/20 px-1.5 py-0.5 font-mono text-[10px] text-purple-700 dark:text-purple-300"
                title="Requires Wizard Approval"
              >
                <HelpCircle className="h-2 w-2" />
                Ask
              </span>
            )}

          {validationStatus === "error" && (
            <div
              className="h-2 w-2 flex-shrink-0 rounded-full bg-red-500 ring-1 ring-red-600"
              aria-label="Error"
            />
          )}
          {validationStatus === "warning" && (
            <div
              className="h-2 w-2 flex-shrink-0 rounded-full bg-amber-500 ring-1 ring-amber-600"
              aria-label="Warning"
            />
          )}
        </div>

        <div className="bg-background/50 border-border/50 mr-1 flex items-center gap-0.5 rounded-md border px-0.5 opacity-0 shadow-sm transition-opacity group-hover:opacity-100">
          <Button
            variant="ghost"
            size="sm"
            className="text-muted-foreground hover:text-foreground pointer-events-auto z-20 h-5 w-5 p-0 text-[10px]"
            onClick={(e) => {
              e.stopPropagation();
              onReorder?.("up");
            }}
            disabled={isFirst}
            aria-label="Move action up"
          >
            <ChevronRight className="h-3 w-3 -rotate-90" />
          </Button>
          <Button
            variant="ghost"
            size="sm"
            className="text-muted-foreground hover:text-foreground pointer-events-auto z-20 h-5 w-5 p-0 text-[10px]"
            onClick={(e) => {
              e.stopPropagation();
              onReorder?.("down");
            }}
            disabled={isLast}
            aria-label="Move action down"
          >
            <ChevronRight className="h-3 w-3 rotate-90" />
          </Button>
        </div>

        <button
          type="button"
          onClick={onDelete}
          className="text-muted-foreground hover:text-destructive rounded p-0.5 opacity-0 transition-opacity group-hover:opacity-100"
          aria-label="Delete action"
        >
          <Trash2 className="h-3 w-3" />
        </button>
      </div>

      {/* Description / Subtext */}
      {def?.description && (
        <div
          className={cn(
            "text-muted-foreground mt-0.5 line-clamp-2 w-full pl-2 text-[10px] leading-snug",
            style.variant !== "default" && style.variant !== "robot" && "pl-4",
          )}
        >
          {def.description}
        </div>
      )}

      {/* Tags for parameters (hide for specialized control blocks that show inline) */}
      {def?.parameters?.length &&
      (style.variant === "default" || style.variant === "robot") ? (
        <div className="flex flex-wrap gap-1 pt-1">
          {def.parameters.slice(0, 3).map((p) => (
            <span
              key={p.id}
              className="bg-background/80 text-muted-foreground ring-border max-w-[80px] truncate rounded px-1 py-0.5 text-[9px] font-medium ring-1"
            >
              {p.name}
            </span>
          ))}
          {def.parameters.length > 3 && (
            <span className="text-muted-foreground text-[9px]">
              +{def.parameters.length - 3}
            </span>
          )}
        </div>
      ) : null}

      {children}
    </div>
  );
}

export function SortableActionChip({
  stepId,
  action,
  parentId,
  selectedActionId,
  onSelectAction,
  onDeleteAction,
  onReorderAction,
  dragHandle,
  isFirst,
  isLast,
}: ActionChipProps) {
  const isSelected = selectedActionId === action.id;

  const insertionProjection = useDesignerStore((s) => s.insertionProjection);
  const steps = useDesignerStore((s) => s.steps);
  const currentStep = steps.find((s) => s.id === stepId);

  // Branch Options Visualization
  const branchOptions = useMemo(() => {
    if (!action.type.includes("branch") || !currentStep) return null;

    const options = (currentStep.trigger as any)?.conditions?.options;
    if (
      !options?.length &&
      !(currentStep.trigger as any)?.conditions?.nextStepId
    ) {
      return (
        <div className="text-muted-foreground/60 bg-background/50 mt-2 rounded border border-dashed py-2 text-center text-[10px] italic">
          No branches configured. Add options in properties.
        </div>
      );
    }

    // Combine explicit options and unconditional nextStepId
    // The original FlowWorkspace logic iterated options. logic there:
    // (step.trigger.conditions as any).options.map...

    return (
      <div className="mt-2 w-full space-y-1">
        {options?.map((opt: any, idx: number) => {
          // Resolve ID to name for display
          let targetName = "Unlinked";
          let targetIndex = -1;

          if (opt.nextStepId) {
            const target = steps.find((s) => s.id === opt.nextStepId);
            if (target) {
              targetName = target.name;
              targetIndex = target.order;
            }
          } else if (typeof opt.nextStepIndex === "number") {
            targetIndex = opt.nextStepIndex;
            targetName = `Step #${targetIndex + 1}`;
          }

          return (
            <div
              key={idx}
              className="bg-background/50 flex items-center justify-between rounded border p-1.5 text-[10px] shadow-sm"
            >
              <div className="flex min-w-0 items-center gap-2">
                <Badge
                  variant="outline"
                  className={cn(
                    "bg-background min-w-[60px] justify-center px-1 py-0 text-[9px] font-bold tracking-wider uppercase",
                    opt.variant === "destructive"
                      ? "border-red-500/30 text-red-600 dark:text-red-400"
                      : "text-foreground border-slate-500/30",
                  )}
                >
                  {opt.label}
                </Badge>
                <ChevronRight className="text-muted-foreground/50 h-3 w-3 flex-shrink-0" />
              </div>

              <div className="flex max-w-[60%] min-w-0 items-center justify-end gap-1.5 text-right">
                <span
                  className="text-foreground/80 truncate font-medium"
                  title={targetName}
                >
                  {targetName}
                </span>
                {targetIndex !== -1 && (
                  <Badge
                    variant="secondary"
                    className="h-3.5 min-w-[18px] justify-center bg-slate-100 px-1 py-0 text-[9px] tabular-nums dark:bg-slate-800"
                  >
                    #{targetIndex + 1}
                  </Badge>
                )}
              </div>
            </div>
          );
        })}

        {/* Visual indicator for unconditional jump if present and no options matched (though usually logic handles this) */}
        {/* For now keeping parity with FlowWorkspace which only showed options */}
      </div>
    );
  }, [action.type, currentStep, steps]);

  const displayChildren = useMemo(() => {
    if (
      insertionProjection?.stepId === stepId &&
      insertionProjection.parentId === action.id
    ) {
      const copy = [...(action.children || [])];
      copy.splice(insertionProjection.index, 0, insertionProjection.action);
      return copy;
    }
    return action.children || [];
  }, [action.children, action.id, stepId, insertionProjection]);

  /* ------------------------------------------------------------------------ */
  /* Main Sortable Logic                                                      */
  /* ------------------------------------------------------------------------ */
  const isPlaceholder = action.id === "projection-placeholder";

  // Compute validation status
  const issues = useDesignerStore((s) => s.validationIssues[action.id]);
  const validationStatus = useMemo(() => {
    if (!issues?.length) return undefined;
    if (issues.some((i) => i.severity === "error")) return "error";
    if (issues.some((i) => i.severity === "warning")) return "warning";
    return "info";
  }, [issues]);

  /* ------------------------------------------------------------------------ */
  /* Sortable (Local) DnD Monitoring                                          */
  /* ------------------------------------------------------------------------ */
  // useSortable disabled per user request to remove action drag-and-drop
  // const { ... } = useSortable(...)

  // Use local dragging state or passed prop
  const isDragging = dragHandle || false;

  /* ------------------------------------------------------------------------ */
  /* Nested Droppable (for control flow containers)                           */
  /* ------------------------------------------------------------------------ */
  const def = actionRegistry.getAction(action.type);
  const nestedDroppableId = `container-${action.id}`;
  const { isOver: isOverNested, setNodeRef: setNestedNodeRef } = useDroppable({
    id: nestedDroppableId,
    disabled: !def?.nestable || isPlaceholder, // Disable droppable for placeholder
    data: {
      type: "container",
      stepId,
      parentId: action.id,
      action, // Pass full action for projection logic
    },
  });

  const shouldRenderChildren = !!def?.nestable;

  if (isPlaceholder) {
    return (
      <div
        className={cn(
          "relative flex w-full flex-col items-start gap-1 rounded border border-dashed px-3 py-2 text-[11px]",
          "border-blue-400 bg-blue-50/50 opacity-70 dark:bg-blue-900/20",
        )}
      >
        <div className="flex w-full items-center gap-2">
          <span className="font-medium text-blue-700 italic">
            {action.name}
          </span>
        </div>
      </div>
    );
  }

  return (
    <ActionChipVisuals
      action={action}
      isSelected={isSelected}
      isDragging={isDragging}
      isOverNested={isOverNested && !isDragging}
      onSelect={(e) => {
        e.stopPropagation();
        onSelectAction(stepId, action.id);
      }}
      onDelete={(e) => {
        e.stopPropagation();
        onDeleteAction(stepId, action.id);
      }}
      onReorder={(direction) => onReorderAction?.(stepId, action.id, direction)}
      isFirst={isFirst}
      isLast={isLast}
      validationStatus={validationStatus}
    >
      {/* Branch Options Visualization */}
      {branchOptions}

      {/* Nested Children Rendering (e.g. for Loops/Parallel) */}
      {shouldRenderChildren && (
        <div
          ref={setNestedNodeRef}
          className={cn(
            "mt-2 w-full space-y-2 rounded border border-dashed p-1.5 transition-colors",
            isOverNested
              ? "border-blue-400 bg-blue-100/50 dark:bg-blue-900/20"
              : "bg-muted/20 dark:bg-muted/10 border-border/50",
          )}
        >
          {displayChildren?.length === 0 ? (
            <div className="text-muted-foreground/60 py-2 text-center text-[10px] italic">
              Empty container
            </div>
          ) : (
            displayChildren?.map((child, idx) => (
              <SortableActionChip
                key={child.id}
                stepId={stepId}
                action={child}
                parentId={action.id}
                selectedActionId={selectedActionId}
                onSelectAction={onSelectAction}
                onDeleteAction={onDeleteAction}
                onReorderAction={onReorderAction}
                isFirst={idx === 0}
                isLast={idx === (displayChildren?.length || 0) - 1}
              />
            ))
          )}
        </div>
      )}
    </ActionChipVisuals>
  );
}
