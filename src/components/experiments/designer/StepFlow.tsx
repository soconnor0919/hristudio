"use client";

import React from "react";
import { useDroppable } from "@dnd-kit/core";
import {
  useSortable,
  SortableContext,
  verticalListSortingStrategy,
} from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { ScrollArea } from "~/components/ui/scroll-area";
import {
  GripVertical,
  ChevronDown,
  ChevronRight,
  Plus,
  Trash2,
  Zap,
  MessageSquare,
  Hand,
  Navigation,
  Volume2,
  Clock,
  Eye,
  Bot,
  User,
  Timer,
  MousePointer,
  Mic,
  Activity,
  Play,
  GitBranch,
} from "lucide-react";
import { cn } from "~/lib/utils";
import type {
  ExperimentStep,
  ExperimentAction,
} from "~/lib/experiment-designer/types";
import { actionRegistry } from "./ActionRegistry";

/* -------------------------------------------------------------------------- */
/* Icon Map (localized to avoid cross-file re-render dependencies)            */
/* -------------------------------------------------------------------------- */
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
  GitBranch,
};

/* -------------------------------------------------------------------------- */
/* DroppableStep                                                              */
/* -------------------------------------------------------------------------- */

interface DroppableStepProps {
  stepId: string;
  children: React.ReactNode;
  isEmpty?: boolean;
}

function DroppableStep({ stepId, children, isEmpty }: DroppableStepProps) {
  const { isOver, setNodeRef } = useDroppable({
    id: `step-${stepId}`,
  });

  return (
    <div
      ref={setNodeRef}
      className={cn(
        "min-h-[60px] rounded border-2 border-dashed transition-colors",
        isOver
          ? "border-blue-500 bg-blue-50 dark:bg-blue-950/20"
          : "border-transparent",
        isEmpty && "bg-muted/20",
      )}
    >
      {isEmpty ? (
        <div className="flex items-center justify-center p-4 text-center">
          <div className="text-muted-foreground">
            <Plus className="mx-auto mb-1 h-5 w-5" />
            <p className="text-xs">Drop actions here</p>
          </div>
        </div>
      ) : (
        children
      )}
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* SortableAction                                                             */
/* -------------------------------------------------------------------------- */

interface SortableActionProps {
  action: ExperimentAction;
  index: number;
  isSelected: boolean;
  onSelect: () => void;
  onDelete: () => void;
}

function SortableAction({
  action,
  index,
  isSelected,
  onSelect,
  onDelete,
}: SortableActionProps) {
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({ id: action.id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
  };

  const def = actionRegistry.getAction(action.type);
  const IconComponent = iconMap[def?.icon ?? "Zap"] ?? Zap;

  const categoryColors = {
    wizard: "bg-blue-500",
    robot: "bg-emerald-500",
    control: "bg-amber-500",
    observation: "bg-purple-500",
  } as const;

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...attributes}
      className={cn(
        "group flex cursor-pointer items-center justify-between rounded border p-2 text-xs transition-colors",
        isSelected
          ? "border-blue-500 bg-blue-50 dark:border-blue-400 dark:bg-blue-950/30"
          : "hover:bg-accent/50",
        isDragging && "opacity-50",
      )}
      onClick={onSelect}
    >
      <div className="flex items-center gap-2">
        <div
          {...listeners}
          className="text-muted-foreground/80 hover:text-foreground cursor-grab rounded p-0.5"
        >
          <GripVertical className="h-3 w-3" />
        </div>
        <Badge variant="outline" className="h-4 text-[10px]">
          {index + 1}
        </Badge>
        {def && (
          <div
            className={cn(
              "flex h-4 w-4 flex-shrink-0 items-center justify-center rounded text-white",
              categoryColors[def.category],
            )}
          >
            <IconComponent className="h-2.5 w-2.5" />
          </div>
        )}
        <span className="flex items-center gap-1 truncate font-medium">
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
        </span>
        <Badge variant="secondary" className="h-4 text-[10px] capitalize">
          {(action.type ?? "").replace(/_/g, " ")}
        </Badge>
      </div>
      <Button
        variant="ghost"
        size="sm"
        className="h-5 w-5 p-0 opacity-0 transition-opacity group-hover:opacity-100"
        onClick={(e) => {
          e.stopPropagation();
          onDelete();
        }}
      >
        <Trash2 className="h-3 w-3" />
      </Button>
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* SortableStep                                                               */
/* -------------------------------------------------------------------------- */

interface SortableStepProps {
  step: ExperimentStep;
  index: number;
  isSelected: boolean;
  selectedActionId: string | null;
  onSelect: () => void;
  onDelete: () => void;
  onUpdate: (updates: Partial<ExperimentStep>) => void;
  onActionSelect: (actionId: string) => void;
  onActionDelete: (actionId: string) => void;
}

function SortableStep({
  step,
  index,
  isSelected,
  selectedActionId,
  onSelect,
  onDelete,
  onUpdate,
  onActionSelect,
  onActionDelete,
}: SortableStepProps) {
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({ id: step.id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
  };

  const stepTypeColors: Record<ExperimentStep["type"], string> = {
    sequential: "border-l-blue-500",
    parallel: "border-l-emerald-500",
    conditional: "border-l-amber-500",
    loop: "border-l-purple-500",
  };

  return (
    <div ref={setNodeRef} style={style} {...attributes}>
      <Card
        className={cn(
          "border-l-4 transition-all",
          stepTypeColors[step.type],
          isSelected
            ? "bg-blue-50/50 ring-2 ring-blue-500 dark:bg-blue-950/20 dark:ring-blue-400"
            : "",
          isDragging && "rotate-2 opacity-50 shadow-lg",
        )}
      >
        <CardHeader className="cursor-pointer pb-2" onClick={() => onSelect()}>
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Button
                variant="ghost"
                size="sm"
                className="h-6 w-6 p-0"
                onClick={(e) => {
                  e.stopPropagation();
                  onUpdate({ expanded: !step.expanded });
                }}
              >
                {step.expanded ? (
                  <ChevronDown className="h-4 w-4" />
                ) : (
                  <ChevronRight className="h-4 w-4" />
                )}
              </Button>
              <Badge variant="outline" className="h-5 text-xs">
                {index + 1}
              </Badge>
              <div>
                <div className="text-sm font-medium">{step.name}</div>
                <div className="text-muted-foreground text-xs">
                  {step.actions.length} actions • {step.type}
                </div>
              </div>
            </div>
            <div className="flex items-center gap-1">
              <Button
                variant="ghost"
                size="sm"
                className="h-6 w-6 p-0"
                onClick={(e) => {
                  e.stopPropagation();
                  onDelete();
                }}
              >
                <Trash2 className="h-3 w-3" />
              </Button>
              <div {...listeners} className="cursor-grab p-1">
                <GripVertical className="text-muted-foreground h-4 w-4" />
              </div>
            </div>
          </div>
        </CardHeader>
        {step.expanded && (
          <CardContent className="pt-0">
            <DroppableStep stepId={step.id} isEmpty={step.actions.length === 0}>
              {step.actions.length > 0 && (
                <SortableContext
                  items={step.actions.map((a) => a.id)}
                  strategy={verticalListSortingStrategy}
                >
                  <div className="space-y-1">
                    {step.actions.map((action, actionIndex) => (
                      <SortableAction
                        key={action.id}
                        action={action}
                        index={actionIndex}
                        isSelected={selectedActionId === action.id}
                        onSelect={() => onActionSelect(action.id)}
                        onDelete={() => onActionDelete(action.id)}
                      />
                    ))}
                  </div>
                </SortableContext>
              )}
            </DroppableStep>
          </CardContent>
        )}
      </Card>
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* StepFlow (Scrollable Container of Steps)                                   */
/* -------------------------------------------------------------------------- */

export interface StepFlowProps {
  steps: ExperimentStep[];
  selectedStepId: string | null;
  selectedActionId: string | null;
  onStepSelect: (id: string) => void;
  onStepDelete: (id: string) => void;
  onStepUpdate: (id: string, updates: Partial<ExperimentStep>) => void;
  onActionSelect: (actionId: string) => void;
  onActionDelete: (stepId: string, actionId: string) => void;
  onActionUpdate?: (
    stepId: string,
    actionId: string,
    updates: Partial<ExperimentAction>,
  ) => void;
  emptyState?: React.ReactNode;
  headerRight?: React.ReactNode;
}

export function StepFlow({
  steps,
  selectedStepId,
  selectedActionId,
  onStepSelect,
  onStepDelete,
  onStepUpdate,
  onActionSelect,
  onActionDelete,
  emptyState,
  headerRight,
}: StepFlowProps) {
  return (
    <Card className="h-[calc(100vh-12rem)]">
      <CardHeader className="pb-2">
        <CardTitle className="flex items-center justify-between text-sm">
          <div className="flex items-center gap-2">
            <GitBranch className="h-4 w-4" />
            Experiment Flow
          </div>
          {headerRight}
        </CardTitle>
      </CardHeader>
      <CardContent className="p-0">
        <ScrollArea className="h-full">
          <div className="p-2">
            {steps.length === 0 ? (
              (emptyState ?? (
                <div className="py-8 text-center">
                  <GitBranch className="text-muted-foreground/50 mx-auto h-8 w-8" />
                  <h3 className="mt-2 text-sm font-medium">No steps yet</h3>
                  <p className="text-muted-foreground mt-1 text-xs">
                    Add your first step to begin designing
                  </p>
                </div>
              ))
            ) : (
              <SortableContext
                items={steps.map((s) => s.id)}
                strategy={verticalListSortingStrategy}
              >
                <div className="space-y-2">
                  {steps.map((step, index) => (
                    <div key={step.id}>
                      <SortableStep
                        step={step}
                        index={index}
                        isSelected={selectedStepId === step.id}
                        selectedActionId={selectedActionId}
                        onSelect={() => onStepSelect(step.id)}
                        onDelete={() => onStepDelete(step.id)}
                        onUpdate={(updates) => onStepUpdate(step.id, updates)}
                        onActionSelect={onActionSelect}
                        onActionDelete={(actionId) =>
                          onActionDelete(step.id, actionId)
                        }
                      />
                      {index < steps.length - 1 && (
                        <div className="flex justify-center py-1">
                          <div className="bg-border h-2 w-px" />
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </SortableContext>
            )}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  );
}
