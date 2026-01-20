"use client";

import React, {
  useCallback,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import {
  useDroppable,
  useDndMonitor,
  type DragEndEvent,
  type DragStartEvent,
  type DragOverEvent,
} from "@dnd-kit/core";
import {
  useSortable,
  SortableContext,
  verticalListSortingStrategy,
} from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import {
  ChevronDown,
  ChevronRight,
  GripVertical,
  Plus,
  Trash2,
  GitBranch,
  Edit3,
} from "lucide-react";
import { cn } from "~/lib/utils";
import {
  type ExperimentStep,
  type ExperimentAction,
} from "~/lib/experiment-designer/types";
import { useDesignerStore } from "../state/store";
import { actionRegistry } from "../ActionRegistry";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Input } from "~/components/ui/input";

/**
 * FlowWorkspace
 *
 * Virtualized step + action workspace with local (Option B) sortable handling.
 * Reordering is processed locally via useDndMonitor (not in DesignerRoot)
 * to keep orchestration layer simpler and reduce cross-component coupling.
 *
 * Features:
 *  - Virtualized step list (absolute positioned variable heights)
 *  - Inline step rename
 *  - Step & action creation / deletion
 *  - Step and action reordering (drag handles)
 *  - Drag-from-library action insertion (handled by root DnD; droppables here)
 *  - Empty step drop affordance + highlight
 *
 * Sortable ID strategy (to avoid collision with palette action ids):
 *  - Sortable Step:   s-step-<stepId>
 *  - Sortable Action: s-act-<actionId>
 *  - Droppable Step:  step-<stepId> (kept for root palette drops)
 */

interface FlowWorkspaceProps {
  className?: string;
  overscan?: number;
  onStepCreate?: (step: ExperimentStep) => void;
  onStepDelete?: (stepId: string) => void;
  onActionCreate?: (stepId: string, action: ExperimentAction) => void;
}

export interface VirtualItem {
  index: number;
  top: number;
  height: number;
  step: ExperimentStep;
  key: string;
  visible: boolean;
}

interface StepRowProps {
  item: VirtualItem;
  selectedStepId: string | null | undefined;
  selectedActionId: string | null | undefined;
  renamingStepId: string | null;
  onSelectStep: (id: string | undefined) => void;
  onSelectAction: (stepId: string, actionId: string | undefined) => void;
  onToggleExpanded: (step: ExperimentStep) => void;
  onRenameStep: (step: ExperimentStep, name: string) => void;
  onDeleteStep: (step: ExperimentStep) => void;
  onDeleteAction: (stepId: string, actionId: string) => void;
  setRenamingStepId: (id: string | null) => void;
  registerMeasureRef: (stepId: string, el: HTMLDivElement | null) => void;
}

const StepRow = React.memo(function StepRow({
  item,
  selectedStepId,
  selectedActionId,
  renamingStepId,
  onSelectStep,
  onSelectAction,
  onToggleExpanded,
  onRenameStep,
  onDeleteStep,
  onDeleteAction,
  setRenamingStepId,
  registerMeasureRef,
}: StepRowProps) {
  const step = item.step;
  const insertionProjection = useDesignerStore((s) => s.insertionProjection);

  const displayActions = useMemo(() => {
    if (
      insertionProjection?.stepId === step.id &&
      insertionProjection.parentId === null
    ) {
      const copy = [...step.actions];
      // Insert placeholder action
      // Ensure specific ID doesn't crash keys if collision (collision unlikely for library items)
      // Actually, standard array key is action.id.
      copy.splice(insertionProjection.index, 0, insertionProjection.action);
      return copy;
    }
    return step.actions;
  }, [step.actions, step.id, insertionProjection]);

  const {
    setNodeRef,
    transform,
    transition,
    attributes,
    listeners,
    isDragging,
  } = useSortable({
    id: sortableStepId(step.id),
    data: {
      type: "step",
      step: step,
    },
  });

  const style: React.CSSProperties = {
    position: "absolute",
    top: item.top,
    left: 0,
    right: 0,
    width: "100%",
    transform: CSS.Transform.toString(transform),
    transition,
    zIndex: isDragging ? 25 : undefined,
  };

  return (
    <div ref={setNodeRef} style={style} data-step-id={step.id}>
      <div
        ref={(el) => registerMeasureRef(step.id, el)}
        className="relative px-3 py-4"
        data-step-id={step.id}
      >
        <StepDroppableArea stepId={step.id} />
        <div
          className={cn(
            "mb-2 rounded border shadow-sm transition-colors",
            selectedStepId === step.id
              ? "border-border bg-accent/30"
              : "hover:bg-accent/30",
            isDragging && "opacity-80 ring-1 ring-blue-300",
          )}
        >
          <div
            className="flex items-center justify-between gap-2 border-b px-2 py-1.5"
            onClick={(e) => {
              const tag = (e.target as HTMLElement).tagName.toLowerCase();
              if (tag === "input" || tag === "textarea" || tag === "button")
                return;
              onSelectStep(step.id);
              onSelectAction(step.id, undefined);
            }}
            role="button"
            tabIndex={0}
          >
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={(e) => {
                  e.stopPropagation();
                  onToggleExpanded(step);
                }}
                className="text-muted-foreground hover:bg-accent/60 hover:text-foreground rounded p-1"
                aria-label={step.expanded ? "Collapse step" : "Expand step"}
              >
                {step.expanded ? (
                  <ChevronDown className="h-4 w-4" />
                ) : (
                  <ChevronRight className="h-4 w-4" />
                )}
              </button>
              <Badge
                variant="outline"
                className="h-5 px-1.5 text-[10px] font-normal"
              >
                {step.order + 1}
              </Badge>
              {renamingStepId === step.id ? (
                <Input
                  autoFocus
                  defaultValue={step.name}
                  className="h-7 w-40 text-xs"
                  onClick={(e) => e.stopPropagation()}
                  onKeyDown={(e) => {
                    if (e.key === "Enter") {
                      onRenameStep(
                        step,
                        (e.target as HTMLInputElement).value.trim() ||
                        step.name,
                      );
                      setRenamingStepId(null);
                    } else if (e.key === "Escape") {
                      setRenamingStepId(null);
                    }
                  }}
                  onBlur={(e) => {
                    onRenameStep(step, e.target.value.trim() || step.name);
                    setRenamingStepId(null);
                  }}
                />
              ) : (
                <div className="flex items-center gap-1">
                  <span className="text-sm font-medium">{step.name}</span>
                  <button
                    type="button"
                    className="text-muted-foreground hover:text-foreground p-1 opacity-0 group-hover:opacity-100"
                    aria-label="Rename step"
                    onClick={(e) => {
                      e.stopPropagation();
                      setRenamingStepId(step.id);
                    }}
                  >
                    <Edit3 className="h-3.5 w-3.5" />
                  </button>
                </div>
              )}
              <span className="text-muted-foreground hidden text-[11px] md:inline">
                {step.actions.length} actions
              </span>
            </div>
            <div className="flex items-center gap-1">
              <Button
                variant="ghost"
                size="sm"
                className="h-7 w-7 p-0 text-[11px] text-red-500 hover:text-red-600"
                onClick={(e) => {
                  e.stopPropagation();
                  onDeleteStep(step);
                }}
                aria-label="Delete step"
              >
                <Trash2 className="h-3.5 w-3.5" />
              </Button>
              <div
                className="text-muted-foreground cursor-grab p-1"
                aria-label="Drag step"
                {...attributes}
                {...listeners}
              >
                <GripVertical className="h-4 w-4" />
              </div>
            </div>
          </div>

          {/* Action List (Collapsible/Virtual content) */}
          {step.expanded && (
            <div className="bg-background/40 min-h-[3rem] space-y-2 p-2 pb-8">
              <SortableContext
                items={displayActions.map((a) => sortableActionId(a.id))}
                strategy={verticalListSortingStrategy}
              >
                <div className="flex w-full flex-col gap-2">
                  {displayActions.length === 0 ? (
                    <div className="flex h-12 items-center justify-center rounded border border-dashed text-xs text-muted-foreground">
                      Drop actions here
                    </div>
                  ) : (
                    displayActions.map((action) => (
                      <SortableActionChip
                        key={action.id}
                        stepId={step.id}
                        action={action}
                        parentId={null}
                        selectedActionId={selectedActionId}
                        onSelectAction={onSelectAction}
                        onDeleteAction={onDeleteAction}
                      />
                    ))
                  )}
                </div>
              </SortableContext>
            </div>
          )}
        </div>
      </div>
    </div>
  );
});

/* -------------------------------------------------------------------------- */
/* Utility                                                                    */
/* -------------------------------------------------------------------------- */

function generateStepId(): string {
  return `step-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`;
}



function sortableStepId(stepId: string) {
  return `s-step-${stepId}`;
}
function sortableActionId(actionId: string) {
  return `s-act-${actionId}`;
}
function parseSortableStep(id: string): string | null {
  return id.startsWith("s-step-") ? id.slice("s-step-".length) : null;
}
function parseSortableAction(id: string): string | null {
  return id.startsWith("s-act-") ? id.slice("s-act-".length) : null;
}

/* -------------------------------------------------------------------------- */
/* Droppable Overlay (for palette action drops)                               */
/* -------------------------------------------------------------------------- */
function StepDroppableArea({ stepId }: { stepId: string }) {
  const { isOver } = useDroppable({ id: `step-${stepId}` });
  return (
    <div
      data-step-drop
      className={cn(
        "pointer-events-none absolute inset-0 rounded-md transition-colors",
        isOver &&
        "bg-blue-50/40 ring-2 ring-blue-400/60 ring-offset-0 dark:bg-blue-950/20",
      )}
    />
  );
}

/* -------------------------------------------------------------------------- */
/* Sortable Action Chip                                                       */
/* -------------------------------------------------------------------------- */

interface ActionChipProps {
  stepId: string;
  action: ExperimentAction;
  parentId: string | null;
  selectedActionId: string | null | undefined;
  onSelectAction: (stepId: string, actionId: string | undefined) => void;
  onDeleteAction: (stepId: string, actionId: string) => void;
  dragHandle?: boolean;
}

function SortableActionChip({
  stepId,
  action,
  parentId,
  selectedActionId,
  onSelectAction,
  onDeleteAction,
  dragHandle,
}: ActionChipProps) {
  const def = actionRegistry.getAction(action.type);
  const isSelected = selectedActionId === action.id;

  const insertionProjection = useDesignerStore((s) => s.insertionProjection);
  const displayChildren = useMemo(() => {
    if (
      insertionProjection?.stepId === stepId &&
      insertionProjection.parentId === action.id
    ) {
      const copy = [...(action.children || [])];
      copy.splice(insertionProjection.index, 0, insertionProjection.action);
      return copy;
    }
    return action.children;
  }, [action.children, action.id, stepId, insertionProjection]);

  /* ------------------------------------------------------------------------ */
  /* Main Sortable Logic                                                      */
  /* ------------------------------------------------------------------------ */
  const isPlaceholder = action.id === "projection-placeholder";

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging: isSortableDragging,
  } = useSortable({
    id: sortableActionId(action.id),
    disabled: isPlaceholder, // Disable sortable for placeholder
    data: {
      type: "action",
      stepId,
      parentId,
      id: action.id,
    },
  });

  // Use local dragging state or passed prop
  const isDragging = isSortableDragging || dragHandle;

  const style = {
    transform: CSS.Translate.toString(transform),
    transition,
  };

  /* ------------------------------------------------------------------------ */
  /* Nested Droppable (for control flow containers)                           */
  /* ------------------------------------------------------------------------ */
  const nestedDroppableId = `container-${action.id}`;
  const {
    isOver: isOverNested,
    setNodeRef: setNestedNodeRef
  } = useDroppable({
    id: nestedDroppableId,
    disabled: !def?.nestable || isPlaceholder, // Disable droppable for placeholder
    data: {
      type: "container",
      stepId,
      parentId: action.id,
      action // Pass full action for projection logic
    }
  });

  const shouldRenderChildren = def?.nestable;

  if (isPlaceholder) {
    const { setNodeRef: setPlaceholderRef } = useDroppable({
      id: "projection-placeholder",
      data: { type: "placeholder" }
    });

    // Render simplified placeholder without hooks refs
    // We still render the content matching the action type for visual fidelity
    return (
      <div
        ref={setPlaceholderRef}
        className="group relative flex w-full flex-col items-start gap-1 rounded border-2 border-dashed border-blue-300 bg-blue-50/50 px-3 py-2 text-[11px] opacity-70"
      >
        <div className="flex w-full items-center gap-2">
          <span className={cn(
            "h-2.5 w-2.5 rounded-full",
            def ? {
              wizard: "bg-blue-500",
              robot: "bg-emerald-500",
              control: "bg-amber-500",
              observation: "bg-purple-500",
            }[def.category] : "bg-gray-400"
          )} />
          <span className="font-medium text-foreground">{def?.name ?? action.name}</span>
        </div>
        {def?.description && (
          <div className="text-muted-foreground line-clamp-3 w-full text-[10px] leading-snug">
            {def.description}
          </div>
        )}
      </div>
    );
  }

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={cn(
        "group relative flex w-full flex-col items-start gap-1 rounded border px-3 py-2 text-[11px]",
        "bg-muted/40 hover:bg-accent/40 cursor-pointer",
        isSelected && "border-border bg-accent/30",
        isDragging && "opacity-70 shadow-lg",
        // Visual feedback for nested drop
        isOverNested && !isDragging && "ring-2 ring-blue-400 ring-offset-1 bg-blue-50/50"
      )}
      onClick={(e) => {
        e.stopPropagation();
        onSelectAction(stepId, action.id);
      }}
      {...attributes}
      role="button"
      aria-pressed={isSelected}
      tabIndex={0}
    >
      <div className="flex w-full items-center gap-2">
        <div
          {...listeners}
          className="text-muted-foreground/70 hover:text-foreground cursor-grab rounded p-0.5"
          aria-label="Drag action"
        >
          <GripVertical className="h-3.5 w-3.5" />
        </div>
        <span
          className={cn(
            "h-2.5 w-2.5 rounded-full",
            def
              ? {
                wizard: "bg-blue-500",
                robot: "bg-emerald-500",
                control: "bg-amber-500",
                observation: "bg-purple-500",
              }[def.category]
              : "bg-slate-400",
          )}
        />
        <span className="flex-1 leading-snug font-medium break-words">
          {action.name}
        </span>
        <button
          type="button"
          onClick={(e) => {
            e.stopPropagation();
            onDeleteAction(stepId, action.id);
          }}
          className="text-muted-foreground hover:text-foreground rounded p-0.5 opacity-0 transition-opacity group-hover:opacity-100"
          aria-label="Delete action"
        >
          <Trash2 className="h-3 w-3" />
        </button>
      </div>
      {def?.description && (
        <div className="text-muted-foreground line-clamp-3 w-full text-[10px] leading-snug">
          {def.description}
        </div>
      )}
      {def?.parameters.length ? (
        <div className="flex flex-wrap gap-1 pt-0.5">
          {def.parameters.slice(0, 4).map((p) => (
            <span
              key={p.id}
              className="bg-background/70 text-muted-foreground ring-border rounded px-1 py-0.5 text-[9px] font-medium ring-1"
            >
              {p.name}
            </span>
          ))}
          {def.parameters.length > 4 && (
            <span className="text-[9px] text-muted-foreground">+{def.parameters.length - 4}</span>
          )}
        </div>
      ) : null}

      {/* Nested Actions Container */}
      {shouldRenderChildren && (
        <div
          ref={setNestedNodeRef}
          className={cn(
            "mt-2 w-full flex flex-col gap-2 pl-4 border-l-2 border-border/40 transition-all min-h-[0.5rem] pb-4",
          )}
        >
          <SortableContext
            items={(displayChildren ?? action.children ?? [])
              .filter(c => c.id !== "projection-placeholder")
              .map(c => sortableActionId(c.id))}
            strategy={verticalListSortingStrategy}
          >
            {(displayChildren || action.children || []).map((child) => (
              <SortableActionChip
                key={child.id}
                stepId={stepId}
                action={child}
                parentId={action.id}
                selectedActionId={selectedActionId}
                onSelectAction={onSelectAction}
                onDeleteAction={onDeleteAction}
              />
            ))}
            {(!displayChildren?.length && !action.children?.length) && (
              <div className="text-[10px] text-muted-foreground/60 italic py-1">
                Drag actions here
              </div>
            )}
          </SortableContext>
        </div>
      )}

    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* FlowWorkspace Component                                                     */
/* -------------------------------------------------------------------------- */

export function FlowWorkspace({
  className,
  overscan = 400,
  onStepCreate,
  onStepDelete,
  onActionCreate: _onActionCreate,
}: FlowWorkspaceProps) {
  /* Store selectors */
  const steps = useDesignerStore((s) => s.steps);
  const selectStep = useDesignerStore((s) => s.selectStep);
  const selectAction = useDesignerStore((s) => s.selectAction);
  const selectedStepId = useDesignerStore((s) => s.selectedStepId);
  const selectedActionId = useDesignerStore((s) => s.selectedActionId);

  const upsertStep = useDesignerStore((s) => s.upsertStep);
  const removeStep = useDesignerStore((s) => s.removeStep);

  const removeAction = useDesignerStore((s) => s.removeAction);
  const reorderStep = useDesignerStore((s) => s.reorderStep);
  const moveAction = useDesignerStore((s) => s.moveAction);
  const recomputeHash = useDesignerStore((s) => s.recomputeHash);

  /* Local state */
  const containerRef = useRef<HTMLDivElement | null>(null);
  const measureRefs = useRef<Map<string, HTMLDivElement>>(new Map());
  const roRef = useRef<ResizeObserver | null>(null);
  const pendingHeightsRef = useRef<Map<string, number> | null>(null);
  const heightsRafRef = useRef<number | null>(null);
  const [heights, setHeights] = useState<Map<string, number>>(new Map());
  const [scrollTop, setScrollTop] = useState(0);
  const [viewportHeight, setViewportHeight] = useState(600);
  const [renamingStepId, setRenamingStepId] = useState<string | null>(null);
  // dragKind state removed (unused after refactor)

  /* Parent lookup for action reorder */
  const actionParentMap = useMemo(() => {
    const map = new Map<string, string>();
    for (const step of steps) {
      for (const a of step.actions) {
        map.set(a.id, step.id);
      }
    }
    return map;
  }, [steps]);

  /* Resize observer for viewport and width changes */
  useLayoutEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const observer = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const cr = entry.contentRect;
        setViewportHeight(cr.height);
        // Do not invalidate all heights on width change; per-step observers will update as needed
      }
    });
    observer.observe(el);

    setViewportHeight(el.clientHeight);
    return () => observer.disconnect();
  }, []);

  /* Per-step measurement observer (attach/detach on ref set) */
  useLayoutEffect(() => {
    roRef.current = new ResizeObserver((entries) => {
      pendingHeightsRef.current ??= new Map();
      for (const entry of entries) {
        const id = entry.target.getAttribute("data-step-id");
        if (!id) continue;
        const h = entry.contentRect.height;
        pendingHeightsRef.current.set(id, h);
      }
      heightsRafRef.current ??= requestAnimationFrame(() => {
        const pending = pendingHeightsRef.current;
        heightsRafRef.current = null;
        pendingHeightsRef.current = null;
        if (!pending) return;
        setHeights((prev) => {
          let changed = false;
          const next = new Map(prev);
          for (const [id, h] of pending) {
            if (prev.get(id) !== h) {
              next.set(id, h);
              changed = true;
            }
          }
          return changed ? next : prev;
        });
      });
    });
    return () => {
      if (heightsRafRef.current) cancelAnimationFrame(heightsRafRef.current);
      heightsRafRef.current = null;
      pendingHeightsRef.current = null;
      roRef.current?.disconnect();
      roRef.current = null;
    };
  }, []);

  /* Scroll */
  const onScroll = useCallback(() => {
    if (!containerRef.current) return;
    setScrollTop(containerRef.current.scrollTop);
  }, []);

  /* Virtual items */
  const estimatedBaseHeight = 140;
  const virtualItems: VirtualItem[] = useMemo(() => {
    const out: VirtualItem[] = [];
    let offset = 0;
    steps.forEach((step, idx) => {
      const h = heights.get(step.id) ?? estimatedBaseHeight;
      const top = offset;
      const visible =
        top + h > scrollTop - overscan &&
        top < scrollTop + viewportHeight + overscan;
      out.push({
        index: idx,
        top,
        height: h,
        step,
        key: step.id,
        visible,
      });
      offset += h;
    });
    return out;
  }, [steps, heights, scrollTop, viewportHeight, overscan]);

  const totalHeight = useMemo(
    () =>
      steps.reduce(
        (sum, step) => sum + (heights.get(step.id) ?? estimatedBaseHeight),
        0,
      ),
    [steps, heights],
  );

  /* CRUD Helpers */
  const createStep = useCallback(
    (insertIndex?: number) => {
      const newStep: ExperimentStep = {
        id: generateStepId(),
        name: `Step ${steps.length + 1}`,
        description: "",
        type: "sequential",
        order: steps.length,
        trigger:
          steps.length === 0
            ? { type: "trial_start", conditions: {} }
            : { type: "previous_step", conditions: {} },
        actions: [],
        expanded: true,
      };
      if (
        typeof insertIndex === "number" &&
        insertIndex >= 0 &&
        insertIndex < steps.length
      ) {
        // Insert with manual reindex
        const reordered = steps
          .slice(0, insertIndex + 1)
          .concat([newStep], steps.slice(insertIndex + 1))
          .map((s, i) => ({ ...s, order: i }));
        reordered.forEach((s) => upsertStep(s));
      } else {
        upsertStep(newStep);
      }
      selectStep(newStep.id);
      onStepCreate?.(newStep);
      void recomputeHash();
    },
    [steps, upsertStep, selectStep, onStepCreate, recomputeHash],
  );

  const deleteStep = useCallback(
    (step: ExperimentStep) => {
      removeStep(step.id);
      onStepDelete?.(step.id);
      if (selectedStepId === step.id) selectStep(undefined);
      void recomputeHash();
    },
    [removeStep, onStepDelete, selectedStepId, selectStep, recomputeHash],
  );

  const toggleExpanded = useCallback(
    (step: ExperimentStep) => {
      upsertStep({ ...step, expanded: !step.expanded });
    },
    [upsertStep],
  );

  const renameStep = useCallback(
    (step: ExperimentStep, name: string) => {
      upsertStep({ ...step, name });
    },
    [upsertStep],
  );

  const deleteAction = useCallback(
    (stepId: string, actionId: string) => {
      removeAction(stepId, actionId);
      if (selectedActionId === actionId) selectAction(stepId, undefined);
      void recomputeHash();
    },
    [removeAction, selectedActionId, selectAction, recomputeHash],
  );

  /* ------------------------------------------------------------------------ */
  /* Sortable (Local) DnD Monitoring                                          */
  /* ------------------------------------------------------------------------ */

  const handleLocalDragStart = useCallback((e: DragStartEvent) => {
    const id = e.active.id.toString();
    if (id.startsWith("action-")) {
      // no-op
    }
  }, []);

  const handleLocalDragEnd = useCallback(
    (e: DragEndEvent) => {
      const { active, over } = e;
      if (!over || !active) {
        return;
      }
      const activeId = active.id.toString();
      const overId = over.id.toString();
      // Step reorder
      if (activeId.startsWith("s-step-") && overId.startsWith("s-step-")) {
        const fromStepId = parseSortableStep(activeId);
        const toStepId = parseSortableStep(overId);
        if (fromStepId && toStepId && fromStepId !== toStepId) {
          const fromIndex = steps.findIndex((s) => s.id === fromStepId);
          const toIndex = steps.findIndex((s) => s.id === toStepId);
          if (fromIndex >= 0 && toIndex >= 0) {
            reorderStep(fromIndex, toIndex);
            void recomputeHash();
          }
        }
      }
      // Action reorder (supports nesting)
      if (activeId.startsWith("s-act-") && overId.startsWith("s-act-")) {
        const activeData = active.data.current;
        const overData = over.data.current;

        if (
          activeData && overData &&
          activeData.stepId === overData.stepId &&
          activeData.type === 'action' && overData.type === 'action'
        ) {
          const stepId = activeData.stepId as string;
          const activeActionId = activeData.action.id;
          const overActionId = overData.action.id;

          if (activeActionId !== overActionId) {
            const newParentId = overData.parentId as string | null;
            const newIndex = overData.sortable.index; // index within that parent's list

            moveAction(stepId, activeActionId, newParentId, newIndex);
            void recomputeHash();
          }
        }
      }
    },
    [steps, reorderStep, moveAction, recomputeHash],
  );

  /* ------------------------------------------------------------------------ */
  /* Drag Over (Live Sorting)                                                  */
  /* ------------------------------------------------------------------------ */
  const handleLocalDragOver = useCallback(
    (event: DragOverEvent) => {
      const { active, over } = event;
      if (!over) return;

      const activeId = active.id.toString();
      const overId = over.id.toString();

      // Only handle action reordering
      if (activeId.startsWith("s-act-") && overId.startsWith("s-act-")) {
        const activeData = active.data.current;
        const overData = over.data.current;

        if (
          activeData &&
          overData &&
          activeData.type === 'action' &&
          overData.type === 'action'
        ) {
          const activeActionId = activeData.action.id;
          const overActionId = overData.action.id;
          const activeStepId = activeData.stepId;
          const overStepId = overData.stepId;
          const activeParentId = activeData.parentId;
          const overParentId = overData.parentId;

          // If moving between different lists (parents/steps), move immediately to visualize snap
          if (activeParentId !== overParentId || activeStepId !== overStepId) {
            // Determine new index
            // verification of safe move handled by store
            moveAction(overStepId, activeActionId, overParentId, overData.sortable.index);
          }
        }
      }
    },
    [moveAction]
  );

  useDndMonitor({
    onDragStart: handleLocalDragStart,
    onDragOver: handleLocalDragOver,
    onDragEnd: handleLocalDragEnd,
    onDragCancel: () => {
      // no-op
    },
  });

  /* ------------------------------------------------------------------------ */
  /* Step Row (Sortable + Virtualized)                                         */
  /* ------------------------------------------------------------------------ */
  // StepRow moved outside of component to prevent re-mounting on every render (flashing fix)

  const registerMeasureRef = useCallback(
    (stepId: string, el: HTMLDivElement | null) => {
      const prev = measureRefs.current.get(stepId) ?? null;
      if (prev && prev !== el) {
        roRef.current?.unobserve(prev);
        measureRefs.current.delete(stepId);
      }
      if (el) {
        measureRefs.current.set(stepId, el);
        roRef.current?.observe(el);
      }
    },
    [],
  );

  /* ------------------------------------------------------------------------ */
  /* Render                                                                    */
  /* ------------------------------------------------------------------------ */
  return (
    <div className={cn("flex h-full min-h-0 flex-col", className)}>
      <div className="flex items-center justify-between border-b px-3 py-2 text-xs">
        <div className="flex items-center gap-3 font-medium">
          <span className="text-muted-foreground flex items-center gap-1">
            <GitBranch className="h-4 w-4" />
            Flow
          </span>
          <span className="text-muted-foreground/70">
            {steps.length} steps •{" "}
            {steps.reduce((s, st) => s + st.actions.length, 0)} actions
          </span>
        </div>
        <div className="flex items-center gap-2">
          <Button
            size="sm"
            className="h-7 px-2 text-[11px]"
            onClick={() => createStep()}
          >
            <Plus className="mr-1 h-3 w-3" />
            Step
          </Button>
        </div>
      </div>

      <div
        ref={containerRef}
        className="relative h-0 min-h-0 flex-1 overflow-x-hidden overflow-y-auto"
        onScroll={onScroll}
      >
        {steps.length === 0 ? (
          <div className="absolute inset-0 flex items-center justify-center p-6">
            <div className="text-center">
              <div className="mx-auto mb-3 flex h-12 w-12 items-center justify-center rounded-full border">
                <GitBranch className="text-muted-foreground h-6 w-6" />
              </div>
              <p className="mb-2 text-sm font-medium">No steps yet</p>
              <p className="text-muted-foreground mb-3 text-xs">
                Create your first step to begin designing the flow.
              </p>
              <Button
                size="sm"
                className="h-7 px-2 text-[11px]"
                onClick={() => createStep()}
              >
                <Plus className="mr-1 h-3 w-3" /> Add Step
              </Button>
            </div>
          </div>
        ) : (
          <SortableContext
            items={steps.map((s) => sortableStepId(s.id))}
            strategy={verticalListSortingStrategy}
          >
            <div style={{ height: totalHeight, position: "relative" }}>
              {virtualItems.map(
                (vi) =>
                  vi.visible && (
                    <StepRow
                      key={vi.key}
                      item={vi}
                      selectedStepId={selectedStepId}
                      selectedActionId={selectedActionId}
                      renamingStepId={renamingStepId}
                      onSelectStep={selectStep}
                      onSelectAction={selectAction}
                      onToggleExpanded={toggleExpanded}
                      onRenameStep={(step, name) => {
                        renameStep(step, name);
                        void recomputeHash();
                      }}
                      onDeleteStep={deleteStep}
                      onDeleteAction={deleteAction}
                      setRenamingStepId={setRenamingStepId}
                      registerMeasureRef={registerMeasureRef}
                    />
                  ),
              )}
            </div>
          </SortableContext>
        )}
      </div>
    </div>
  );
}

// Wrap in React.memo to prevent unnecessary re-renders causing flashing
export default React.memo(FlowWorkspace);

