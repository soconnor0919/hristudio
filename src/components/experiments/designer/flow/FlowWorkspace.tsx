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

interface VirtualItem {
  index: number;
  top: number;
  height: number;
  step: ExperimentStep;
  key: string;
  visible: boolean;
}

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
  action: ExperimentAction;
  isSelected: boolean;
  onSelect: () => void;
  onDelete: () => void;
  dragHandle?: boolean;
}

function SortableActionChip({
  action,
  isSelected,
  onSelect,
  onDelete,
}: ActionChipProps) {
  const def = actionRegistry.getAction(action.type);
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({
    id: sortableActionId(action.id),
  });

  const style: React.CSSProperties = {
    transform: CSS.Transform.toString(transform),
    transition,
    zIndex: isDragging ? 30 : undefined,
  };

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={cn(
        "group relative flex w-full flex-col items-start gap-1 rounded border px-3 py-2 text-[11px]",
        "bg-muted/40 hover:bg-accent/40 cursor-pointer",
        isSelected && "border-border bg-accent/30",
        isDragging && "opacity-70 shadow-lg",
      )}
      onClick={onSelect}
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
            onDelete();
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
            <span className="text-muted-foreground text-[9px]">
              +{def.parameters.length - 4} more
            </span>
          )}
        </div>
      ) : null}
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
  const reorderAction = useDesignerStore((s) => s.reorderAction);
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
        trigger: { type: "trial_start", conditions: {} },
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
      // Action reorder (within same parent only)
      if (activeId.startsWith("s-act-") && overId.startsWith("s-act-")) {
        const fromActionId = parseSortableAction(activeId);
        const toActionId = parseSortableAction(overId);
        if (fromActionId && toActionId && fromActionId !== toActionId) {
          const fromParent = actionParentMap.get(fromActionId);
          const toParent = actionParentMap.get(toActionId);
          if (fromParent && toParent && fromParent === toParent) {
            const step = steps.find((s) => s.id === fromParent);
            if (step) {
              const fromIdx = step.actions.findIndex(
                (a) => a.id === fromActionId,
              );
              const toIdx = step.actions.findIndex((a) => a.id === toActionId);
              if (fromIdx >= 0 && toIdx >= 0) {
                reorderAction(step.id, fromIdx, toIdx);
                void recomputeHash();
              }
            }
          }
        }
      }
    },
    [steps, reorderStep, reorderAction, actionParentMap, recomputeHash],
  );

  useDndMonitor({
    onDragStart: handleLocalDragStart,
    onDragEnd: handleLocalDragEnd,
    onDragCancel: () => {
      // no-op
    },
  });

  /* ------------------------------------------------------------------------ */
  /* Step Row (Sortable + Virtualized)                                         */
  /* ------------------------------------------------------------------------ */
  function StepRow({ item }: { item: VirtualItem }) {
    const step = item.step;
    const {
      setNodeRef,
      transform,
      transition,
      attributes,
      listeners,
      isDragging,
    } = useSortable({
      id: sortableStepId(step.id),
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

    const setMeasureRef = (el: HTMLDivElement | null) => {
      const prev = measureRefs.current.get(step.id) ?? null;
      if (prev && prev !== el) {
        roRef.current?.unobserve(prev);
        measureRefs.current.delete(step.id);
      }
      if (el) {
        measureRefs.current.set(step.id, el);
        roRef.current?.observe(el);
      }
    };

    return (
      <div ref={setNodeRef} style={style} data-step-id={step.id}>
        <div
          ref={setMeasureRef}
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
                // Avoid selecting step when interacting with controls or inputs
                const tag = (e.target as HTMLElement).tagName.toLowerCase();
                if (tag === "input" || tag === "textarea" || tag === "button")
                  return;
                selectStep(step.id);
                selectAction(step.id, undefined);
              }}
              role="button"
              tabIndex={0}
            >
              <div className="flex items-center gap-2">
                <button
                  type="button"
                  onClick={(e) => {
                    e.stopPropagation();
                    toggleExpanded(step);
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
                        renameStep(
                          step,
                          (e.target as HTMLInputElement).value.trim() ||
                            step.name,
                        );
                        setRenamingStepId(null);
                        void recomputeHash();
                      } else if (e.key === "Escape") {
                        setRenamingStepId(null);
                      }
                    }}
                    onBlur={(e) => {
                      renameStep(step, e.target.value.trim() || step.name);
                      setRenamingStepId(null);
                      void recomputeHash();
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
                    deleteStep(step);
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

            {step.expanded && (
              <div className="space-y-2 px-3 py-3">
                <div className="flex flex-wrap gap-2">
                  {step.actions.length > 0 && (
                    <SortableContext
                      items={step.actions.map((a) => sortableActionId(a.id))}
                      strategy={verticalListSortingStrategy}
                    >
                      <div className="flex w-full flex-col gap-2">
                        {step.actions.map((action) => (
                          <SortableActionChip
                            key={action.id}
                            action={action}
                            isSelected={
                              selectedStepId === step.id &&
                              selectedActionId === action.id
                            }
                            onSelect={() => {
                              selectStep(step.id);
                              selectAction(step.id, action.id);
                            }}
                            onDelete={() => deleteAction(step.id, action.id)}
                          />
                        ))}
                      </div>
                    </SortableContext>
                  )}
                </div>
                {/* Persistent centered bottom drop hint */}
                <div className="mt-3 flex w-full items-center justify-center">
                  <div className="text-muted-foreground border-muted-foreground/30 rounded border border-dashed px-2 py-1 text-[11px]">
                    Drop actions here
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    );
  }

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
                (vi) => vi.visible && <StepRow key={vi.key} item={vi} />,
              )}
            </div>
          </SortableContext>
        )}
      </div>
    </div>
  );
}

export default FlowWorkspace;
