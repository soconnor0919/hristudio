import React, { useCallback, useMemo } from "react";
import { useDesignerStore } from "../state/store";
import { StepFlow } from "../StepFlow";
import { useDroppable } from "@dnd-kit/core";
import type {
  ExperimentAction,
  ExperimentStep,
} from "~/lib/experiment-designer/types";

/**
 * Hidden droppable anchors so actions dragged from the ActionLibraryPanel
 * can land on steps even though StepFlow is still a legacy component.
 * This avoids having to deeply modify StepFlow during the transitional phase.
 */
function HiddenDroppableAnchors({ stepIds }: { stepIds: string[] }) {
  return (
    <>
      {stepIds.map((id) => (
        <SingleAnchor key={id} id={id} />
      ))}
    </>
  );
}

function SingleAnchor({ id }: { id: string }) {
  // Register a droppable area matching the StepFlow internal step id pattern
  useDroppable({
    id: `step-${id}`,
  });
  // Render nothing (zero-size element) – DnD kit only needs the registration
  return null;
}

/**
 * FlowListView (Transitional)
 *
 * This component is a TEMPORARY compatibility wrapper around the legacy
 * StepFlow component while the new virtualized / dual-mode (List vs Graph)
 * flow workspace is implemented.
 *
 * Responsibilities (current):
 *  - Read step + selection state from the designer store
 *  - Provide mutation handlers (upsert, delete, reorder placeholder)
 *  - Emit structured callbacks (reserved for future instrumentation)
 *
 * Planned Enhancements:
 *  - Virtualization for large step counts
 *  - Inline step creation affordances between steps
 *  - Multi-select + bulk operations
 *  - Drag reordering at step level (currently delegated to DnD kit)
 *  - Graph mode toggle (will lift state to higher DesignerRoot)
 *  - Performance memoization / fine-grained selectors
 *
 * Until the new system is complete, this wrapper allows incremental
 * replacement without breaking existing behavior.
 */

export interface FlowListViewProps {
  /**
   * Optional callbacks for higher-level orchestration (e.g. autosave triggers)
   */
  onStepMutated?: (
    step: ExperimentStep,
    kind: "create" | "update" | "delete",
  ) => void;
  onActionMutated?: (
    action: ExperimentAction,
    step: ExperimentStep,
    kind: "create" | "update" | "delete",
  ) => void;
  className?: string;
}

export function FlowListView({
  onStepMutated,
  onActionMutated,
  className,
}: FlowListViewProps) {
  /* ----------------------------- Store Selectors ---------------------------- */
  const steps = useDesignerStore((s) => s.steps);
  const selectedStepId = useDesignerStore((s) => s.selectedStepId);
  const selectedActionId = useDesignerStore((s) => s.selectedActionId);

  const selectStep = useDesignerStore((s) => s.selectStep);
  const selectAction = useDesignerStore((s) => s.selectAction);

  const upsertStep = useDesignerStore((s) => s.upsertStep);
  const removeStep = useDesignerStore((s) => s.removeStep);
  const upsertAction = useDesignerStore((s) => s.upsertAction);
  const removeAction = useDesignerStore((s) => s.removeAction);

  /* ------------------------------- Handlers --------------------------------- */

  const handleStepUpdate = useCallback(
    (stepId: string, updates: Partial<ExperimentStep>) => {
      const existing = steps.find((s) => s.id === stepId);
      if (!existing) return;
      const next: ExperimentStep = { ...existing, ...updates };
      upsertStep(next);
      onStepMutated?.(next, "update");
    },
    [steps, upsertStep, onStepMutated],
  );

  const handleStepDelete = useCallback(
    (stepId: string) => {
      const existing = steps.find((s) => s.id === stepId);
      if (!existing) return;
      removeStep(stepId);
      onStepMutated?.(existing, "delete");
    },
    [steps, removeStep, onStepMutated],
  );

  const handleActionDelete = useCallback(
    (stepId: string, actionId: string) => {
      const step = steps.find((s) => s.id === stepId);
      const action = step?.actions.find((a) => a.id === actionId);
      removeAction(stepId, actionId);
      if (step && action) {
        onActionMutated?.(action, step, "delete");
      }
    },
    [steps, removeAction, onActionMutated],
  );

  const totalActions = useMemo(
    () => steps.reduce((sum, s) => sum + s.actions.length, 0),
    [steps],
  );

  /* ------------------------------- Render ----------------------------------- */

  return (
    <div className={className} data-flow-mode="list">
      {/* NOTE: Header / toolbar will be hoisted into the main workspace toolbar in later iterations */}
      <div className="flex items-center justify-between border-b px-3 py-2 text-xs">
        <div className="flex items-center gap-3 font-medium">
          <span className="text-muted-foreground">Flow (List View)</span>
          <span className="text-muted-foreground/70">
            {steps.length} steps • {totalActions} actions
          </span>
        </div>
        <div className="text-muted-foreground/60 text-[10px]">
          Transitional component
        </div>
      </div>
      <div className="h-[calc(100%-2.5rem)]">
        {/* Hidden droppable anchors to enable dropping actions onto steps */}
        <HiddenDroppableAnchors stepIds={steps.map((s) => s.id)} />
        <StepFlow
          steps={steps}
          selectedStepId={selectedStepId ?? null}
          selectedActionId={selectedActionId ?? null}
          onStepSelect={(id) => selectStep(id)}
          onActionSelect={(actionId) =>
            selectedStepId && actionId
              ? selectAction(selectedStepId, actionId)
              : undefined
          }
          onStepDelete={handleStepDelete}
          onStepUpdate={handleStepUpdate}
          onActionDelete={handleActionDelete}
          emptyState={
            <div className="text-muted-foreground py-10 text-center text-sm">
              No steps yet. Use the + Step button to add your first step.
            </div>
          }
          headerRight={
            <div className="text-muted-foreground/70 text-[11px]">
              (Add Step control will move to global toolbar)
            </div>
          }
        />
      </div>
    </div>
  );
}

export default FlowListView;
