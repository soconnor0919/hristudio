"use client";

/**
 * @deprecated
 * BlockDesigner is being phased out in favor of DesignerShell (see DesignerShell.tsx).
 * TODO: Remove this file after full migration of add/update/delete handlers, hashing,
 * validation, drift detection, and export logic to the new architecture.
 */

/**
 * BlockDesigner (Modular Refactor)
 *
 * Responsibilities:
 * - Own overall experiment design state (steps + actions)
 * - Coordinate drag & drop between ActionLibrary (source) and StepFlow (targets)
 * - Persist design via experiments.update mutation (optionally compiling execution graph)
 * - Trigger server-side validation (experiments.validateDesign) to obtain integrity hash
 * - Track & surface "hash drift" (design changed since last validation or mismatch with stored integrityHash)
 *
 * Extracted Modules:
 * - ActionRegistry      -> ./ActionRegistry.ts
 * - ActionLibrary       -> ./ActionLibrary.tsx
 * - StepFlow            -> ./StepFlow.tsx
 * - PropertiesPanel     -> ./PropertiesPanel.tsx
 *
 * Enhancements Added Here:
 * - Hash drift indicator logic (Validated / Drift / Unvalidated)
 * - Modular wiring replacing previous monolithic file
 */

import React, { useState, useCallback, useEffect, useMemo } from "react";
import {
  DndContext,
  closestCenter,
  PointerSensor,
  useSensor,
  useSensors,
  type DragEndEvent,
  type DragStartEvent,
} from "@dnd-kit/core";
import { arrayMove } from "@dnd-kit/sortable";

import { toast } from "sonner";
import { Save, Download, Play, Plus } from "lucide-react";

import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { ScrollArea } from "~/components/ui/scroll-area";
import { PageHeader, ActionButton } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";

import {
  type ExperimentDesign,
  type ExperimentStep,
  type ExperimentAction,
  type ActionDefinition,
} from "~/lib/experiment-designer/types";

import { api } from "~/trpc/react";
import { ActionLibrary } from "./ActionLibrary";
import { StepFlow } from "./StepFlow";
import { PropertiesPanel } from "./PropertiesPanel";
import { actionRegistry } from "./ActionRegistry";

/* -------------------------------------------------------------------------- */
/* Utilities                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * Build a lightweight JSON string representing the current design for drift checks.
 * We include full steps & actions; param value churn will intentionally flag drift
 * (acceptable trade-off for now; can switch to structural signature if too noisy).
 */
function serializeDesignSteps(steps: ExperimentStep[]): string {
  return JSON.stringify(
    steps.map((s) => ({
      id: s.id,
      order: s.order,
      type: s.type,
      trigger: {
        type: s.trigger.type,
        conditionKeys: Object.keys(s.trigger.conditions).sort(),
      },
      actions: s.actions.map((a) => ({
        id: a.id,
        type: a.type,
        sourceKind: a.source.kind,
        pluginId: a.source.pluginId,
        pluginVersion: a.source.pluginVersion,
        transport: a.execution.transport,
        parameterKeys: Object.keys(a.parameters).sort(),
      })),
    })),
  );
}

/* -------------------------------------------------------------------------- */
/* Props                                                                      */
/* -------------------------------------------------------------------------- */

interface BlockDesignerProps {
  experimentId: string;
  initialDesign?: ExperimentDesign;
  onSave?: (design: ExperimentDesign) => void;
}

/* -------------------------------------------------------------------------- */
/* Component                                                                  */
/* -------------------------------------------------------------------------- */

export function BlockDesigner({
  experimentId,
  initialDesign,
  onSave,
}: BlockDesignerProps) {
  /* ---------------------------- Experiment Query ---------------------------- */
  const { data: experiment } = api.experiments.get.useQuery({
    id: experimentId,
  });

  /* ------------------------------ Local Design ------------------------------ */
  const [design, setDesign] = useState<ExperimentDesign>(() => {
    const defaultDesign: ExperimentDesign = {
      id: experimentId,
      name: "New Experiment",
      description: "",
      steps: [],
      version: 1,
      lastSaved: new Date(),
    };
    return initialDesign ?? defaultDesign;
  });

  const [selectedStepId, setSelectedStepId] = useState<string | null>(null);
  const [selectedActionId, setSelectedActionId] = useState<string | null>(null);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);

  /* ------------------------- Validation / Drift Tracking -------------------- */
  const [isValidating, setIsValidating] = useState(false);
  const [lastValidatedHash, setLastValidatedHash] = useState<string | null>(
    null,
  );
  const [lastValidatedDesignJson, setLastValidatedDesignJson] = useState<
    string | null
  >(null);

  // Recompute drift conditions
  const currentDesignJson = useMemo(
    () => serializeDesignSteps(design.steps),
    [design.steps],
  );

  const hasIntegrityHash = !!experiment?.integrityHash;
  const hashMismatch =
    hasIntegrityHash &&
    lastValidatedHash &&
    experiment?.integrityHash !== lastValidatedHash;
  const designChangedSinceValidation =
    !!lastValidatedDesignJson && lastValidatedDesignJson !== currentDesignJson;

  const drift =
    hasIntegrityHash && (hashMismatch ? true : designChangedSinceValidation);

  /* ---------------------------- Active Drag State --------------------------- */
  // Removed unused activeId state (drag overlay removed in modular refactor)

  /* ------------------------------- tRPC Mutations --------------------------- */
  const updateExperiment = api.experiments.update.useMutation({
    onSuccess: () => {
      toast.success("Experiment saved");
      setHasUnsavedChanges(false);
    },
    onError: (err) => {
      toast.error(`Failed to save: ${err.message}`);
    },
  });
  const trpcUtils = api.useUtils();

  /* ------------------------------- Plugins Load ----------------------------- */
  const { data: studyPlugins } = api.robots.plugins.getStudyPlugins.useQuery(
    { studyId: experiment?.studyId ?? "" },
    { enabled: !!experiment?.studyId },
  );

  /* ---------------------------- Registry Loading ---------------------------- */
  useEffect(() => {
    actionRegistry.loadCoreActions().catch((err) => {
      console.error("Core actions load failed:", err);
      toast.error("Failed to load core action library");
    });
  }, []);

  useEffect(() => {
    if (experiment?.studyId && (studyPlugins?.length ?? 0) > 0) {
      actionRegistry.loadPluginActions(
        experiment.studyId,
        (studyPlugins ?? []).map((sp) => ({
          plugin: {
            id: sp.plugin.id,
            robotId: sp.plugin.robotId,
            version: sp.plugin.version,
            actionDefinitions: Array.isArray(sp.plugin.actionDefinitions)
              ? sp.plugin.actionDefinitions
              : undefined,
          },
        })) ?? [],
      );
    }
  }, [experiment?.studyId, studyPlugins]);

  /* ------------------------------ Breadcrumbs ------------------------------- */
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    {
      label: experiment?.study?.name ?? "Study",
      href: `/studies/${experiment?.studyId}`,
    },
    { label: "Experiments", href: `/studies/${experiment?.studyId}` },
    { label: design.name, href: `/experiments/${experimentId}` },
    { label: "Designer" },
  ]);

  /* ------------------------------ DnD Sensors ------------------------------- */
  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: { distance: 5 },
    }),
  );

  const handleDragStart = useCallback((_event: DragStartEvent) => {
    // activeId tracking removed (drag overlay no longer used)
  }, []);

  /* ------------------------------ Helpers ----------------------------------- */

  const addActionToStep = useCallback(
    (stepId: string, def: ActionDefinition) => {
      const newAction: ExperimentAction = {
        id: `action_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`,
        type: def.type,
        name: def.name,
        parameters: {},
        category: def.category,
        source: def.source,
        execution: def.execution ?? { transport: "internal" },
        parameterSchemaRaw: def.parameterSchemaRaw,
      };
      // Default param values
      def.parameters.forEach((p) => {
        if (p.value !== undefined) {
          newAction.parameters[p.id] = p.value;
        }
      });
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((s) =>
          s.id === stepId ? { ...s, actions: [...s.actions, newAction] } : s,
        ),
      }));
      setHasUnsavedChanges(true);
      toast.success(`Added ${def.name}`);
    },
    [],
  );

  const handleDragEnd = useCallback(
    (event: DragEndEvent) => {
      const { active, over } = event;
      // activeId reset removed (no longer tracked)
      if (!over) return;

      const activeIdStr = active.id.toString();
      const overIdStr = over.id.toString();

      // From library to step droppable
      if (activeIdStr.startsWith("action-") && overIdStr.startsWith("step-")) {
        const actionId = activeIdStr.replace("action-", "");
        const stepId = overIdStr.replace("step-", "");
        const def = actionRegistry.getAction(actionId);
        if (def) {
          addActionToStep(stepId, def);
        }
        return;
      }

      // Step reorder (both plain ids of steps)
      if (
        !activeIdStr.startsWith("action-") &&
        !overIdStr.startsWith("step-") &&
        !overIdStr.startsWith("action-")
      ) {
        const oldIndex = design.steps.findIndex((s) => s.id === activeIdStr);
        const newIndex = design.steps.findIndex((s) => s.id === overIdStr);
        if (oldIndex !== -1 && newIndex !== -1 && oldIndex !== newIndex) {
          setDesign((prev) => ({
            ...prev,
            steps: arrayMove(prev.steps, oldIndex, newIndex).map(
              (s, index) => ({ ...s, order: index }),
            ),
          }));
          setHasUnsavedChanges(true);
        }
        return;
      }

      // Action reorder (within same step)
      if (
        !activeIdStr.startsWith("action-") &&
        !overIdStr.startsWith("step-") &&
        activeIdStr !== overIdStr
      ) {
        // Identify which step these actions belong to
        const containingStep = design.steps.find((s) =>
          s.actions.some((a) => a.id === activeIdStr),
        );
        const targetStep = design.steps.find((s) =>
          s.actions.some((a) => a.id === overIdStr),
        );
        if (
          containingStep &&
          targetStep &&
          containingStep.id === targetStep.id
        ) {
          const oldActionIndex = containingStep.actions.findIndex(
            (a) => a.id === activeIdStr,
          );
          const newActionIndex = containingStep.actions.findIndex(
            (a) => a.id === overIdStr,
          );
          if (
            oldActionIndex !== -1 &&
            newActionIndex !== -1 &&
            oldActionIndex !== newActionIndex
          ) {
            setDesign((prev) => ({
              ...prev,
              steps: prev.steps.map((s) =>
                s.id === containingStep.id
                  ? {
                      ...s,
                      actions: arrayMove(
                        s.actions,
                        oldActionIndex,
                        newActionIndex,
                      ),
                    }
                  : s,
              ),
            }));
            setHasUnsavedChanges(true);
          }
        }
      }
    },
    [design.steps, addActionToStep],
  );

  const addStep = useCallback(() => {
    const newStep: ExperimentStep = {
      id: `step_${Date.now()}_${Math.random().toString(36).slice(2, 6)}`,
      name: `Step ${design.steps.length + 1}`,
      description: "",
      type: "sequential",
      order: design.steps.length,
      trigger: {
        type: design.steps.length === 0 ? "trial_start" : "previous_step",
        conditions: {},
      },
      actions: [],
      expanded: true,
    };
    setDesign((prev) => ({
      ...prev,
      steps: [...prev.steps, newStep],
    }));
    setHasUnsavedChanges(true);
  }, [design.steps.length]);

  const updateStep = useCallback(
    (stepId: string, updates: Partial<ExperimentStep>) => {
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((s) =>
          s.id === stepId ? { ...s, ...updates } : s,
        ),
      }));
      setHasUnsavedChanges(true);
    },
    [],
  );

  const deleteStep = useCallback(
    (stepId: string) => {
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.filter((s) => s.id !== stepId),
      }));
      if (selectedStepId === stepId) setSelectedStepId(null);
      setHasUnsavedChanges(true);
    },
    [selectedStepId],
  );

  const updateAction = useCallback(
    (stepId: string, actionId: string, updates: Partial<ExperimentAction>) => {
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((s) =>
          s.id === stepId
            ? {
                ...s,
                actions: s.actions.map((a) =>
                  a.id === actionId ? { ...a, ...updates } : a,
                ),
              }
            : s,
        ),
      }));
      setHasUnsavedChanges(true);
    },
    [],
  );

  const deleteAction = useCallback(
    (stepId: string, actionId: string) => {
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((s) =>
          s.id === stepId
            ? {
                ...s,
                actions: s.actions.filter((a) => a.id !== actionId),
              }
            : s,
        ),
      }));
      if (selectedActionId === actionId) setSelectedActionId(null);
      setHasUnsavedChanges(true);
    },
    [selectedActionId],
  );

  /* ------------------------------- Validation ------------------------------- */
  const runValidation = useCallback(async () => {
    setIsValidating(true);
    try {
      const result = await trpcUtils.experiments.validateDesign.fetch({
        experimentId,
        visualDesign: { steps: design.steps },
      });

      if (!result.valid) {
        toast.error(
          `Validation failed: ${result.issues.slice(0, 3).join(", ")}${
            result.issues.length > 3 ? "…" : ""
          }`,
        );
        return;
      }

      if (result.integrityHash) {
        setLastValidatedHash(result.integrityHash);
        setLastValidatedDesignJson(currentDesignJson);
        toast.success(
          `Validated • Hash: ${result.integrityHash.slice(0, 10)}…`,
        );
      } else {
        toast.success("Validated (no hash produced)");
      }
    } catch (err) {
      toast.error(
        `Validation error: ${
          err instanceof Error ? err.message : "Unknown error"
        }`,
      );
    } finally {
      setIsValidating(false);
    }
  }, [experimentId, design.steps, trpcUtils, currentDesignJson]);

  /* --------------------------------- Saving --------------------------------- */
  const saveDesign = useCallback(() => {
    const visualDesign = {
      steps: design.steps,
      version: design.version,
      lastSaved: new Date().toISOString(),
    };
    updateExperiment.mutate({
      id: experimentId,
      visualDesign,
      createSteps: true,
      compileExecution: true,
    });
    const updatedDesign = { ...design, lastSaved: new Date() };
    setDesign(updatedDesign);
    onSave?.(updatedDesign);
  }, [design, experimentId, onSave, updateExperiment]);

  /* --------------------------- Selection Resolution ------------------------- */
  const selectedStep = design.steps.find((s) => s.id === selectedStepId);
  const selectedAction = selectedStep?.actions.find(
    (a) => a.id === selectedActionId,
  );

  /* ------------------------------- Header Badges ---------------------------- */
  const validationBadge = drift ? (
    <Badge
      variant="destructive"
      className="text-xs"
      title="Design has drifted since last validation or differs from stored hash"
    >
      Drift
    </Badge>
  ) : lastValidatedHash ? (
    <Badge
      variant="outline"
      className="border-green-400 text-xs text-green-700 dark:text-green-400"
      title="Design matches last validated structure"
    >
      Validated
    </Badge>
  ) : (
    <Badge variant="outline" className="text-xs" title="Not yet validated">
      Unvalidated
    </Badge>
  );

  /* ---------------------------------- Render -------------------------------- */
  return (
    <DndContext
      sensors={sensors}
      collisionDetection={closestCenter}
      onDragStart={handleDragStart}
      onDragEnd={handleDragEnd}
    >
      <div className="space-y-4">
        <PageHeader
          title={design.name}
          description="Design your experiment using steps and categorized actions"
          icon={Play}
          actions={
            <div className="flex flex-wrap items-center gap-2">
              {validationBadge}
              {experiment?.integrityHash && (
                <Badge variant="outline" className="text-xs">
                  Hash: {experiment.integrityHash.slice(0, 10)}…
                </Badge>
              )}
              {experiment?.executionGraphSummary && (
                <Badge variant="outline" className="text-xs">
                  Exec: {experiment.executionGraphSummary.steps ?? 0}s /
                  {experiment.executionGraphSummary.actions ?? 0}a
                </Badge>
              )}
              {Array.isArray(experiment?.pluginDependencies) &&
                experiment.pluginDependencies.length > 0 && (
                  <Badge variant="secondary" className="text-xs">
                    {experiment.pluginDependencies.length} plugins
                  </Badge>
                )}
              <Badge variant="secondary" className="text-xs">
                {design.steps.length} steps
              </Badge>
              {hasUnsavedChanges && (
                <Badge
                  variant="outline"
                  className="border-orange-300 text-orange-600"
                >
                  Unsaved
                </Badge>
              )}
              <ActionButton
                onClick={saveDesign}
                disabled={!hasUnsavedChanges || updateExperiment.isPending}
              >
                <Save className="mr-2 h-4 w-4" />
                {updateExperiment.isPending ? "Saving…" : "Save"}
              </ActionButton>
              <ActionButton
                variant="outline"
                onClick={() => {
                  setHasUnsavedChanges(false); // immediate feedback
                  void runValidation();
                }}
                disabled={isValidating}
              >
                <Play className="mr-2 h-4 w-4" />
                {isValidating ? "Validating…" : "Revalidate"}
              </ActionButton>
              <ActionButton variant="outline">
                <Download className="mr-2 h-4 w-4" />
                Export
              </ActionButton>
            </div>
          }
        />

        <div className="grid grid-cols-12 gap-4">
          {/* Action Library */}
          <div className="col-span-3">
            <Card className="h-[calc(100vh-12rem)]">
              <CardHeader className="pb-2">
                <CardTitle className="flex items-center gap-2 text-sm">
                  <Plus className="h-4 w-4" />
                  Action Library
                </CardTitle>
              </CardHeader>
              <CardContent className="p-0">
                <ActionLibrary />
              </CardContent>
            </Card>
          </div>

          {/* Flow */}
          <div className="col-span-6">
            <StepFlow
              steps={design.steps}
              selectedStepId={selectedStepId}
              selectedActionId={selectedActionId}
              onStepSelect={(id) => {
                setSelectedStepId(id);
                setSelectedActionId(null);
              }}
              onStepDelete={deleteStep}
              onStepUpdate={updateStep}
              onActionSelect={(actionId) => setSelectedActionId(actionId)}
              onActionDelete={deleteAction}
              emptyState={
                <div className="py-8 text-center">
                  <Play className="text-muted-foreground/50 mx-auto h-8 w-8" />
                  <h3 className="mt-2 text-sm font-medium">No steps yet</h3>
                  <p className="text-muted-foreground mt-1 text-xs">
                    Add your first step to begin designing
                  </p>
                  <Button className="mt-2" size="sm" onClick={addStep}>
                    <Plus className="mr-1 h-3 w-3" />
                    Add First Step
                  </Button>
                </div>
              }
              headerRight={
                <Button size="sm" onClick={addStep} className="h-6 text-xs">
                  <Plus className="mr-1 h-3 w-3" />
                  Add Step
                </Button>
              }
            />
          </div>

          {/* Properties */}
          <div className="col-span-3">
            <Card className="h-[calc(100vh-12rem)]">
              <CardHeader className="pb-2">
                <CardTitle className="flex items-center gap-2 text-sm">
                  Properties
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <ScrollArea className="h-full pr-1">
                  <PropertiesPanel
                    design={design}
                    selectedStep={selectedStep}
                    selectedAction={selectedAction}
                    onActionUpdate={updateAction}
                    onStepUpdate={updateStep}
                  />
                </ScrollArea>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </DndContext>
  );
}
