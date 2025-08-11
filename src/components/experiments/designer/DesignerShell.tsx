"use client";

/**
 * DesignerShell
 *
 * High-level orchestration component for the Experiment Designer redesign.
 * Replaces prior monolithic `BlockDesigner` responsibilities and delegates:
 *  - Data loading (experiment + study plugins)
 *  - Store initialization (steps, persisted/validated hashes)
 *  - Hash & drift status display
 *  - Save / validate / export actions (callback props)
 *  - Layout composition (Action Library | Step Flow | Properties Panel)
 *
 * This file intentionally does NOT contain:
 *  - Raw drag & drop logic (belongs to StepFlow & related internal modules)
 *  - Parameter field rendering logic (PropertiesPanel / ParameterFieldFactory)
 *  - Action registry loading internals (ActionRegistry singleton)
 *
 * Future Extensions:
 *  - Conflict modal
 *  - Bulk drift reconciliation
 *  - Command palette (action insertion)
 *  - Auto-save throttle controls
 */

import React, { useCallback, useEffect, useMemo, useState } from "react";
import { Play, Save, Download, RefreshCw } from "lucide-react";
import { DndContext, closestCenter } from "@dnd-kit/core";
import type { DragEndEvent, DragOverEvent } from "@dnd-kit/core";
import { toast } from "sonner";

import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { PageHeader, ActionButton } from "~/components/ui/page-header";

import { api } from "~/trpc/react";
import type {
  ExperimentDesign,
  ExperimentStep,
  ExperimentAction,
  ActionDefinition,
} from "~/lib/experiment-designer/types";

import { useDesignerStore } from "./state/store";
import { computeDesignHash } from "./state/hashing";

import { actionRegistry } from "./ActionRegistry";
import { ActionLibrary } from "./ActionLibrary";
import { StepFlow } from "./StepFlow";
import { PropertiesPanel } from "./PropertiesPanel";
import { ValidationPanel } from "./ValidationPanel";
import { DependencyInspector } from "./DependencyInspector";
import { validateExperimentDesign } from "./state/validators";

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

export interface DesignerShellProps {
  experimentId: string;
  initialDesign?: ExperimentDesign;
  /**
   * Called after a successful persisted save (server acknowledged).
   */
  onPersist?: (design: ExperimentDesign) => void;
  /**
   * Whether to auto-run compilation on save.
   */
  autoCompile?: boolean;
}

/* -------------------------------------------------------------------------- */
/* Utility                                                                    */
/* -------------------------------------------------------------------------- */

function buildEmptyDesign(
  experimentId: string,
  name?: string,
  description?: string | null,
): ExperimentDesign {
  return {
    id: experimentId,
    name: name?.trim().length ? name : "Untitled Experiment",
    description: description ?? "",
    version: 1,
    steps: [],
    lastSaved: new Date(),
  };
}

function adaptExistingDesign(experiment: {
  id: string;
  name: string;
  description: string | null;
  visualDesign: unknown;
}): ExperimentDesign | undefined {
  if (
    !experiment?.visualDesign ||
    typeof experiment.visualDesign !== "object" ||
    !("steps" in (experiment.visualDesign as Record<string, unknown>))
  ) {
    return undefined;
  }
  const vd = experiment.visualDesign as {
    steps?: ExperimentStep[];
    version?: number;
    lastSaved?: string;
  };
  if (!vd.steps || !Array.isArray(vd.steps)) return undefined;
  return {
    id: experiment.id,
    name: experiment.name,
    description: experiment.description ?? "",
    steps: vd.steps,
    version: vd.version ?? 1,
    lastSaved:
      vd.lastSaved && typeof vd.lastSaved === "string"
        ? new Date(vd.lastSaved)
        : new Date(),
  };
}

/* -------------------------------------------------------------------------- */
/* DesignerShell                                                              */
/* -------------------------------------------------------------------------- */

export function DesignerShell({
  experimentId,
  initialDesign,
  onPersist,
  autoCompile = true,
}: DesignerShellProps) {
  /* ---------------------------- Remote Experiment --------------------------- */
  const {
    data: experiment,
    isLoading: loadingExperiment,
    refetch: refetchExperiment,
  } = api.experiments.get.useQuery({ id: experimentId });

  /* ------------------------------ Store Access ------------------------------ */
  const steps = useDesignerStore((s) => s.steps);
  const setSteps = useDesignerStore((s) => s.setSteps);
  const recomputeHash = useDesignerStore((s) => s.recomputeHash);
  const currentDesignHash = useDesignerStore((s) => s.currentDesignHash);
  const lastPersistedHash = useDesignerStore((s) => s.lastPersistedHash);
  const lastValidatedHash = useDesignerStore((s) => s.lastValidatedHash);
  const setPersistedHash = useDesignerStore((s) => s.setPersistedHash);
  const setValidatedHash = useDesignerStore((s) => s.setValidatedHash);
  const selectedActionId = useDesignerStore((s) => s.selectedActionId);
  const selectedStepId = useDesignerStore((s) => s.selectedStepId);
  const selectStep = useDesignerStore((s) => s.selectStep);
  const selectAction = useDesignerStore((s) => s.selectAction);
  const validationIssues = useDesignerStore((s) => s.validationIssues);
  const actionSignatureDrift = useDesignerStore((s) => s.actionSignatureDrift);
  const upsertStep = useDesignerStore((s) => s.upsertStep);
  const removeStep = useDesignerStore((s) => s.removeStep);
  const upsertAction = useDesignerStore((s) => s.upsertAction);
  const removeAction = useDesignerStore((s) => s.removeAction);

  /* ------------------------------ Step Creation ------------------------------ */
  const createNewStep = useCallback(() => {
    const newStep: ExperimentStep = {
      id: `step-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      name: `Step ${steps.length + 1}`,
      description: "",
      type: "sequential",
      order: steps.length,
      trigger: {
        type: "trial_start",
        conditions: {},
      },
      actions: [],
      expanded: true,
    };
    upsertStep(newStep);
    selectStep(newStep.id);
    toast.success(`Created ${newStep.name}`);
  }, [steps.length, upsertStep, selectStep]);

  /* ------------------------------ DnD Handlers ------------------------------ */
  const handleDragEnd = useCallback(
    (event: DragEndEvent) => {
      const { active, over } = event;

      if (!over) return;

      // Handle action drag to step
      if (
        active.id.toString().startsWith("action-") &&
        over.id.toString().startsWith("step-")
      ) {
        const actionData = active.data.current?.action as ActionDefinition;
        const stepId = over.id.toString().replace("step-", "");

        if (!actionData) return;

        const step = steps.find((s) => s.id === stepId);
        if (!step) return;

        // Create new action instance
        const newAction: ExperimentAction = {
          id: `action-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
          type: actionData.type,
          name: actionData.name,
          category: actionData.category,
          parameters: {},
          source: actionData.source,
          execution: actionData.execution ?? {
            transport: "internal",
            retryable: false,
          },
        };

        upsertAction(stepId, newAction);
        selectStep(stepId);
        selectAction(stepId, newAction.id);
        toast.success(`Added ${actionData.name} to ${step.name}`);
      }
    },
    [steps, upsertAction, selectStep, selectAction],
  );

  const handleDragOver = useCallback((_event: DragOverEvent) => {
    // This could be used for visual feedback during drag
  }, []);

  /* ------------------------------- Local State ------------------------------ */
  const [designMeta, setDesignMeta] = useState<{
    name: string;
    description: string;
    version: number;
  }>(() => {
    const init =
      initialDesign ??
      (experiment ? adaptExistingDesign(experiment) : undefined) ??
      buildEmptyDesign(experimentId, experiment?.name, experiment?.description);
    return {
      name: init.name,
      description: init.description,
      version: init.version,
    };
  });

  const [isValidating, setIsValidating] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [isExporting, setIsExporting] = useState(false);
  const [initialized, setInitialized] = useState(false);

  /* ----------------------------- Experiment Update -------------------------- */
  const updateExperiment = api.experiments.update.useMutation({
    onSuccess: async () => {
      toast.success("Experiment saved");
      await refetchExperiment();
    },
    onError: (err) => {
      toast.error(`Save failed: ${err.message}`);
    },
  });

  /* ------------------------------ Plugin Loading ---------------------------- */
  const { data: studyPlugins } = api.robots.plugins.getStudyPlugins.useQuery(
    { studyId: experiment?.studyId ?? "" },
    { enabled: !!experiment?.studyId },
  );

  // Load core actions once
  useEffect(() => {
    actionRegistry
      .loadCoreActions()
      .catch((err) => console.error("Core action load failed:", err));
  }, []);

  // Load study plugin actions when available
  useEffect(() => {
    if (!experiment?.studyId) return;
    if (!studyPlugins || studyPlugins.length === 0) return;
    actionRegistry.loadPluginActions(
      experiment.studyId,
      studyPlugins.map((sp) => ({
        plugin: {
          id: sp.plugin.id,
          robotId: sp.plugin.robotId,
          version: sp.plugin.version,
          actionDefinitions: Array.isArray(sp.plugin.actionDefinitions)
            ? sp.plugin.actionDefinitions
            : undefined,
        },
      })),
    );
  }, [experiment?.studyId, studyPlugins]);

  /* ------------------------- Initialize Store Steps ------------------------- */
  useEffect(() => {
    if (initialized) return;
    if (loadingExperiment) return;
    const resolvedInitial =
      initialDesign ??
      (experiment ? adaptExistingDesign(experiment) : undefined) ??
      buildEmptyDesign(experimentId, experiment?.name, experiment?.description);
    setDesignMeta({
      name: resolvedInitial.name,
      description: resolvedInitial.description,
      version: resolvedInitial.version,
    });
    setSteps(resolvedInitial.steps);
    // Set persisted hash if experiment already has integrityHash
    if (experiment?.integrityHash) {
      setPersistedHash(experiment.integrityHash);
      setValidatedHash(experiment.integrityHash);
    }
    setInitialized(true);
    // Kick off first hash compute
    void recomputeHash();
  }, [
    initialized,
    loadingExperiment,
    experiment,
    initialDesign,
    experimentId,
    setSteps,
    setPersistedHash,
    setValidatedHash,
    recomputeHash,
  ]);

  /* ----------------------------- Drift Computation -------------------------- */
  const driftState = useMemo(() => {
    if (!lastValidatedHash || !currentDesignHash) {
      return {
        status: "unvalidated" as const,
        drift: false,
      };
    }
    if (currentDesignHash !== lastValidatedHash) {
      return { status: "drift" as const, drift: true };
    }
    return { status: "validated" as const, drift: false };
  }, [lastValidatedHash, currentDesignHash]);

  /* ------------------------------ Derived Flags ----------------------------- */
  const hasUnsavedChanges =
    !!currentDesignHash && lastPersistedHash !== currentDesignHash;

  const totalActions = steps.reduce((sum, s) => sum + s.actions.length, 0);

  /* ------------------------------- Validation ------------------------------- */
  const validateDesign = useCallback(async () => {
    if (!experimentId) return;
    setIsValidating(true);
    try {
      // Run local validation
      const validationResult = validateExperimentDesign(steps, {
        steps,
        actionDefinitions: actionRegistry.getAllActions(),
      });

      // Compute hash for integrity
      const hash = await computeDesignHash(steps);
      setValidatedHash(hash);

      if (validationResult.valid) {
        toast.success(`Validated • ${hash.slice(0, 10)}… • No issues found`);
      } else {
        toast.warning(
          `Validated with ${validationResult.errorCount} errors, ${validationResult.warningCount} warnings`,
        );
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
  }, [experimentId, steps, setValidatedHash]);

  /* ---------------------------------- Save ---------------------------------- */
  const persist = useCallback(async () => {
    if (!experimentId) return;
    setIsSaving(true);
    try {
      const visualDesign = {
        steps,
        version: designMeta.version,
        lastSaved: new Date().toISOString(),
      };
      updateExperiment.mutate({
        id: experimentId,
        visualDesign,
        createSteps: true,
        compileExecution: autoCompile,
      });
      // Optimistic hash recompute to reflect state
      await recomputeHash();
      onPersist?.({
        id: experimentId,
        name: designMeta.name,
        description: designMeta.description,
        version: designMeta.version,
        steps,
        lastSaved: new Date(),
      });
    } finally {
      setIsSaving(false);
    }
  }, [
    experimentId,
    steps,
    designMeta,
    recomputeHash,
    updateExperiment,
    onPersist,
    autoCompile,
  ]);

  /* -------------------------------- Export ---------------------------------- */
  const handleExport = useCallback(async () => {
    setIsExporting(true);
    try {
      const designHash = currentDesignHash ?? (await computeDesignHash(steps));
      const bundle = {
        format: "hristudio.design.v1",
        exportedAt: new Date().toISOString(),
        experiment: {
          id: experimentId,
          name: designMeta.name,
          version: designMeta.version,
          integrityHash: designHash,
          steps,
          pluginDependencies:
            experiment?.pluginDependencies?.slice().sort() ?? [],
        },
        compiled: null, // Will be implemented when execution graph is available
      };
      const blob = new Blob([JSON.stringify(bundle, null, 2)], {
        type: "application/json",
      });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `${designMeta.name
        .replace(/[^a-z0-9-_]+/gi, "_")
        .toLowerCase()}_design.json`;
      document.body.appendChild(a);
      a.click();
      a.remove();
      URL.revokeObjectURL(url);
      toast.success("Exported design bundle");
    } catch (err) {
      toast.error(
        `Export failed: ${
          err instanceof Error ? err.message : "Unknown error"
        }`,
      );
    } finally {
      setIsExporting(false);
    }
  }, [
    currentDesignHash,
    steps,
    experimentId,
    designMeta,
    experiment?.pluginDependencies,
  ]);

  /* ---------------------------- Incremental Hashing ------------------------- */
  // Optionally re-hash after step mutations (basic heuristic)
  useEffect(() => {
    if (!initialized) return;
    void recomputeHash();
  }, [steps.length, initialized, recomputeHash]);

  /* ------------------------------- Header Badges ---------------------------- */
  const hashBadge =
    driftState.status === "drift" ? (
      <Badge variant="destructive" title="Design drift detected">
        Drift
      </Badge>
    ) : driftState.status === "validated" ? (
      <Badge
        variant="outline"
        className="border-green-400 text-green-700 dark:text-green-400"
        title="Design validated"
      >
        Validated
      </Badge>
    ) : (
      <Badge variant="outline" title="Not validated">
        Unvalidated
      </Badge>
    );

  /* ------------------------------- Render ----------------------------------- */
  if (loadingExperiment && !initialized) {
    return (
      <div className="py-24 text-center">
        <p className="text-muted-foreground text-sm">
          Loading experiment design…
        </p>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      <PageHeader
        title={designMeta.name}
        description="Design your experiment by composing ordered steps with provenance-aware actions."
        icon={Play}
        actions={
          <div className="flex flex-wrap items-center gap-2">
            {hashBadge}
            {experiment?.integrityHash && (
              <Badge variant="outline" className="text-xs">
                Hash: {experiment.integrityHash.slice(0, 10)}…
              </Badge>
            )}
            <Badge variant="secondary" className="text-xs">
              {steps.length} steps
            </Badge>
            <Badge variant="secondary" className="text-xs">
              {totalActions} actions
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
              onClick={persist}
              disabled={!hasUnsavedChanges || isSaving}
            >
              <Save className="mr-2 h-4 w-4" />
              {isSaving ? "Saving…" : "Save"}
            </ActionButton>
            <ActionButton
              variant="outline"
              onClick={validateDesign}
              disabled={isValidating}
            >
              <RefreshCw className="mr-2 h-4 w-4" />
              {isValidating ? "Validating…" : "Validate"}
            </ActionButton>
            <ActionButton
              variant="outline"
              onClick={handleExport}
              disabled={isExporting}
            >
              <Download className="mr-2 h-4 w-4" />
              {isExporting ? "Exporting…" : "Export"}
            </ActionButton>
          </div>
        }
      />

      <DndContext
        collisionDetection={closestCenter}
        onDragEnd={handleDragEnd}
        onDragOver={handleDragOver}
      >
        <div className="grid grid-cols-12 gap-4">
          {/* Action Library */}
          <div className="col-span-3">
            <Card className="h-[calc(100vh-12rem)]">
              <CardHeader className="pb-2">
                <CardTitle className="flex items-center gap-2 text-sm">
                  Action Library
                </CardTitle>
              </CardHeader>
              <CardContent className="p-0">
                <ActionLibrary />
              </CardContent>
            </Card>
          </div>

          {/* Step Flow */}
          <div className="col-span-6">
            <StepFlow
              steps={steps}
              selectedStepId={selectedStepId ?? null}
              selectedActionId={selectedActionId ?? null}
              onStepSelect={(id: string) => selectStep(id)}
              onActionSelect={(id: string) =>
                selectedStepId && id
                  ? selectAction(selectedStepId, id)
                  : undefined
              }
              onStepDelete={(stepId: string) => {
                removeStep(stepId);
                toast.success("Step deleted");
              }}
              onStepUpdate={(
                stepId: string,
                updates: Partial<ExperimentStep>,
              ) => {
                const step = steps.find((s) => s.id === stepId);
                if (!step) return;
                upsertStep({ ...step, ...updates });
              }}
              onActionDelete={(stepId: string, actionId: string) => {
                removeAction(stepId, actionId);
                toast.success("Action deleted");
              }}
              emptyState={
                <div className="text-muted-foreground py-10 text-center text-sm">
                  Add your first step to begin designing.
                </div>
              }
              headerRight={
                <Button
                  size="sm"
                  className="h-6 text-xs"
                  onClick={createNewStep}
                >
                  + Step
                </Button>
              }
            />
          </div>

          {/* Properties Panel */}
          <div className="col-span-3">
            <Tabs defaultValue="properties" className="h-[calc(100vh-12rem)]">
              <Card className="h-full">
                <CardHeader className="pb-2">
                  <TabsList className="grid w-full grid-cols-3">
                    <TabsTrigger value="properties" className="text-xs">
                      Properties
                    </TabsTrigger>
                    <TabsTrigger value="validation" className="text-xs">
                      Issues
                    </TabsTrigger>
                    <TabsTrigger value="dependencies" className="text-xs">
                      Dependencies
                    </TabsTrigger>
                  </TabsList>
                </CardHeader>
                <CardContent className="p-0">
                  <TabsContent value="properties" className="m-0 h-full">
                    <ScrollArea className="h-full p-3">
                      <PropertiesPanel
                        design={{
                          id: experimentId,
                          name: designMeta.name,
                          description: designMeta.description,
                          version: designMeta.version,
                          steps,
                          lastSaved: new Date(),
                        }}
                        selectedStep={steps.find(
                          (s) => s.id === selectedStepId,
                        )}
                        selectedAction={
                          steps
                            .find(
                              (s: ExperimentStep) => s.id === selectedStepId,
                            )
                            ?.actions.find(
                              (a: ExperimentAction) =>
                                a.id === selectedActionId,
                            ) ?? undefined
                        }
                        onActionUpdate={(stepId, actionId, updates) => {
                          const step = steps.find((s) => s.id === stepId);
                          if (!step) return;
                          const action = step.actions.find(
                            (a) => a.id === actionId,
                          );
                          if (!action) return;
                          upsertAction(stepId, { ...action, ...updates });
                        }}
                        onStepUpdate={(stepId, updates) => {
                          const step = steps.find((s) => s.id === stepId);
                          if (!step) return;
                          upsertStep({ ...step, ...updates });
                        }}
                      />
                    </ScrollArea>
                  </TabsContent>

                  <TabsContent value="validation" className="m-0 h-full">
                    <ValidationPanel
                      issues={validationIssues}
                      onIssueClick={(issue) => {
                        if (issue.stepId) {
                          selectStep(issue.stepId);
                          if (issue.actionId) {
                            selectAction(issue.stepId, issue.actionId);
                          }
                        }
                      }}
                    />
                  </TabsContent>

                  <TabsContent value="dependencies" className="m-0 h-full">
                    <DependencyInspector
                      steps={steps}
                      actionSignatureDrift={actionSignatureDrift}
                      actionDefinitions={actionRegistry.getAllActions()}
                      onReconcileAction={(actionId) => {
                        // TODO: Implement drift reconciliation
                        toast.info(
                          `Reconciliation for action ${actionId} - TODO`,
                        );
                      }}
                      onRefreshDependencies={() => {
                        // TODO: Implement dependency refresh
                        toast.info("Dependency refresh - TODO");
                      }}
                      onInstallPlugin={(pluginId) => {
                        // TODO: Implement plugin installation
                        toast.info(`Install plugin ${pluginId} - TODO`);
                      }}
                    />
                  </TabsContent>
                </CardContent>
              </Card>
            </Tabs>
          </div>
        </div>
      </DndContext>
    </div>
  );
}

export default DesignerShell;
