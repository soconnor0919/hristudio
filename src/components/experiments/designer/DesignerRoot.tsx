"use client";

import React, { useCallback, useEffect, useMemo, useState } from "react";
import { toast } from "sonner";
import { Play, Plus } from "lucide-react";

import { PageHeader, ActionButton } from "~/components/ui/page-header";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { api } from "~/trpc/react";

import { PanelsContainer } from "./layout/PanelsContainer";
import { DndContext, closestCenter, type DragEndEvent } from "@dnd-kit/core";
import { BottomStatusBar } from "./layout/BottomStatusBar";
import { ActionLibraryPanel } from "./panels/ActionLibraryPanel";
import { InspectorPanel } from "./panels/InspectorPanel";
import { FlowListView } from "./flow/FlowListView";

import {
  type ExperimentDesign,
  type ExperimentStep,
  type ExperimentAction,
} from "~/lib/experiment-designer/types";

import { useDesignerStore } from "./state/store";
import { actionRegistry } from "./ActionRegistry";
import { computeDesignHash } from "./state/hashing";
import { validateExperimentDesign } from "./state/validators";

/**
 * DesignerRoot
 *
 * New high-level orchestrator for the Experiment Designer refactor.
 * Replaces the previous monolithic DesignerShell with a composable,
 * panel-based layout (left: action library, center: flow workspace,
 * right: contextual inspector, bottom: status bar).
 *
 * Responsibilities:
 *  - Remote experiment fetch + initial design hydration
 *  - Store initialization (steps, persisted / validated hash)
 *  - Save / validate / export orchestration
 *  - Keyboard shortcut wiring
 *  - Action / plugin registry initialization
 *
 * Non-Responsibilities:
 *  - Raw per-field editing (delegated to sub-panels)
 *  - Drag/drop internals (delegated to flow + library components)
 *  - Low-level hashing/incremental logic (state/store)
 *
 * Future Enhancements (planned hooks / modules):
 *  - Command Palette (action insertion & navigation)
 *  - Virtualized step list
 *  - Graph view toggle
 *  - Drift diff / reconciliation modal
 *  - Autosave throttling controls
 */

export interface DesignerRootProps {
  experimentId: string;
  initialDesign?: ExperimentDesign;
  autoCompile?: boolean;
  onPersist?: (design: ExperimentDesign) => void;
}

interface RawExperiment {
  id: string;
  name: string;
  description: string | null;
  studyId: string;
  integrityHash?: string | null;
  pluginDependencies?: string[] | null;
  visualDesign?: unknown;
}

/* -------------------------------------------------------------------------- */
/* Util: Adapt Existing Stored Visual Design                                  */
/* -------------------------------------------------------------------------- */

function adaptExistingDesign(exp: RawExperiment): ExperimentDesign | undefined {
  if (
    !exp.visualDesign ||
    typeof exp.visualDesign !== "object" ||
    !("steps" in (exp.visualDesign as Record<string, unknown>))
  ) {
    return undefined;
  }
  const vd = exp.visualDesign as {
    steps?: ExperimentStep[];
    version?: number;
    lastSaved?: string;
  };
  if (!Array.isArray(vd.steps)) return undefined;
  return {
    id: exp.id,
    name: exp.name,
    description: exp.description ?? "",
    steps: vd.steps,
    version: vd.version ?? 1,
    lastSaved:
      vd.lastSaved && typeof vd.lastSaved === "string"
        ? new Date(vd.lastSaved)
        : new Date(),
  };
}

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

/* -------------------------------------------------------------------------- */
/* Component                                                                  */
/* -------------------------------------------------------------------------- */

export function DesignerRoot({
  experimentId,
  initialDesign,
  autoCompile = true,
  onPersist,
}: DesignerRootProps) {
  /* ----------------------------- Remote Experiment ------------------------- */
  const {
    data: experiment,
    isLoading: loadingExperiment,
    refetch: refetchExperiment,
  } = api.experiments.get.useQuery({ id: experimentId });

  const updateExperiment = api.experiments.update.useMutation({
    onSuccess: async () => {
      toast.success("Experiment saved");
      await refetchExperiment();
    },
    onError: (err) => {
      toast.error(`Save failed: ${err.message}`);
    },
  });

  const { data: studyPlugins } = api.robots.plugins.getStudyPlugins.useQuery(
    { studyId: experiment?.studyId ?? "" },
    { enabled: !!experiment?.studyId },
  );

  /* ------------------------------ Store Access ----------------------------- */
  const steps = useDesignerStore((s) => s.steps);
  const setSteps = useDesignerStore((s) => s.setSteps);
  const recomputeHash = useDesignerStore((s) => s.recomputeHash);
  const lastPersistedHash = useDesignerStore((s) => s.lastPersistedHash);
  const currentDesignHash = useDesignerStore((s) => s.currentDesignHash);
  const lastValidatedHash = useDesignerStore((s) => s.lastValidatedHash);
  const setPersistedHash = useDesignerStore((s) => s.setPersistedHash);
  const setValidatedHash = useDesignerStore((s) => s.setValidatedHash);
  const upsertStep = useDesignerStore((s) => s.upsertStep);
  const upsertAction = useDesignerStore((s) => s.upsertAction);

  /* ------------------------------- Local Meta ------------------------------ */
  const [designMeta, setDesignMeta] = useState<{
    name: string;
    description: string;
    version: number;
  }>(() => {
    // Determine initial local meta (prefer server design)
    const existing =
      initialDesign ??
      (experiment
        ? adaptExistingDesign(experiment as RawExperiment)
        : undefined);
    const base =
      existing ??
      buildEmptyDesign(
        experimentId,
        experiment?.name,
        experiment?.description ?? "",
      );
    return {
      name: base.name,
      description: base.description,
      version: base.version,
    };
  });

  const [initialized, setInitialized] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [isValidating, setIsValidating] = useState(false);
  const [isExporting, setIsExporting] = useState(false);
  const [lastSavedAt, setLastSavedAt] = useState<Date | undefined>(undefined);

  /* ----------------------------- Initialization ---------------------------- */
  useEffect(() => {
    if (initialized || loadingExperiment) return;
    const adapted =
      initialDesign ??
      (experiment
        ? adaptExistingDesign(experiment as RawExperiment)
        : undefined);
    const resolved =
      adapted ??
      buildEmptyDesign(
        experimentId,
        experiment?.name,
        experiment?.description ?? "",
      );
    setDesignMeta({
      name: resolved.name,
      description: resolved.description,
      version: resolved.version,
    });
    setSteps(resolved.steps);
    if ((experiment as RawExperiment | undefined)?.integrityHash) {
      const ih = (experiment as RawExperiment).integrityHash!;
      setPersistedHash(ih);
      setValidatedHash(ih);
    }
    setInitialized(true);
    // Kick initial hash
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

  /* ---------------------------- Action Registry ---------------------------- */
  // Load core actions once
  useEffect(() => {
    actionRegistry
      .loadCoreActions()
      .catch((err) => console.error("Core action load failed:", err));
  }, []);

  // Load plugin actions when study plugins available
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

  /* ----------------------------- Derived State ----------------------------- */
  const hasUnsavedChanges =
    !!currentDesignHash && lastPersistedHash !== currentDesignHash;

  const driftStatus = useMemo<"unvalidated" | "drift" | "validated">(() => {
    if (!currentDesignHash || !lastValidatedHash) return "unvalidated";
    if (currentDesignHash !== lastValidatedHash) return "drift";
    return "validated";
  }, [currentDesignHash, lastValidatedHash]);

  /* ------------------------------- Step Ops -------------------------------- */
  const createNewStep = useCallback(() => {
    const newStep: ExperimentStep = {
      id: `step-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`,
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
    toast.success(`Created ${newStep.name}`);
  }, [steps.length, upsertStep]);

  /* ------------------------------- Validation ------------------------------ */
  const validateDesign = useCallback(async () => {
    if (!initialized) return;
    setIsValidating(true);
    try {
      const currentSteps = [...steps];
      const result = validateExperimentDesign(currentSteps, {
        steps: currentSteps,
        actionDefinitions: actionRegistry.getAllActions(),
      });
      const hash = await computeDesignHash(currentSteps);
      setValidatedHash(hash);
      if (result.valid) {
        toast.success(`Validated • ${hash.slice(0, 10)}… • No issues`);
      } else {
        toast.warning(
          `Validated with ${result.errorCount} errors, ${result.warningCount} warnings`,
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
  }, [initialized, steps, setValidatedHash]);

  /* --------------------------------- Save ---------------------------------- */
  const persist = useCallback(async () => {
    if (!initialized) return;
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
      // Optimistic hash recompute
      await recomputeHash();
      setLastSavedAt(new Date());
      onPersist?.({
        id: experimentId,
        name: designMeta.name,
        description: designMeta.description,
        steps,
        version: designMeta.version,
        lastSaved: new Date(),
      });
    } finally {
      setIsSaving(false);
    }
  }, [
    initialized,
    steps,
    designMeta,
    experimentId,
    updateExperiment,
    recomputeHash,
    onPersist,
    autoCompile,
  ]);

  /* -------------------------------- Export --------------------------------- */
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
            (experiment as RawExperiment | undefined)?.pluginDependencies
              ?.slice()
              .sort() ?? [],
        },
        compiled: null,
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
  }, [currentDesignHash, steps, experimentId, designMeta, experiment]);

  /* ---------------------------- Incremental Hash --------------------------- */
  useEffect(() => {
    if (!initialized) return;
    void recomputeHash();
  }, [steps.length, initialized, recomputeHash]);

  /* -------------------------- Keyboard Shortcuts --------------------------- */
  const keyHandler = useCallback(
    (e: globalThis.KeyboardEvent) => {
      if (
        (e.metaKey || e.ctrlKey) &&
        e.key.toLowerCase() === "s" &&
        hasUnsavedChanges
      ) {
        e.preventDefault();
        void persist();
      } else if (e.key === "v" && !e.metaKey && !e.ctrlKey) {
        e.preventDefault();
        void validateDesign();
      } else if (e.key === "e" && !e.metaKey && !e.ctrlKey) {
        e.preventDefault();
        void handleExport();
      } else if (e.key === "n" && e.shiftKey) {
        e.preventDefault();
        createNewStep();
      }
    },
    [hasUnsavedChanges, persist, validateDesign, handleExport, createNewStep],
  );

  useEffect(() => {
    const listener = (ev: globalThis.KeyboardEvent) => keyHandler(ev);
    window.addEventListener("keydown", listener, { passive: true });
    return () => {
      window.removeEventListener("keydown", listener);
    };
  }, [keyHandler]);

  /* ------------------------------ Header Badges ---------------------------- */

  /* ----------------------------- Drag Handlers ----------------------------- */
  const handleDragEnd = useCallback(
    (event: DragEndEvent) => {
      const { active, over } = event;
      if (!over) return;

      // Expect dragged action (library) onto a step droppable
      const activeId = active.id.toString();
      const overId = over.id.toString();

      if (
        activeId.startsWith("action-") &&
        overId.startsWith("step-") &&
        active.data.current?.action
      ) {
        const actionDef = active.data.current.action as {
          id: string;
          type: string;
          name: string;
          category: string;
          description?: string;
          source: { kind: string; pluginId?: string; pluginVersion?: string };
          execution?: { transport: string; retryable?: boolean };
          parameters: Array<{ id: string; name: string }>;
        };

        const stepId = overId.replace("step-", "");
        const targetStep = steps.find((s) => s.id === stepId);
        if (!targetStep) return;

        const execution: ExperimentAction["execution"] =
          actionDef.execution &&
          (actionDef.execution.transport === "internal" ||
            actionDef.execution.transport === "rest" ||
            actionDef.execution.transport === "ros2")
            ? {
                transport: actionDef.execution.transport,
                retryable: actionDef.execution.retryable ?? false,
              }
            : {
                transport: "internal",
                retryable: false,
              };
        const newAction: ExperimentAction = {
          id: `action-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
          type: actionDef.type,
          name: actionDef.name,
          category: actionDef.category as ExperimentAction["category"],
          parameters: {},
          source: actionDef.source as ExperimentAction["source"],
          execution,
        };

        upsertAction(stepId, newAction);
        toast.success(`Added ${actionDef.name} to ${targetStep.name}`);
      }
    },
    [steps, upsertAction],
  );
  const validationBadge =
    driftStatus === "drift" ? (
      <Badge variant="destructive">Drift</Badge>
    ) : driftStatus === "validated" ? (
      <Badge
        variant="outline"
        className="border-green-400 text-green-700 dark:text-green-400"
      >
        Validated
      </Badge>
    ) : (
      <Badge variant="outline">Unvalidated</Badge>
    );

  /* ------------------------------- Render ---------------------------------- */
  if (loadingExperiment && !initialized) {
    return (
      <div className="text-muted-foreground py-24 text-center text-sm">
        Loading experiment design…
      </div>
    );
  }

  return (
    <div className="flex h-[calc(100vh-6rem)] flex-col gap-3">
      <PageHeader
        title={designMeta.name}
        description="Compose ordered steps with provenance-aware actions."
        icon={Play}
        actions={
          <div className="flex flex-wrap items-center gap-2">
            {validationBadge}
            {experiment?.integrityHash && (
              <Badge variant="outline" className="text-xs">
                Hash: {experiment.integrityHash.slice(0, 10)}…
              </Badge>
            )}
            <Badge variant="secondary" className="text-xs">
              {steps.length} steps
            </Badge>
            <Badge variant="secondary" className="text-xs">
              {steps.reduce((s, st) => s + st.actions.length, 0)} actions
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
              onClick={() => persist()}
              disabled={!hasUnsavedChanges || isSaving}
            >
              {isSaving ? "Saving…" : "Save"}
            </ActionButton>
            <ActionButton
              variant="outline"
              onClick={() => validateDesign()}
              disabled={isValidating}
            >
              {isValidating ? "Validating…" : "Validate"}
            </ActionButton>
            <ActionButton
              variant="outline"
              onClick={() => handleExport()}
              disabled={isExporting}
            >
              {isExporting ? "Exporting…" : "Export"}
            </ActionButton>
            <Button
              size="sm"
              variant="default"
              className="h-8 text-xs"
              onClick={createNewStep}
            >
              <Plus className="mr-1 h-4 w-4" />
              Step
            </Button>
          </div>
        }
      />

      <div className="flex min-h-0 flex-1 flex-col overflow-hidden rounded-md border">
        <DndContext
          collisionDetection={closestCenter}
          onDragEnd={handleDragEnd}
        >
          <PanelsContainer
            left={<ActionLibraryPanel />}
            center={<FlowListView />}
            right={<InspectorPanel />}
            initialLeftWidth={260}
            initialRightWidth={360}
            className="flex-1"
          />
        </DndContext>
        <BottomStatusBar
          onSave={() => persist()}
          onValidate={() => validateDesign()}
          onExport={() => handleExport()}
          lastSavedAt={lastSavedAt}
          saving={isSaving}
          validating={isValidating}
          exporting={isExporting}
        />
      </div>
    </div>
  );
}

export default DesignerRoot;
