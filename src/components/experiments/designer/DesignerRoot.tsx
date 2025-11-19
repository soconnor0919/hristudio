"use client";

import React, {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { toast } from "sonner";
import { Play, RefreshCw } from "lucide-react";

import { cn } from "~/lib/utils";
import { PageHeader } from "~/components/ui/page-header";

import { Button } from "~/components/ui/button";
import { api } from "~/trpc/react";

import { PanelsContainer } from "./layout/PanelsContainer";
import {
  DndContext,
  DragOverlay,
  pointerWithin,
  useSensor,
  useSensors,
  MouseSensor,
  TouchSensor,
  KeyboardSensor,
  type DragEndEvent,
  type DragStartEvent,
} from "@dnd-kit/core";
import { BottomStatusBar } from "./layout/BottomStatusBar";
import { ActionLibraryPanel } from "./panels/ActionLibraryPanel";
import { InspectorPanel } from "./panels/InspectorPanel";
import { FlowWorkspace } from "./flow/FlowWorkspace";

import {
  type ExperimentDesign,
  type ExperimentStep,
  type ExperimentAction,
} from "~/lib/experiment-designer/types";

import { useDesignerStore } from "./state/store";
import { actionRegistry } from "./ActionRegistry";
import { computeDesignHash } from "./state/hashing";
import {
  validateExperimentDesign,
  groupIssuesByEntity,
} from "./state/validators";

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
    onError: (err) => {
      toast.error(`Save failed: ${err.message}`);
    },
  });

  const { data: studyPluginsRaw } = api.robots.plugins.getStudyPlugins.useQuery(
    { studyId: experiment?.studyId ?? "" },
    { enabled: !!experiment?.studyId },
  );

  // Map studyPlugins to format expected by DependencyInspector
  const studyPlugins = useMemo(
    () =>
      studyPluginsRaw?.map((sp) => ({
        id: sp.plugin.id,
        robotId: sp.plugin.robotId ?? "",
        name: sp.plugin.name,
        version: sp.plugin.version,
      })),
    [studyPluginsRaw],
  );

  /* ------------------------------ Store Access ----------------------------- */
  const steps = useDesignerStore((s) => s.steps);
  const setSteps = useDesignerStore((s) => s.setSteps);
  const recomputeHash = useDesignerStore((s) => s.recomputeHash);
  const lastPersistedHash = useDesignerStore((s) => s.lastPersistedHash);
  const currentDesignHash = useDesignerStore((s) => s.currentDesignHash);

  const setPersistedHash = useDesignerStore((s) => s.setPersistedHash);
  const setValidatedHash = useDesignerStore((s) => s.setValidatedHash);
  const upsertStep = useDesignerStore((s) => s.upsertStep);
  const upsertAction = useDesignerStore((s) => s.upsertAction);
  const selectStep = useDesignerStore((s) => s.selectStep);
  const selectAction = useDesignerStore((s) => s.selectAction);
  const setValidationIssues = useDesignerStore((s) => s.setValidationIssues);
  const clearAllValidationIssues = useDesignerStore(
    (s) => s.clearAllValidationIssues,
  );
  const selectedStepId = useDesignerStore((s) => s.selectedStepId);
  const selectedActionId = useDesignerStore((s) => s.selectedActionId);

  const libraryRootRef = useRef<HTMLDivElement | null>(null);
  const toggleLibraryScrollLock = useCallback((lock: boolean) => {
    const viewport = libraryRootRef.current?.querySelector(
      '[data-slot="scroll-area-viewport"]',
    ) as HTMLElement | null;
    if (viewport) {
      if (lock) {
        viewport.style.overflowY = "hidden";
        viewport.style.overscrollBehavior = "contain";
      } else {
        viewport.style.overflowY = "";
        viewport.style.overscrollBehavior = "";
      }
    }
  }, []);

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
  const [isReady, setIsReady] = useState(false); // Track when everything is loaded

  const [lastSavedAt, setLastSavedAt] = useState<Date | undefined>(undefined);
  const [inspectorTab, setInspectorTab] = useState<
    "properties" | "issues" | "dependencies"
  >("properties");
  /**
   * Active action being dragged from the Action Library (for DragOverlay rendering).
   * Captures a lightweight subset for visual feedback.
   */
  const [dragOverlayAction, setDragOverlayAction] = useState<{
    id: string;
    name: string;
    category: string;
    description?: string;
  } | null>(null);

  /* ----------------------------- Initialization ---------------------------- */
  useEffect(() => {
    if (initialized) return;
    if (loadingExperiment && !initialDesign) return;

    console.log('[DesignerRoot] 🚀 INITIALIZING', {
      hasExperiment: !!experiment,
      hasInitialDesign: !!initialDesign,
      loadingExperiment,
    });

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
    // NOTE: We don't call recomputeHash() here because the automatic
    // hash recomputation useEffect will trigger when setSteps() updates the steps array
    console.log('[DesignerRoot] 🚀 Initialization complete, steps set');
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
    if (!studyPluginsRaw) return;
    // @ts-expect-error - studyPluginsRaw type from tRPC is compatible but TypeScript can't infer it
    actionRegistry.loadPluginActions(experiment.studyId, studyPluginsRaw);
  }, [experiment?.studyId, studyPluginsRaw]);

  /* ------------------------- Ready State Management ------------------------ */
  // Mark as ready once initialized and plugins are loaded
  useEffect(() => {
    if (!initialized || isReady) return;

    // Check if plugins are loaded by verifying the action registry has plugin actions
    const debugInfo = actionRegistry.getDebugInfo();
    const hasPlugins = debugInfo.pluginActionsLoaded;

    if (hasPlugins) {
      // Small delay to ensure all components have rendered
      const timer = setTimeout(() => {
        setIsReady(true);
        console.log('[DesignerRoot] ✅ Designer ready (plugins loaded), fading in');
      }, 150);
      return () => clearTimeout(timer);
    }
  }, [initialized, isReady, studyPluginsRaw]);

  /* ----------------------- Automatic Hash Recomputation -------------------- */
  // Automatically recompute hash when steps change (debounced to avoid excessive computation)
  useEffect(() => {
    if (!initialized) return;

    console.log('[DesignerRoot] Steps changed, scheduling hash recomputation', {
      stepsCount: steps.length,
      actionsCount: steps.reduce((sum, s) => sum + s.actions.length, 0),
    });

    const timeoutId = setTimeout(async () => {
      console.log('[DesignerRoot] Executing debounced hash recomputation');
      const result = await recomputeHash();
      if (result) {
        console.log('[DesignerRoot] Hash recomputed:', {
          newHash: result.designHash.slice(0, 16),
          fullHash: result.designHash,
        });
      }
    }, 300); // Debounce 300ms

    return () => clearTimeout(timeoutId);
  }, [steps, initialized, recomputeHash]);


  /* ----------------------------- Derived State ----------------------------- */
  const hasUnsavedChanges =
    !!currentDesignHash && lastPersistedHash !== currentDesignHash;

  // Debug logging to track hash updates and save button state
  useEffect(() => {
    console.log('[DesignerRoot] Hash State:', {
      currentDesignHash: currentDesignHash?.slice(0, 10),
      lastPersistedHash: lastPersistedHash?.slice(0, 10),
      hasUnsavedChanges,
      stepsCount: steps.length,
    });
  }, [currentDesignHash, lastPersistedHash, hasUnsavedChanges, steps.length]);

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
    selectStep(newStep.id);
    setInspectorTab("properties");
    toast.success(`Created ${newStep.name}`);
  }, [steps.length, upsertStep, selectStep]);

  /* ------------------------------- Validation ------------------------------ */
  const validateDesign = useCallback(async () => {
    if (!initialized) return;
    setIsValidating(true);
    try {
      const currentSteps = [...steps];
      // Ensure core actions are loaded before validating
      await actionRegistry.loadCoreActions();
      const result = validateExperimentDesign(currentSteps, {
        steps: currentSteps,
        actionDefinitions: actionRegistry.getAllActions(),
      });
      // Debug: log validation results for troubleshooting

      console.debug("[DesignerRoot] validation", {
        valid: result.valid,
        errors: result.errorCount,
        warnings: result.warningCount,
        infos: result.infoCount,
        issues: result.issues,
      });
      // Persist issues to store for inspector rendering
      const grouped = groupIssuesByEntity(result.issues);
      clearAllValidationIssues();
      for (const [entityId, arr] of Object.entries(grouped)) {
        setValidationIssues(
          entityId,
          arr.map((i) => ({
            entityId,
            severity: i.severity,
            message: i.message,
            code: undefined,
          })),
        );
      }
      const hash = await computeDesignHash(currentSteps);
      setValidatedHash(hash);
      if (result.valid) {
        toast.success(`Validated • ${hash.slice(0, 10)}… • 0 errors`);
      } else {
        toast.warning(
          `Validated with ${result.errorCount} errors, ${result.warningCount} warnings`,
        );
      }
    } catch (err) {
      toast.error(
        `Validation error: ${err instanceof Error ? err.message : "Unknown error"
        }`,
      );
    } finally {
      setIsValidating(false);
    }
  }, [
    initialized,
    steps,
    setValidatedHash,
    setValidationIssues,
    clearAllValidationIssues,
  ]);

  /* --------------------------------- Save ---------------------------------- */
  const persist = useCallback(async () => {
    if (!initialized) return;

    console.log('[DesignerRoot] 💾 SAVE initiated', {
      stepsCount: steps.length,
      actionsCount: steps.reduce((sum, s) => sum + s.actions.length, 0),
      currentHash: currentDesignHash?.slice(0, 16),
      lastPersistedHash: lastPersistedHash?.slice(0, 16),
    });

    setIsSaving(true);
    try {
      const visualDesign = {
        steps,
        version: designMeta.version,
        lastSaved: new Date().toISOString(),
      };

      console.log('[DesignerRoot] 💾 Sending to server...', {
        experimentId,
        stepsCount: steps.length,
        version: designMeta.version,
      });

      // Wait for mutation to complete
      await updateExperiment.mutateAsync({
        id: experimentId,
        visualDesign,
        createSteps: true,
        compileExecution: autoCompile,
      });

      console.log('[DesignerRoot] 💾 Server save successful');

      // NOTE: We do NOT refetch here because it would reset the local steps state
      // to the server state, which would cause the hash to match the persisted hash,
      // preventing the save button from re-enabling on subsequent changes.
      // The local state is already the source of truth after a successful save.

      // Recompute hash and update persisted hash
      const hashResult = await recomputeHash();
      if (hashResult?.designHash) {
        console.log('[DesignerRoot] 💾 Updated persisted hash:', {
          newPersistedHash: hashResult.designHash.slice(0, 16),
          fullHash: hashResult.designHash,
        });
        setPersistedHash(hashResult.designHash);
      }

      setLastSavedAt(new Date());
      toast.success("Experiment saved");

      console.log('[DesignerRoot] 💾 SAVE complete');

      onPersist?.({
        id: experimentId,
        name: designMeta.name,
        description: designMeta.description,
        steps,
        version: designMeta.version,
        lastSaved: new Date(),
      });
    } catch (error) {
      console.error('[DesignerRoot] 💾 SAVE failed:', error);
      // Error already handled by mutation onError
    } finally {
      setIsSaving(false);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    initialized,
    steps,
    designMeta,
    experimentId,
    recomputeHash,
    currentDesignHash,
    setPersistedHash,
    refetchExperiment,
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
        `Export failed: ${err instanceof Error ? err.message : "Unknown error"
        }`,
      );
    } finally {
      setIsExporting(false);
    }
  }, [currentDesignHash, steps, experimentId, designMeta, experiment]);

  /* ---------------------------- Incremental Hash --------------------------- */
  // Serialize steps for stable comparison
  const stepsHash = useMemo(() => JSON.stringify(steps), [steps]);

  useEffect(() => {
    if (!initialized) return;
    void recomputeHash();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [stepsHash, initialized]);

  useEffect(() => {
    if (selectedStepId || selectedActionId) {
      setInspectorTab("properties");
    }
  }, [selectedStepId, selectedActionId]);

  // Auto-open properties tab when a step or action becomes selected
  useEffect(() => {
    if (selectedStepId || selectedActionId) {
      setInspectorTab("properties");
    }
  }, [selectedStepId, selectedActionId]);

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

  const sensors = useSensors(
    useSensor(MouseSensor, { activationConstraint: { distance: 6 } }),
    useSensor(TouchSensor, {
      activationConstraint: { delay: 150, tolerance: 5 },
    }),
    useSensor(KeyboardSensor),
  );

  /* ----------------------------- Drag Handlers ----------------------------- */
  const handleDragStart = useCallback(
    (event: DragStartEvent) => {
      const { active } = event;
      if (
        active.id.toString().startsWith("action-") &&
        active.data.current?.action
      ) {
        const a = active.data.current.action as {
          id: string;
          name: string;
          category: string;
          description?: string;
        };
        toggleLibraryScrollLock(true);
        setDragOverlayAction({
          id: a.id,
          // prefer definition name; fallback to id
          name: a.name || a.id,
          category: a.category,
          description: a.description,
        });
      }
    },
    [toggleLibraryScrollLock],
  );

  const handleDragEnd = useCallback(
    async (event: DragEndEvent) => {
      const { active, over } = event;
      console.debug("[DesignerRoot] dragEnd", {
        active: active?.id,
        over: over?.id ?? null,
      });
      // Clear overlay immediately
      toggleLibraryScrollLock(false);
      setDragOverlayAction(null);
      if (!over) {
        console.debug("[DesignerRoot] dragEnd: no drop target (ignored)");
        return;
      }

      // Expect dragged action (library) onto a step droppable
      const activeId = active.id.toString();
      const overId = over.id.toString();

      if (activeId.startsWith("action-") && active.data.current?.action) {
        // Resolve stepId from possible over ids: step-<id>, s-step-<id>, or s-act-<actionId>
        let stepId: string | null = null;
        if (overId.startsWith("step-")) {
          stepId = overId.slice("step-".length);
        } else if (overId.startsWith("s-step-")) {
          stepId = overId.slice("s-step-".length);
        } else if (overId.startsWith("s-act-")) {
          const actionId = overId.slice("s-act-".length);
          const parent = steps.find((s) =>
            s.actions.some((a) => a.id === actionId),
          );
          stepId = parent?.id ?? null;
        }
        if (!stepId) return;

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
        // Select the newly added action and open properties
        selectStep(stepId);
        selectAction(stepId, newAction.id);
        setInspectorTab("properties");
        await recomputeHash();
        toast.success(`Added ${actionDef.name} to ${targetStep.name}`);
      }
    },
    [
      steps,
      upsertAction,
      recomputeHash,
      selectStep,
      selectAction,
      toggleLibraryScrollLock,
    ],
  );
  // validation status badges removed (unused)

  /* ------------------------------- Render ---------------------------------- */
  if (loadingExperiment && !initialized) {
    return (
      <div className="text-muted-foreground py-24 text-center text-sm">
        Loading experiment design…
      </div>
    );
  }

  const actions = (
    <div className="flex flex-wrap items-center gap-2">
      <Button
        size="sm"
        variant="default"
        className="h-8 px-3 text-xs"
        onClick={() => validateDesign()}
        disabled={isValidating}
      >
        Validate
      </Button>
      <Button
        size="sm"
        variant="secondary"
        className="h-8 px-3 text-xs"
        onClick={() => persist()}
        disabled={!hasUnsavedChanges || isSaving}
      >
        Save
      </Button>
    </div>
  );

  return (
    <div className="flex h-full w-full flex-col overflow-hidden">
      <PageHeader
        title={designMeta.name}
        description={designMeta.description || "No description"}
        icon={Play}
        actions={actions}
      />

      <div className="relative flex flex-1 flex-col overflow-hidden">
        {/* Loading Overlay */}
        {!isReady && (
          <div className="absolute inset-0 z-50 flex items-center justify-center bg-background">
            <div className="flex flex-col items-center gap-4">
              <RefreshCw className="h-8 w-8 animate-spin text-primary" />
              <p className="text-muted-foreground text-sm">Loading designer...</p>
            </div>
          </div>
        )}

        {/* Main Content - Fade in when ready */}
        <div
          className={cn(
            "flex flex-1 flex-col overflow-hidden transition-opacity duration-500",
            isReady ? "opacity-100" : "opacity-0"
          )}
        >
          <div className="flex h-[calc(100vh-12rem)] w-full max-w-full flex-col overflow-hidden rounded-md border">
            <DndContext
              sensors={sensors}
              collisionDetection={pointerWithin}
              onDragStart={handleDragStart}
              onDragEnd={handleDragEnd}
              onDragCancel={() => toggleLibraryScrollLock(false)}
            >
              <PanelsContainer
                showDividers
                className="min-h-0 flex-1"
                left={
                  <div ref={libraryRootRef} data-library-root className="h-full">
                    <ActionLibraryPanel />
                  </div>
                }
                center={<FlowWorkspace />}
                right={
                  <div className="h-full">
                    <InspectorPanel
                      activeTab={inspectorTab}
                      onTabChange={setInspectorTab}
                      studyPlugins={studyPlugins}
                    />
                  </div>
                }
              />
              <DragOverlay>
                {dragOverlayAction ? (
                  <div className="bg-background pointer-events-none rounded border px-2 py-1 text-xs shadow-lg select-none">
                    {dragOverlayAction.name}
                  </div>
                ) : null}
              </DragOverlay>
            </DndContext>
            <div className="flex-shrink-0 border-t">
              <BottomStatusBar
                onSave={() => persist()}
                onValidate={() => validateDesign()}
                onExport={() => handleExport()}
                onRecalculateHash={() => recomputeHash()}
                lastSavedAt={lastSavedAt}
                saving={isSaving}
                validating={isValidating}
                exporting={isExporting}
              />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default DesignerRoot;
