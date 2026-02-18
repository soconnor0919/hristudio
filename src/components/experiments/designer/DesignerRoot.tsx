"use client";

import React, {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { toast } from "sonner";
import {
  Play,
  RefreshCw,
  HelpCircle,
  PanelLeftClose,
  PanelLeftOpen,
  PanelRightClose,
  PanelRightOpen,
  Maximize2,
  Minimize2,
  Settings
} from "lucide-react";

import { cn } from "~/lib/utils";
import { PageHeader } from "~/components/ui/page-header";
import { useTour } from "~/components/onboarding/TourProvider";
import { SettingsModal } from "./SettingsModal";

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
  closestCenter,
  type DragEndEvent,
  type DragStartEvent,
  type DragOverEvent,
} from "@dnd-kit/core";
import { BottomStatusBar } from "./layout/BottomStatusBar";
import { ActionLibraryPanel } from "./panels/ActionLibraryPanel";
import { InspectorPanel } from "./panels/InspectorPanel";
import { FlowWorkspace, StepCardPreview } from "./flow/FlowWorkspace";
import { SortableActionChip } from "./flow/ActionChip";
import { GripVertical } from "lucide-react";

import {
  type ExperimentDesign,
  type ExperimentStep,
  type ExperimentAction,
} from "~/lib/experiment-designer/types";

import { useDesignerStore } from "./state/store";
import { actionRegistry, useActionRegistry } from "./ActionRegistry";
import { computeDesignHash } from "./state/hashing";
import {
  validateExperimentDesign,
  groupIssuesByEntity,
} from "./state/validators";
import { convertDatabaseToSteps } from "~/lib/experiment-designer/block-converter";

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
  experiment?: {
    id: string;
    name: string;
    description: string | null;
    status: string;
    studyId: string;
    createdAt: Date;
    updatedAt: Date;
    study: {
      id: string;
      name: string;
    };
  };
  designStats?: {
    stepCount: number;
    actionCount: number;
  };
}

interface RawExperiment {
  id: string;
  name: string;
  description: string | null;
  studyId: string;
  integrityHash?: string | null;
  pluginDependencies?: string[] | null;
  visualDesign?: unknown;
  steps?: unknown[]; // DB steps from relation
}

/* -------------------------------------------------------------------------- */
/* Util: Adapt Existing Stored Visual Design                                  */
/* -------------------------------------------------------------------------- */

function adaptExistingDesign(exp: RawExperiment): ExperimentDesign | undefined {
  console.log('[adaptExistingDesign] Entry - exp.steps:', exp.steps);

  // 1. Prefer database steps (Source of Truth) if valid, to ensure we have the latest
  //    plugin provenance data (which might be missing from stale visualDesign snapshots).
  // 1. Prefer database steps (Source of Truth) if valid.
  if (Array.isArray(exp.steps) && exp.steps.length > 0) {
    console.log('[adaptExistingDesign] Has steps array, length:', exp.steps.length);
    try {
      // Check if steps are already converted (have trigger property) to avoid double-conversion data loss
      const firstStep = exp.steps[0] as any;
      let dbSteps: ExperimentStep[];

      if (firstStep && typeof firstStep === 'object' && 'trigger' in firstStep) {
        // Already converted by server
        dbSteps = exp.steps as ExperimentStep[];
      } else {
        // Raw DB steps, need conversion
        console.log('[adaptExistingDesign] Taking raw DB conversion path');
        dbSteps = convertDatabaseToSteps(exp.steps);

        // DEBUG: Check children after conversion
        dbSteps.forEach((step) => {
          step.actions.forEach((action) => {
            if (["sequence", "parallel", "loop", "branch"].includes(action.type)) {
              console.log(`[adaptExistingDesign] Post-conversion ${action.type} (${action.name}) children:`, action.children);
            }
          });
        });
      }

      return {
        id: exp.id,
        name: exp.name,
        description: exp.description ?? "",
        steps: dbSteps,
        version: 1, // Reset version on re-hydration
        lastSaved: new Date(),
      };
    } catch (err) {
      console.warn('[DesignerRoot] Failed to convert/hydrate steps, falling back to visualDesign:', err);
    }
  }

  // 2. Fallback to visualDesign blob if DB steps unavailable or conversion failed
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
  experiment: experimentMetadata,
  designStats,
}: DesignerRootProps) {
  // Subscribe to registry updates to ensure re-renders when actions load
  useActionRegistry();

  const { startTour } = useTour();

  /* ----------------------------- Remote Experiment ------------------------- */
  const {
    data: experiment,
    isLoading: loadingExperiment,
    refetch: refetchExperiment,
  } = api.experiments.get.useQuery(
    { id: experimentId },
    {
      // Debug Mode: Disable all caching to ensure fresh data from DB
      refetchOnMount: true,
      refetchOnWindowFocus: true,
      staleTime: 0,
      gcTime: 0, // Garbage collect immediately
    }
  );

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
        actionDefinitions: sp.plugin.actionDefinitions as any[],
        metadata: sp.plugin.metadata as Record<string, any>,
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
  const reorderStep = useDesignerStore((s) => s.reorderStep);
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

  const [leftCollapsed, setLeftCollapsed] = useState(false);
  const [rightCollapsed, setRightCollapsed] = useState(false);
  const [settingsOpen, setSettingsOpen] = useState(false);

  // Responsive initialization: Collapse left sidebar on smaller screens (<1280px)
  useEffect(() => {
    const checkWidth = () => {
      if (window.innerWidth < 1280) {
        setLeftCollapsed(true);
      }
    };
    // Check once on mount
    checkWidth();
    // Optional: Add resize listener if we want live responsiveness
    // window.addEventListener('resize', checkWidth);
    // return () => window.removeEventListener('resize', checkWidth);
  }, []);
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

  const [activeSortableItem, setActiveSortableItem] = useState<{
    type: 'step' | 'action';
    data: any;
  } | null>(null);

  /* ----------------------------- Initialization ---------------------------- */
  useEffect(() => {
    console.log('[DesignerRoot] useEffect triggered', { initialized, loadingExperiment, hasExperiment: !!experiment, hasInitialDesign: !!initialDesign });

    if (initialized) return;
    if (loadingExperiment && !initialDesign) return;

    console.log('[DesignerRoot] Proceeding with initialization');

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
    // console.log('[DesignerRoot] 🚀 Initialization complete, steps set');
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

  // Load plugin actions only after we have the flattened, processed plugin list
  useEffect(() => {
    if (!experiment?.studyId) return;
    if (!studyPlugins) return;

    // Pass the flattened plugins which match the structure ActionRegistry expects
    actionRegistry.loadPluginActions(experiment.studyId, studyPlugins);
  }, [experiment?.studyId, studyPlugins]);

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
      }, 150);
      return () => clearTimeout(timer);
    }
  }, [initialized, isReady, studyPlugins]);

  /* ----------------------- Automatic Hash Recomputation -------------------- */
  // Automatically recompute hash when steps change (debounced to avoid excessive computation)
  useEffect(() => {
    if (!initialized) return;

    // console.log('[DesignerRoot] Steps changed, scheduling hash recomputation');

    const timeoutId = setTimeout(async () => {
      // console.log('[DesignerRoot] Executing debounced hash recomputation');
      const result = await recomputeHash();
      if (result) {
        // console.log('[DesignerRoot] Hash recomputed:', result.designHash.slice(0, 16));
      }
    }, 300); // Debounce 300ms

    return () => clearTimeout(timeoutId);
  }, [steps, initialized, recomputeHash]);


  /* ----------------------------- Derived State ----------------------------- */
  const hasUnsavedChanges =
    !!currentDesignHash && lastPersistedHash !== currentDesignHash;

  // Debug logging to track hash updates and save button state
  useEffect(() => {
    // console.log('[DesignerRoot] Hash State:', {
    //   currentDesignHash: currentDesignHash?.slice(0, 10),
    //   lastPersistedHash: lastPersistedHash?.slice(0, 10),
    //   hasUnsavedChanges,
    // });
  }, [currentDesignHash, lastPersistedHash, hasUnsavedChanges]);

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

      // Debug: Improved structured logging for validation results
      console.group("🧪 Experiment Validation Results");
      if (result.valid) {
        console.log(`%c✓ VALID (0 errors, ${result.warningCount} warnings, ${result.infoCount} hints)`, "color: green; font-weight: bold; font-size: 12px;");
      } else {
        console.log(`%c✗ INVALID (${result.errorCount} errors, ${result.warningCount} warnings)`, "color: red; font-weight: bold; font-size: 12px;");
      }

      if (result.issues.length > 0) {
        console.table(
          result.issues.map(i => ({
            Severity: i.severity.toUpperCase(),
            Category: i.category,
            Message: i.message,
            Suggest: i.suggestion,
            Location: i.actionId ? `Action ${i.actionId}` : (i.stepId ? `Step ${i.stepId}` : 'Global')
          }))
        );
      } else {
        console.log("No issues found. Design is perfectly compliant.");
      }
      console.groupEnd();
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

  // Trigger initial validation when ready (plugins loaded) to ensure no stale errors
  // Trigger initial validation when ready (plugins loaded) to ensure no stale errors
  // DISABLED: User prefers manual validation to avoid noise on improved sequential architecture
  // useEffect(() => {
  //   if (isReady) {
  //     void validateDesign();
  //   }
  // }, [isReady, validateDesign]);

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

      // Auto-validate after save to clear "Modified" (drift) status
      void validateDesign();

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

  // Intentionally removed redundant recomputeHash useEffect that was causing excessive refreshes
  // The debounced useEffect (lines 352-372) handles this correctly.

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
      }
      // 'v' (validate), 'e' (export), 'Shift+N' (new step) shortcuts removed to prevent accidents
    },
    [hasUnsavedChanges, persist],
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
  /* ----------------------------- Drag Handlers ----------------------------- */
  const handleDragStart = useCallback(
    (event: DragStartEvent) => {
      const { active } = event;
      const activeId = active.id.toString();
      const activeData = active.data.current;

      console.log("[DesignerRoot] DragStart", { activeId, activeData });

      if (
        activeId.startsWith("action-") &&
        activeData?.action
      ) {
        const a = activeData.action as {
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
      } else if (activeId.startsWith("s-step-")) {
        console.log("[DesignerRoot] Setting active sortable STEP", activeData);
        setActiveSortableItem({
          type: 'step',
          data: activeData
        });
      } else if (activeId.startsWith("s-act-")) {
        console.log("[DesignerRoot] Setting active sortable ACTION", activeData);
        setActiveSortableItem({
          type: 'action',
          data: activeData
        });
      }
    },
    [toggleLibraryScrollLock],
  );

  const handleDragOver = useCallback((event: DragOverEvent) => {
    const { active, over } = event;
    const store = useDesignerStore.getState();
    const activeId = active.id.toString();

    if (!over) {
      if (store.insertionProjection) {
        store.setInsertionProjection(null);
      }
      return;
    }

    // 3. Library -> Flow Projection (Action)
    if (!activeId.startsWith("action-")) {
      if (store.insertionProjection) {
        store.setInsertionProjection(null);
      }
      return;
    }



    const overId = over.id.toString();
    const activeDef = active.data.current?.action;

    if (!activeDef) return;

    let stepId: string | null = null;
    let parentId: string | null = null;
    let index = 0;

    // Detect target based on over id
    if (overId.startsWith("s-act-")) {
      const data = over.data.current;
      if (data && data.stepId) {
        stepId = data.stepId;
        parentId = data.parentId ?? null; // Use parentId from the action we are hovering over
        // Use sortable index (insertion point provided by dnd-kit sortable strategy)
        index = data.sortable?.index ?? 0;
      }
    } else if (overId.startsWith("container-")) {
      // Dropping into a container (e.g. Loop)
      const data = over.data.current;
      if (data && data.stepId) {
        stepId = data.stepId;
        parentId = data.parentId ?? overId.slice("container-".length);
        // If dropping into container, appending is a safe default if specific index logic is missing
        // But actually we can find length if we want. For now, 0 or append logic?
        // If container is empty, index 0 is correct.
        // If not empty, we are hitting the container *background*, so append?
        // The projection logic will insert at 'index'. If index is past length, it appends.
        // Let's set a large index to ensure append, or look up length.
        // Lookup requires finding the action in store. Expensive?
        // Let's assume index 0 for now (prepend) or implement lookup.
        // Better: lookup action -> children length.
        const actionId = parentId;
        const step = store.steps.find(s => s.id === stepId);
        // Find action recursive? Store has `findActionById` helper but it is not exported/accessible easily here?
        // Actually, `store.steps` is available.
        // We can implement a quick BFS/DFS or just assume 0. 
        // If dragging over the container *background* (empty space), append is usually expected.
        // Let's try 9999?
        index = 9999;
      }
    } else if (overId.startsWith("s-step-") || overId.startsWith("step-")) {
      // Container drop (Step)
      stepId = overId.startsWith("s-step-")
        ? overId.slice("s-step-".length)
        : overId.slice("step-".length);
      const step = store.steps.find((s) => s.id === stepId);
      index = step ? step.actions.length : 0;

    } else if (overId === "projection-placeholder") {
      // Hovering over our own projection placeholder -> keep current state
      return;
    }

    if (stepId) {
      const current = store.insertionProjection;
      // Optimization: avoid redundant updates if projection matches
      if (
        current &&
        current.stepId === stepId &&
        current.parentId === parentId &&
        current.index === index
      ) {
        return;
      }

      store.setInsertionProjection({
        stepId,
        parentId,
        index,
        action: {
          id: "projection-placeholder",
          type: activeDef.type,
          name: activeDef.name,
          category: activeDef.category,
          description: "Drop here",
          source: activeDef.source || { kind: "library" },
          parameters: {},
          execution: activeDef.execution,
        } as any,
      });
    } else {
      if (store.insertionProjection) store.setInsertionProjection(null);
    }
  }, []);

  const handleDragEnd = useCallback(
    async (event: DragEndEvent) => {
      const { active, over } = event;

      // Clear overlay immediately
      toggleLibraryScrollLock(false);
      setDragOverlayAction(null);
      setActiveSortableItem(null);

      // Capture and clear projection
      const store = useDesignerStore.getState();
      const projection = store.insertionProjection;
      store.setInsertionProjection(null);

      if (!over) {
        return;
      }

      const activeId = active.id.toString();

      // Handle Step Reordering (Active is a sortable step)
      if (activeId.startsWith("s-step-")) {
        const overId = over.id.toString();
        // Allow reordering over both sortable steps (s-step-) and drop zones (step-)
        if (!overId.startsWith("s-step-") && !overId.startsWith("step-")) return;

        // Strip prefixes to get raw IDs
        const rawActiveId = activeId.replace(/^s-step-/, "");
        const rawOverId = overId.replace(/^s-step-/, "").replace(/^step-/, "");

        console.log("[DesignerRoot] DragEnd - Step Sort", { activeId, overId, rawActiveId, rawOverId });

        const oldIndex = steps.findIndex((s) => s.id === rawActiveId);
        const newIndex = steps.findIndex((s) => s.id === rawOverId);

        console.log("[DesignerRoot] Indices", { oldIndex, newIndex });

        if (oldIndex !== -1 && newIndex !== -1 && oldIndex !== newIndex) {
          console.log("[DesignerRoot] Reordering...");
          reorderStep(oldIndex, newIndex);
        }
        return;
      }

      // 1. Determine Target (Step, Parent, Index)
      let stepId: string | null = null;
      let parentId: string | null = null;
      let index: number | undefined = undefined;

      if (projection) {
        stepId = projection.stepId;
        parentId = projection.parentId;
        index = projection.index;
      } else {
        // Fallback: resolution from overId (if projection failed or raced)
        const overId = over.id.toString();
        if (overId.startsWith("step-")) {
          stepId = overId.slice("step-".length);
        } else if (overId.startsWith("s-step-")) {
          stepId = overId.slice("s-step-".length);
        } else if (overId.startsWith("s-act-")) {
          // This might fail if s-act-projection, but that should have covered by projection check above
          const actionId = overId.slice("s-act-".length);
          const parent = steps.find((s) =>
            s.actions.some((a) => a.id === actionId),
          );
          stepId = parent?.id ?? null;
        }
      }

      if (!stepId) return;
      const targetStep = steps.find((s) => s.id === stepId);
      if (!targetStep) return;

      // 2. Instantiate Action
      if (active.id.toString().startsWith("action-") && active.data.current?.action) {
        const actionDef = active.data.current.action as {
          id: string; // type
          type: string;
          name: string;
          category: string;
          description?: string;
          source: { kind: string; pluginId?: string; pluginVersion?: string };
          execution?: { transport: string; retryable?: boolean };
          parameters: Array<{ id: string; name: string }>;
        };

        const fullDef = actionRegistry.getAction(actionDef.type);
        const defaultParams: Record<string, unknown> = {};
        if (fullDef?.parameters) {
          for (const param of fullDef.parameters) {
            if (param.value !== undefined) {
              defaultParams[param.id] = param.value;
            }
          }
        }

        const execution: ExperimentAction["execution"] =
          actionDef.execution &&
            (actionDef.execution.transport === "internal" ||
              actionDef.execution.transport === "rest" ||
              actionDef.execution.transport === "ros2")
            ? {
              transport: actionDef.execution.transport,
              retryable: actionDef.execution.retryable ?? false,
            }
            : undefined;

        const newId = `action-${Date.now().toString(36)}-${Math.random().toString(36).substr(2, 9)}`;
        const newAction: ExperimentAction = {
          id: newId,
          type: actionDef.type, // this is the 'type' key
          name: actionDef.name,
          category: actionDef.category as any,
          description: "",
          parameters: defaultParams,
          source: actionDef.source ? {
            kind: actionDef.source.kind as any,
            pluginId: actionDef.source.pluginId,
            pluginVersion: actionDef.source.pluginVersion,
            baseActionId: actionDef.id
          } : { kind: "core" },
          execution,
          children: [],
        };

        // 3. Commit
        upsertAction(stepId, newAction, parentId, index);

        // Auto-select
        selectAction(stepId, newAction.id);

        void recomputeHash();
      }
    },
    [steps, upsertAction, selectAction, recomputeHash, toggleLibraryScrollLock, reorderStep],
  );
  // validation status badges removed (unused)
  /* ------------------------------- Panels ---------------------------------- */
  const leftPanel = useMemo(
    () => (
      <div id="tour-designer-blocks" ref={libraryRootRef} data-library-root className="h-full">
        <ActionLibraryPanel />
      </div>
    ),
    [],
  );

  const centerPanel = useMemo(
    () => (
      <div id="tour-designer-canvas" className="h-full">
        <FlowWorkspace />
      </div>
    ),
    [],
  );

  const rightPanel = useMemo(
    () => (
      <div id="tour-designer-properties" className="h-full">
        <InspectorPanel
          activeTab={inspectorTab}
          onTabChange={setInspectorTab}
          studyPlugins={studyPlugins}
          onClearAll={clearAllValidationIssues}
        />
      </div>
    ),
    [inspectorTab, studyPlugins, clearAllValidationIssues],
  );

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
      {experimentMetadata && (
        <Button
          variant="ghost"
          size="icon"
          onClick={() => setSettingsOpen(true)}
          title="Experiment Settings"
        >
          <Settings className="h-5 w-5" />
        </Button>
      )}
      <Button
        variant="ghost"
        size="icon"
        onClick={() => startTour("designer")}
        title="Start Tour"
      >
        <HelpCircle className="h-5 w-5" />
      </Button>
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
    <div className="relative flex h-[calc(100vh-5rem)] w-full flex-col overflow-hidden bg-background">
      {/* Subtle Background Gradients */}
      <div className="absolute top-0 left-1/2 -z-10 h-[400px] w-[800px] -translate-x-1/2 rounded-full bg-primary/10 blur-3xl opacity-20 dark:opacity-10" />
      <div className="absolute bottom-0 right-0 -z-10 h-[250px] w-[250px] rounded-full bg-violet-500/5 blur-3xl" />
      <PageHeader
        title={designMeta.name}
        description={designMeta.description || "No description"}
        icon={Play}
        actions={actions}
        className="flex-none pb-4"
      />

      {/* Main Grid Container - 2-4-2 Split */}
      {/* Main Grid Container - 2-4-2 Split */}
      <div className="flex-1 min-h-0 w-full px-2 overflow-hidden">
        <DndContext
          sensors={sensors}
          collisionDetection={closestCenter}
          onDragStart={handleDragStart}
          onDragOver={handleDragOver}
          onDragEnd={handleDragEnd}
          onDragCancel={() => toggleLibraryScrollLock(false)}
        >
          <div className="grid grid-cols-8 gap-4 h-full w-full transition-all duration-300 ease-in-out">
            {/* Left Panel (Library) */}
            {!leftCollapsed && (
              <div className={cn(
                "flex flex-col overflow-hidden rounded-lg border bg-background shadow-sm",
                rightCollapsed ? "col-span-3" : "col-span-2"
              )}>
                <div className="flex items-center justify-between border-b px-3 py-2 bg-muted/30">
                  <span className="text-sm font-medium">Action Library</span>
                  <Button
                    variant="ghost"
                    size="icon"
                    className="h-6 w-6"
                    onClick={() => setLeftCollapsed(true)}
                  >
                    <PanelLeftClose className="h-4 w-4" />
                  </Button>
                </div>
                <div className="flex-1 overflow-hidden min-h-0 bg-muted/10">
                  {leftPanel}
                </div>
              </div>
            )}

            {/* Center Panel (Workspace) */}
            <div className={cn(
              "flex flex-col overflow-hidden rounded-lg border bg-background shadow-sm",
              leftCollapsed && rightCollapsed ? "col-span-8" :
                leftCollapsed ? "col-span-6" :
                  rightCollapsed ? "col-span-5" :
                    "col-span-4"
            )}>
              <div className="flex items-center justify-between border-b px-3 py-2 bg-muted/30">
                {leftCollapsed && (
                  <Button
                    variant="ghost"
                    size="icon"
                    className="h-6 w-6 mr-2"
                    onClick={() => setLeftCollapsed(false)}
                    title="Open Library"
                  >
                    <PanelLeftOpen className="h-4 w-4" />
                  </Button>
                )}
                <span className="text-sm font-medium">Flow Workspace</span>
                {rightCollapsed && (
                  <div className="flex items-center">
                    <Button variant="ghost" size="icon" className="h-7 w-7" onClick={() => startTour('designer')}>
                      <HelpCircle className="h-4 w-4" />
                    </Button>
                    {rightCollapsed && (
                      <Button
                        variant="ghost"
                        size="icon"
                        className="h-6 w-6 ml-2"
                        onClick={() => setRightCollapsed(false)}
                        title="Open Inspector"
                      >
                        <PanelRightOpen className="h-4 w-4" />
                      </Button>
                    )}
                  </div>
                )}
              </div>
              <div className="flex-1 overflow-hidden min-h-0 relative">
                {centerPanel}
              </div>
              <div className="border-t">
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

            {/* Right Panel (Inspector) */}
            {!rightCollapsed && (
              <div className={cn(
                "flex flex-col overflow-hidden rounded-lg border bg-background shadow-sm",
                leftCollapsed ? "col-span-2" : "col-span-2"
              )}>
                <div className="flex items-center justify-between border-b px-3 py-2 bg-muted/30">
                  <span className="text-sm font-medium">Inspector</span>
                  <Button
                    variant="ghost"
                    size="icon"
                    className="h-6 w-6"
                    onClick={() => setRightCollapsed(true)}
                  >
                    <PanelRightClose className="h-4 w-4" />
                  </Button>
                </div>
                <div className="flex-1 overflow-hidden min-h-0 bg-muted/10">
                  {rightPanel}
                </div>
              </div>
            )}
          </div>

          <DragOverlay dropAnimation={null}>
            {dragOverlayAction ? (
              // Library Item Drag
              <div className="bg-background pointer-events-none flex items-center gap-2 rounded border px-3 py-2 text-xs font-medium shadow-lg select-none ring-2 ring-blue-500/20">
                <div
                  className={cn(
                    "flex h-4 w-4 items-center justify-center rounded text-white",
                    dragOverlayAction.category === "robot" && "bg-emerald-600",
                    dragOverlayAction.category === "control" && "bg-amber-500",
                    dragOverlayAction.category === "observation" &&
                    "bg-purple-600",
                  )}
                />
                {dragOverlayAction.name}
              </div>
            ) : activeSortableItem?.type === 'action' ? (
              // Existing Action Sort
              <div className="w-[300px] opacity-90 pointer-events-none">
                <SortableActionChip
                  stepId={activeSortableItem.data.stepId}
                  action={activeSortableItem.data.action}
                  parentId={activeSortableItem.data.parentId}
                  selectedActionId={selectedActionId}
                  onSelectAction={() => { }}
                  onDeleteAction={() => { }}
                  dragHandle={true}
                />
              </div>
            ) : activeSortableItem?.type === 'step' ? (
              // Existing Step Sort
              <div className="w-[400px] pointer-events-none opacity-90">
                <StepCardPreview step={activeSortableItem.data.step} dragHandle />
              </div>
            ) : null}
          </DragOverlay>
        </DndContext>
      </div>

      {/* Settings Modal */}
      {experimentMetadata && (
        <SettingsModal
          open={settingsOpen}
          onOpenChange={setSettingsOpen}
          experiment={experimentMetadata}
          designStats={designStats}
        />
      )}
    </div>
  );
}

export default DesignerRoot;
