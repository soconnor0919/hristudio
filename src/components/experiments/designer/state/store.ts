"use client";
/**
 * Experiment Designer Zustand Store
 *
 * Centralized state management for the redesigned experiment designer.
 * Responsibilities:
 * - Steps & actions structural state
 * - Selection state (step / action)
 * - Dirty tracking
 * - Hashing & drift (incremental design hash computation)
 * - Validation issue storage
 * - Plugin action signature drift detection
 * - Save / conflict / versioning control flags
 *
 * This store intentionally avoids direct network calls; consumers orchestrate
 * server mutations & pass results back into the store (pure state container).
 */

import { create } from "zustand";
import type {
  ExperimentStep,
  ExperimentAction,
} from "~/lib/experiment-designer/types";
import {
  computeIncrementalDesignHash,
  type IncrementalHashMaps,
  type IncrementalHashResult,
  computeActionSignature,
} from "./hashing";

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

export interface ValidationIssue {
  entityId: string;
  severity: "error" | "warning" | "info";
  message: string;
  code?: string;
}

export type VersionStrategy = "auto" | "forceIncrement" | "none";

export interface ConflictState {
  serverHash: string;
  localHash: string;
  at: Date;
}

export interface DesignerState {
  // Core structural
  steps: ExperimentStep[];

  // Selection
  selectedStepId?: string;
  selectedActionId?: string;

  // Dirty tracking (entity IDs)
  dirtyEntities: Set<string>;

  // Hashing
  lastPersistedHash?: string;
  currentDesignHash?: string;
  lastValidatedHash?: string;
  incremental?: IncrementalHashMaps;

  // Validation & drift
  validationIssues: Record<string, ValidationIssue[]>;
  actionSignatureIndex: Map<string, string>; // actionType or instance -> signature hash
  actionSignatureDrift: Set<string>; // action instance IDs with drift

  // Saving & conflicts
  pendingSave: boolean;
  conflict?: ConflictState;
  versionStrategy: VersionStrategy;
  autoSaveEnabled: boolean;

  // Flags
  busyHashing: boolean;
  busyValidating: boolean;

  /* ------------------------------ Mutators --------------------------------- */

  // Selection
  selectStep: (id?: string) => void;
  selectAction: (stepId: string, actionId?: string) => void;

  // Steps
  setSteps: (steps: ExperimentStep[]) => void;
  upsertStep: (step: ExperimentStep) => void;
  removeStep: (stepId: string) => void;
  reorderStep: (from: number, to: number) => void;

  // Actions
  upsertAction: (stepId: string, action: ExperimentAction) => void;
  removeAction: (stepId: string, actionId: string) => void;
  reorderAction: (stepId: string, from: number, to: number) => void;

  // Dirty
  markDirty: (id: string) => void;
  clearDirty: (id: string) => void;
  clearAllDirty: () => void;

  // Hashing
  recomputeHash: (options?: {
    forceFull?: boolean;
  }) => Promise<IncrementalHashResult | null>;
  setPersistedHash: (hash: string) => void;
  setValidatedHash: (hash: string) => void;

  // Validation
  setValidationIssues: (entityId: string, issues: ValidationIssue[]) => void;
  clearValidationIssues: (entityId: string) => void;
  clearAllValidationIssues: () => void;

  // Drift detection (action definition signature)
  setActionSignature: (actionId: string, signature: string) => void;
  detectActionSignatureDrift: (
    action: ExperimentAction,
    latestSignature: string,
  ) => void;
  clearActionSignatureDrift: (actionId: string) => void;

  // Save workflow
  setPendingSave: (pending: boolean) => void;
  recordConflict: (serverHash: string, localHash: string) => void;
  clearConflict: () => void;
  setVersionStrategy: (strategy: VersionStrategy) => void;
  setAutoSaveEnabled: (enabled: boolean) => void;

  // Bulk apply from server (authoritative sync after save/fetch)
  applyServerSync: (payload: {
    steps: ExperimentStep[];
    persistedHash?: string;
    validatedHash?: string;
  }) => void;
}

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

function cloneSteps(steps: ExperimentStep[]): ExperimentStep[] {
  return steps.map((s) => ({
    ...s,
    actions: s.actions.map((a) => ({ ...a })),
  }));
}

function reindexSteps(steps: ExperimentStep[]): ExperimentStep[] {
  return steps
    .slice()
    .sort((a, b) => a.order - b.order)
    .map((s, idx) => ({ ...s, order: idx }));
}

function reindexActions(actions: ExperimentAction[]): ExperimentAction[] {
  // ExperimentAction type does not define orderIndex; preserve array order only
  return actions.map((a) => ({ ...a }));
}

function updateActionList(
  existing: ExperimentAction[],
  action: ExperimentAction,
): ExperimentAction[] {
  const idx = existing.findIndex((a) => a.id === action.id);
  if (idx >= 0) {
    const copy = [...existing];
    copy[idx] = { ...action };
    return copy;
  }
  return [...existing, { ...action }];
}

/* -------------------------------------------------------------------------- */
/* Store Implementation                                                       */
/* -------------------------------------------------------------------------- */

export const useDesignerStore = create<DesignerState>((set, get) => ({
  steps: [],
  dirtyEntities: new Set<string>(),
  validationIssues: {},
  actionSignatureIndex: new Map(),
  actionSignatureDrift: new Set(),
  pendingSave: false,
  versionStrategy: "auto_minor" as VersionStrategy,
  autoSaveEnabled: true,
  busyHashing: false,
  busyValidating: false,

  /* ------------------------------ Selection -------------------------------- */
  selectStep: (id) =>
    set({
      selectedStepId: id,
      selectedActionId: id ? get().selectedActionId : undefined,
    }),
  selectAction: (stepId, actionId) =>
    set({
      selectedStepId: stepId,
      selectedActionId: actionId,
    }),

  /* -------------------------------- Steps ---------------------------------- */
  setSteps: (steps) =>
    set(() => ({
      steps: reindexSteps(cloneSteps(steps)),
      dirtyEntities: new Set<string>(), // assume authoritative load
    })),

  upsertStep: (step) =>
    set((state) => {
      const idx = state.steps.findIndex((s) => s.id === step.id);
      let steps: ExperimentStep[];
      if (idx >= 0) {
        steps = [...state.steps];
        steps[idx] = { ...step };
      } else {
        steps = [...state.steps, { ...step, order: state.steps.length }];
      }
      return {
        steps: reindexSteps(steps),
        dirtyEntities: new Set([...state.dirtyEntities, step.id]),
      };
    }),

  removeStep: (stepId) =>
    set((state) => {
      const steps = state.steps.filter((s) => s.id !== stepId);
      const dirty = new Set(state.dirtyEntities);
      dirty.add(stepId);
      return {
        steps: reindexSteps(steps),
        dirtyEntities: dirty,
        selectedStepId:
          state.selectedStepId === stepId ? undefined : state.selectedStepId,
        selectedActionId: undefined,
      };
    }),

  reorderStep: (from: number, to: number) =>
    set((state: DesignerState) => {
      if (
        from < 0 ||
        to < 0 ||
        from >= state.steps.length ||
        to >= state.steps.length ||
        from === to
      ) {
        return state;
      }
      const stepsDraft = [...state.steps];
      const [moved] = stepsDraft.splice(from, 1);
      if (!moved) return state;
      stepsDraft.splice(to, 0, moved);
      const reindexed = reindexSteps(stepsDraft);
      return {
        steps: reindexed,
        dirtyEntities: new Set<string>([
          ...state.dirtyEntities,
          ...reindexed.map((s) => s.id),
        ]),
      };
    }),

  /* ------------------------------- Actions --------------------------------- */
  upsertAction: (stepId: string, action: ExperimentAction) =>
    set((state: DesignerState) => {
      const stepsDraft: ExperimentStep[] = state.steps.map((s) =>
        s.id === stepId
          ? {
              ...s,
              actions: reindexActions(updateActionList(s.actions, action)),
            }
          : s,
      );
      return {
        steps: stepsDraft,
        dirtyEntities: new Set<string>([
          ...state.dirtyEntities,
          action.id,
          stepId,
        ]),
      };
    }),

  removeAction: (stepId: string, actionId: string) =>
    set((state: DesignerState) => {
      const stepsDraft: ExperimentStep[] = state.steps.map((s) =>
        s.id === stepId
          ? {
              ...s,
              actions: reindexActions(
                s.actions.filter((a) => a.id !== actionId),
              ),
            }
          : s,
      );
      const dirty = new Set<string>(state.dirtyEntities);
      dirty.add(actionId);
      dirty.add(stepId);
      return {
        steps: stepsDraft,
        dirtyEntities: dirty,
        selectedActionId:
          state.selectedActionId === actionId
            ? undefined
            : state.selectedActionId,
      };
    }),

  reorderAction: (stepId: string, from: number, to: number) =>
    set((state: DesignerState) => {
      const stepsDraft: ExperimentStep[] = state.steps.map((s) => {
        if (s.id !== stepId) return s;
        if (
          from < 0 ||
          to < 0 ||
          from >= s.actions.length ||
          to >= s.actions.length ||
          from === to
        ) {
          return s;
        }
        const actionsDraft = [...s.actions];
        const [moved] = actionsDraft.splice(from, 1);
        if (!moved) return s;
        actionsDraft.splice(to, 0, moved);
        return { ...s, actions: reindexActions(actionsDraft) };
      });
      return {
        steps: stepsDraft,
        dirtyEntities: new Set<string>([...state.dirtyEntities, stepId]),
      };
    }),

  /* -------------------------------- Dirty ---------------------------------- */
  markDirty: (id: string) =>
    set((state: DesignerState) => ({
      dirtyEntities: state.dirtyEntities.has(id)
        ? state.dirtyEntities
        : new Set<string>([...state.dirtyEntities, id]),
    })),
  clearDirty: (id: string) =>
    set((state: DesignerState) => {
      if (!state.dirtyEntities.has(id)) return state;
      const next = new Set(state.dirtyEntities);
      next.delete(id);
      return { dirtyEntities: next };
    }),
  clearAllDirty: () => set({ dirtyEntities: new Set<string>() }),

  /* ------------------------------- Hashing --------------------------------- */
  recomputeHash: async (options?: { forceFull?: boolean }) => {
    const { steps, incremental } = get();
    if (steps.length === 0) {
      set({ currentDesignHash: undefined });
      return null;
    }
    set({ busyHashing: true });
    try {
      const result = await computeIncrementalDesignHash(
        steps,
        options?.forceFull ? undefined : incremental,
      );
      set({
        currentDesignHash: result.designHash,
        incremental: {
          actionHashes: result.actionHashes,
          stepHashes: result.stepHashes,
        },
      });
      return result;
    } finally {
      set({ busyHashing: false });
    }
  },

  setPersistedHash: (hash: string) => set({ lastPersistedHash: hash }),
  setValidatedHash: (hash: string) => set({ lastValidatedHash: hash }),

  /* ----------------------------- Validation -------------------------------- */
  setValidationIssues: (entityId: string, issues: ValidationIssue[]) =>
    set((state: DesignerState) => ({
      validationIssues: {
        ...state.validationIssues,
        [entityId]: issues,
      },
    })),
  clearValidationIssues: (entityId: string) =>
    set((state: DesignerState) => {
      if (!state.validationIssues[entityId]) return state;
      const next = { ...state.validationIssues };
      delete next[entityId];
      return { validationIssues: next };
    }),
  clearAllValidationIssues: () => set({ validationIssues: {} }),

  /* ------------------------- Action Signature Drift ------------------------ */
  setActionSignature: (actionId: string, signature: string) =>
    set((state: DesignerState) => {
      const index = new Map(state.actionSignatureIndex);
      index.set(actionId, signature);
      return { actionSignatureIndex: index };
    }),
  detectActionSignatureDrift: (
    action: ExperimentAction,
    latestSignature: string,
  ) =>
    set((state: DesignerState) => {
      const current = state.actionSignatureIndex.get(action.id);
      if (!current) {
        const idx = new Map(state.actionSignatureIndex);
        idx.set(action.id, latestSignature);
        return { actionSignatureIndex: idx };
      }
      if (current === latestSignature) return {};
      const drift = new Set(state.actionSignatureDrift);
      drift.add(action.id);
      return { actionSignatureDrift: drift };
    }),
  clearActionSignatureDrift: (actionId: string) =>
    set((state: DesignerState) => {
      if (!state.actionSignatureDrift.has(actionId)) return state;
      const next = new Set(state.actionSignatureDrift);
      next.delete(actionId);
      return { actionSignatureDrift: next };
    }),

  /* ------------------------------- Save Flow -------------------------------- */
  setPendingSave: (pending: boolean) => set({ pendingSave: pending }),
  recordConflict: (serverHash: string, localHash: string) =>
    set({
      conflict: { serverHash, localHash, at: new Date() },
      pendingSave: false,
    }),
  clearConflict: () => set({ conflict: undefined }),
  setVersionStrategy: (strategy: VersionStrategy) =>
    set({ versionStrategy: strategy }),
  setAutoSaveEnabled: (enabled: boolean) => set({ autoSaveEnabled: enabled }),

  /* ------------------------------ Server Sync ------------------------------ */
  applyServerSync: (payload: {
    steps: ExperimentStep[];
    persistedHash?: string;
    validatedHash?: string;
  }) =>
    set((state: DesignerState) => {
      const syncedSteps = reindexSteps(cloneSteps(payload.steps));
      const dirty = new Set<string>();
      return {
        steps: syncedSteps,
        lastPersistedHash: payload.persistedHash ?? state.lastPersistedHash,
        lastValidatedHash: payload.validatedHash ?? state.lastValidatedHash,
        dirtyEntities: dirty,
        conflict: undefined,
      };
    }),
}));

/* -------------------------------------------------------------------------- */
/* Convenience Selectors                                                      */
/* -------------------------------------------------------------------------- */

export const useDesignerSteps = (): ExperimentStep[] =>
  useDesignerStore((s) => s.steps);

export const useDesignerSelection = (): {
  selectedStepId: string | undefined;
  selectedActionId: string | undefined;
} =>
  useDesignerStore((s) => ({
    selectedStepId: s.selectedStepId,
    selectedActionId: s.selectedActionId,
  }));

export const useDesignerHashes = (): {
  currentDesignHash: string | undefined;
  lastPersistedHash: string | undefined;
  lastValidatedHash: string | undefined;
} =>
  useDesignerStore((s) => ({
    currentDesignHash: s.currentDesignHash,
    lastPersistedHash: s.lastPersistedHash,
    lastValidatedHash: s.lastValidatedHash,
  }));

export const useDesignerDrift = (): {
  hasDrift: boolean;
  actionSignatureDrift: Set<string>;
} =>
  useDesignerStore((s) => ({
    hasDrift:
      !!s.lastValidatedHash &&
      !!s.currentDesignHash &&
      s.currentDesignHash !== s.lastValidatedHash,
    actionSignatureDrift: s.actionSignatureDrift,
  }));

/* -------------------------------------------------------------------------- */
/* Signature Helper (on-demand)                                               */
/* -------------------------------------------------------------------------- */

/**
 * Compute a signature for an action definition or instance (schema + provenance).
 * Store modules can call this to register baseline signatures.
 */
export async function computeBaselineActionSignature(
  action: ExperimentAction,
): Promise<string> {
  return computeActionSignature({
    type: action.type,
    category: action.category,
    parameterSchemaRaw: action.parameterSchemaRaw,
    execution: action.execution,
    baseActionId: action.source.baseActionId,
    pluginVersion: action.source.pluginVersion,
    pluginId: action.source.pluginId,
  });
}
