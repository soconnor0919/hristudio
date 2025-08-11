"use client";

import React, { useCallback, useMemo } from "react";
import {
  Save,
  RefreshCw,
  Download,
  Hash,
  AlertTriangle,
  CheckCircle2,
  UploadCloud,
  Wand2,
  Sparkles,
  GitBranch,
  Keyboard,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { cn } from "~/lib/utils";
import { useDesignerStore } from "../state/store";

/**
 * BottomStatusBar
 *
 * Compact, persistent status + quick-action bar for the Experiment Designer.
 * Shows:
 *  - Validation / drift / unsaved state
 *  - Short design hash & version
 *  - Aggregate counts (steps / actions)
 *  - Last persisted hash (if available)
 *  - Quick actions (Save, Validate, Export, Command Palette)
 *
 * The bar is intentionally UI-only: callback props are used so that higher-level
 * orchestration (e.g. DesignerRoot / Shell) controls actual side effects.
 */

export interface BottomStatusBarProps {
  onSave?: () => void;
  onValidate?: () => void;
  onExport?: () => void;
  onOpenCommandPalette?: () => void;
  onToggleVersionStrategy?: () => void;
  className?: string;
  saving?: boolean;
  validating?: boolean;
  exporting?: boolean;
  /**
   * Optional externally supplied last saved Date for relative display.
   */
  lastSavedAt?: Date;
}

export function BottomStatusBar({
  onSave,
  onValidate,
  onExport,
  onOpenCommandPalette,
  onToggleVersionStrategy,
  className,
  saving,
  validating,
  exporting,
  lastSavedAt,
}: BottomStatusBarProps) {
  /* ------------------------------------------------------------------------ */
  /* Store Selectors                                                          */
  /* ------------------------------------------------------------------------ */
  const steps = useDesignerStore((s) => s.steps);
  const lastPersistedHash = useDesignerStore((s) => s.lastPersistedHash);
  const currentDesignHash = useDesignerStore((s) => s.currentDesignHash);
  const lastValidatedHash = useDesignerStore((s) => s.lastValidatedHash);
  const pendingSave = useDesignerStore((s) => s.pendingSave);
  const versionStrategy = useDesignerStore((s) => s.versionStrategy);
  const autoSaveEnabled = useDesignerStore((s) => s.autoSaveEnabled);

  const actionCount = useMemo(
    () => steps.reduce((sum, st) => sum + st.actions.length, 0),
    [steps],
  );

  const hasUnsaved = useMemo(
    () =>
      Boolean(currentDesignHash) &&
      currentDesignHash !== lastPersistedHash &&
      !pendingSave,
    [currentDesignHash, lastPersistedHash, pendingSave],
  );

  const validationStatus = useMemo<"unvalidated" | "valid" | "drift">(() => {
    if (!currentDesignHash || !lastValidatedHash) return "unvalidated";
    if (currentDesignHash !== lastValidatedHash) return "drift";
    return "valid";
  }, [currentDesignHash, lastValidatedHash]);

  const shortHash = useMemo(
    () => (currentDesignHash ? currentDesignHash.slice(0, 8) : "—"),
    [currentDesignHash],
  );

  const lastPersistedShort = useMemo(
    () => (lastPersistedHash ? lastPersistedHash.slice(0, 8) : null),
    [lastPersistedHash],
  );

  /* ------------------------------------------------------------------------ */
  /* Derived Display Helpers                                                  */
  /* ------------------------------------------------------------------------ */
  function formatRelative(date?: Date): string {
    if (!date) return "—";
    const now = Date.now();
    const diffMs = now - date.getTime();
    if (diffMs < 30_000) return "just now";
    const mins = Math.floor(diffMs / 60_000);
    if (mins < 60) return `${mins}m ago`;
    const hrs = Math.floor(mins / 60);
    if (hrs < 24) return `${hrs}h ago`;
    const days = Math.floor(hrs / 24);
    return `${days}d ago`;
  }

  const relSaved = formatRelative(lastSavedAt);

  const validationBadge = (() => {
    switch (validationStatus) {
      case "valid":
        return (
          <Badge
            variant="outline"
            className="border-green-400 text-green-600 dark:text-green-400"
            title="Validated (hash stable)"
          >
            <CheckCircle2 className="mr-1 h-3 w-3" />
            Validated
          </Badge>
        );
      case "drift":
        return (
          <Badge
            variant="destructive"
            className="border-amber-400 bg-amber-50 text-amber-700 dark:bg-amber-950/30 dark:text-amber-400"
            title="Drift since last validation"
          >
            <AlertTriangle className="mr-1 h-3 w-3" />
            Drift
          </Badge>
        );
      default:
        return (
          <Badge variant="outline" title="Not validated yet">
            <Hash className="mr-1 h-3 w-3" />
            Unvalidated
          </Badge>
        );
    }
  })();

  const unsavedBadge =
    hasUnsaved && !pendingSave ? (
      <Badge
        variant="outline"
        className="border-orange-300 text-orange-600 dark:text-orange-400"
        title="Unsaved changes"
      >
        ● Unsaved
      </Badge>
    ) : null;

  const savingIndicator =
    pendingSave || saving ? (
      <Badge
        variant="secondary"
        className="animate-pulse"
        title="Saving changes"
      >
        <RefreshCw className="mr-1 h-3 w-3 animate-spin" />
        Saving…
      </Badge>
    ) : null;

  /* ------------------------------------------------------------------------ */
  /* Handlers                                                                  */
  /* ------------------------------------------------------------------------ */
  const handleSave = useCallback(() => {
    if (onSave) onSave();
  }, [onSave]);

  const handleValidate = useCallback(() => {
    if (onValidate) onValidate();
  }, [onValidate]);

  const handleExport = useCallback(() => {
    if (onExport) onExport();
  }, [onExport]);

  const handlePalette = useCallback(() => {
    if (onOpenCommandPalette) onOpenCommandPalette();
  }, [onOpenCommandPalette]);

  const handleToggleVersionStrategy = useCallback(() => {
    if (onToggleVersionStrategy) onToggleVersionStrategy();
  }, [onToggleVersionStrategy]);

  /* ------------------------------------------------------------------------ */
  /* Render                                                                    */
  /* ------------------------------------------------------------------------ */

  return (
    <div
      className={cn(
        "border-border/60 bg-muted/40 backdrop-blur supports-[backdrop-filter]:bg-muted/30",
        "flex h-10 w-full flex-shrink-0 items-center gap-3 border-t px-3 text-xs",
        "font-medium",
        className,
      )}
      aria-label="Designer status bar"
    >
      {/* Left Cluster: Validation & Hash */}
      <div className="flex items-center gap-2">
        {validationBadge}
        {unsavedBadge}
        {savingIndicator}
        <Separator orientation="vertical" className="h-4" />
        <div
          className="flex items-center gap-1 font-mono text-[11px]"
          title="Current design hash"
        >
          <Hash className="h-3 w-3 text-muted-foreground" />
          {shortHash}
          {lastPersistedShort && lastPersistedShort !== shortHash && (
            <span
              className="text-muted-foreground/70"
              title="Last persisted hash"
            >
              / {lastPersistedShort}
            </span>
          )}
        </div>
      </div>

      {/* Middle Cluster: Aggregate Counts */}
      <div className="flex items-center gap-3 text-muted-foreground">
        <div
          className="flex items-center gap-1"
          title="Steps in current design"
        >
          <GitBranch className="h-3 w-3" />
          {steps.length} steps
        </div>
        <div
          className="flex items-center gap-1"
          title="Total actions across all steps"
        >
          <Sparkles className="h-3 w-3" />
          {actionCount} actions
        </div>
        <div
          className="hidden items-center gap-1 sm:flex"
          title="Auto-save setting"
        >
          <UploadCloud className="h-3 w-3" />
          {autoSaveEnabled ? "auto-save on" : "auto-save off"}
        </div>
        <div
          className="hidden cursor-pointer items-center gap-1 sm:flex"
          title={`Version strategy: ${versionStrategy}`}
          onClick={handleToggleVersionStrategy}
        >
          <Wand2 className="h-3 w-3" />
          {versionStrategy.replace(/_/g, " ")}
        </div>
        <div
          className="hidden items-center gap-1 text-[10px] font-normal tracking-wide text-muted-foreground/80 md:flex"
          title="Relative time since last save"
        >
          Saved {relSaved}
        </div>
      </div>

      {/* Flexible Spacer */}
      <div className="flex-1" />

      {/* Right Cluster: Quick Actions */}
      <div className="flex items-center gap-1">
        <Button
          variant="ghost"
          size="sm"
          className="h-7 px-2"
          disabled={!hasUnsaved && !pendingSave}
          onClick={handleSave}
          aria-label="Save (s)"
        >
          <Save className="mr-1 h-3 w-3" />
          Save
        </Button>
        <Button
          variant="ghost"
          size="sm"
          className="h-7 px-2"
          onClick={handleValidate}
          disabled={validating}
          aria-label="Validate (v)"
        >
            <RefreshCw
              className={cn(
                "mr-1 h-3 w-3",
                validating && "animate-spin",
              )}
            />
          Validate
        </Button>
        <Button
          variant="ghost"
          size="sm"
          className="h-7 px-2"
          onClick={handleExport}
          disabled={exporting}
          aria-label="Export (e)"
        >
          <Download className="mr-1 h-3 w-3" />
          Export
        </Button>
        <Separator orientation="vertical" className="mx-1 h-4" />
        <Button
          variant="outline"
          size="sm"
          className="h-7 px-2"
          onClick={handlePalette}
          aria-label="Command Palette (⌘K)"
        >
          <Keyboard className="mr-1 h-3 w-3" />
          Commands
        </Button>
      </div>
    </div>
  );
}

export default BottomStatusBar;
