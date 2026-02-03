"use client";

import React, { useCallback, useMemo } from "react";
import {
  Save,
  RefreshCw,
  Download,
  AlertTriangle,
  CheckCircle2,
  Hash,
  GitBranch,
  Sparkles,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { cn } from "~/lib/utils";
import { useDesignerStore } from "../state/store";

export interface BottomStatusBarProps {
  onSave?: () => void;
  onValidate?: () => void;
  onExport?: () => void;
  onOpenCommandPalette?: () => void;
  onRecalculateHash?: () => void;
  className?: string;
  saving?: boolean;
  validating?: boolean;
  exporting?: boolean;
  lastSavedAt?: Date;
}

export function BottomStatusBar({
  onSave,
  onValidate,
  onExport,
  className,
  saving,
  validating,
  exporting,
}: BottomStatusBarProps) {
  const steps = useDesignerStore((s) => s.steps);
  const lastPersistedHash = useDesignerStore((s) => s.lastPersistedHash);
  const currentDesignHash = useDesignerStore((s) => s.currentDesignHash);
  const lastValidatedHash = useDesignerStore((s) => s.lastValidatedHash);
  const pendingSave = useDesignerStore((s) => s.pendingSave);

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

  const validationBadge = (() => {
    switch (validationStatus) {
      case "valid":
        return (
          <div className="flex items-center gap-1.5 text-green-600 dark:text-green-400">
            <CheckCircle2 className="h-3.5 w-3.5" />
            <span className="hidden sm:inline">Valid</span>
          </div>
        );
      case "drift":
        return (
          <div className="flex items-center gap-1.5 text-amber-600 dark:text-amber-400">
            <AlertTriangle className="h-3.5 w-3.5" />
            <span className="hidden sm:inline">Modified</span>
          </div>
        );
      default:
        return (
          <div className="flex items-center gap-1.5 text-muted-foreground">
            <Hash className="h-3.5 w-3.5" />
            <span className="hidden sm:inline">Unvalidated</span>
          </div>
        );
    }
  })();

  const unsavedBadge =
    hasUnsaved && !pendingSave ? (
      <Badge
        variant="outline"
        className="h-5 gap-1 border-orange-300 px-1.5 text-[10px] font-normal text-orange-600 dark:text-orange-400"
      >
        Unsaved
      </Badge>
    ) : null;

  const savingIndicator =
    pendingSave || saving ? (
      <div className="flex items-center gap-1.5 text-muted-foreground animate-pulse">
        <RefreshCw className="h-3 w-3 animate-spin" />
        <span>Saving...</span>
      </div>
    ) : null;

  return (
    <div
      className={cn(
        "border-border/60 bg-muted/40 supports-[backdrop-filter]:bg-muted/30 backdrop-blur",
        "flex h-9 w-full flex-shrink-0 items-center gap-4 border-t px-3 text-xs font-medium",
        className,
      )}
    >
      {/* Status Indicators */}
      <div className="flex items-center gap-3 min-w-0">
        {validationBadge}
        {unsavedBadge}
        {savingIndicator}
      </div>

      <Separator orientation="vertical" className="h-4 opacity-50" />

      {/* Stats */}
      <div className="text-muted-foreground flex items-center gap-3 truncate">
        <span className="flex items-center gap-1.5">
          <GitBranch className="h-3.5 w-3.5 opacity-70" />
          {steps.length}
        </span>
        <span className="flex items-center gap-1.5">
          <Sparkles className="h-3.5 w-3.5 opacity-70" />
          {actionCount}
        </span>
      </div>

      <div className="flex-1" />

      {/* Actions */}
      <div className="flex items-center gap-1">
        <Button
          variant="ghost"
          size="sm"
          className="h-7 px-2 text-xs"
          onClick={onExport}
          disabled={exporting}
          title="Export JSON"
        >
          <Download className="mr-1.5 h-3.5 w-3.5" />
          Export
        </Button>
      </div>
    </div>
  );
}

export default BottomStatusBar;
