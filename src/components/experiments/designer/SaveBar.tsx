"use client";

import React, { useState } from "react";
import {
  Save,
  Download,
  Upload,
  AlertCircle,
  Clock,
  GitBranch,
  RefreshCw,
  CheckCircle,
  AlertTriangle,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card } from "~/components/ui/card";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Switch } from "~/components/ui/switch";
import { Label } from "~/components/ui/label";
import { Separator } from "~/components/ui/separator";
import { cn } from "~/lib/utils";

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

export type VersionStrategy = "manual" | "auto_minor" | "auto_patch";
export type SaveState = "clean" | "dirty" | "saving" | "conflict" | "error";

export interface SaveBarProps {
  /**
   * Current save state
   */
  saveState: SaveState;
  /**
   * Whether auto-save is enabled
   */
  autoSaveEnabled: boolean;
  /**
   * Current version strategy
   */
  versionStrategy: VersionStrategy;
  /**
   * Number of unsaved changes
   */
  dirtyCount: number;
  /**
   * Current design hash for integrity
   */
  currentHash?: string;
  /**
   * Last persisted hash
   */
  persistedHash?: string;
  /**
   * Last save timestamp
   */
  lastSaved?: Date;
  /**
   * Whether there's a conflict with server state
   */
  hasConflict?: boolean;
  /**
   * Current experiment version
   */
  currentVersion: number;
  /**
   * Called when user manually saves
   */
  onSave: () => void;
  /**
   * Called when user exports the design
   */
  onExport: () => void;
  /**
   * Called when user imports a design
   */
  onImport?: (file: File) => void;
  /**
   * Called when auto-save setting changes
   */
  onAutoSaveChange: (enabled: boolean) => void;
  /**
   * Called when version strategy changes
   */
  onVersionStrategyChange: (strategy: VersionStrategy) => void;
  /**
   * Called when user resolves a conflict
   */
  onResolveConflict?: () => void;
  /**
   * Called when user wants to validate the design
   */
  onValidate?: () => void;
  className?: string;
}

/* -------------------------------------------------------------------------- */
/* Save State Configuration                                                   */
/* -------------------------------------------------------------------------- */

const saveStateConfig = {
  clean: {
    icon: CheckCircle,
    color: "text-green-600 dark:text-green-400",
    label: "Saved",
    description: "All changes saved",
  },
  dirty: {
    icon: AlertCircle,
    color: "text-amber-600 dark:text-amber-400",
    label: "Unsaved",
    description: "You have unsaved changes",
  },
  saving: {
    icon: RefreshCw,
    color: "text-blue-600 dark:text-blue-400",
    label: "Saving",
    description: "Saving changes...",
  },
  conflict: {
    icon: AlertTriangle,
    color: "text-red-600 dark:text-red-400",
    label: "Conflict",
    description: "Server conflict detected",
  },
  error: {
    icon: AlertTriangle,
    color: "text-red-600 dark:text-red-400",
    label: "Error",
    description: "Save failed",
  },
} as const;

/* -------------------------------------------------------------------------- */
/* Version Strategy Options                                                   */
/* -------------------------------------------------------------------------- */

const versionStrategyOptions = [
  {
    value: "manual" as const,
    label: "Manual",
    description: "Only increment version when explicitly requested",
  },
  {
    value: "auto_minor" as const,
    label: "Auto Minor",
    description: "Auto-increment minor version on structural changes",
  },
  {
    value: "auto_patch" as const,
    label: "Auto Patch",
    description: "Auto-increment patch version on any save",
  },
];

/* -------------------------------------------------------------------------- */
/* Utility Functions                                                          */
/* -------------------------------------------------------------------------- */

function formatLastSaved(date?: Date): string {
  if (!date) return "Never";

  const now = new Date();
  const diffMs = now.getTime() - date.getTime();
  const diffMins = Math.floor(diffMs / (1000 * 60));

  if (diffMins < 1) return "Just now";
  if (diffMins < 60) return `${diffMins}m ago`;

  const diffHours = Math.floor(diffMins / 60);
  if (diffHours < 24) return `${diffHours}h ago`;

  const diffDays = Math.floor(diffHours / 24);
  return `${diffDays}d ago`;
}

function getNextVersion(
  current: number,
  strategy: VersionStrategy,
  hasStructuralChanges = false,
): number {
  switch (strategy) {
    case "manual":
      return current;
    case "auto_minor":
      return hasStructuralChanges ? current + 1 : current;
    case "auto_patch":
      return current + 1;
    default:
      return current;
  }
}

/* -------------------------------------------------------------------------- */
/* Import Handler                                                             */
/* -------------------------------------------------------------------------- */

function ImportButton({ onImport }: { onImport?: (file: File) => void }) {
  const handleFileSelect = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file && onImport) {
      onImport(file);
    }
    // Reset input to allow re-selecting the same file
    event.target.value = "";
  };

  if (!onImport) return null;

  return (
    <div>
      <input
        type="file"
        accept=".json"
        onChange={handleFileSelect}
        className="hidden"
        id="import-design"
      />
      <Button
        variant="outline"
        size="sm"
        className="h-8"
        onClick={() => document.getElementById("import-design")?.click()}
      >
        <Upload className="mr-2 h-3 w-3" />
        Import
      </Button>
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* SaveBar Component                                                          */
/* -------------------------------------------------------------------------- */

export function SaveBar({
  saveState,
  autoSaveEnabled,
  versionStrategy,
  dirtyCount,
  currentHash,
  persistedHash,
  lastSaved,
  hasConflict,
  currentVersion,
  onSave,
  onExport,
  onImport,
  onAutoSaveChange,
  onVersionStrategyChange,
  onResolveConflict,
  onValidate,
  className,
}: SaveBarProps) {
  const [showSettings, setShowSettings] = useState(false);

  const config = saveStateConfig[saveState];
  const IconComponent = config.icon;

  const hasUnsavedChanges = saveState === "dirty" || dirtyCount > 0;
  const canSave = hasUnsavedChanges && saveState !== "saving";
  const hashesMatch =
    currentHash && persistedHash && currentHash === persistedHash;

  return (
    <Card className={cn("rounded-t-none border-t-0", className)}>
      <div className="flex items-center justify-between p-3">
        {/* Left: Save Status & Info */}
        <div className="flex items-center gap-3">
          {/* Save State Indicator */}
          <div className="flex items-center gap-2">
            <IconComponent
              className={cn(
                "h-4 w-4",
                config.color,
                saveState === "saving" && "animate-spin",
              )}
            />
            <div className="text-sm">
              <span className="font-medium">{config.label}</span>
              {dirtyCount > 0 && (
                <span className="text-muted-foreground ml-1">
                  ({dirtyCount} changes)
                </span>
              )}
            </div>
          </div>

          <Separator orientation="vertical" className="h-4" />

          {/* Version Info */}
          <div className="flex items-center gap-2 text-sm">
            <GitBranch className="text-muted-foreground h-3 w-3" />
            <span className="text-muted-foreground">Version</span>
            <Badge variant="outline" className="h-5 text-xs">
              v{currentVersion}
            </Badge>
          </div>

          {/* Last Saved */}
          <div className="text-muted-foreground flex items-center gap-2 text-sm">
            <Clock className="h-3 w-3" />
            <span>{formatLastSaved(lastSaved)}</span>
          </div>

          {/* Hash Status */}
          {currentHash && (
            <div className="flex items-center gap-1">
              <Badge
                variant={hashesMatch ? "outline" : "secondary"}
                className="h-5 font-mono text-[10px]"
              >
                {currentHash.slice(0, 8)}
              </Badge>
            </div>
          )}
        </div>

        {/* Right: Actions */}
        <div className="flex items-center gap-2">
          {/* Conflict Resolution */}
          {hasConflict && onResolveConflict && (
            <Button
              variant="destructive"
              size="sm"
              className="h-8"
              onClick={onResolveConflict}
            >
              <AlertTriangle className="mr-2 h-3 w-3" />
              Resolve Conflict
            </Button>
          )}

          {/* Validate */}
          {onValidate && (
            <Button
              variant="outline"
              size="sm"
              className="h-8"
              onClick={onValidate}
            >
              <CheckCircle className="mr-2 h-3 w-3" />
              Validate
            </Button>
          )}

          {/* Import */}
          <ImportButton onImport={onImport} />

          {/* Export */}
          <Button
            variant="outline"
            size="sm"
            className="h-8"
            onClick={onExport}
          >
            <Download className="mr-2 h-3 w-3" />
            Export
          </Button>

          {/* Save */}
          <Button
            variant={canSave ? "default" : "outline"}
            size="sm"
            className="h-8"
            onClick={onSave}
            disabled={!canSave}
          >
            <Save className="mr-2 h-3 w-3" />
            {saveState === "saving" ? "Saving..." : "Save"}
          </Button>

          {/* Settings Toggle */}
          <Button
            variant="ghost"
            size="sm"
            className="h-8 w-8 p-0"
            onClick={() => setShowSettings(!showSettings)}
          >
            <RefreshCw className="h-3 w-3" />
          </Button>
        </div>
      </div>

      {/* Settings Panel */}
      {showSettings && (
        <>
          <Separator />
          <div className="bg-muted/30 space-y-3 p-3">
            <div className="grid grid-cols-2 gap-4">
              {/* Auto-Save Toggle */}
              <div className="space-y-2">
                <Label className="text-xs font-medium">Auto-Save</Label>
                <div className="flex items-center space-x-2">
                  <Switch
                    id="auto-save"
                    checked={autoSaveEnabled}
                    onCheckedChange={onAutoSaveChange}
                  />
                  <Label
                    htmlFor="auto-save"
                    className="text-muted-foreground text-xs"
                  >
                    Save automatically when idle
                  </Label>
                </div>
              </div>

              {/* Version Strategy */}
              <div className="space-y-2">
                <Label className="text-xs font-medium">Version Strategy</Label>
                <Select
                  value={versionStrategy}
                  onValueChange={onVersionStrategyChange}
                >
                  <SelectTrigger className="h-8">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    {versionStrategyOptions.map((option) => (
                      <SelectItem key={option.value} value={option.value}>
                        <div>
                          <div className="font-medium">{option.label}</div>
                          <div className="text-muted-foreground text-xs">
                            {option.description}
                          </div>
                        </div>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
            </div>

            {/* Preview Next Version */}
            {versionStrategy !== "manual" && (
              <div className="text-muted-foreground text-xs">
                Next save will create version{" "}
                <Badge variant="outline" className="h-4 text-[10px]">
                  v
                  {getNextVersion(
                    currentVersion,
                    versionStrategy,
                    hasUnsavedChanges,
                  )}
                </Badge>
              </div>
            )}

            {/* Status Details */}
            <div className="text-muted-foreground text-xs">
              {config.description}
              {hasUnsavedChanges && autoSaveEnabled && (
                <span> • Auto-save enabled</span>
              )}
            </div>
          </div>
        </>
      )}
    </Card>
  );
}
