"use client";

import React, { useState, useMemo } from "react";
import {
  AlertCircle,
  AlertTriangle,
  Info,
  Filter,
  X,
  Search,
  CheckCircle2,
} from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";

import { Input } from "~/components/ui/input";
import { cn } from "~/lib/utils";

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

export interface ValidationIssue {
  severity: "error" | "warning" | "info";
  message: string;
  category?: "structural" | "parameter" | "semantic" | "execution";
  field?: string;
  actionId?: string;
  stepId?: string;
}

export interface ValidationPanelProps {
  /**
   * Map of entity ID to validation issues for that entity.
   */
  issues: Record<string, ValidationIssue[]>;
  /**
   * Called when user clicks on an issue to navigate to the problematic entity.
   */
  onIssueClick?: (issue: ValidationIssue) => void;
  /**
   * Called to clear a specific issue (if clearable).
   */
  onIssueClear?: (entityId: string, issueIndex: number) => void;
  /**
   * Called to clear all issues for an entity.
   */
  onEntityClear?: (entityId: string) => void;
  /**
   * Optional function to map entity IDs to human-friendly names (e.g., step/action names).
   */
  entityLabelForId?: (entityId: string) => string;
  className?: string;
}

/* -------------------------------------------------------------------------- */
/* Severity Configuration                                                     */
/* -------------------------------------------------------------------------- */

const severityConfig = {
  error: {
    icon: AlertCircle,
    color: "text-red-600 dark:text-red-400",
    bgColor: "bg-red-100 dark:bg-red-950/60",
    borderColor: "border-red-300 dark:border-red-700",
    badgeVariant: "destructive" as const,
    label: "Error",
  },
  warning: {
    icon: AlertTriangle,
    color: "text-amber-600 dark:text-amber-400",
    bgColor: "bg-amber-100 dark:bg-amber-950/60",
    borderColor: "border-amber-300 dark:border-amber-700",
    badgeVariant: "secondary" as const,
    label: "Warning",
  },
  info: {
    icon: Info,
    color: "text-blue-600 dark:text-blue-400",
    bgColor: "bg-blue-100 dark:bg-blue-950/60",
    borderColor: "border-blue-300 dark:border-blue-700",
    badgeVariant: "outline" as const,
    label: "Info",
  },
} as const;

/* -------------------------------------------------------------------------- */
/* Utility Functions                                                          */
/* -------------------------------------------------------------------------- */

function flattenIssues(issuesMap: Record<string, ValidationIssue[]>) {
  const flattened: Array<
    ValidationIssue & { entityId: string; index: number }
  > = [];

  Object.entries(issuesMap).forEach(([entityId, issues]) => {
    issues.forEach((issue, index) => {
      flattened.push({ ...issue, entityId, index });
    });
  });

  return flattened;
}



/* -------------------------------------------------------------------------- */
/* Issue Item Component                                                       */
/* -------------------------------------------------------------------------- */

interface IssueItemProps {
  issue: ValidationIssue & { entityId: string; index: number };
  onIssueClick?: (issue: ValidationIssue) => void;
  onIssueClear?: (entityId: string, issueIndex: number) => void;
  entityLabelForId?: (entityId: string) => string;
}

function IssueItem({
  issue,
  onIssueClick,
  onIssueClear,
  entityLabelForId,
}: IssueItemProps) {
  const config = severityConfig[issue.severity];
  const IconComponent = config.icon;

  return (
    <div
      className={cn(
        "group flex w-full max-w-full min-w-0 items-start gap-2 rounded-md border p-2 break-words transition-colors",
        config.borderColor,
        config.bgColor,
        onIssueClick && "cursor-pointer hover:shadow-sm",
      )}
      onClick={() => onIssueClick?.(issue)}
    >
      <div className="flex-shrink-0">
        <IconComponent className={cn("h-4 w-4", config.color)} />
      </div>

      <div className="min-w-0 flex-1">
        <div className="flex items-start justify-between gap-2">
          <div className="min-w-0 flex-1">
            <p className="text-[12px] leading-snug break-words whitespace-normal">
              {issue.message}
            </p>

            <div className="mt-1 flex flex-wrap items-center gap-1">
              <Badge variant={config.badgeVariant} className="text-[10px]">
                {config.label}
              </Badge>

              {issue.category && (
                <Badge variant="outline" className="text-[10px] capitalize">
                  {issue.category}
                </Badge>
              )}

              <Badge
                variant="secondary"
                className="max-w-full text-[10px] break-words whitespace-normal"
              >
                {entityLabelForId?.(issue.entityId) ?? "Unknown"}
              </Badge>

              {issue.field && (
                <Badge variant="outline" className="text-[10px]">
                  {issue.field}
                </Badge>
              )}
            </div>
          </div>

          {onIssueClear && (
            <Button
              variant="ghost"
              size="sm"
              className="h-5 w-5 p-0 opacity-0 transition-opacity group-hover:opacity-100"
              onClick={(e) => {
                e.stopPropagation();
                onIssueClear(issue.entityId, issue.index);
              }}
            >
              <X className="h-3 w-3" />
            </Button>
          )}
        </div>
      </div>
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* ValidationPanel Component                                                  */
/* -------------------------------------------------------------------------- */

export function ValidationPanel({
  issues,
  onIssueClick,
  onIssueClear,
  onEntityClear: _onEntityClear,
  entityLabelForId,
  className,
}: ValidationPanelProps) {
  const [severityFilter, setSeverityFilter] = useState<
    "all" | "error" | "warning" | "info"
  >("all");
  const [categoryFilter] = useState<
    "all" | "structural" | "parameter" | "semantic" | "execution"
  >("all");
  const [search, setSearch] = useState("");

  // Flatten and filter issues
  const flatIssues = useMemo(() => {
    const flat = flattenIssues(issues);
    const q = search.trim().toLowerCase();
    return flat.filter((issue) => {
      if (severityFilter !== "all" && issue.severity !== severityFilter)
        return false;
      if (categoryFilter !== "all" && issue.category !== categoryFilter)
        return false;
      if (!q) return true;
      const hay =
        `${issue.message} ${issue.field ?? ""} ${issue.category ?? ""} ${issue.entityId}`.toLowerCase();
      return hay.includes(q);
    });
  }, [issues, severityFilter, categoryFilter, search]);

  // Count by severity
  const counts = useMemo(() => {
    const flat = flattenIssues(issues);
    return {
      total: flat.length,
      error: flat.filter((i) => i.severity === "error").length,
      warning: flat.filter((i) => i.severity === "warning").length,
      info: flat.filter((i) => i.severity === "info").length,
    };
  }, [issues]);

  React.useEffect(() => {
    // Debug: surface validation state to console

    console.log("[ValidationPanel] issues", issues, { flatIssues, counts });
  }, [issues, flatIssues, counts]);



  return (
    <div
      className={cn(
        "flex h-full min-h-0 min-w-0 flex-col overflow-hidden",
        className,
      )}
    >
      {/* Header (emulate ActionLibraryPanel) */}
      <div className="bg-background/60 border-b p-2">
        <div className="relative mb-2">
          <Search className="text-muted-foreground absolute top-1/2 left-2 h-3.5 w-3.5 -translate-y-1/2" />
          <Input
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            placeholder="Search issues"
            className="h-8 w-full pl-7 text-xs"
            aria-label="Search issues"
          />
        </div>

        <div className="mb-2 grid grid-cols-2 gap-1">
          <Button
            variant={severityFilter === "all" ? "default" : "ghost"}
            size="sm"
            className="h-7 justify-start gap-1 text-[11px]"
            onClick={() => setSeverityFilter("all")}
            aria-pressed={severityFilter === "all"}
          >
            <Filter className="h-3 w-3" /> All
            <span className="ml-auto text-[10px] font-normal opacity-80">
              {counts.total}
            </span>
          </Button>
          <Button
            variant={severityFilter === "error" ? "default" : "ghost"}
            size="sm"
            className={cn(
              "h-7 justify-start gap-1 text-[11px]",
              severityFilter === "error" &&
                "bg-red-600 text-white hover:opacity-90",
            )}
            onClick={() => setSeverityFilter("error")}
            aria-pressed={severityFilter === "error"}
          >
            <AlertCircle className="h-3 w-3" /> Errors
            <span className="ml-auto text-[10px] font-normal opacity-80">
              {counts.error}
            </span>
          </Button>
          <Button
            variant={severityFilter === "warning" ? "default" : "ghost"}
            size="sm"
            className={cn(
              "h-7 justify-start gap-1 text-[11px]",
              severityFilter === "warning" &&
                "bg-amber-500 text-white hover:opacity-90",
            )}
            onClick={() => setSeverityFilter("warning")}
            aria-pressed={severityFilter === "warning"}
          >
            <AlertTriangle className="h-3 w-3" /> Warn
            <span className="ml-auto text-[10px] font-normal opacity-80">
              {counts.warning}
            </span>
          </Button>
          <Button
            variant={severityFilter === "info" ? "default" : "ghost"}
            size="sm"
            className={cn(
              "h-7 justify-start gap-1 text-[11px]",
              severityFilter === "info" &&
                "bg-blue-600 text-white hover:opacity-90",
            )}
            onClick={() => setSeverityFilter("info")}
            aria-pressed={severityFilter === "info"}
          >
            <Info className="h-3 w-3" /> Info
            <span className="ml-auto text-[10px] font-normal opacity-80">
              {counts.info}
            </span>
          </Button>
        </div>
      </div>

      {/* Issues List */}
      <div className="min-h-0 flex-1 overflow-x-hidden overflow-y-auto">
        <div className="flex min-w-0 flex-col gap-2 p-2 pr-2">
          {counts.total === 0 ? (
            <div className="py-8 text-center">
              <div className="mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full bg-emerald-100 dark:bg-emerald-950/20">
                <CheckCircle2 className="h-4 w-4 text-emerald-600 dark:text-emerald-400" />
              </div>
              <p className="text-sm font-medium text-emerald-700 dark:text-emerald-300">
                All clear — no issues
              </p>
              <p className="text-muted-foreground text-xs">
                Validate again after changes.
              </p>
            </div>
          ) : flatIssues.length === 0 ? (
            <div className="py-8 text-center">
              <div className="bg-muted mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full">
                <Filter className="h-4 w-4" />
              </div>
              <p className="text-sm font-medium">No issues match filters</p>
              <p className="text-muted-foreground text-xs">
                Adjust your filters
              </p>
            </div>
          ) : (
            flatIssues.map((issue) => (
              <IssueItem
                key={`${issue.entityId}-${issue.index}`}
                issue={issue}
                onIssueClick={onIssueClick}
                onIssueClear={onIssueClear}
                entityLabelForId={entityLabelForId}
              />
            ))
          )}
        </div>
      </div>
    </div>
  );
}
