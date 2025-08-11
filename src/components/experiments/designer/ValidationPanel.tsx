"use client";

import React, { useState, useMemo } from "react";
import { AlertCircle, AlertTriangle, Info, Filter, X } from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Separator } from "~/components/ui/separator";
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
  className?: string;
}

/* -------------------------------------------------------------------------- */
/* Severity Configuration                                                     */
/* -------------------------------------------------------------------------- */

const severityConfig = {
  error: {
    icon: AlertCircle,
    color: "text-red-600 dark:text-red-400",
    bgColor: "bg-red-50 dark:bg-red-950/20",
    borderColor: "border-red-200 dark:border-red-800",
    badgeVariant: "destructive" as const,
    label: "Error",
  },
  warning: {
    icon: AlertTriangle,
    color: "text-amber-600 dark:text-amber-400",
    bgColor: "bg-amber-50 dark:bg-amber-950/20",
    borderColor: "border-amber-200 dark:border-amber-800",
    badgeVariant: "secondary" as const,
    label: "Warning",
  },
  info: {
    icon: Info,
    color: "text-blue-600 dark:text-blue-400",
    bgColor: "bg-blue-50 dark:bg-blue-950/20",
    borderColor: "border-blue-200 dark:border-blue-800",
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

function getEntityDisplayName(entityId: string): string {
  if (entityId.startsWith("step-")) {
    return `Step ${entityId.replace("step-", "")}`;
  }
  if (entityId.startsWith("action-")) {
    return `Action ${entityId.replace("action-", "")}`;
  }
  return entityId;
}

/* -------------------------------------------------------------------------- */
/* Issue Item Component                                                       */
/* -------------------------------------------------------------------------- */

interface IssueItemProps {
  issue: ValidationIssue & { entityId: string; index: number };
  onIssueClick?: (issue: ValidationIssue) => void;
  onIssueClear?: (entityId: string, issueIndex: number) => void;
}

function IssueItem({ issue, onIssueClick, onIssueClear }: IssueItemProps) {
  const config = severityConfig[issue.severity];
  const IconComponent = config.icon;

  return (
    <div
      className={cn(
        "group flex items-start gap-3 rounded-md border p-3 transition-colors",
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
            <p className="text-sm leading-relaxed">{issue.message}</p>

            <div className="mt-1 flex flex-wrap items-center gap-1">
              <Badge variant={config.badgeVariant} className="h-4 text-[10px]">
                {config.label}
              </Badge>

              {issue.category && (
                <Badge variant="outline" className="h-4 text-[10px] capitalize">
                  {issue.category}
                </Badge>
              )}

              <Badge variant="secondary" className="h-4 text-[10px]">
                {getEntityDisplayName(issue.entityId)}
              </Badge>

              {issue.field && (
                <Badge variant="outline" className="h-4 text-[10px]">
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
  className,
}: ValidationPanelProps) {
  const [severityFilter, setSeverityFilter] = useState<
    "all" | "error" | "warning" | "info"
  >("all");
  const [categoryFilter, setCategoryFilter] = useState<
    "all" | "structural" | "parameter" | "semantic" | "execution"
  >("all");

  // Flatten and filter issues
  const flatIssues = useMemo(() => {
    const flat = flattenIssues(issues);

    return flat.filter((issue) => {
      if (severityFilter !== "all" && issue.severity !== severityFilter) {
        return false;
      }
      if (categoryFilter !== "all" && issue.category !== categoryFilter) {
        return false;
      }
      return true;
    });
  }, [issues, severityFilter, categoryFilter]);

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

  // Available categories
  const availableCategories = useMemo(() => {
    const flat = flattenIssues(issues);
    const categories = new Set(flat.map((i) => i.category).filter(Boolean));
    return Array.from(categories) as Array<
      "structural" | "parameter" | "semantic" | "execution"
    >;
  }, [issues]);

  return (
    <Card className={cn("h-[calc(100vh-12rem)]", className)}>
      <CardHeader className="pb-2">
        <CardTitle className="flex items-center justify-between text-sm">
          <div className="flex items-center gap-2">
            <AlertCircle className="h-4 w-4" />
            Validation Issues
          </div>
          <div className="flex items-center gap-1">
            {counts.error > 0 && (
              <Badge variant="destructive" className="h-4 text-[10px]">
                {counts.error}
              </Badge>
            )}
            {counts.warning > 0 && (
              <Badge variant="secondary" className="h-4 text-[10px]">
                {counts.warning}
              </Badge>
            )}
            {counts.info > 0 && (
              <Badge variant="outline" className="h-4 text-[10px]">
                {counts.info}
              </Badge>
            )}
          </div>
        </CardTitle>
      </CardHeader>

      <CardContent className="p-0">
        {/* Filters */}
        {counts.total > 0 && (
          <>
            <div className="border-b p-3">
              <div className="flex flex-wrap gap-2">
                {/* Severity Filter */}
                <div className="flex items-center gap-1">
                  <Filter className="text-muted-foreground h-3 w-3" />
                  <Button
                    variant={severityFilter === "all" ? "default" : "ghost"}
                    size="sm"
                    className="h-6 px-2 text-xs"
                    onClick={() => setSeverityFilter("all")}
                  >
                    All ({counts.total})
                  </Button>
                  {counts.error > 0 && (
                    <Button
                      variant={
                        severityFilter === "error" ? "destructive" : "ghost"
                      }
                      size="sm"
                      className="h-6 px-2 text-xs"
                      onClick={() => setSeverityFilter("error")}
                    >
                      Errors ({counts.error})
                    </Button>
                  )}
                  {counts.warning > 0 && (
                    <Button
                      variant={
                        severityFilter === "warning" ? "secondary" : "ghost"
                      }
                      size="sm"
                      className="h-6 px-2 text-xs"
                      onClick={() => setSeverityFilter("warning")}
                    >
                      Warnings ({counts.warning})
                    </Button>
                  )}
                  {counts.info > 0 && (
                    <Button
                      variant={severityFilter === "info" ? "outline" : "ghost"}
                      size="sm"
                      className="h-6 px-2 text-xs"
                      onClick={() => setSeverityFilter("info")}
                    >
                      Info ({counts.info})
                    </Button>
                  )}
                </div>

                {/* Category Filter */}
                {availableCategories.length > 0 && (
                  <>
                    <Separator orientation="vertical" className="h-6" />
                    <div className="flex items-center gap-1">
                      <Button
                        variant={categoryFilter === "all" ? "default" : "ghost"}
                        size="sm"
                        className="h-6 px-2 text-xs"
                        onClick={() => setCategoryFilter("all")}
                      >
                        All Categories
                      </Button>
                      {availableCategories.map((category) => (
                        <Button
                          key={category}
                          variant={
                            categoryFilter === category ? "outline" : "ghost"
                          }
                          size="sm"
                          className="h-6 px-2 text-xs capitalize"
                          onClick={() => setCategoryFilter(category)}
                        >
                          {category}
                        </Button>
                      ))}
                    </div>
                  </>
                )}
              </div>
            </div>
          </>
        )}

        {/* Issues List */}
        <ScrollArea className="h-full">
          <div className="p-3">
            {counts.total === 0 ? (
              <div className="py-8 text-center">
                <div className="mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full bg-green-100 dark:bg-green-950/20">
                  <Info className="h-4 w-4 text-green-600 dark:text-green-400" />
                </div>
                <p className="text-sm font-medium text-green-700 dark:text-green-300">
                  No validation issues
                </p>
                <p className="text-muted-foreground text-xs">
                  Your experiment design looks good!
                </p>
              </div>
            ) : flatIssues.length === 0 ? (
              <div className="py-8 text-center">
                <div className="bg-muted mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full">
                  <Filter className="h-4 w-4" />
                </div>
                <p className="text-sm font-medium">No issues match filters</p>
                <p className="text-muted-foreground text-xs">
                  Try adjusting your filter criteria
                </p>
              </div>
            ) : (
              <div className="space-y-2">
                {flatIssues.map((issue) => (
                  <IssueItem
                    key={`${issue.entityId}-${issue.index}`}
                    issue={issue}
                    onIssueClick={onIssueClick}
                    onIssueClear={onIssueClear}
                  />
                ))}
              </div>
            )}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  );
}
