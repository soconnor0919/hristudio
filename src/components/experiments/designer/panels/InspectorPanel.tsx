"use client";

import React, { useMemo, useState, useCallback } from "react";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";
import { ScrollArea } from "~/components/ui/scroll-area";
import { cn } from "~/lib/utils";
import { useDesignerStore } from "../state/store";
import { actionRegistry } from "../ActionRegistry";
import { PropertiesPanel } from "../PropertiesPanel";
import { ValidationPanel } from "../ValidationPanel";
import { DependencyInspector } from "../DependencyInspector";
import type {
  ExperimentStep,
  ExperimentAction,
} from "~/lib/experiment-designer/types";
import {
  Settings,
  AlertTriangle,
  GitBranch,
  PackageSearch,
} from "lucide-react";

/**
 * InspectorPanel
 *
 * Collapsible / dockable right-side panel presenting contextual information:
 *  - Properties (Step or Action)
 *  - Validation Issues
 *  - Dependencies (action definitions & drift)
 *
 * This is a skeleton implementation bridging existing sub-panels. Future
 * enhancements (planned):
 *  - Lazy loading heavy panels
 *  - Diff / reconciliation modal for action signature drift
 *  - Parameter schema visualization popovers
 *  - Step / Action navigation breadcrumbs
 *  - Split / pop-out inspector
 */

export interface InspectorPanelProps {
  className?: string;
  /**
   * Optional forced active tab; if undefined, internal state manages it.
   */
  activeTab?: "properties" | "issues" | "dependencies";
  /**
   * Called when user changes tab (only if activeTab not externally controlled).
   */
  onTabChange?: (tab: "properties" | "issues" | "dependencies") => void;
  /**
   * Whether to auto-switch to properties tab when selection changes.
   */
  autoFocusOnSelection?: boolean;
}

export function InspectorPanel({
  className,
  activeTab,
  onTabChange,
  autoFocusOnSelection = true,
}: InspectorPanelProps) {
  /* ------------------------------------------------------------------------ */
  /* Store Selectors                                                          */
  /* ------------------------------------------------------------------------ */
  const steps = useDesignerStore((s) => s.steps);
  const selectedStepId = useDesignerStore((s) => s.selectedStepId);
  const selectedActionId = useDesignerStore((s) => s.selectedActionId);
  const validationIssues = useDesignerStore((s) => s.validationIssues);
  const actionSignatureDrift = useDesignerStore((s) => s.actionSignatureDrift);

  const upsertStep = useDesignerStore((s) => s.upsertStep);
  const upsertAction = useDesignerStore((s) => s.upsertAction);
  const selectStep = useDesignerStore((s) => s.selectStep);
  const selectAction = useDesignerStore((s) => s.selectAction);

  /* ------------------------------------------------------------------------ */
  /* Derived Selection                                                        */
  /* ------------------------------------------------------------------------ */
  const selectedStep: ExperimentStep | undefined = useMemo(
    () => steps.find((s) => s.id === selectedStepId),
    [steps, selectedStepId],
  );

  const selectedAction: ExperimentAction | undefined = useMemo(
    () => selectedStep?.actions.find((a) => a.id === selectedActionId),
    [selectedStep, selectedActionId],
  );

  /* ------------------------------------------------------------------------ */
  /* Local Active Tab State (uncontrolled mode)                               */
  /* ------------------------------------------------------------------------ */
  const INSPECTOR_TAB_STORAGE_KEY = "hristudio-designer-inspector-tab-v1";
  const [internalTab, setInternalTab] = useState<
    "properties" | "issues" | "dependencies"
  >(() => {
    try {
      const raw =
        typeof window !== "undefined"
          ? localStorage.getItem(INSPECTOR_TAB_STORAGE_KEY)
          : null;
      if (raw === "properties" || raw === "issues" || raw === "dependencies") {
        return raw;
      }
    } catch {
      /* noop */
    }
    if (selectedStepId) return "properties";
    return "issues";
  });

  const effectiveTab = activeTab ?? internalTab;

  // Auto switch to properties on new selection if permitted
  React.useEffect(() => {
    if (!autoFocusOnSelection) return;
    if (selectedStepId || selectedActionId) {
      setInternalTab("properties");
      // Scroll properties panel to top and focus first field
      requestAnimationFrame(() => {
        const activeTabpanel = document.querySelector(
          '[role="tabpanel"][data-state="active"]',
        );
        if (!(activeTabpanel instanceof HTMLElement)) return;
        const viewportEl = activeTabpanel.querySelector(
          '[data-slot="scroll-area-viewport"]',
        );
        if (viewportEl instanceof HTMLElement) {
          viewportEl.scrollTop = 0;
          const firstField = viewportEl.querySelector(
            "input, select, textarea, button",
          );
          if (firstField instanceof HTMLElement) {
            firstField.focus();
          }
        }
      });
    }
  }, [selectedStepId, selectedActionId, autoFocusOnSelection]);

  const handleTabChange = useCallback(
    (val: string) => {
      if (val === "properties" || val === "issues" || val === "dependencies") {
        if (activeTab) {
          onTabChange?.(val);
        } else {
          setInternalTab(val);
          try {
            localStorage.setItem(INSPECTOR_TAB_STORAGE_KEY, val);
          } catch {
            /* noop */
          }
        }
      }
    },
    [activeTab, onTabChange],
  );

  /* ------------------------------------------------------------------------ */
  /* Mutation Handlers (pass-through to store)                                */
  /* ------------------------------------------------------------------------ */
  const handleActionUpdate = useCallback(
    (stepId: string, actionId: string, updates: Partial<ExperimentAction>) => {
      const step = steps.find((s) => s.id === stepId);
      if (!step) return;
      const action = step.actions.find((a) => a.id === actionId);
      if (!action) return;
      upsertAction(stepId, { ...action, ...updates });
    },
    [steps, upsertAction],
  );

  const handleStepUpdate = useCallback(
    (stepId: string, updates: Partial<ExperimentStep>) => {
      const step = steps.find((s) => s.id === stepId);
      if (!step) return;
      upsertStep({ ...step, ...updates });
    },
    [steps, upsertStep],
  );

  /* ------------------------------------------------------------------------ */
  /* Counts & Badges                                                          */
  /* ------------------------------------------------------------------------ */
  const issueCount = useMemo(
    () =>
      Object.values(validationIssues).reduce((sum, arr) => sum + arr.length, 0),
    [validationIssues],
  );

  const driftCount = actionSignatureDrift.size;

  /* ------------------------------------------------------------------------ */
  /* Empty States                                                             */
  /* ------------------------------------------------------------------------ */
  const propertiesEmpty = !selectedStep && !selectedAction;

  /* ------------------------------------------------------------------------ */
  /* Render                                                                   */
  /* ------------------------------------------------------------------------ */
  return (
    <div
      className={cn(
        "bg-background/40 border-border relative flex h-full min-w-0 flex-col overflow-hidden border-l backdrop-blur-sm",
        className,
      )}
      style={{ contain: "layout paint size" }}
      role="complementary"
      aria-label="Inspector panel"
    >
      {/* Tab Header */}
      <div className="border-b px-2 py-1.5">
        <Tabs
          value={effectiveTab}
          onValueChange={handleTabChange}
          className="w-full"
        >
          <TabsList className="flex h-9 w-full items-center gap-1 overflow-hidden">
            <TabsTrigger
              value="properties"
              className="flex min-w-0 flex-1 items-center justify-center gap-1 truncate text-[11px]"
              title="Properties (Step / Action)"
            >
              <Settings className="h-3 w-3 flex-shrink-0" />
              <span className="hidden sm:inline">Props</span>
            </TabsTrigger>
            <TabsTrigger
              value="issues"
              className="flex min-w-0 flex-1 items-center justify-center gap-1 truncate text-[11px]"
              title="Validation Issues"
            >
              <AlertTriangle className="h-3 w-3 flex-shrink-0" />
              <span className="hidden sm:inline">
                Issues{issueCount > 0 ? ` (${issueCount})` : ""}
              </span>
              {issueCount > 0 && (
                <span className="xs:hidden text-amber-600 dark:text-amber-400">
                  {issueCount}
                </span>
              )}
            </TabsTrigger>
            <TabsTrigger
              value="dependencies"
              className="flex min-w-0 flex-1 items-center justify-center gap-1 truncate text-[11px]"
              title="Dependencies / Drift"
            >
              <PackageSearch className="h-3 w-3 flex-shrink-0" />
              <span className="hidden sm:inline">
                Deps{driftCount > 0 ? ` (${driftCount})` : ""}
              </span>
              {driftCount > 0 && (
                <span className="xs:hidden text-purple-600 dark:text-purple-400">
                  {driftCount}
                </span>
              )}
            </TabsTrigger>
          </TabsList>
        </Tabs>
      </div>

      {/* Content */}
      <div className="flex min-h-0 flex-1 flex-col">
        {/*
          Force consistent width for tab bodies to prevent reflow when
          switching between content with different intrinsic widths.
        */}
        <Tabs value={effectiveTab}>
          {/* Properties */}
          <TabsContent
            value="properties"
            className="m-0 flex min-h-0 flex-1 flex-col data-[state=inactive]:hidden"
          >
            {propertiesEmpty ? (
              <div className="text-muted-foreground flex h-full flex-col items-center justify-center gap-3 p-4 text-center">
                <div className="bg-background/70 flex h-10 w-10 items-center justify-center rounded-full border">
                  <GitBranch className="h-5 w-5" />
                </div>
                <div className="space-y-1">
                  <p className="text-sm font-medium">Select a Step or Action</p>
                  <p className="text-xs">
                    Click within the flow to edit its properties here.
                  </p>
                </div>
              </div>
            ) : (
              <ScrollArea className="flex-1">
                <div className="w-full px-3 py-3">
                  <PropertiesPanel
                    design={{
                      id: "design",
                      name: "Design",
                      description: "",
                      version: 1,
                      steps,
                      lastSaved: new Date(),
                    }}
                    selectedStep={selectedStep}
                    selectedAction={selectedAction}
                    onActionUpdate={handleActionUpdate}
                    onStepUpdate={handleStepUpdate}
                  />
                </div>
              </ScrollArea>
            )}
          </TabsContent>

          {/* Issues */}
          <TabsContent
            value="issues"
            className="m-0 flex min-h-0 flex-1 flex-col data-[state=inactive]:hidden"
          >
            <ValidationPanel
              issues={validationIssues}
              entityLabelForId={(entityId) => {
                if (entityId.startsWith("action-")) {
                  for (const s of steps) {
                    const a = s.actions.find((x) => x.id === entityId);
                    if (a) return `${a.name} • ${s.name}`;
                  }
                }
                if (entityId.startsWith("step-")) {
                  const st = steps.find((s) => s.id === entityId);
                  if (st) return st.name;
                }
                return "Unknown";
              }}
              onIssueClick={(issue) => {
                if (issue.stepId) {
                  selectStep(issue.stepId);
                  if (issue.actionId) {
                    selectAction(issue.stepId, issue.actionId);
                  } else {
                    selectAction(issue.stepId, undefined);
                  }
                  if (autoFocusOnSelection) {
                    handleTabChange("properties");
                  }
                }
              }}
            />
          </TabsContent>

          {/* Dependencies */}
          <TabsContent
            value="dependencies"
            className="m-0 flex min-h-0 flex-1 flex-col data-[state=inactive]:hidden"
          >
            <ScrollArea className="flex-1">
              <div className="w-full px-3 py-3">
                <DependencyInspector
                  steps={steps}
                  actionSignatureDrift={actionSignatureDrift}
                  actionDefinitions={actionRegistry.getAllActions()}
                  onReconcileAction={(actionId) => {
                    // Placeholder: future diff modal / signature update

                    console.log("Reconcile TODO for action:", actionId);
                  }}
                  onRefreshDependencies={() => {
                    console.log("Refresh dependencies TODO");
                  }}
                  onInstallPlugin={(pluginId) => {
                    console.log("Install plugin TODO:", pluginId);
                  }}
                />
              </div>
            </ScrollArea>
          </TabsContent>
        </Tabs>
      </div>

      {/* Footer (lightweight) */}
      <div className="text-muted-foreground border-t px-3 py-1.5 text-[10px]">
        Inspector • {selectedStep ? "Step" : selectedAction ? "Action" : "None"}{" "}
        • {issueCount} issues • {driftCount} drift
      </div>
    </div>
  );
}

export default InspectorPanel;
