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
  ListChecks,
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
    () =>
      selectedStep?.actions.find(
        (a) => a.id === selectedActionId,
      ) as ExperimentAction | undefined,
    [selectedStep, selectedActionId],
  );

  /* ------------------------------------------------------------------------ */
  /* Local Active Tab State (uncontrolled mode)                               */
  /* ------------------------------------------------------------------------ */
  const [internalTab, setInternalTab] = useState<
    "properties" | "issues" | "dependencies"
  >(() => {
    if (selectedStepId) return "properties";
    return "issues";
  });

  const effectiveTab = activeTab ?? internalTab;

  // Auto switch to properties on new selection if permitted
  React.useEffect(() => {
    if (!autoFocusOnSelection) return;
    if (selectedStepId || selectedActionId) {
      setInternalTab("properties");
    }
  }, [selectedStepId, selectedActionId, autoFocusOnSelection]);

  const handleTabChange = useCallback(
    (val: string) => {
      if (
        val === "properties" ||
        val === "issues" ||
        val === "dependencies"
      ) {
        if (activeTab) {
          onTabChange?.(val);
        } else {
          setInternalTab(val);
        }
      }
    },
    [activeTab, onTabChange],
  );

  /* ------------------------------------------------------------------------ */
  /* Mutation Handlers (pass-through to store)                                */
  /* ------------------------------------------------------------------------ */
  const handleActionUpdate = useCallback(
    (
      stepId: string,
      actionId: string,
      updates: Partial<ExperimentAction>,
    ) => {
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
      Object.values(validationIssues).reduce(
        (sum, arr) => sum + arr.length,
        0,
      ),
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
        "flex h-full flex-col border-l bg-background/40 backdrop-blur-sm",
        className,
      )}
    >
      {/* Tab Header */}
      <div className="border-b px-2 py-1.5">
        <Tabs
          value={effectiveTab}
          onValueChange={handleTabChange}
          className="w-full"
        >
          <TabsList className="grid h-8 grid-cols-3">
            <TabsTrigger
              value="properties"
              className="flex items-center gap-1 text-[11px]"
              title="Properties (Step / Action)"
            >
              <Settings className="h-3 w-3" />
              <span className="hidden sm:inline">Props</span>
            </TabsTrigger>
            <TabsTrigger
              value="issues"
              className="flex items-center gap-1 text-[11px]"
              title="Validation Issues"
            >
              <AlertTriangle className="h-3 w-3" />
              <span className="hidden sm:inline">
                Issues{issueCount > 0 ? ` (${issueCount})` : ""}
              </span>
              {issueCount > 0 && (
                <span className="text-amber-600 dark:text-amber-400 sm:hidden">
                  {issueCount}
                </span>
              )}
            </TabsTrigger>
            <TabsTrigger
              value="dependencies"
              className="flex items-center gap-1 text-[11px]"
              title="Dependencies / Drift"
            >
              <PackageSearch className="h-3 w-3" />
              <span className="hidden sm:inline">
                Deps{driftCount > 0 ? ` (${driftCount})` : ""}
              </span>
              {driftCount > 0 && (
                <span className="text-purple-600 dark:text-purple-400 sm:hidden">
                  {driftCount}
                </span>
              )}
            </TabsTrigger>
          </TabsList>
        </Tabs>
      </div>

      {/* Content */}
      <div className="flex min-h-0 flex-1 flex-col">
        <Tabs value={effectiveTab}>
          {/* Properties */}
            <TabsContent
              value="properties"
              className="m-0 flex h-full flex-col data-[state=inactive]:hidden"
            >
              {propertiesEmpty ? (
                <div className="text-muted-foreground flex h-full flex-col items-center justify-center gap-3 p-4 text-center">
                  <div className="flex h-10 w-10 items-center justify-center rounded-full border bg-background/70">
                    <GitBranch className="h-5 w-5" />
                  </div>
                  <div className="space-y-1">
                    <p className="text-sm font-medium">
                      Select a Step or Action
                    </p>
                    <p className="text-xs">
                      Click within the flow to edit its properties here.
                    </p>
                  </div>
                </div>
              ) : (
                <ScrollArea className="flex-1">
                  <div className="p-3">
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
            className="m-0 flex h-full flex-col data-[state=inactive]:hidden"
          >
            <ScrollArea className="flex-1">
              <div className="p-3">
                <ValidationPanel
                  issues={validationIssues}
                  onIssueClick={(issue) => {
                    if (issue.stepId) {
                      selectStep(issue.stepId);
                      if (issue.actionId) {
                        selectAction(issue.stepId, issue.actionId);
                        if (autoFocusOnSelection) {
                          handleTabChange("properties");
                        }
                      }
                    }
                  }}
                />
              </div>
            </ScrollArea>
          </TabsContent>

          {/* Dependencies */}
          <TabsContent
            value="dependencies"
            className="m-0 flex h-full flex-col data-[state=inactive]:hidden"
          >
            <ScrollArea className="flex-1">
              <div className="p-3">
                <DependencyInspector
                  steps={steps}
                  actionSignatureDrift={actionSignatureDrift}
                  actionDefinitions={actionRegistry.getAllActions()}
                  onReconcileAction={(actionId) => {
                    // Placeholder: future diff modal / signature update
                    // eslint-disable-next-line no-console
                    console.log("Reconcile TODO for action:", actionId);
                  }}
                  onRefreshDependencies={() => {
                    // eslint-disable-next-line no-console
                    console.log("Refresh dependencies TODO");
                  }}
                  onInstallPlugin={(pluginId) => {
                    // eslint-disable-next-line no-console
                    console.log("Install plugin TODO:", pluginId);
                  }}
                />
              </div>
            </ScrollArea>
          </TabsContent>
        </Tabs>
      </div>

      {/* Footer (lightweight) */}
      <div className="border-t px-3 py-1.5 text-[10px] text-muted-foreground">
        Inspector • {selectedStep ? "Step" : selectedAction ? "Action" : "None"}{" "}
        • {issueCount} issues • {driftCount} drift
      </div>
    </div>
  );
}

export default InspectorPanel;
