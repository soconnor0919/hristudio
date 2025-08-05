"use client";

import React, { useState, useCallback, useEffect, useMemo } from "react";
import {
  ReactFlow,
  Background,
  Controls,
  MiniMap,
  useNodesState,
  useEdgesState,
  addEdge,
  type Node,
  type Edge,
  type Connection,
  type NodeTypes,
  MarkerType,
  Panel,
  Handle,
  Position,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import "./flow-theme.css";
import {
  Bot,
  Users,
  Shuffle,
  GitBranch,
  Play,
  Zap,
  Eye,
  Clock,
  Plus,
  Save,
  Undo,
  Redo,
  Download,
  Upload,
  Settings,
  Trash2,
  Copy,
  Edit3,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Label } from "~/components/ui/label";
import { Input } from "~/components/ui/input";
import { Textarea } from "~/components/ui/textarea";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Separator } from "~/components/ui/separator";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
  DropdownMenuSeparator,
} from "~/components/ui/dropdown-menu";
import {
  Sheet,
  SheetContent,
  SheetDescription,
  SheetHeader,
  SheetTitle,
  SheetTrigger,
} from "~/components/ui/sheet";
import { toast } from "sonner";

// Types
type StepType =
  | "wizard"
  | "robot"
  | "parallel"
  | "conditional"
  | "start"
  | "end";
type ActionType =
  | "speak"
  | "move"
  | "gesture"
  | "look_at"
  | "wait"
  | "instruction"
  | "question"
  | "observe";

interface FlowAction {
  id: string;
  type: ActionType;
  name: string;
  parameters: Record<string, unknown>;
  order: number;
}

interface FlowStep {
  id: string;
  type: StepType;
  name: string;
  description?: string;
  duration?: number;
  actions: FlowAction[];
  parameters: Record<string, unknown>;
  position: { x: number; y: number };
}

interface FlowDesign {
  id: string;
  name: string;
  description?: string;
  steps: FlowStep[];
  version: number;
  lastSaved: Date;
}

// Step type configurations
const stepTypeConfig = {
  start: {
    label: "Start",
    icon: Play,
    color: "#10b981",
    bgColor: "bg-green-500",
    lightColor: "bg-green-50 border-green-200",
    description: "Experiment starting point",
  },
  wizard: {
    label: "Wizard Action",
    icon: Users,
    color: "#3b82f6",
    bgColor: "bg-blue-500",
    lightColor: "bg-blue-50 border-blue-200",
    description: "Actions performed by human wizard",
  },
  robot: {
    label: "Robot Action",
    icon: Bot,
    color: "#8b5cf6",
    bgColor: "bg-purple-500",
    lightColor: "bg-purple-50 border-purple-200",
    description: "Actions performed by robot",
  },
  parallel: {
    label: "Parallel Steps",
    icon: Shuffle,
    color: "#f59e0b",
    bgColor: "bg-amber-500",
    lightColor: "bg-amber-50 border-amber-200",
    description: "Execute multiple steps simultaneously",
  },
  conditional: {
    label: "Conditional Branch",
    icon: GitBranch,
    color: "#ef4444",
    bgColor: "bg-red-500",
    lightColor: "bg-red-50 border-red-200",
    description: "Branching logic based on conditions",
  },
  end: {
    label: "End",
    icon: Play,
    color: "#6b7280",
    bgColor: "bg-gray-500",
    lightColor: "bg-gray-50 border-gray-200",
    description: "Experiment end point",
  },
};

const actionTypeConfig = {
  speak: {
    label: "Speak",
    icon: Play,
    description: "Text-to-speech output",
    defaultParams: { text: "Hello, I'm ready to help!" },
  },
  move: {
    label: "Move",
    icon: Play,
    description: "Move to location or position",
    defaultParams: { x: 0, y: 0, speed: 1 },
  },
  gesture: {
    label: "Gesture",
    icon: Zap,
    description: "Physical gesture or animation",
    defaultParams: { gesture: "wave", duration: 2 },
  },
  look_at: {
    label: "Look At",
    icon: Eye,
    description: "Orient gaze or camera",
    defaultParams: { target: "participant" },
  },
  wait: {
    label: "Wait",
    icon: Clock,
    description: "Pause for specified duration",
    defaultParams: { duration: 3 },
  },
  instruction: {
    label: "Instruction",
    icon: Settings,
    description: "Display instruction for wizard",
    defaultParams: { text: "Follow the protocol", allowSkip: true },
  },
  question: {
    label: "Question",
    icon: Plus,
    description: "Ask participant a question",
    defaultParams: { question: "How do you feel?", recordResponse: true },
  },
  observe: {
    label: "Observe",
    icon: Eye,
    description: "Observe and record behavior",
    defaultParams: { target: "participant", duration: 5, notes: "" },
  },
};

// Custom Node Components
interface StepNodeProps {
  data: {
    step: FlowStep;
    onEdit: (step: FlowStep) => void;
    onDelete: (stepId: string) => void;
    onDuplicate: (step: FlowStep) => void;
    isSelected: boolean;
  };
}

function StepNode({ data }: StepNodeProps) {
  const { step, onEdit, onDelete, onDuplicate, isSelected } = data;
  const config = stepTypeConfig[step.type];

  return (
    <div className="relative">
      {/* Connection Handles */}
      <Handle
        type="target"
        position={Position.Left}
        className="!bg-primary !border-background !h-3 !w-3 !border-2"
        id="input"
      />
      <Handle
        type="source"
        position={Position.Right}
        className="!bg-primary !border-background !h-3 !w-3 !border-2"
        id="output"
      />

      <Card
        className={`min-w-[200px] border transition-all duration-200 ${
          isSelected ? "ring-primary shadow-2xl ring-2" : "hover:shadow-lg"
        }`}
      >
        <CardHeader className="pb-2">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <div
                className={`${config.bgColor} flex h-8 w-8 shrink-0 items-center justify-center rounded shadow-lg`}
              >
                <config.icon className="h-4 w-4 text-white" />
              </div>
              <div>
                <CardTitle className="text-sm font-medium">
                  {step.name}
                </CardTitle>
                <Badge variant="outline" className="text-xs">
                  {config.label}
                </Badge>
              </div>
            </div>

            <DropdownMenu>
              <DropdownMenuTrigger asChild>
                <Button variant="ghost" size="sm" className="h-8 w-8 p-0">
                  <Settings className="h-4 w-4" />
                </Button>
              </DropdownMenuTrigger>
              <DropdownMenuContent align="end">
                <DropdownMenuItem onClick={() => onEdit(step)}>
                  <Edit3 className="mr-2 h-4 w-4" />
                  Edit Step
                </DropdownMenuItem>
                <DropdownMenuItem onClick={() => onDuplicate(step)}>
                  <Copy className="mr-2 h-4 w-4" />
                  Duplicate
                </DropdownMenuItem>
                <DropdownMenuSeparator />
                <DropdownMenuItem
                  onClick={() => onDelete(step.id)}
                  className="text-destructive focus:text-destructive"
                >
                  <Trash2 className="mr-2 h-4 w-4" />
                  Delete Step
                </DropdownMenuItem>
              </DropdownMenuContent>
            </DropdownMenu>
          </div>
        </CardHeader>

        {step.description && (
          <CardContent className="pt-0 pb-2">
            <p className="text-muted-foreground text-xs">{step.description}</p>
          </CardContent>
        )}

        {step.actions.length > 0 && (
          <CardContent className="pt-0">
            <div className="text-muted-foreground text-xs">
              {step.actions.length} action{step.actions.length !== 1 ? "s" : ""}
            </div>
            <div className="mt-1 flex flex-wrap gap-1">
              {step.actions.slice(0, 3).map((action) => {
                const actionConfig = actionTypeConfig[action.type];
                return (
                  <Badge
                    key={action.id}
                    variant="secondary"
                    className="text-xs"
                  >
                    <actionConfig.icon className="mr-1 h-3 w-3" />
                    {actionConfig.label}
                  </Badge>
                );
              })}
              {step.actions.length > 3 && (
                <Badge variant="secondary" className="text-xs">
                  +{step.actions.length - 3} more
                </Badge>
              )}
            </div>
          </CardContent>
        )}
      </Card>
    </div>
  );
}

// Node types configuration
const nodeTypes: NodeTypes = {
  stepNode: StepNode,
};

// Main Flow Designer Component
interface FlowDesignerProps {
  experimentId: string;
  initialDesign: FlowDesign;
  onSave?: (design: FlowDesign) => Promise<void>;
  isSaving?: boolean;
}

export function FlowDesigner({
  experimentId,
  initialDesign,
  onSave,
  isSaving = false,
}: FlowDesignerProps) {
  const [design, setDesign] = useState<FlowDesign>(initialDesign);
  const [selectedStepId, setSelectedStepId] = useState<string>();
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);
  const [editingStep, setEditingStep] = useState<FlowStep | null>(null);

  // React Flow state
  const [reactFlowInstance, setReactFlowInstance] = useState(null);

  const selectedStep = useMemo(() => {
    return design.steps.find((step) => step.id === selectedStepId);
  }, [design.steps, selectedStepId]);

  const createStep = useCallback(
    (type: StepType, position: { x: number; y: number }): FlowStep => {
      const config = stepTypeConfig[type];
      const stepNumber = design.steps.length + 1;

      return {
        id: `step-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        type,
        name: `${config.label} ${stepNumber}`,
        actions: [],
        parameters: {},
        position,
      };
    },
    [design.steps.length],
  );

  const handleStepTypeAdd = useCallback(
    (type: StepType) => {
      const newPosition = {
        x: design.steps.length * 250 + 100,
        y: 100,
      };
      const newStep = createStep(type, newPosition);

      setDesign((prev) => ({
        ...prev,
        steps: [...prev.steps, newStep],
      }));
      setHasUnsavedChanges(true);
      setSelectedStepId(newStep.id);
      toast.success(`Added ${stepTypeConfig[type].label}`);
    },
    [createStep, design.steps.length],
  );

  const handleStepDelete = useCallback(
    (stepId: string) => {
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.filter((step) => step.id !== stepId),
      }));
      setHasUnsavedChanges(true);

      if (selectedStepId === stepId) {
        setSelectedStepId(undefined);
      }

      toast.success("Step deleted");
    },
    [selectedStepId],
  );

  const handleStepDuplicate = useCallback((step: FlowStep) => {
    const newStep: FlowStep = {
      ...step,
      id: `step-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      name: `${step.name} (Copy)`,
      position: {
        x: step.position.x + 250,
        y: step.position.y,
      },
      actions: step.actions.map((action) => ({
        ...action,
        id: `action-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      })),
    };

    setDesign((prev) => ({
      ...prev,
      steps: [...prev.steps, newStep],
    }));
    setHasUnsavedChanges(true);
    toast.success("Step duplicated");
  }, []);

  const handleNodeClick = useCallback(
    (_event: React.MouseEvent, node: Node) => {
      setSelectedStepId(node.id);
    },
    [],
  );

  // Convert design steps to React Flow nodes
  const nodes: Node[] = useMemo(() => {
    return design.steps.map((step) => ({
      id: step.id,
      type: "stepNode",
      position: step.position,
      data: {
        step,
        onEdit: setEditingStep,
        onDelete: handleStepDelete,
        onDuplicate: handleStepDuplicate,
        isSelected: selectedStepId === step.id,
      },
    }));
  }, [design.steps, selectedStepId, handleStepDelete, handleStepDuplicate]);

  // Auto-connect sequential steps based on position
  const edges: Edge[] = useMemo(() => {
    const sortedSteps = [...design.steps].sort(
      (a, b) => a.position.x - b.position.x,
    );
    const newEdges: Edge[] = [];

    for (let i = 0; i < sortedSteps.length - 1; i++) {
      const sourceStep = sortedSteps[i];
      const targetStep = sortedSteps[i + 1];

      if (sourceStep && targetStep) {
        // Only auto-connect if steps are reasonably close horizontally
        const distance = Math.abs(
          targetStep.position.x - sourceStep.position.x,
        );
        if (distance < 400) {
          newEdges.push({
            id: `${sourceStep.id}-${targetStep.id}`,
            source: sourceStep.id,
            sourceHandle: "output",
            target: targetStep.id,
            targetHandle: "input",
            type: "smoothstep",
            animated: true,
            markerEnd: {
              type: MarkerType.ArrowClosed,
              width: 20,
              height: 20,
              color: "hsl(var(--muted-foreground))",
            },
            style: {
              strokeWidth: 2,
              stroke: "hsl(var(--muted-foreground))",
            },
          });
        }
      }
    }

    return newEdges;
  }, [design.steps]);

  const handleNodesChange = useCallback((changes: any[]) => {
    // Update step positions when nodes are moved
    const positionChanges = changes.filter(
      (change) => change.type === "position" && change.position,
    );
    if (positionChanges.length > 0) {
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((step) => {
          const positionChange = positionChanges.find(
            (change) => change.id === step.id,
          );
          if (positionChange && positionChange.position) {
            return { ...step, position: positionChange.position };
          }
          return step;
        }),
      }));
      setHasUnsavedChanges(true);
    }
  }, []);

  const handleConnect = useCallback((params: Connection) => {
    if (!params.source || !params.target) return;

    // Update the design to reflect the new connection order
    setDesign((prev) => {
      const sourceStep = prev.steps.find((s) => s.id === params.source);
      const targetStep = prev.steps.find((s) => s.id === params.target);

      if (sourceStep && targetStep) {
        // Automatically adjust positions to create a logical flow
        const updatedSteps = prev.steps.map((step) => {
          if (step.id === params.target) {
            return {
              ...step,
              position: {
                x: Math.max(sourceStep.position.x + 300, step.position.x),
                y: step.position.y,
              },
            };
          }
          return step;
        });

        setHasUnsavedChanges(true);
        toast.success("Steps connected successfully");
        return { ...prev, steps: updatedSteps };
      }

      return prev;
    });
  }, []);

  const handleSave = async () => {
    if (!onSave) return;

    try {
      const updatedDesign = {
        ...design,
        version: design.version + 1,
        lastSaved: new Date(),
      };

      await onSave(updatedDesign);
      setDesign(updatedDesign);
      setHasUnsavedChanges(false);
    } catch (error) {
      console.error("Save error:", error);
    }
  };

  const handleStepUpdate = useCallback((updatedStep: FlowStep) => {
    setDesign((prev) => ({
      ...prev,
      steps: prev.steps.map((step) =>
        step.id === updatedStep.id ? updatedStep : step,
      ),
    }));
    setHasUnsavedChanges(true);
    setEditingStep(null);
  }, []);

  return (
    <div className="flex h-full flex-col">
      {/* Toolbar */}
      <div className="bg-background/95 supports-[backdrop-filter]:bg-background/60 border-b p-4 backdrop-blur">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <h2 className="font-semibold">{design.name}</h2>
            {hasUnsavedChanges && (
              <Badge
                variant="outline"
                className="border-amber-500 bg-amber-500/10 text-amber-600 dark:text-amber-400"
              >
                Unsaved Changes
              </Badge>
            )}
          </div>

          <div className="flex items-center gap-2">
            <Button variant="outline" size="sm" disabled>
              <Undo className="mr-2 h-4 w-4" />
              Undo
            </Button>
            <Button variant="outline" size="sm" disabled>
              <Redo className="mr-2 h-4 w-4" />
              Redo
            </Button>
            <Button variant="outline" size="sm">
              <Download className="mr-2 h-4 w-4" />
              Export
            </Button>
            <Button variant="outline" size="sm">
              <Upload className="mr-2 h-4 w-4" />
              Import
            </Button>
            <Button
              onClick={handleSave}
              disabled={isSaving || !hasUnsavedChanges}
              size="sm"
            >
              <Save className="mr-2 h-4 w-4" />
              {isSaving ? "Saving..." : "Save"}
            </Button>
          </div>
        </div>
      </div>

      {/* Main Flow Area */}
      <div className="relative flex-1">
        <ReactFlow
          nodes={nodes}
          edges={edges}
          onNodesChange={handleNodesChange}
          onConnect={handleConnect}
          onNodeClick={handleNodeClick}
          nodeTypes={nodeTypes}
          fitView
          fitViewOptions={{ padding: 0.2 }}
          connectionLineType={"smoothstep" as any}
          snapToGrid={true}
          snapGrid={[20, 20]}
          defaultEdgeOptions={{
            type: "smoothstep",
            animated: true,
            style: { strokeWidth: 2 },
          }}
          className="[&_.react-flow\_\_background]:bg-background [&_.react-flow\_\_controls]:bg-background [&_.react-flow\_\_controls]:border-border [&_.react-flow\_\_controls-button]:bg-background [&_.react-flow\_\_controls-button]:border-border [&_.react-flow\_\_controls-button]:text-foreground [&_.react-flow\_\_controls-button:hover]:bg-accent [&_.react-flow\_\_minimap]:bg-background [&_.react-flow\_\_minimap]:border-border [&_.react-flow\_\_edge-path]:stroke-muted-foreground [&_.react-flow\_\_controls]:shadow-sm [&_.react-flow\_\_edge-path]:stroke-2"
        >
          <Background
            variant={"dots" as any}
            gap={20}
            size={1}
            className="[&>*]:fill-muted-foreground/20"
          />
          <Controls className="bg-background border-border rounded-lg shadow-lg" />
          <MiniMap
            nodeColor={(node) => {
              const step = design.steps.find((s) => s.id === node.id);
              return step
                ? stepTypeConfig[step.type].color
                : "hsl(var(--muted))";
            }}
            className="bg-background border-border rounded-lg shadow-lg"
          />

          {/* Step Library Panel */}
          <Panel
            position="top-left"
            className="bg-card/95 supports-[backdrop-filter]:bg-card/80 rounded-lg border p-4 shadow-lg backdrop-blur"
          >
            <div className="space-y-3">
              <h4 className="text-sm font-medium">Add Step</h4>
              <div className="grid grid-cols-2 gap-2">
                {Object.entries(stepTypeConfig).map(([type, config]) => (
                  <Button
                    key={type}
                    variant="outline"
                    size="sm"
                    className="h-auto justify-start p-2"
                    onClick={() => handleStepTypeAdd(type as StepType)}
                  >
                    <div className="flex items-center gap-2">
                      <div
                        className={`${config.bgColor} flex h-6 w-6 shrink-0 items-center justify-center rounded shadow-lg`}
                      >
                        <config.icon className="h-3 w-3 text-white" />
                      </div>
                      <span className="text-xs">{config.label}</span>
                    </div>
                  </Button>
                ))}
              </div>
            </div>
          </Panel>

          {/* Info Panel */}
          <Panel
            position="top-right"
            className="bg-card/95 supports-[backdrop-filter]:bg-card/80 rounded-lg border p-4 shadow-lg backdrop-blur"
          >
            <div className="space-y-2 text-sm">
              <div className="flex justify-between">
                <span className="text-muted-foreground">Steps:</span>
                <span className="font-medium">{design.steps.length}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Actions:</span>
                <span className="font-medium">
                  {design.steps.reduce(
                    (sum, step) => sum + step.actions.length,
                    0,
                  )}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Version:</span>
                <span className="font-medium">v{design.version}</span>
              </div>
            </div>
          </Panel>
        </ReactFlow>

        {/* Properties Sheet */}
        {selectedStep && (
          <Sheet
            open={!!selectedStep}
            onOpenChange={() => setSelectedStepId(undefined)}
          >
            <SheetContent>
              <SheetHeader>
                <SheetTitle>Step Properties</SheetTitle>
                <SheetDescription>
                  Configure the selected step and its actions
                </SheetDescription>
              </SheetHeader>

              <div className="space-y-6 px-4 pb-4">
                <div className="space-y-2">
                  <Label htmlFor="step-name">Name</Label>
                  <Input
                    id="step-name"
                    value={selectedStep.name}
                    onChange={(e) => {
                      const updatedStep = {
                        ...selectedStep,
                        name: e.target.value,
                      };
                      handleStepUpdate(updatedStep);
                    }}
                  />
                </div>

                <div className="space-y-2">
                  <Label htmlFor="step-description">Description</Label>
                  <Textarea
                    id="step-description"
                    value={selectedStep.description ?? ""}
                    onChange={(e) => {
                      const updatedStep = {
                        ...selectedStep,
                        description: e.target.value,
                      };
                      handleStepUpdate(updatedStep);
                    }}
                    placeholder="Optional description..."
                    rows={3}
                  />
                </div>

                <div className="space-y-2">
                  <Label>Step Type</Label>
                  <div className="flex items-center gap-2">
                    <div
                      className={`${stepTypeConfig[selectedStep.type].bgColor} flex h-8 w-8 shrink-0 items-center justify-center rounded shadow-lg`}
                    >
                      {React.createElement(
                        stepTypeConfig[selectedStep.type].icon,
                        {
                          className: "h-4 w-4 text-white",
                        },
                      )}
                    </div>
                    <span className="text-sm font-medium">
                      {stepTypeConfig[selectedStep.type].label}
                    </span>
                  </div>
                </div>

                <Separator />

                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <Label>Actions ({selectedStep.actions.length})</Label>
                    <Button size="sm" variant="outline">
                      <Plus className="mr-2 h-4 w-4" />
                      Add Action
                    </Button>
                  </div>

                  <ScrollArea className="mt-2 h-[200px]">
                    <div className="space-y-2 pr-4">
                      {selectedStep.actions.map((action) => {
                        const actionConfig = actionTypeConfig[action.type];
                        return (
                          <div
                            key={action.id}
                            className="bg-muted/50 flex items-center gap-2 rounded border p-2"
                          >
                            <actionConfig.icon className="text-muted-foreground h-4 w-4 shrink-0" />
                            <span className="min-w-0 flex-1 truncate text-sm font-medium">
                              {action.name}
                            </span>
                            <Button
                              variant="ghost"
                              size="sm"
                              className="h-6 w-6 p-0"
                            >
                              <Edit3 className="h-3 w-3" />
                            </Button>
                          </div>
                        );
                      })}
                    </div>
                  </ScrollArea>
                </div>
              </div>
            </SheetContent>
          </Sheet>
        )}

        {/* Step Edit Dialog */}
        {editingStep && (
          <Sheet open={!!editingStep} onOpenChange={() => setEditingStep(null)}>
            <SheetContent>
              <SheetHeader>
                <SheetTitle>Edit Step</SheetTitle>
                <SheetDescription>
                  Modify step properties and actions
                </SheetDescription>
              </SheetHeader>

              <div className="space-y-6 px-4 pb-4">
                <div className="space-y-2">
                  <Label htmlFor="edit-step-name">Name</Label>
                  <Input
                    id="edit-step-name"
                    value={editingStep.name}
                    onChange={(e) => {
                      setEditingStep({ ...editingStep, name: e.target.value });
                    }}
                  />
                </div>

                <div className="space-y-2">
                  <Label htmlFor="edit-step-description">Description</Label>
                  <Textarea
                    id="edit-step-description"
                    value={editingStep.description ?? ""}
                    onChange={(e) => {
                      setEditingStep({
                        ...editingStep,
                        description: e.target.value,
                      });
                    }}
                    placeholder="Optional description..."
                    rows={3}
                  />
                </div>

                <div className="flex gap-2 pt-6">
                  <Button
                    onClick={() => {
                      handleStepUpdate(editingStep);
                    }}
                    className="flex-1"
                  >
                    Save Changes
                  </Button>
                  <Button
                    variant="outline"
                    onClick={() => setEditingStep(null)}
                  >
                    Cancel
                  </Button>
                </div>
              </div>
            </SheetContent>
          </Sheet>
        )}
      </div>
    </div>
  );
}

export type { FlowDesign, FlowStep, FlowAction, StepType, ActionType };
