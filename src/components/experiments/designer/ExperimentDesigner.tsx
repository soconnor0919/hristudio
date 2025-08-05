"use client";

import React, { useState, useCallback, useMemo } from "react";
import {
  DndContext,
  type DragEndEvent,
  type DragStartEvent,
  useSensors,
  useSensor,
  PointerSensor,
  closestCorners,
  DragOverlay,
} from "@dnd-kit/core";
import {
  SortableContext,
  verticalListSortingStrategy,
  arrayMove,
} from "@dnd-kit/sortable";
import { useSortable } from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import {
  Play,
  Bot,
  Shuffle,
  GitBranch,
  Plus,
  GripVertical,
  Edit3,
  Trash2,
  Copy,
  Settings,
  Eye,
  Save,
  Clock,
  Users,
  Zap,
  ChevronRight,
  ChevronDown,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Label } from "~/components/ui/label";
import { Input } from "~/components/ui/input";
import { Textarea } from "~/components/ui/textarea";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Separator } from "~/components/ui/separator";
import {
  ResizablePanelGroup,
  ResizablePanel,
  ResizableHandle,
} from "~/components/ui/resizable";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
  DropdownMenuSeparator,
} from "~/components/ui/dropdown-menu";
import {
  Collapsible,
  CollapsibleContent,
  CollapsibleTrigger,
} from "~/components/ui/collapsible";
import { toast } from "sonner";

// Types
type StepType = "wizard" | "robot" | "parallel" | "conditional";
type ActionType =
  | "speak"
  | "move"
  | "gesture"
  | "look_at"
  | "wait"
  | "instruction"
  | "question"
  | "observe";

interface Action {
  id: string;
  type: ActionType;
  name: string;
  parameters: Record<string, unknown>;
  order: number;
}

interface ExperimentStep {
  id: string;
  type: StepType;
  name: string;
  description?: string;
  duration?: number;
  order: number;
  actions: Action[];
  parameters: Record<string, unknown>;
  expanded: boolean;
}

interface ExperimentDesign {
  id: string;
  name: string;
  description?: string;
  steps: ExperimentStep[];
  version: number;
  lastSaved: Date;
}

// Configuration
const stepTypeConfig = {
  wizard: {
    label: "Wizard Action",
    icon: Users,
    description: "Actions performed by the human wizard",
    color: "bg-blue-500",
    lightColor: "bg-blue-50 border-blue-200",
  },
  robot: {
    label: "Robot Action",
    icon: Bot,
    description: "Actions performed by the robot",
    color: "bg-green-500",
    lightColor: "bg-green-50 border-green-200",
  },
  parallel: {
    label: "Parallel Steps",
    icon: Shuffle,
    description: "Execute multiple steps simultaneously",
    color: "bg-purple-500",
    lightColor: "bg-purple-50 border-purple-200",
  },
  conditional: {
    label: "Conditional Branch",
    icon: GitBranch,
    description: "Branching logic based on conditions",
    color: "bg-orange-500",
    lightColor: "bg-orange-50 border-orange-200",
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

// Step Library Component
interface StepLibraryProps {
  onStepTypeSelect: (type: StepType) => void;
}

function StepLibrary({ onStepTypeSelect }: StepLibraryProps) {
  return (
    <div className="space-y-4">
      <div className="flex items-center gap-2">
        <Plus className="h-4 w-4" />
        <h4 className="font-medium">Add Step</h4>
      </div>

      <div className="grid gap-2">
        {Object.entries(stepTypeConfig).map(([type, config]) => (
          <Button
            key={type}
            variant="outline"
            className="h-auto justify-start p-3"
            onClick={() => onStepTypeSelect(type as StepType)}
          >
            <div className="flex items-start gap-3 text-left">
              <div
                className={`${config.color} flex h-8 w-8 shrink-0 items-center justify-center rounded`}
              >
                <config.icon className="h-4 w-4 text-white" />
              </div>
              <div className="min-w-0 flex-1">
                <div className="font-medium">{config.label}</div>
                <div className="text-muted-foreground text-xs">
                  {config.description}
                </div>
              </div>
            </div>
          </Button>
        ))}
      </div>
    </div>
  );
}

// Action Library Component
interface ActionLibraryProps {
  stepId: string;
  onActionAdd: (type: ActionType, stepId: string) => void;
}

function ActionLibrary({ stepId, onActionAdd }: ActionLibraryProps) {
  return (
    <div className="space-y-2">
      <Label className="text-muted-foreground text-xs font-medium">
        Add Action
      </Label>
      <div className="grid grid-cols-2 gap-1">
        {Object.entries(actionTypeConfig).map(([type, config]) => (
          <Button
            key={type}
            variant="ghost"
            size="sm"
            className="h-auto justify-start p-2"
            onClick={() => onActionAdd(type as ActionType, stepId)}
          >
            <div className="flex items-center gap-2">
              <config.icon className="h-3 w-3" />
              <span className="text-xs">{config.label}</span>
            </div>
          </Button>
        ))}
      </div>
    </div>
  );
}

// Action Card Component
interface ActionCardProps {
  action: Action;
  onEdit: (action: Action) => void;
  onDelete: (actionId: string) => void;
  onDuplicate: (action: Action) => void;
}

function ActionCard({
  action,
  onEdit,
  onDelete,
  onDuplicate,
}: ActionCardProps) {
  const config = actionTypeConfig[action.type];

  return (
    <div className="bg-background/50 group hover:bg-accent/50 flex items-center gap-2 rounded border p-2">
      <config.icon className="text-muted-foreground h-3 w-3 shrink-0" />
      <span className="min-w-0 flex-1 truncate text-xs font-medium">
        {action.name}
      </span>

      <div className="flex items-center gap-1 opacity-0 group-hover:opacity-100">
        <Button
          variant="ghost"
          size="icon"
          className="h-6 w-6"
          onClick={() => onEdit(action)}
        >
          <Edit3 className="h-3 w-3" />
        </Button>
        <Button
          variant="ghost"
          size="icon"
          className="h-6 w-6"
          onClick={() => onDuplicate(action)}
        >
          <Copy className="h-3 w-3" />
        </Button>
        <Button
          variant="ghost"
          size="icon"
          className="h-6 w-6"
          onClick={() => onDelete(action.id)}
        >
          <Trash2 className="h-3 w-3" />
        </Button>
      </div>
    </div>
  );
}

// Sortable Step Card Component
interface SortableStepCardProps {
  step: ExperimentStep;
  isSelected: boolean;
  onSelect: (stepId: string) => void;
  onEdit: (step: ExperimentStep) => void;
  onDelete: (stepId: string) => void;
  onDuplicate: (step: ExperimentStep) => void;
  onToggleExpanded: (stepId: string) => void;
  onActionEdit: (action: Action) => void;
  onActionDelete: (stepId: string, actionId: string) => void;
  onActionDuplicate: (stepId: string, action: Action) => void;
  onActionAdd: (type: ActionType, stepId: string) => void;
}

function SortableStepCard({
  step,
  isSelected,
  onSelect,
  onEdit,
  onDelete,
  onDuplicate,
  onToggleExpanded,
  onActionEdit,
  onActionDelete,
  onActionDuplicate,
  onActionAdd,
}: SortableStepCardProps) {
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({ id: step.id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
  };

  const config = stepTypeConfig[step.type];

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={`${isDragging ? "opacity-50" : ""} ${isSelected ? "ring-primary ring-2" : ""}`}
    >
      <Card className="group hover:shadow-sm">
        <CardHeader className="pb-3">
          <div className="flex items-start gap-3">
            <button
              {...attributes}
              {...listeners}
              className="text-muted-foreground hover:text-foreground mt-1 cursor-grab active:cursor-grabbing"
            >
              <GripVertical className="h-4 w-4" />
            </button>

            <div
              className={`${config.color} flex h-8 w-8 shrink-0 items-center justify-center rounded`}
            >
              <config.icon className="h-4 w-4 text-white" />
            </div>

            <div className="min-w-0 flex-1">
              <div className="flex items-start justify-between gap-2">
                <div
                  className="min-w-0 flex-1 cursor-pointer"
                  onClick={() => onSelect(step.id)}
                >
                  <h4 className="leading-none font-medium">{step.name}</h4>
                  {step.description && (
                    <p className="text-muted-foreground mt-1 text-sm">
                      {step.description}
                    </p>
                  )}
                  <div className="text-muted-foreground mt-2 flex items-center gap-2 text-xs">
                    <Badge variant="secondary" className="text-xs">
                      {config.label}
                    </Badge>
                    {step.actions.length > 0 && (
                      <span>
                        {step.actions.length} action
                        {step.actions.length !== 1 ? "s" : ""}
                      </span>
                    )}
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
                      className="text-destructive"
                    >
                      <Trash2 className="mr-2 h-4 w-4" />
                      Delete Step
                    </DropdownMenuItem>
                  </DropdownMenuContent>
                </DropdownMenu>
              </div>
            </div>
          </div>
        </CardHeader>

        <Collapsible
          open={step.expanded}
          onOpenChange={() => onToggleExpanded(step.id)}
        >
          <CollapsibleTrigger asChild>
            <Button
              variant="ghost"
              className="w-full justify-between px-6 py-2"
            >
              <span className="text-sm">Actions ({step.actions.length})</span>
              {step.expanded ? (
                <ChevronDown className="h-4 w-4" />
              ) : (
                <ChevronRight className="h-4 w-4" />
              )}
            </Button>
          </CollapsibleTrigger>
          <CollapsibleContent>
            <CardContent className="pt-0">
              <div className="space-y-2">
                {step.actions.map((action) => (
                  <ActionCard
                    key={action.id}
                    action={action}
                    onEdit={onActionEdit}
                    onDelete={(actionId) => onActionDelete(step.id, actionId)}
                    onDuplicate={(action) => onActionDuplicate(step.id, action)}
                  />
                ))}

                <ActionLibrary stepId={step.id} onActionAdd={onActionAdd} />
              </div>
            </CardContent>
          </CollapsibleContent>
        </Collapsible>
      </Card>
    </div>
  );
}

// Experiment Canvas Component
interface ExperimentCanvasProps {
  design: ExperimentDesign;
  selectedStepId?: string;
  onStepSelect: (stepId: string) => void;
  onStepEdit: (step: ExperimentStep) => void;
  onStepDelete: (stepId: string) => void;
  onStepDuplicate: (step: ExperimentStep) => void;
  onStepToggleExpanded: (stepId: string) => void;
  onActionEdit: (action: Action) => void;
  onActionDelete: (stepId: string, actionId: string) => void;
  onActionDuplicate: (stepId: string, action: Action) => void;
  onActionAdd: (type: ActionType, stepId: string) => void;
}

function ExperimentCanvas({
  design,
  selectedStepId,
  onStepSelect,
  onStepEdit,
  onStepDelete,
  onStepDuplicate,
  onStepToggleExpanded,
  onActionEdit,
  onActionDelete,
  onActionDuplicate,
  onActionAdd,
}: ExperimentCanvasProps) {
  return (
    <div className="flex h-full flex-col">
      <div className="border-b p-4">
        <h3 className="font-medium">Experiment Steps</h3>
        <p className="text-muted-foreground text-sm">
          Drag to reorder, click to select, expand to edit actions
        </p>
      </div>

      <ScrollArea className="flex-1 p-4">
        {design.steps.length === 0 ? (
          <div className="flex h-64 flex-col items-center justify-center text-center">
            <Bot className="text-muted-foreground mb-4 h-12 w-12" />
            <h3 className="font-medium">No steps yet</h3>
            <p className="text-muted-foreground text-sm">
              Add your first step from the library on the left
            </p>
          </div>
        ) : (
          <SortableContext
            items={design.steps.map((s) => s.id)}
            strategy={verticalListSortingStrategy}
          >
            <div className="space-y-4">
              {design.steps.map((step) => (
                <SortableStepCard
                  key={step.id}
                  step={step}
                  isSelected={selectedStepId === step.id}
                  onSelect={onStepSelect}
                  onEdit={onStepEdit}
                  onDelete={onStepDelete}
                  onDuplicate={onStepDuplicate}
                  onToggleExpanded={onStepToggleExpanded}
                  onActionEdit={onActionEdit}
                  onActionDelete={onActionDelete}
                  onActionDuplicate={onActionDuplicate}
                  onActionAdd={onActionAdd}
                />
              ))}
            </div>
          </SortableContext>
        )}
      </ScrollArea>
    </div>
  );
}

// Main Designer Component
interface ExperimentDesignerProps {
  experimentId: string;
  initialDesign: ExperimentDesign;
  onSave?: (design: ExperimentDesign) => Promise<void>;
  isSaving?: boolean;
}

export function ExperimentDesigner({
  experimentId,
  initialDesign,
  onSave,
  isSaving = false,
}: ExperimentDesignerProps) {
  const [design, setDesign] = useState<ExperimentDesign>(
    initialDesign || {
      id: experimentId,
      name: "New Experiment",
      steps: [],
      version: 1,
      lastSaved: new Date(),
    },
  );

  const [selectedStepId, setSelectedStepId] = useState<string>();
  const [activeId, setActiveId] = useState<string | null>(null);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);

  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 8,
      },
    }),
  );

  const selectedStep = useMemo(() => {
    return design.steps.find((step) => step.id === selectedStepId);
  }, [design.steps, selectedStepId]);

  const createStep = useCallback(
    (type: StepType): ExperimentStep => {
      const config = stepTypeConfig[type];
      const newOrder = Math.max(...design.steps.map((s) => s.order), 0) + 1;

      return {
        id: `step-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        type,
        name: `${config.label} ${newOrder}`,
        order: newOrder,
        actions: [],
        parameters: {},
        expanded: true,
      };
    },
    [design.steps],
  );

  const createAction = useCallback(
    (type: ActionType, stepId: string): Action => {
      const config = actionTypeConfig[type];
      const step = design.steps.find((s) => s.id === stepId);
      const newOrder =
        Math.max(...(step?.actions.map((a) => a.order) ?? [0]), 0) + 1;

      return {
        id: `action-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        type,
        name: config.label,
        parameters: { ...config.defaultParams },
        order: newOrder,
      };
    },
    [design.steps],
  );

  const handleStepTypeSelect = useCallback(
    (type: StepType) => {
      const newStep = createStep(type);
      setDesign((prev) => ({
        ...prev,
        steps: [...prev.steps, newStep],
      }));
      setHasUnsavedChanges(true);
      setSelectedStepId(newStep.id);
      toast.success(`Added ${stepTypeConfig[type].label}`);
    },
    [createStep],
  );

  const handleActionAdd = useCallback(
    (type: ActionType, stepId: string) => {
      const newAction = createAction(type, stepId);

      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((step) =>
          step.id === stepId
            ? { ...step, actions: [...step.actions, newAction] }
            : step,
        ),
      }));
      setHasUnsavedChanges(true);
      toast.success(`Added ${actionTypeConfig[type].label}`);
    },
    [createAction],
  );

  const handleStepEdit = useCallback((step: ExperimentStep) => {
    // TODO: Open step edit dialog
    console.log("Edit step:", step);
    setSelectedStepId(step.id);
  }, []);

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

  const handleStepDuplicate = useCallback(
    (step: ExperimentStep) => {
      const newStep: ExperimentStep = {
        ...step,
        id: `step-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        name: `${step.name} (Copy)`,
        order: Math.max(...design.steps.map((s) => s.order), 0) + 1,
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
    },
    [design.steps],
  );

  const handleStepToggleExpanded = useCallback((stepId: string) => {
    setDesign((prev) => ({
      ...prev,
      steps: prev.steps.map((step) =>
        step.id === stepId ? { ...step, expanded: !step.expanded } : step,
      ),
    }));
  }, []);

  const handleActionEdit = useCallback((action: Action) => {
    // TODO: Open action edit dialog
    console.log("Edit action:", action);
  }, []);

  const handleActionDelete = useCallback((stepId: string, actionId: string) => {
    setDesign((prev) => ({
      ...prev,
      steps: prev.steps.map((step) =>
        step.id === stepId
          ? { ...step, actions: step.actions.filter((a) => a.id !== actionId) }
          : step,
      ),
    }));
    setHasUnsavedChanges(true);
    toast.success("Action deleted");
  }, []);

  const handleActionDuplicate = useCallback(
    (stepId: string, action: Action) => {
      const newAction = {
        ...action,
        id: `action-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        name: `${action.name} (Copy)`,
        order:
          Math.max(
            ...(design.steps
              .find((s) => s.id === stepId)
              ?.actions.map((a) => a.order) ?? [0]),
            0,
          ) + 1,
      };

      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((step) =>
          step.id === stepId
            ? { ...step, actions: [...step.actions, newAction] }
            : step,
        ),
      }));
      setHasUnsavedChanges(true);
      toast.success("Action duplicated");
    },
    [design.steps],
  );

  const handleDragStart = (event: DragStartEvent) => {
    setActiveId(event.active.id as string);
  };

  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;
    setActiveId(null);

    if (!over || active.id === over.id) return;

    setDesign((prev) => {
      const steps = [...prev.steps];
      const activeIndex = steps.findIndex((step) => step.id === active.id);
      const overIndex = steps.findIndex((step) => step.id === over.id);

      if (activeIndex !== -1 && overIndex !== -1) {
        const reorderedSteps = arrayMove(steps, activeIndex, overIndex);

        // Update order numbers
        reorderedSteps.forEach((step, index) => {
          step.order = index + 1;
        });

        return { ...prev, steps: reorderedSteps };
      }

      return prev;
    });

    setHasUnsavedChanges(true);
  };

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

  return (
    <div className="flex h-full flex-col">
      {/* Toolbar */}
      <div className="border-b p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <h2 className="font-semibold">{design.name}</h2>
            {hasUnsavedChanges && (
              <Badge
                variant="outline"
                className="border-orange-600 text-orange-600"
              >
                Unsaved Changes
              </Badge>
            )}
          </div>

          <div className="flex items-center gap-2">
            <Button variant="outline" size="sm">
              <Eye className="mr-2 h-4 w-4" />
              Preview
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

      {/* Main Content */}
      <div className="flex-1 overflow-hidden">
        <DndContext
          sensors={sensors}
          collisionDetection={closestCorners}
          onDragStart={handleDragStart}
          onDragEnd={handleDragEnd}
        >
          <ResizablePanelGroup direction="horizontal" className="h-full">
            {/* Left Sidebar - Step Library */}
            <ResizablePanel defaultSize={25} minSize={20}>
              <div className="bg-muted/20 h-full border-r">
                <ScrollArea className="h-full p-4">
                  <StepLibrary onStepTypeSelect={handleStepTypeSelect} />

                  <Separator className="my-6" />

                  <div className="space-y-4">
                    <div className="flex items-center gap-2">
                      <Settings className="h-4 w-4" />
                      <h4 className="font-medium">Experiment Info</h4>
                    </div>

                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-muted-foreground">Steps:</span>
                        <span className="font-medium">
                          {design.steps.length}
                        </span>
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
                      <div className="flex justify-between">
                        <span className="text-muted-foreground">
                          Last Saved:
                        </span>
                        <span className="text-xs font-medium">
                          {design.lastSaved.toLocaleTimeString()}
                        </span>
                      </div>
                    </div>
                  </div>
                </ScrollArea>
              </div>
            </ResizablePanel>

            <ResizableHandle />

            {/* Main Canvas */}
            <ResizablePanel defaultSize={55} minSize={40}>
              <ExperimentCanvas
                design={design}
                selectedStepId={selectedStepId}
                onStepSelect={setSelectedStepId}
                onStepEdit={handleStepEdit}
                onStepDelete={handleStepDelete}
                onStepDuplicate={handleStepDuplicate}
                onStepToggleExpanded={handleStepToggleExpanded}
                onActionEdit={handleActionEdit}
                onActionDelete={handleActionDelete}
                onActionDuplicate={handleActionDuplicate}
                onActionAdd={handleActionAdd}
              />
            </ResizablePanel>

            <ResizableHandle />

            {/* Right Sidebar - Properties Panel */}
            <ResizablePanel defaultSize={20} minSize={15}>
              <div className="bg-muted/20 h-full border-l">
                <ScrollArea className="h-full p-4">
                  {selectedStep ? (
                    <div className="space-y-4">
                      <div className="flex items-center gap-2">
                        <Edit3 className="h-4 w-4" />
                        <h4 className="font-medium">Step Properties</h4>
                      </div>

                      <div className="space-y-3">
                        <div>
                          <Label htmlFor="step-name">Name</Label>
                          <Input
                            id="step-name"
                            value={selectedStep.name}
                            onChange={(e) => {
                              setDesign((prev) => ({
                                ...prev,
                                steps: prev.steps.map((step) =>
                                  step.id === selectedStepId
                                    ? { ...step, name: e.target.value }
                                    : step,
                                ),
                              }));
                              setHasUnsavedChanges(true);
                            }}
                          />
                        </div>

                        <div>
                          <Label htmlFor="step-description">Description</Label>
                          <Textarea
                            id="step-description"
                            value={selectedStep.description ?? ""}
                            onChange={(e) => {
                              setDesign((prev) => ({
                                ...prev,
                                steps: prev.steps.map((step) =>
                                  step.id === selectedStepId
                                    ? { ...step, description: e.target.value }
                                    : step,
                                ),
                              }));
                              setHasUnsavedChanges(true);
                            }}
                            placeholder="Optional description..."
                            rows={3}
                          />
                        </div>

                        <div>
                          <Label>Step Type</Label>
                          <div className="mt-1 flex items-center gap-2">
                            {React.createElement(
                              stepTypeConfig[selectedStep.type].icon,
                              {
                                className: "h-4 w-4",
                              },
                            )}
                            <span className="text-sm font-medium">
                              {stepTypeConfig[selectedStep.type].label}
                            </span>
                          </div>
                        </div>

                        <div>
                          <Label>Actions</Label>
                          <div className="text-muted-foreground text-sm">
                            {selectedStep.actions.length} action
                            {selectedStep.actions.length !== 1 ? "s" : ""}
                          </div>
                        </div>
                      </div>
                    </div>
                  ) : (
                    <div className="flex h-32 flex-col items-center justify-center text-center">
                      <Settings className="text-muted-foreground mb-2 h-8 w-8" />
                      <p className="text-muted-foreground text-sm">
                        Select a step to edit its properties
                      </p>
                    </div>
                  )}
                </ScrollArea>
              </div>
            </ResizablePanel>
          </ResizablePanelGroup>

          <DragOverlay>
            {activeId ? (
              <Card className="opacity-90 shadow-lg">
                <CardContent className="p-3">
                  <div className="flex items-center gap-2">
                    <GripVertical className="text-muted-foreground h-4 w-4" />
                    <span className="text-sm font-medium">Moving step...</span>
                  </div>
                </CardContent>
              </Card>
            ) : null}
          </DragOverlay>
        </DndContext>
      </div>
    </div>
  );
}

export type { ExperimentDesign, ExperimentStep, Action, StepType, ActionType };
