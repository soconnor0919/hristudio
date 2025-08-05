"use client";

import React, { useState, useCallback, useMemo } from "react";
import {
  DndContext,
  type DragEndEvent,
  type DragOverEvent,
  type DragStartEvent,
  PointerSensor,
  useSensor,
  useSensors,
  DragOverlay,
  closestCorners,
} from "@dnd-kit/core";
import {
  SortableContext,
  verticalListSortingStrategy,
  useSortable,
} from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import {
  Bot,
  User,
  Plus,
  GripVertical,
  Edit3,
  Trash2,
  Play,
  Clock,
  MessageSquare,
  ArrowRight,
  Settings,
  Copy,
  Eye,
  Save,
  FileText,
  Zap,
  GitBranch,
  Shuffle,
  ChevronDown,
  ChevronRight,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Input } from "~/components/ui/input";
import { Textarea } from "~/components/ui/textarea";
import { Label } from "~/components/ui/label";
import {
  Accordion,
  AccordionContent,
  AccordionItem,
  AccordionTrigger,
} from "~/components/ui/accordion";
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
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";
import { toast } from "sonner";

export type StepType = "wizard" | "robot" | "parallel" | "conditional";
export type ActionType =
  | "speak"
  | "move"
  | "gesture"
  | "look_at"
  | "wait"
  | "instruction"
  | "question"
  | "observe";

export interface Action {
  id: string;
  type: ActionType;
  name: string;
  description?: string;
  parameters: Record<string, any>;
  duration?: number;
  order: number;
}

export interface ExperimentStep {
  id: string;
  type: StepType;
  name: string;
  description?: string;
  order: number;
  duration?: number;
  actions: Action[];
  parameters: Record<string, any>;
  expanded?: boolean;
}

export interface ExperimentDesign {
  id: string;
  name: string;
  description?: string;
  steps: ExperimentStep[];
  version: number;
  lastSaved: Date;
  robotPlatform?: string;
}

const stepTypeConfig = {
  wizard: {
    label: "Wizard Step",
    description: "Manual instructions for the wizard operator",
    icon: User,
    color: "bg-blue-50 text-blue-700 border-blue-200",
    allowedActions: ["instruction", "question", "observe"],
  },
  robot: {
    label: "Robot Step",
    description: "Automated robot behaviors and actions",
    icon: Bot,
    color: "bg-green-50 text-green-700 border-green-200",
    allowedActions: ["speak", "move", "gesture", "look_at", "wait"],
  },
  parallel: {
    label: "Parallel Steps",
    description: "Execute multiple actions simultaneously",
    icon: Shuffle,
    color: "bg-purple-50 text-purple-700 border-purple-200",
    allowedActions: [],
  },
  conditional: {
    label: "Conditional Branch",
    description: "Execute steps based on conditions",
    icon: GitBranch,
    color: "bg-orange-50 text-orange-700 border-orange-200",
    allowedActions: [],
  },
} as const;

const actionTypeConfig = {
  speak: {
    label: "Speak",
    description: "Robot speaks text or plays audio",
    icon: MessageSquare,
    defaultParams: { text: "", voice: "default", speed: 1.0 },
  },
  move: {
    label: "Move",
    description: "Robot moves to a position or along a path",
    icon: ArrowRight,
    defaultParams: { target: "", speed: 0.5, precision: "normal" },
  },
  gesture: {
    label: "Gesture",
    description: "Robot performs a predefined gesture",
    icon: Zap,
    defaultParams: { gesture: "", intensity: 0.5 },
  },
  look_at: {
    label: "Look At",
    description: "Robot looks at a target or direction",
    icon: Eye,
    defaultParams: { target: "", duration: 2.0 },
  },
  wait: {
    label: "Wait",
    description: "Wait for a specified duration",
    icon: Clock,
    defaultParams: { duration: 1.0 },
  },
  instruction: {
    label: "Instruction",
    description: "Display instruction for wizard",
    icon: FileText,
    defaultParams: { text: "", allowSkip: true },
  },
  question: {
    label: "Question",
    description: "Wizard asks participant a question",
    icon: MessageSquare,
    defaultParams: { question: "", recordResponse: true },
  },
  observe: {
    label: "Observe",
    description: "Wizard observes and records behavior",
    icon: Eye,
    defaultParams: { target: "", duration: 5.0, notes: "" },
  },
} as const;

interface StepLibraryProps {
  onStepTypeSelect: (type: StepType) => void;
}

function StepLibrary({ onStepTypeSelect }: StepLibraryProps) {
  return (
    <div className="space-y-4">
      <div className="flex items-center gap-2">
        <Plus className="h-4 w-4" />
        <h3 className="font-semibold">Step Library</h3>
      </div>

      <Accordion type="single" collapsible className="w-full">
        <AccordionItem value="basic-steps">
          <AccordionTrigger>Basic Steps</AccordionTrigger>
          <AccordionContent>
            <div className="space-y-2">
              {(["wizard", "robot"] as const).map((type) => {
                const config = stepTypeConfig[type];
                const Icon = config.icon;
                return (
                  <Button
                    key={type}
                    variant="outline"
                    className={`h-auto w-full justify-start gap-2 p-3 ${config.color}`}
                    onClick={() => onStepTypeSelect(type)}
                  >
                    <Icon className="h-4 w-4" />
                    <div className="text-left">
                      <div className="font-medium">{config.label}</div>
                      <div className="text-xs opacity-70">
                        {config.description}
                      </div>
                    </div>
                  </Button>
                );
              })}
            </div>
          </AccordionContent>
        </AccordionItem>

        <AccordionItem value="advanced-steps">
          <AccordionTrigger>Advanced Steps</AccordionTrigger>
          <AccordionContent>
            <div className="space-y-2">
              {(["parallel", "conditional"] as const).map((type) => {
                const config = stepTypeConfig[type];
                const Icon = config.icon;
                return (
                  <Button
                    key={type}
                    variant="outline"
                    className={`h-auto w-full justify-start gap-2 p-3 ${config.color}`}
                    onClick={() => onStepTypeSelect(type)}
                  >
                    <Icon className="h-4 w-4" />
                    <div className="text-left">
                      <div className="font-medium">{config.label}</div>
                      <div className="text-xs opacity-70">
                        {config.description}
                      </div>
                    </div>
                  </Button>
                );
              })}
            </div>
          </AccordionContent>
        </AccordionItem>
      </Accordion>
    </div>
  );
}

interface ActionLibraryProps {
  stepType: StepType;
  onActionSelect: (type: ActionType) => void;
}

function ActionLibrary({ stepType, onActionSelect }: ActionLibraryProps) {
  const allowedActions = stepTypeConfig[stepType]?.allowedActions || [];

  if (allowedActions.length === 0) {
    return (
      <div className="text-muted-foreground py-4 text-center">
        <p>No actions available for this step type</p>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      <div className="flex items-center gap-2">
        <Zap className="h-4 w-4" />
        <h4 className="font-medium">Available Actions</h4>
      </div>

      <div className="grid grid-cols-1 gap-2">
        {allowedActions.map((actionType) => {
          const config = actionTypeConfig[actionType];
          const Icon = config.icon;
          return (
            <Button
              key={actionType}
              variant="ghost"
              size="sm"
              className="h-auto justify-start gap-2 p-2"
              onClick={() => onActionSelect(actionType)}
            >
              <Icon className="h-3 w-3" />
              <div className="text-left">
                <div className="text-sm font-medium">{config.label}</div>
                <div className="text-muted-foreground text-xs">
                  {config.description}
                </div>
              </div>
            </Button>
          );
        })}
      </div>
    </div>
  );
}

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
  const Icon = config.icon;

  return (
    <Card className="ml-6 border-l-4 border-l-blue-200">
      <CardContent className="p-3">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Icon className="text-muted-foreground h-4 w-4" />
            <div>
              <div className="text-sm font-medium">{action.name}</div>
              <div className="text-muted-foreground text-xs">
                {config.label}
              </div>
            </div>
          </div>

          <div className="flex items-center gap-1">
            {action.duration && (
              <Badge variant="outline" className="text-xs">
                {action.duration}s
              </Badge>
            )}
            <DropdownMenu>
              <DropdownMenuTrigger asChild>
                <Button variant="ghost" size="sm" className="h-6 w-6 p-0">
                  <Settings className="h-3 w-3" />
                </Button>
              </DropdownMenuTrigger>
              <DropdownMenuContent align="end">
                <DropdownMenuItem onClick={() => onEdit(action)}>
                  <Edit3 className="mr-2 h-3 w-3" />
                  Edit
                </DropdownMenuItem>
                <DropdownMenuItem onClick={() => onDuplicate(action)}>
                  <Copy className="mr-2 h-3 w-3" />
                  Duplicate
                </DropdownMenuItem>
                <DropdownMenuSeparator />
                <DropdownMenuItem
                  onClick={() => onDelete(action.id)}
                  className="text-red-600"
                >
                  <Trash2 className="mr-2 h-3 w-3" />
                  Delete
                </DropdownMenuItem>
              </DropdownMenuContent>
            </DropdownMenu>
          </div>
        </div>

        {action.description && (
          <p className="text-muted-foreground mt-2 text-xs">
            {action.description}
          </p>
        )}
      </CardContent>
    </Card>
  );
}

interface StepCardProps {
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
  onActionAdd: (stepId: string, actionType: ActionType) => void;
}

function StepCard({
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
}: StepCardProps) {
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
  const Icon = config.icon;
  const totalDuration = step.actions.reduce(
    (sum, action) => sum + (action.duration || 0),
    0,
  );

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={`${isDragging ? "opacity-50" : ""}`}
    >
      <Card
        className={`cursor-pointer transition-all duration-200 ${isSelected ? "shadow-md ring-2 ring-blue-500" : "hover:shadow-sm"} ${config.color} `}
        onClick={() => onSelect(step.id)}
      >
        <CardHeader className="pb-3">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <div
                {...attributes}
                {...listeners}
                className="cursor-grab active:cursor-grabbing"
              >
                <GripVertical className="text-muted-foreground h-4 w-4" />
              </div>

              <Icon className="h-5 w-5" />

              <div className="flex-1">
                <CardTitle className="text-base">{step.name}</CardTitle>
                {step.description && (
                  <p className="text-muted-foreground mt-1 text-sm">
                    {step.description}
                  </p>
                )}
              </div>
            </div>

            <div className="flex items-center gap-2">
              <Badge variant="outline" className="text-xs">
                {step.actions.length} action
                {step.actions.length !== 1 ? "s" : ""}
              </Badge>

              {totalDuration > 0 && (
                <Badge variant="outline" className="text-xs">
                  ~{totalDuration}s
                </Badge>
              )}

              <Button
                variant="ghost"
                size="sm"
                onClick={(e) => {
                  e.stopPropagation();
                  onToggleExpanded(step.id);
                }}
                className="h-6 w-6 p-0"
              >
                {step.expanded ? (
                  <ChevronDown className="h-3 w-3" />
                ) : (
                  <ChevronRight className="h-3 w-3" />
                )}
              </Button>

              <DropdownMenu>
                <DropdownMenuTrigger asChild>
                  <Button
                    variant="ghost"
                    size="sm"
                    className="h-6 w-6 p-0"
                    onClick={(e) => e.stopPropagation()}
                  >
                    <Settings className="h-3 w-3" />
                  </Button>
                </DropdownMenuTrigger>
                <DropdownMenuContent align="end">
                  <DropdownMenuItem onClick={() => onEdit(step)}>
                    <Edit3 className="mr-2 h-3 w-3" />
                    Edit Step
                  </DropdownMenuItem>
                  <DropdownMenuItem onClick={() => onDuplicate(step)}>
                    <Copy className="mr-2 h-3 w-3" />
                    Duplicate
                  </DropdownMenuItem>
                  <DropdownMenuSeparator />
                  <DropdownMenuItem
                    onClick={() => onDelete(step.id)}
                    className="text-red-600"
                  >
                    <Trash2 className="mr-2 h-3 w-3" />
                    Delete
                  </DropdownMenuItem>
                </DropdownMenuContent>
              </DropdownMenu>
            </div>
          </div>
        </CardHeader>

        {step.expanded && (
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

              <div className="mt-3 ml-6">
                <ActionLibrary
                  stepType={step.type}
                  onActionSelect={(actionType) =>
                    onActionAdd(step.id, actionType)
                  }
                />
              </div>
            </div>
          </CardContent>
        )}
      </Card>
    </div>
  );
}

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
  onActionAdd: (stepId: string, actionType: ActionType) => void;
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
  const sortedSteps = [...design.steps].sort((a, b) => a.order - b.order);
  const stepIds = sortedSteps.map((step) => step.id);

  return (
    <div className="flex h-full flex-col">
      <div className="flex items-center justify-between border-b p-4">
        <div>
          <h2 className="text-lg font-semibold">{design.name}</h2>
          {design.description && (
            <p className="text-muted-foreground text-sm">
              {design.description}
            </p>
          )}
        </div>

        <div className="flex items-center gap-2">
          <Badge variant="outline">
            {design.steps.length} step{design.steps.length !== 1 ? "s" : ""}
          </Badge>
          <Badge variant="outline">v{design.version}</Badge>
        </div>
      </div>

      <ScrollArea className="flex-1 p-4">
        {sortedSteps.length === 0 ? (
          <div className="flex h-64 flex-col items-center justify-center text-center">
            <div className="bg-muted mb-4 rounded-full p-4">
              <FileText className="text-muted-foreground h-8 w-8" />
            </div>
            <h3 className="mb-2 font-medium">No steps in experiment</h3>
            <p className="text-muted-foreground mb-4 max-w-sm text-sm">
              Start building your experiment by dragging step types from the
              library on the left.
            </p>
          </div>
        ) : (
          <SortableContext
            items={stepIds}
            strategy={verticalListSortingStrategy}
          >
            <div className="space-y-4">
              {sortedSteps.map((step, index) => (
                <div key={step.id}>
                  <StepCard
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

                  {index < sortedSteps.length - 1 && (
                    <div className="flex justify-center py-2">
                      <ArrowRight className="text-muted-foreground h-4 w-4" />
                    </div>
                  )}
                </div>
              ))}
            </div>
          </SortableContext>
        )}
      </ScrollArea>
    </div>
  );
}

interface ExperimentDesignerProps {
  experimentId: string;
  initialDesign?: ExperimentDesign;
  onSave?: (design: ExperimentDesign) => Promise<void>;
}

export function ExperimentDesigner({
  experimentId,
  initialDesign,
  onSave,
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
  const [isSaving, setIsSaving] = useState(false);
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
        Math.max(...(step?.actions.map((a) => a.order) || [0]), 0) + 1;

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
    (stepId: string, actionType: ActionType) => {
      const newAction = createAction(actionType, stepId);
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.map((step) =>
          step.id === stepId
            ? { ...step, actions: [...step.actions, newAction] }
            : step,
        ),
      }));
      setHasUnsavedChanges(true);
      toast.success(`Added ${actionTypeConfig[actionType].label} action`);
    },
    [createAction],
  );

  const handleStepEdit = useCallback((step: ExperimentStep) => {
    // TODO: Open step edit dialog
    console.log("Edit step:", step);
  }, []);

  const handleStepDelete = useCallback(
    (stepId: string) => {
      setDesign((prev) => ({
        ...prev,
        steps: prev.steps.filter((s) => s.id !== stepId),
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
      const newStep = {
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
              ?.actions.map((a) => a.order) || [0]),
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
        // Reorder steps
        const [movedStep] = steps.splice(activeIndex, 1);
        steps.splice(overIndex, 0, movedStep!);

        // Update order numbers
        steps.forEach((step, index) => {
          step.order = index + 1;
        });
      }

      return { ...prev, steps };
    });

    setHasUnsavedChanges(true);
  };

  const handleSave = async () => {
    if (!onSave) return;

    setIsSaving(true);
    try {
      const updatedDesign = {
        ...design,
        version: design.version + 1,
        lastSaved: new Date(),
      };

      await onSave(updatedDesign);
      setDesign(updatedDesign);
      setHasUnsavedChanges(false);
      toast.success("Experiment saved successfully");
    } catch (error) {
      toast.error("Failed to save experiment");
      console.error("Save error:", error);
    } finally {
      setIsSaving(false);
    }
  };

  return (
    <div className="flex h-screen flex-col">
      {/* Header */}
      <div className="bg-background border-b p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <h1 className="text-xl font-semibold">Experiment Designer</h1>
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
            <Button onClick={handleSave} disabled={isSaving} size="sm">
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
                            value={selectedStep.description || ""}
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
