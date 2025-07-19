"use client";

import { useState, useCallback, useRef } from "react";
import {
  DndContext,
  DragOverlay,
  useDraggable,
  useDroppable,
  DragStartEvent,
  DragEndEvent,
  closestCenter,
  PointerSensor,
  useSensor,
  useSensors,
} from "@dnd-kit/core";
import {
  SortableContext,
  verticalListSortingStrategy,
  useSortable,
} from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import {
  Play,
  Bot,
  GitBranch,
  Shuffle,
  Settings,
  Plus,
  Save,
  Undo,
  Redo,
  Eye,
  Trash2,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";

// Step type definitions
export type StepType = "wizard" | "robot" | "parallel" | "conditional";

export interface ExperimentStep {
  id: string;
  type: StepType;
  name: string;
  description?: string;
  order: number;
  parameters: Record<string, any>;
  duration?: number;
  parentId?: string;
  children?: string[];
}

export interface ExperimentDesign {
  id: string;
  name: string;
  steps: ExperimentStep[];
  version: number;
  lastSaved: Date;
}

const stepTypeConfig = {
  wizard: {
    label: "Wizard Action",
    description: "Manual control by wizard operator",
    icon: Play,
    color: "bg-blue-100 text-blue-700 border-blue-200",
    defaultParams: {
      instruction: "",
      allowSkip: true,
      timeout: null,
    },
  },
  robot: {
    label: "Robot Action",
    description: "Automated robot behavior",
    icon: Bot,
    color: "bg-green-100 text-green-700 border-green-200",
    defaultParams: {
      action: "",
      parameters: {},
      waitForCompletion: true,
    },
  },
  parallel: {
    label: "Parallel Steps",
    description: "Execute multiple steps simultaneously",
    icon: Shuffle,
    color: "bg-purple-100 text-purple-700 border-purple-200",
    defaultParams: {
      waitForAll: true,
      maxDuration: null,
    },
  },
  conditional: {
    label: "Conditional Branch",
    description: "Execute steps based on conditions",
    icon: GitBranch,
    color: "bg-orange-100 text-orange-700 border-orange-200",
    defaultParams: {
      condition: "",
      trueSteps: [],
      falseSteps: [],
    },
  },
};

interface StepLibraryItemProps {
  type: StepType;
  onDragStart?: () => void;
}

function StepLibraryItem({ type, onDragStart }: StepLibraryItemProps) {
  const config = stepTypeConfig[type];
  const Icon = config.icon;

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    isDragging,
  } = useDraggable({
    id: `library-${type}`,
    data: { type: "library-item", stepType: type },
  });

  const style = {
    transform: CSS.Translate.toString(transform),
    opacity: isDragging ? 0.5 : 1,
  };

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...listeners}
      {...attributes}
      className={`
        cursor-grab active:cursor-grabbing
        rounded-lg border-2 border-dashed p-3 transition-all
        hover:border-solid hover:shadow-sm
        ${config.color}
      `}
      onMouseDown={onDragStart}
    >
      <div className="flex items-center space-x-2">
        <Icon className="h-4 w-4" />
        <span className="text-sm font-medium">{config.label}</span>
      </div>
      <p className="mt-1 text-xs opacity-80">{config.description}</p>
    </div>
  );
}

interface ExperimentStepCardProps {
  step: ExperimentStep;
  onEdit: (step: ExperimentStep) => void;
  onDelete: (stepId: string) => void;
  isSelected: boolean;
  onClick: () => void;
}

function ExperimentStepCard({
  step,
  onEdit,
  onDelete,
  isSelected,
  onClick
}: ExperimentStepCardProps) {
  const config = stepTypeConfig[step.type];
  const Icon = config.icon;

  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({
    id: step.id,
    data: { type: "step", step },
  });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.5 : 1,
  };

  return (
    <Card
      ref={setNodeRef}
      style={style}
      className={`
        cursor-pointer transition-all
        ${isSelected ? "ring-2 ring-blue-500 shadow-md" : "hover:shadow-sm"}
      `}
      onClick={onClick}
    >
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <div className={`p-1 rounded ${config.color}`}>
              <Icon className="h-3 w-3" />
            </div>
            <div>
              <CardTitle className="text-sm">{step.name}</CardTitle>
              <Badge variant="outline" className="text-xs">
                {config.label}
              </Badge>
            </div>
          </div>
          <div
            className="drag-handle cursor-grab active:cursor-grabbing p-1"
            {...attributes}
            {...listeners}
          >
            <div className="grid grid-cols-2 gap-0.5">
              {[...Array(6)].map((_, i) => (
                <div key={i} className="h-0.5 w-0.5 bg-slate-400 rounded-full" />
              ))}
            </div>
          </div>
        </div>
      </CardHeader>
      <CardContent className="pt-0">
        {step.description && (
          <p className="text-xs text-slate-600 mb-2">{step.description}</p>
        )}
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-2 text-xs text-slate-500">
            <span>Step {step.order}</span>
            {step.duration && <span>• {step.duration}s</span>}
          </div>
          <div className="flex items-center space-x-1">
            <Button
              size="sm"
              variant="ghost"
              onClick={(e) => {
                e.stopPropagation();
                onEdit(step);
              }}
              className="h-6 w-6 p-0"
            >
              <Settings className="h-3 w-3" />
            </Button>
            <Button
              size="sm"
              variant="ghost"
              onClick={(e) => {
                e.stopPropagation();
                onDelete(step.id);
              }}
              className="h-6 w-6 p-0 text-red-600 hover:text-red-700"
            >
              <Trash2 className="h-3 w-3" />
            </Button>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}

interface ExperimentCanvasProps {
  steps: ExperimentStep[];
  selectedStepId?: string;
  onStepSelect: (stepId: string) => void;
  onStepEdit: (step: ExperimentStep) => void;
  onStepDelete: (stepId: string) => void;
  onStepsReorder: (steps: ExperimentStep[]) => void;
}

function ExperimentCanvas({
  steps,
  selectedStepId,
  onStepSelect,
  onStepEdit,
  onStepDelete,
  onStepsReorder,
}: ExperimentCanvasProps) {
  const { setNodeRef } = useDroppable({
    id: "experiment-canvas",
  });

  const stepIds = steps.map((step) => step.id);

  return (
    <div
      ref={setNodeRef}
      className="flex-1 bg-slate-50 rounded-lg border-2 border-dashed border-slate-300 p-4 min-h-[500px]"
    >
      {steps.length === 0 ? (
        <div className="flex items-center justify-center h-full text-center">
          <div>
            <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-slate-200">
              <Play className="h-8 w-8 text-slate-400" />
            </div>
            <h3 className="text-lg font-semibold text-slate-700 mb-2">
              Start Designing Your Experiment
            </h3>
            <p className="text-slate-500 max-w-sm">
              Drag step types from the library on the left to begin creating your experimental protocol.
            </p>
          </div>
        </div>
      ) : (
        <SortableContext items={stepIds} strategy={verticalListSortingStrategy}>
          <div className="space-y-3">
            {steps
              .sort((a, b) => a.order - b.order)
              .map((step) => (
                <ExperimentStepCard
                  key={step.id}
                  step={step}
                  onEdit={onStepEdit}
                  onDelete={onStepDelete}
                  isSelected={selectedStepId === step.id}
                  onClick={() => onStepSelect(step.id)}
                />
              ))}
          </div>
        </SortableContext>
      )}
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
    }
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
    })
  );

  const createStep = useCallback((type: StepType): ExperimentStep => {
    const config = stepTypeConfig[type];
    const newOrder = Math.max(...design.steps.map(s => s.order), 0) + 1;

    return {
      id: `step-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      type,
      name: `${config.label} ${newOrder}`,
      order: newOrder,
      parameters: { ...config.defaultParams },
    };
  }, [design.steps]);

  const handleDragStart = (event: DragStartEvent) => {
    setActiveId(event.active.id as string);
  };

  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;
    setActiveId(null);

    if (!over) return;

    // Handle dropping library item onto canvas
    if (active.data.current?.type === "library-item" && over.id === "experiment-canvas") {
      const stepType = active.data.current.stepType as StepType;
      const newStep = createStep(stepType);

      setDesign(prev => ({
        ...prev,
        steps: [...prev.steps, newStep],
      }));
      setHasUnsavedChanges(true);
      return;
    }

    // Handle reordering steps
    if (active.data.current?.type === "step" && over.data.current?.type === "step") {
      const activeStep = design.steps.find(s => s.id === active.id);
      const overStep = design.steps.find(s => s.id === over.id);

      if (!activeStep || !overStep) return;

      const newSteps = [...design.steps];
      const activeIndex = newSteps.findIndex(s => s.id === active.id);
      const overIndex = newSteps.findIndex(s => s.id === over.id);

      // Swap positions
      [newSteps[activeIndex], newSteps[overIndex]] = [newSteps[overIndex], newSteps[activeIndex]];

      // Update order numbers
      newSteps.forEach((step, index) => {
        step.order = index + 1;
      });

      setDesign(prev => ({
        ...prev,
        steps: newSteps,
      }));
      setHasUnsavedChanges(true);
    }
  };

  const handleStepEdit = (step: ExperimentStep) => {
    // TODO: Open step configuration modal
    console.log("Edit step:", step);
  };

  const handleStepDelete = (stepId: string) => {
    setDesign(prev => ({
      ...prev,
      steps: prev.steps.filter(s => s.id !== stepId),
    }));
    setHasUnsavedChanges(true);

    if (selectedStepId === stepId) {
      setSelectedStepId(undefined);
    }
  };

  const handleSave = async () => {
    if (!onSave || !hasUnsavedChanges) return;

    setIsSaving(true);
    try {
      const updatedDesign = {
        ...design,
        lastSaved: new Date(),
        version: design.version + 1,
      };

      await onSave(updatedDesign);
      setDesign(updatedDesign);
      setHasUnsavedChanges(false);
    } catch (error) {
      console.error("Failed to save experiment:", error);
    } finally {
      setIsSaving(false);
    }
  };

  return (
    <div className="h-full flex flex-col">
      {/* Toolbar */}
      <div className="flex items-center justify-between p-4 border-b bg-white">
        <div className="flex items-center space-x-4">
          <h2 className="text-lg font-semibold">{design.name}</h2>
          {hasUnsavedChanges && (
            <Badge variant="outline" className="text-orange-600 border-orange-600">
              Unsaved Changes
            </Badge>
          )}
        </div>

        <div className="flex items-center space-x-2">
          <Button variant="outline" size="sm" disabled>
            <Undo className="h-4 w-4 mr-1" />
            Undo
          </Button>
          <Button variant="outline" size="sm" disabled>
            <Redo className="h-4 w-4 mr-1" />
            Redo
          </Button>
          <Separator orientation="vertical" className="h-6" />
          <Button variant="outline" size="sm">
            <Eye className="h-4 w-4 mr-1" />
            Preview
          </Button>
          <Button
            onClick={handleSave}
            disabled={!hasUnsavedChanges || isSaving}
            size="sm"
          >
            <Save className="h-4 w-4 mr-1" />
            {isSaving ? "Saving..." : "Save"}
          </Button>
        </div>
      </div>

      <div className="flex-1 flex">
        <DndContext
          sensors={sensors}
          collisionDetection={closestCenter}
          onDragStart={handleDragStart}
          onDragEnd={handleDragEnd}
        >
          {/* Step Library Sidebar */}
          <div className="w-64 bg-white border-r p-4">
            <div className="mb-4">
              <h3 className="font-semibold text-slate-900 mb-2">Step Library</h3>
              <p className="text-sm text-slate-600">
                Drag steps onto the canvas to build your experiment
              </p>
            </div>

            <div className="space-y-3">
              {(Object.keys(stepTypeConfig) as StepType[]).map((type) => (
                <StepLibraryItem key={type} type={type} />
              ))}
            </div>

            <Separator className="my-4" />

            <div>
              <h4 className="font-medium text-slate-900 mb-2">Experiment Stats</h4>
              <div className="space-y-1 text-sm text-slate-600">
                <div className="flex justify-between">
                  <span>Total Steps:</span>
                  <span className="font-medium">{design.steps.length}</span>
                </div>
                <div className="flex justify-between">
                  <span>Estimated Duration:</span>
                  <span className="font-medium">
                    {design.steps.reduce((acc, step) => acc + (step.duration || 0), 0)}s
                  </span>
                </div>
                <div className="flex justify-between">
                  <span>Version:</span>
                  <span className="font-medium">v{design.version}</span>
                </div>
              </div>
            </div>
          </div>

          {/* Main Canvas */}
          <div className="flex-1 p-4">
            <ExperimentCanvas
              steps={design.steps}
              selectedStepId={selectedStepId}
              onStepSelect={setSelectedStepId}
              onStepEdit={handleStepEdit}
              onStepDelete={handleStepDelete}
              onStepsReorder={(newSteps) => {
                setDesign(prev => ({ ...prev, steps: newSteps }));
                setHasUnsavedChanges(true);
              }}
            />
          </div>

          <DragOverlay>
            {activeId ? (
              <div className="opacity-80">
                {activeId.startsWith("library-") ? (
                  <div className="p-3 bg-white border rounded-lg shadow-lg">
                    <div className="flex items-center space-x-2">
                      <Plus className="h-4 w-4" />
                      <span className="text-sm font-medium">New Step</span>
                    </div>
                  </div>
                ) : (
                  <Card className="w-64 shadow-lg">
                    <CardContent className="p-3">
                      <div className="font-medium text-sm">Moving step...</div>
                    </CardContent>
                  </Card>
                )}
              </div>
            ) : null}
          </DragOverlay>
        </DndContext>
      </div>
    </div>
  );
}
