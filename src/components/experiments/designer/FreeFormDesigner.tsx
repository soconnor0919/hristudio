"use client";

import {
    closestCenter, DndContext,
    DragOverlay, PointerSensor, useDraggable,
    useDroppable, useSensor,
    useSensors, type DragEndEvent, type DragStartEvent
} from "@dnd-kit/core";
import {
    Bot, Clock, Edit3, Grid, MessageSquare, Play, Redo, Save, Trash2, Undo, ZoomIn,
    ZoomOut
} from "lucide-react";
import { useCallback, useEffect, useRef, useState } from "react";
import { Button } from "~/components/ui/button";
import {
    Dialog,
    DialogContent,
    DialogDescription,
    DialogFooter,
    DialogHeader,
    DialogTitle
} from "~/components/ui/dialog";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Separator } from "~/components/ui/separator";
import { Textarea } from "~/components/ui/textarea";
import {
    Tooltip,
    TooltipContent,
    TooltipProvider,
    TooltipTrigger
} from "~/components/ui/tooltip";

// Free-form element types
export type ElementType =
  | "text"
  | "action"
  | "timer"
  | "decision"
  | "note"
  | "group";

export interface CanvasElement {
  id: string;
  type: ElementType;
  title: string;
  content: string;
  position: { x: number; y: number };
  size: { width: number; height: number };
  style: {
    backgroundColor: string;
    textColor: string;
    borderColor: string;
    fontSize: number;
  };
  metadata: Record<string, any>;
  connections: string[]; // IDs of connected elements
}

export interface Connection {
  id: string;
  from: string;
  to: string;
  label?: string;
  style: {
    color: string;
    width: number;
    type: "solid" | "dashed" | "dotted";
  };
}

export interface ExperimentDesign {
  id: string;
  name: string;
  elements: CanvasElement[];
  connections: Connection[];
  canvasSettings: {
    zoom: number;
    gridSize: number;
    showGrid: boolean;
    backgroundColor: string;
  };
  version: number;
  lastSaved: Date;
}

const elementTypeConfig = {
  text: {
    label: "Text Block",
    description: "Add instructions or information",
    icon: MessageSquare,
    defaultStyle: {
      backgroundColor: "#f8fafc",
      textColor: "#1e293b",
      borderColor: "#e2e8f0",
    },
  },
  action: {
    label: "Action Step",
    description: "Define an action to be performed",
    icon: Play,
    defaultStyle: {
      backgroundColor: "#dbeafe",
      textColor: "#1e40af",
      borderColor: "#3b82f6",
    },
  },
  timer: {
    label: "Timer",
    description: "Add timing constraints",
    icon: Clock,
    defaultStyle: {
      backgroundColor: "#fef3c7",
      textColor: "#92400e",
      borderColor: "#f59e0b",
    },
  },
  decision: {
    label: "Decision Point",
    description: "Create branching logic",
    icon: Bot,
    defaultStyle: {
      backgroundColor: "#dcfce7",
      textColor: "#166534",
      borderColor: "#22c55e",
    },
  },
  note: {
    label: "Research Note",
    description: "Add researcher annotations",
    icon: Edit3,
    defaultStyle: {
      backgroundColor: "#fce7f3",
      textColor: "#be185d",
      borderColor: "#ec4899",
    },
  },
  group: {
    label: "Group Container",
    description: "Group related elements",
    icon: Grid,
    defaultStyle: {
      backgroundColor: "#f3f4f6",
      textColor: "#374151",
      borderColor: "#9ca3af",
    },
  },
};

interface FreeFormDesignerProps {
  experiment: {
    id: string;
    name: string;
    description: string;
  };
  onSave?: (design: ExperimentDesign) => void;
  initialDesign?: ExperimentDesign;
}

// Draggable element from toolbar
function ToolboxElement({ type }: { type: ElementType }) {
  const config = elementTypeConfig[type];
  const { attributes, listeners, setNodeRef, transform, isDragging } =
    useDraggable({
      id: `toolbox-${type}`,
      data: { type: "toolbox", elementType: type },
    });

  const style = {
    transform: transform
      ? `translate3d(${transform.x}px, ${transform.y}px, 0)`
      : undefined,
    opacity: isDragging ? 0.5 : 1,
  };

  return (
    <TooltipProvider>
      <Tooltip>
        <TooltipTrigger asChild>
          <div
            ref={setNodeRef}
            style={style}
            {...listeners}
            {...attributes}
            className="flex cursor-grab flex-col items-center gap-2 rounded-lg border-2 border-dashed border-gray-300 p-3 transition-colors hover:border-gray-400 hover:bg-gray-50"
          >
            <config.icon className="h-6 w-6 text-gray-600" />
            <span className="text-xs font-medium text-gray-700">
              {config.label}
            </span>
          </div>
        </TooltipTrigger>
        <TooltipContent>
          <p>{config.description}</p>
        </TooltipContent>
      </Tooltip>
    </TooltipProvider>
  );
}

// Canvas element component
function CanvasElementComponent({
  element,
  isSelected,
  onSelect,
  onEdit,
  onDelete,
}: {
  element: CanvasElement;
  isSelected: boolean;
  onSelect: () => void;
  onEdit: () => void;
  onDelete: () => void;
}) {
  const config = elementTypeConfig[element.type];
  const { attributes, listeners, setNodeRef, transform, isDragging } =
    useDraggable({
      id: element.id,
      data: { type: "canvas-element", element },
    });

  const style = {
    transform: transform
      ? `translate3d(${transform.x}px, ${transform.y}px, 0)`
      : undefined,
    position: "absolute" as const,
    left: element.position.x,
    top: element.position.y,
    width: element.size.width,
    height: element.size.height,
    backgroundColor: element.style.backgroundColor,
    color: element.style.textColor,
    borderColor: element.style.borderColor,
    fontSize: element.style.fontSize,
    opacity: isDragging ? 0.7 : 1,
    zIndex: isSelected ? 10 : 1,
  };

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={`cursor-pointer rounded-lg border-2 p-3 shadow-sm transition-all ${
        isSelected ? "ring-2 ring-blue-500 ring-offset-2" : ""
      }`}
      onClick={onSelect}
      {...listeners}
      {...attributes}
    >
      <div className="flex items-start gap-2">
        <config.icon className="h-4 w-4 flex-shrink-0" />
        <div className="min-w-0 flex-1">
          <h4 className="truncate text-sm font-medium">{element.title}</h4>
          <p className="mt-1 line-clamp-3 text-xs opacity-75">
            {element.content}
          </p>
        </div>
      </div>

      {isSelected && (
        <div className="absolute -top-2 -right-2 flex gap-1">
          <Button
            size="sm"
            variant="secondary"
            className="h-6 w-6 p-0"
            onClick={(e) => {
              e.stopPropagation();
              onEdit();
            }}
          >
            <Edit3 className="h-3 w-3" />
          </Button>
          <Button
            size="sm"
            variant="destructive"
            className="h-6 w-6 p-0"
            onClick={(e) => {
              e.stopPropagation();
              onDelete();
            }}
          >
            <Trash2 className="h-3 w-3" />
          </Button>
        </div>
      )}
    </div>
  );
}

// Canvas drop zone
function DesignCanvas({
  children,
  onDrop,
}: {
  children: React.ReactNode;
  onDrop: (position: { x: number; y: number }) => void;
}) {
  const { setNodeRef, isOver } = useDroppable({
    id: "design-canvas",
  });

  const handleCanvasClick = useCallback(
    (e: React.MouseEvent) => {
      if (e.target === e.currentTarget) {
        const rect = e.currentTarget.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        onDrop({ x, y });
      }
    },
    [onDrop],
  );

  return (
    <div
      ref={setNodeRef}
      className={`relative h-full w-full overflow-hidden bg-gray-50 ${
        isOver ? "bg-blue-50" : ""
      }`}
      style={{
        backgroundImage:
          "radial-gradient(circle, #d1d5db 1px, transparent 1px)",
        backgroundSize: "20px 20px",
      }}
      onClick={handleCanvasClick}
    >
      {children}
    </div>
  );
}

// Element editor dialog
function ElementEditor({
  element,
  isOpen,
  onClose,
  onSave,
}: {
  element: CanvasElement | null;
  isOpen: boolean;
  onClose: () => void;
  onSave: (element: CanvasElement) => void;
}) {
  const [editingElement, setEditingElement] = useState<CanvasElement | null>(
    element,
  );

  useEffect(() => {
    setEditingElement(element);
  }, [element]);

  if (!editingElement) return null;

  const handleSave = () => {
    onSave(editingElement);
    onClose();
  };

  return (
    <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="max-w-md">
        <DialogHeader>
          <DialogTitle>Edit Element</DialogTitle>
          <DialogDescription>
            Customize the properties of this element.
          </DialogDescription>
        </DialogHeader>

        <div className="space-y-4">
          <div>
            <Label htmlFor="title">Title</Label>
            <Input
              id="title"
              value={editingElement.title}
              onChange={(e) =>
                setEditingElement({
                  ...editingElement,
                  title: e.target.value,
                })
              }
            />
          </div>

          <div>
            <Label htmlFor="content">Content</Label>
            <Textarea
              id="content"
              value={editingElement.content}
              onChange={(e) =>
                setEditingElement({
                  ...editingElement,
                  content: e.target.value,
                })
              }
              rows={3}
            />
          </div>

          <div className="grid grid-cols-2 gap-4">
            <div>
              <Label htmlFor="width">Width</Label>
              <Input
                id="width"
                type="number"
                value={editingElement.size.width}
                onChange={(e) =>
                  setEditingElement({
                    ...editingElement,
                    size: {
                      ...editingElement.size,
                      width: parseInt(e.target.value) || 200,
                    },
                  })
                }
              />
            </div>
            <div>
              <Label htmlFor="height">Height</Label>
              <Input
                id="height"
                type="number"
                value={editingElement.size.height}
                onChange={(e) =>
                  setEditingElement({
                    ...editingElement,
                    size: {
                      ...editingElement.size,
                      height: parseInt(e.target.value) || 100,
                    },
                  })
                }
              />
            </div>
          </div>

          <div>
            <Label htmlFor="backgroundColor">Background Color</Label>
            <Input
              id="backgroundColor"
              type="color"
              value={editingElement.style.backgroundColor}
              onChange={(e) =>
                setEditingElement({
                  ...editingElement,
                  style: {
                    ...editingElement.style,
                    backgroundColor: e.target.value,
                  },
                })
              }
            />
          </div>
        </div>

        <DialogFooter>
          <Button variant="outline" onClick={onClose}>
            Cancel
          </Button>
          <Button onClick={handleSave}>Save Changes</Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}

export function FreeFormDesigner({
  experiment,
  onSave,
  initialDesign,
}: FreeFormDesignerProps) {
  const [design, setDesign] = useState<ExperimentDesign>(
    initialDesign || {
      id: experiment.id,
      name: experiment.name,
      elements: [],
      connections: [],
      canvasSettings: {
        zoom: 1,
        gridSize: 20,
        showGrid: true,
        backgroundColor: "#f9fafb",
      },
      version: 1,
      lastSaved: new Date(),
    },
  );

  const [selectedElement, setSelectedElement] = useState<string | null>(null);
  const [editingElement, setEditingElement] = useState<CanvasElement | null>(
    null,
  );
  const [isEditDialogOpen, setIsEditDialogOpen] = useState(false);
  const [draggedElement, setDraggedElement] = useState<any>(null);
  const canvasRef = useRef<HTMLDivElement>(null);

  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 8,
      },
    }),
  );

  const generateId = () =>
    `element-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

  const handleDragStart = (event: DragStartEvent) => {
    setDraggedElement(event.active.data.current);
  };

  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;

    if (!over || over.id !== "design-canvas") {
      setDraggedElement(null);
      return;
    }

    const rect = canvasRef.current?.getBoundingClientRect();
    if (!rect) return;

    const x =
      event.delta.x + (active.rect.current.translated?.left || 0) - rect.left;
    const y =
      event.delta.y + (active.rect.current.translated?.top || 0) - rect.top;

    const dragData = active.data.current;

    if (dragData?.type === "toolbox") {
      // Create new element from toolbox
      createNewElement(dragData.elementType, { x, y });
    } else if (dragData?.type === "canvas-element") {
      // Move existing element
      moveElement(dragData.element.id, { x, y });
    }

    setDraggedElement(null);
  };

  const createNewElement = (
    type: ElementType,
    position: { x: number; y: number },
  ) => {
    const config = elementTypeConfig[type];
    const newElement: CanvasElement = {
      id: generateId(),
      type,
      title: `New ${config.label}`,
      content: "Click to edit this element",
      position,
      size: { width: 200, height: 100 },
      style: {
        ...config.defaultStyle,
        fontSize: 14,
      },
      metadata: {},
      connections: [],
    };

    setDesign((prev) => ({
      ...prev,
      elements: [...prev.elements, newElement],
    }));
  };

  const moveElement = (
    elementId: string,
    newPosition: { x: number; y: number },
  ) => {
    setDesign((prev) => ({
      ...prev,
      elements: prev.elements.map((el) =>
        el.id === elementId ? { ...el, position: newPosition } : el,
      ),
    }));
  };

  const deleteElement = (elementId: string) => {
    setDesign((prev) => ({
      ...prev,
      elements: prev.elements.filter((el) => el.id !== elementId),
      connections: prev.connections.filter(
        (conn) => conn.from !== elementId && conn.to !== elementId,
      ),
    }));
    setSelectedElement(null);
  };

  const editElement = (element: CanvasElement) => {
    setEditingElement(element);
    setIsEditDialogOpen(true);
  };

  const saveElement = (updatedElement: CanvasElement) => {
    setDesign((prev) => ({
      ...prev,
      elements: prev.elements.map((el) =>
        el.id === updatedElement.id ? updatedElement : el,
      ),
    }));
  };

  const handleSave = () => {
    const updatedDesign = {
      ...design,
      lastSaved: new Date(),
    };
    setDesign(updatedDesign);
    onSave?.(updatedDesign);
  };

  const handleCanvasDrop = (position: { x: number; y: number }) => {
    // Deselect when clicking empty space
    setSelectedElement(null);
  };

  return (
    <div className="flex h-screen bg-white">
      {/* Toolbar */}
      <div className="w-64 border-r bg-gray-50 p-4">
        <div className="space-y-4">
          <div>
            <h3 className="font-medium text-gray-900">Element Toolbox</h3>
            <p className="text-sm text-gray-500">Drag elements to the canvas</p>
          </div>

          <div className="grid grid-cols-2 gap-3">
            {Object.entries(elementTypeConfig).map(([type, config]) => (
              <ToolboxElement key={type} type={type as ElementType} />
            ))}
          </div>

          <Separator />

          <div className="space-y-2">
            <Button onClick={handleSave} className="w-full">
              <Save className="mr-2 h-4 w-4" />
              Save Design
            </Button>

            <div className="grid grid-cols-2 gap-2">
              <Button variant="outline" size="sm">
                <Undo className="h-4 w-4" />
              </Button>
              <Button variant="outline" size="sm">
                <Redo className="h-4 w-4" />
              </Button>
            </div>

            <div className="grid grid-cols-2 gap-2">
              <Button variant="outline" size="sm">
                <ZoomIn className="h-4 w-4" />
              </Button>
              <Button variant="outline" size="sm">
                <ZoomOut className="h-4 w-4" />
              </Button>
            </div>
          </div>

          <Separator />

          <div className="space-y-2">
            <h4 className="text-sm font-medium text-gray-900">Design Info</h4>
            <div className="space-y-1 text-xs text-gray-500">
              <div>Elements: {design.elements.length}</div>
              <div>Last saved: {design.lastSaved.toLocaleTimeString()}</div>
              <div>Version: {design.version}</div>
            </div>
          </div>
        </div>
      </div>

      {/* Canvas */}
      <div className="relative flex-1">
        <DndContext
          sensors={sensors}
          collisionDetection={closestCenter}
          onDragStart={handleDragStart}
          onDragEnd={handleDragEnd}
        >
          <div ref={canvasRef} className="h-full">
            <DesignCanvas onDrop={handleCanvasDrop}>
              {design.elements.map((element) => (
                <CanvasElementComponent
                  key={element.id}
                  element={element}
                  isSelected={selectedElement === element.id}
                  onSelect={() => setSelectedElement(element.id)}
                  onEdit={() => editElement(element)}
                  onDelete={() => deleteElement(element.id)}
                />
              ))}
            </DesignCanvas>
          </div>

          <DragOverlay>
            {draggedElement?.type === "toolbox" && (
              <div className="rounded-lg border bg-white p-3 shadow-lg">
                {(() => {
                  const IconComponent =
                    elementTypeConfig[draggedElement.elementType as ElementType]
                      .icon;
                  return <IconComponent className="h-6 w-6" />;
                })()}
              </div>
            )}
            {draggedElement?.type === "canvas-element" && (
              <div className="rounded-lg border bg-white p-3 opacity-75 shadow-lg">
                {draggedElement.element.title}
              </div>
            )}
          </DragOverlay>
        </DndContext>
      </div>

      {/* Element Editor Dialog */}
      <ElementEditor
        element={editingElement}
        isOpen={isEditDialogOpen}
        onClose={() => setIsEditDialogOpen(false)}
        onSave={saveElement}
      />
    </div>
  );
}
