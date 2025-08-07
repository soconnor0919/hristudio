"use client";

import React, { useState, useCallback, useMemo } from "react";
import {
  DndContext,
  closestCenter,
  KeyboardSensor,
  PointerSensor,
  useSensor,
  useSensors,
  type DragEndEvent,
  type DragStartEvent,
  DragOverlay,
  useDraggable,
  useDroppable,
  type CollisionDetection,
  type DroppableContainer,
} from "@dnd-kit/core";
import {
  SortableContext,
  sortableKeyboardCoordinates,
  verticalListSortingStrategy,
  useSortable,
  arrayMove,
} from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
// Removed unused resizable imports
import { ScrollArea } from "~/components/ui/scroll-area";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Separator } from "~/components/ui/separator";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { toast } from "sonner";
import { cn } from "~/lib/utils";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { api } from "~/trpc/react";
import { useEffect } from "react";
import { PageHeader, ActionButton } from "~/components/ui/page-header";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import {
  Play,
  Users,
  Bot,
  GitBranch,
  Activity,
  Zap,
  Plus,
  Save,
  Download,
  Settings,
  GripVertical,
  Trash2,
  Clock,
  Palette,
} from "lucide-react";

// Types
type BlockShape = "action" | "control" | "hat" | "cap" | "boolean" | "value";
type BlockCategory =
  | "event"
  | "wizard"
  | "robot"
  | "control"
  | "sensor"
  | "logic";

interface BlockParameter {
  id: string;
  name: string;
  type: "text" | "number" | "select" | "boolean";
  value?: string | number | boolean;
  placeholder?: string;
  options?: string[];
  min?: number;
  max?: number;
  step?: number;
}

export interface ExperimentBlock {
  id: string;
  type: string;
  category: BlockCategory;
  shape: BlockShape;
  displayName: string;
  description: string;
  icon: string;
  color: string;
  parameters: BlockParameter[];
  children?: ExperimentBlock[];
  nestable?: boolean;
  order: number;
}

export interface BlockDesign {
  id: string;
  name: string;
  description: string;
  blocks: ExperimentBlock[];
  version: number;
  lastSaved: Date;
}

// Block Registry
class BlockRegistry {
  private static instance: BlockRegistry;
  private blocks = new Map<string, PluginBlockDefinition>();

  static getInstance(): BlockRegistry {
    if (!BlockRegistry.instance) {
      BlockRegistry.instance = new BlockRegistry();
      BlockRegistry.instance.initializeCoreBlocks();
    }
    return BlockRegistry.instance;
  }

  private initializeCoreBlocks() {
    const coreBlocks: PluginBlockDefinition[] = [
      // Events
      {
        type: "when_trial_starts",
        shape: "hat",
        category: "event",
        displayName: "when trial starts",
        description: "Triggered when the trial begins",
        icon: "Play",
        color: "#22c55e",
        parameters: [],
      },

      // Wizard Actions
      {
        type: "wizard_say",
        shape: "action",
        category: "wizard",
        displayName: "say",
        description: "Wizard speaks to participant",
        icon: "Users",
        color: "#a855f7",
        parameters: [
          {
            id: "message",
            name: "Message",
            type: "text",
            value: "",
            placeholder: "What should the wizard say?",
          },
        ],
      },
      {
        type: "wizard_gesture",
        shape: "action",
        category: "wizard",
        displayName: "gesture",
        description: "Wizard performs a gesture",
        icon: "Users",
        color: "#a855f7",
        parameters: [
          {
            id: "type",
            name: "Gesture",
            type: "select",
            value: "wave",
            options: ["wave", "point", "nod", "thumbs_up"],
          },
        ],
      },

      // Robot Actions
      {
        type: "robot_say",
        shape: "action",
        category: "robot",
        displayName: "say",
        description: "Robot speaks using text-to-speech",
        icon: "Bot",
        color: "#3b82f6",
        parameters: [
          {
            id: "text",
            name: "Text",
            type: "text",
            value: "",
            placeholder: "What should the robot say?",
          },
        ],
      },
      {
        type: "robot_move",
        shape: "action",
        category: "robot",
        displayName: "move",
        description: "Robot moves in specified direction",
        icon: "Bot",
        color: "#3b82f6",
        parameters: [
          {
            id: "direction",
            name: "Direction",
            type: "select",
            value: "forward",
            options: ["forward", "backward", "left", "right"],
          },
          {
            id: "distance",
            name: "Distance (m)",
            type: "number",
            value: 1,
            min: 0.1,
            max: 5,
            step: 0.1,
          },
        ],
      },
      {
        type: "robot_look_at",
        shape: "action",
        category: "robot",
        displayName: "look at",
        description: "Robot looks at target",
        icon: "Bot",
        color: "#3b82f6",
        parameters: [
          {
            id: "target",
            name: "Target",
            type: "select",
            value: "participant",
            options: ["participant", "object", "door"],
          },
        ],
      },

      // Control Flow
      {
        type: "wait",
        shape: "action",
        category: "control",
        displayName: "wait",
        description: "Pause execution for specified time",
        icon: "Clock",
        color: "#f97316",
        parameters: [
          {
            id: "seconds",
            name: "Seconds",
            type: "number",
            value: 1,
            min: 0.1,
            max: 60,
            step: 0.1,
          },
        ],
      },
      {
        type: "repeat",
        shape: "control",
        category: "control",
        displayName: "repeat",
        description: "Execute contained blocks multiple times",
        icon: "GitBranch",
        color: "#f97316",
        parameters: [
          {
            id: "times",
            name: "Times",
            type: "number",
            value: 3,
            min: 1,
            max: 20,
          },
        ],
        nestable: true,
      },
      {
        type: "if",
        shape: "control",
        category: "control",
        displayName: "if",
        description: "Conditional execution",
        icon: "GitBranch",
        color: "#f97316",
        parameters: [
          {
            id: "condition",
            name: "Condition",
            type: "select",
            value: "participant_speaks",
            options: ["participant_speaks", "object_detected", "timer_expired"],
          },
        ],
        nestable: true,
      },

      // Sensors
      {
        type: "observe",
        shape: "action",
        category: "sensor",
        displayName: "observe",
        description: "Record behavioral observations",
        icon: "Activity",
        color: "#16a34a",
        parameters: [
          {
            id: "what",
            name: "What to observe",
            type: "text",
            value: "",
            placeholder: "e.g., participant engagement",
          },
          {
            id: "duration",
            name: "Duration (s)",
            type: "number",
            value: 5,
            min: 1,
            max: 60,
          },
        ],
      },
    ];

    coreBlocks.forEach((block) => this.blocks.set(block.type, block));
  }

  registerBlock(blockDef: PluginBlockDefinition) {
    this.blocks.set(blockDef.type, blockDef);
  }

  getBlock(type: string): PluginBlockDefinition | undefined {
    return this.blocks.get(type);
  }

  getBlocksByCategory(category: BlockCategory): PluginBlockDefinition[] {
    return Array.from(this.blocks.values()).filter(
      (b) => b.category === category,
    );
  }

  createBlock(type: string, order: number): ExperimentBlock {
    const blockDef = this.blocks.get(type);
    if (!blockDef) {
      throw new Error(`Block type ${type} not found`);
    }

    return {
      id: `block_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      type: blockDef.type,
      category: blockDef.category,
      shape: blockDef.shape,
      displayName: blockDef.displayName,
      description: blockDef.description,
      icon: blockDef.icon,
      color: blockDef.color,
      parameters: blockDef.parameters.map((param) => ({ ...param })),
      children: blockDef.nestable ? [] : undefined,
      nestable: blockDef.nestable,
      order,
    };
  }
}

interface PluginBlockDefinition {
  type: string;
  shape: BlockShape;
  category: BlockCategory;
  displayName: string;
  description: string;
  icon: string;
  color: string;
  parameters: BlockParameter[];
  nestable?: boolean;
}

// Icon mapping
const IconComponents: Record<
  string,
  React.ComponentType<{ className?: string }>
> = {
  Play,
  Users,
  Bot,
  GitBranch,
  Activity,
  Zap,
  Clock,
};

// Draggable Palette Block
interface DraggablePaletteBlockProps {
  blockDef: PluginBlockDefinition;
  showParameterPreview?: boolean;
}

function DraggablePaletteBlock({
  blockDef,
  showParameterPreview,
}: DraggablePaletteBlockProps) {
  const { attributes, listeners, setNodeRef, transform, isDragging } =
    useDraggable({
      id: `palette-${blockDef.type}`,
      data: { blockType: blockDef.type, isFromPalette: true },
    });

  const style = {
    transform: transform
      ? `translate3d(${transform.x}px, ${transform.y}px, 0)`
      : undefined,
    zIndex: isDragging ? 1000 : undefined,
  };

  const IconComponent = IconComponents[blockDef.icon] ?? Bot;

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...listeners}
      {...attributes}
      className={cn(
        "group border-border bg-card cursor-grab rounded-lg border p-3 shadow-sm transition-all",
        "hover:bg-accent hover:shadow-md active:cursor-grabbing",
        isDragging && "opacity-50",
      )}
    >
      <div className="flex items-center gap-3">
        <div
          className="flex h-8 w-8 items-center justify-center rounded-lg shadow-sm"
          style={{ backgroundColor: blockDef.color }}
        >
          <IconComponent className="text-primary-foreground h-4 w-4" />
        </div>
        <div className="min-w-0 flex-1">
          <div className="group-hover:text-accent-foreground truncate text-sm font-semibold">
            {blockDef.displayName}
          </div>
          <div className="text-muted-foreground mt-0.5 truncate text-xs">
            {blockDef.description}
          </div>
          {showParameterPreview && blockDef.parameters.length > 0 && (
            <div className="mt-1 flex flex-wrap gap-1">
              {blockDef.parameters.slice(0, 2).map((param, idx) => (
                <Badge key={idx} variant="outline" className="text-xs">
                  {param.name}
                </Badge>
              ))}
              {blockDef.parameters.length > 2 && (
                <span className="text-muted-foreground text-xs">
                  +{blockDef.parameters.length - 2}
                </span>
              )}
            </div>
          )}
        </div>
        <GripVertical className="text-muted-foreground h-4 w-4 opacity-0 transition-opacity group-hover:opacity-100" />
      </div>
    </div>
  );
}

// Block Palette
interface BlockPaletteProps {
  showParameterPreview?: boolean;
}

function BlockPalette({ showParameterPreview = false }: BlockPaletteProps) {
  const registry = BlockRegistry.getInstance();
  const categories: BlockCategory[] = [
    "event",
    "wizard",
    "robot",
    "control",
    "sensor",
  ];
  const [activeCategory, setActiveCategory] = useState<BlockCategory>("wizard");

  const categoryConfig = {
    event: { label: "Events", icon: Play, color: "bg-green-500" },
    wizard: { label: "Wizard", icon: Users, color: "bg-purple-500" },
    robot: { label: "Robot", icon: Bot, color: "bg-blue-500" },
    control: { label: "Control", icon: GitBranch, color: "bg-orange-500" },
    sensor: { label: "Sensors", icon: Activity, color: "bg-green-600" },
    logic: { label: "Logic", icon: Zap, color: "bg-pink-500" },
  };

  return (
    <div className="flex h-full flex-col">
      <div className="border-b p-3">
        <div className="grid grid-cols-1 gap-1">
          {categories.map((category) => {
            const config = categoryConfig[category];
            const IconComponent = config.icon;
            const isActive = activeCategory === category;
            return (
              <Button
                key={category}
                variant={isActive ? "default" : "ghost"}
                size="sm"
                className={cn(
                  "h-8 justify-start text-xs font-medium transition-all",
                  isActive
                    ? "bg-primary text-primary-foreground hover:bg-primary/90 shadow-sm"
                    : "hover:bg-accent hover:text-accent-foreground text-muted-foreground",
                )}
                onClick={() => setActiveCategory(category)}
              >
                <IconComponent className="mr-2 h-3.5 w-3.5" />
                {config.label}
              </Button>
            );
          })}
        </div>
      </div>

      <ScrollArea className="flex-1">
        <div className="space-y-2 p-3">
          {registry.getBlocksByCategory(activeCategory).map((blockDef) => (
            <DraggablePaletteBlock
              key={blockDef.type}
              blockDef={blockDef}
              showParameterPreview={showParameterPreview}
            />
          ))}
        </div>
      </ScrollArea>
    </div>
  );
}

// Droppable Container for control blocks
interface DroppableContainerProps {
  id: string;
  isEmpty: boolean;
  children?: React.ReactNode;
  isMainCanvas?: boolean;
}

function DroppableContainer({
  id,
  isEmpty,
  children,
  isMainCanvas = false,
}: DroppableContainerProps) {
  const { isOver, setNodeRef } = useDroppable({ id });

  if (isMainCanvas && !isEmpty) {
    // Main canvas with content - no special styling
    return (
      <div ref={setNodeRef} className="min-h-full">
        {children}
      </div>
    );
  }

  return (
    <div
      ref={setNodeRef}
      className={cn(
        "min-h-[40px] rounded-lg border-2 border-dashed p-3 transition-colors",
        isOver ? "border-primary bg-accent/20" : "border-muted-foreground/30",
        isEmpty && "bg-muted/20",
        isMainCanvas && isEmpty && "min-h-[120px]",
      )}
    >
      {isEmpty ? (
        <div className="flex items-center justify-center text-center">
          {isMainCanvas ? (
            <div>
              <Plus className="text-muted-foreground mx-auto mb-2 h-8 w-8" />
              <p className="text-muted-foreground text-sm">
                Drag blocks from the palette to build your experiment
              </p>
            </div>
          ) : (
            <div className="text-muted-foreground py-1 text-center text-xs">
              Drop blocks here
            </div>
          )}
        </div>
      ) : (
        children
      )}
    </div>
  );
}

// Sortable Block Component
interface SortableBlockProps {
  block: ExperimentBlock;
  isSelected: boolean;
  selectedBlockId: string | null;
  onSelect: () => void;
  onDelete: () => void;
  onAddToControl?: (parentId: string, childId: string) => void;
  onRemoveFromControl?: (parentId: string, childId: string) => void;
  level?: number;
}

function SortableBlock({
  block,
  isSelected,
  selectedBlockId,
  onSelect,
  onDelete,
  onAddToControl,
  onRemoveFromControl,
  level = 0,
}: SortableBlockProps) {
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({
    id: block.id,
  });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
  };

  const IconComponent = IconComponents[block.icon] ?? Bot;

  const renderParameterPreview = () => {
    if (block.parameters.length === 0) return null;

    return block.parameters.slice(0, 2).map((param) => (
      <span
        key={param.id}
        className="inline-flex items-center rounded bg-black/20 px-1.5 py-0.5 text-xs text-white"
      >
        {param.type === "text" && param.value && `"${param.value}"`}
        {param.type === "number" && param.value}
        {param.type === "select" && param.value}
        {param.type === "boolean" && (param.value ? "✓" : "✗")}
      </span>
    ));
  };

  const baseClasses = cn(
    "group relative flex items-center gap-3 rounded-lg border px-4 py-3 text-sm font-medium transition-all",
    "hover:shadow-md cursor-pointer select-none",
    isSelected && "ring-2 ring-primary ring-offset-2 ring-offset-background",
    isDragging && "opacity-30 shadow-2xl scale-105 rotate-1",
    level > 0 && "ml-4",
  );

  const renderBlock = () => {
    switch (block.shape) {
      case "hat":
        return (
          <div className="relative">
            <div
              className="absolute -top-2 right-4 left-4 h-2 rounded-t-lg"
              style={{ backgroundColor: block.color }}
            />
            <div
              className={cn(baseClasses, "pt-4 text-white")}
              style={{ backgroundColor: block.color }}
            >
              <div
                {...listeners}
                className="cursor-grab rounded p-1 hover:bg-black/10 active:cursor-grabbing"
              >
                <GripVertical className="h-4 w-4 opacity-50 group-hover:opacity-100" />
              </div>
              <div className="flex flex-1 items-center gap-2">
                <IconComponent className="h-4 w-4 flex-shrink-0" />
                <span className="truncate font-medium">
                  {block.displayName}
                </span>
              </div>
            </div>
          </div>
        );

      case "control":
        return (
          <div className="space-y-2">
            <div
              className={cn(baseClasses, "text-white")}
              style={{ backgroundColor: block.color }}
            >
              <div
                {...listeners}
                className="cursor-grab rounded p-1 hover:bg-black/10 active:cursor-grabbing"
              >
                <GripVertical className="h-4 w-4 opacity-50 group-hover:opacity-100" />
              </div>
              <div className="flex flex-1 items-center gap-2">
                <IconComponent className="h-4 w-4 flex-shrink-0" />
                <span className="truncate font-medium">
                  {block.displayName}
                </span>
                <div className="flex items-center gap-1">
                  {renderParameterPreview()}
                </div>
              </div>
              <Button
                variant="ghost"
                size="sm"
                className="h-6 w-6 p-0 text-white opacity-0 group-hover:opacity-100 hover:bg-black/20"
                onClick={(e) => {
                  e.stopPropagation();
                  onDelete();
                }}
              >
                <Trash2 className="h-3 w-3" />
              </Button>
            </div>
            {block.nestable && (
              <div className="ml-6">
                <DroppableContainer
                  id={`control-${block.id}`}
                  isEmpty={!block.children || block.children.length === 0}
                >
                  {block.children && block.children.length > 0 && (
                    <SortableContext
                      items={block.children.map((c) => c.id)}
                      strategy={verticalListSortingStrategy}
                    >
                      <div className="space-y-2">
                        {block.children.map((child) => (
                          <SortableBlock
                            key={child.id}
                            block={child}
                            isSelected={selectedBlockId === child.id}
                            selectedBlockId={selectedBlockId}
                            onSelect={() => onSelect()}
                            onDelete={() =>
                              onRemoveFromControl?.(block.id, child.id)
                            }
                            onAddToControl={onAddToControl}
                            onRemoveFromControl={onRemoveFromControl}
                            level={level + 1}
                          />
                        ))}
                      </div>
                    </SortableContext>
                  )}
                </DroppableContainer>
              </div>
            )}
          </div>
        );

      default:
        return (
          <div
            className={cn(baseClasses, "text-white")}
            style={{ backgroundColor: block.color }}
          >
            <div
              {...listeners}
              className="cursor-grab rounded p-1 hover:bg-black/10 active:cursor-grabbing"
            >
              <GripVertical className="h-4 w-4 opacity-50 group-hover:opacity-100" />
            </div>
            <div className="flex flex-1 items-center gap-2">
              <IconComponent className="h-4 w-4 flex-shrink-0" />
              <span className="truncate font-medium">{block.displayName}</span>
              <div className="flex items-center gap-1">
                {renderParameterPreview()}
              </div>
            </div>
            <Button
              variant="ghost"
              size="sm"
              className="h-6 w-6 p-0 text-white opacity-0 group-hover:opacity-100 hover:bg-black/20"
              onClick={(e) => {
                e.stopPropagation();
                onDelete();
              }}
            >
              <Trash2 className="h-3 w-3" />
            </Button>
          </div>
        );
    }
  };

  return (
    <div ref={setNodeRef} style={style} {...attributes} onClick={onSelect}>
      {renderBlock()}
    </div>
  );
}

// Main Designer Component
interface EnhancedBlockDesignerProps {
  experimentId: string;
  initialDesign?: BlockDesign;
  onSave?: (design: BlockDesign) => void;
}

export function EnhancedBlockDesigner({
  experimentId,
  initialDesign,
  onSave,
}: EnhancedBlockDesignerProps) {
  const registry = BlockRegistry.getInstance();

  // Add error logging for debugging
  useEffect(() => {
    console.log("Designer mounted with:", { experimentId, initialDesign });
  }, [experimentId, initialDesign]);

  const [design, setDesign] = useState<BlockDesign>(() => {
    const defaultDesign = {
      id: experimentId,
      name: "New Experiment",
      description: "",
      blocks: [] as ExperimentBlock[],
      version: 1,
      lastSaved: new Date(),
    };

    if (initialDesign) {
      console.log("Using existing design:", initialDesign);
      return initialDesign;
    }

    // Create default "when trial starts" block if no initial design
    try {
      defaultDesign.blocks = [registry.createBlock("when_trial_starts", 0)];
      console.log("Created default design with when_trial_starts block");
    } catch (error) {
      console.error("Failed to create default block:", error);
      defaultDesign.blocks = [];
    }
    return defaultDesign;
  });

  const [selectedBlockId, setSelectedBlockId] = useState<string | null>(null);
  const [activeId, setActiveId] = useState<string | null>(null);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);

  // API mutation for saving
  const updateExperiment = api.experiments.update.useMutation({
    onSuccess: () => {
      setHasUnsavedChanges(false);
      toast.success("Design saved successfully");
    },
    onError: (error) => {
      toast.error("Failed to save design: " + error.message);
    },
  });

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Experiments", href: "/experiments" },
    { label: design.name, href: `/experiments/${experimentId}` },
    { label: "Designer" },
  ]);

  // DnD sensors with improved collision detection
  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: { distance: 5 },
    }),
    useSensor(KeyboardSensor, {
      coordinateGetter: sortableKeyboardCoordinates,
    }),
  );

  // Helper functions for nested block operations
  const findBlockById = useCallback(
    (id: string, blocks: ExperimentBlock[]): ExperimentBlock | null => {
      for (const block of blocks) {
        if (block.id === id) return block;
        if (block.children) {
          const found = findBlockById(id, block.children);
          if (found) return found;
        }
      }
      return null;
    },
    [],
  );

  const removeBlockFromStructure = useCallback(
    (id: string, blocks: ExperimentBlock[]): ExperimentBlock[] => {
      return blocks
        .filter((block) => block.id !== id)
        .map((block) => ({
          ...block,
          children: block.children
            ? removeBlockFromStructure(id, block.children)
            : block.children,
        }));
    },
    [],
  );

  // Custom collision detection for nested blocks
  const customCollisionDetection = useCallback<CollisionDetection>((args) => {
    if (!args.pointerCoordinates) return closestCenter(args);

    // First check for droppable containers (control blocks and main canvas)
    const droppableCollisions =
      args.droppableContainers
        ?.filter(
          (container) =>
            container.id?.toString().startsWith("control-") ||
            container.id === "main-canvas",
        )
        ?.map((container) => {
          // Handle rect being a ref or direct object
          const rect =
            "current" in container.rect
              ? container.rect.current
              : container.rect;
          if (!rect) return null;

          return {
            id: container.id,
            data: container.data,
            rect,
          };
        })
        ?.filter(Boolean) ?? [];

    if (droppableCollisions.length > 0) {
      // Return the closest droppable container
      let closest:
        | ((typeof droppableCollisions)[0] & { distance: number })
        | null = null;

      for (const current of droppableCollisions) {
        if (!current?.rect) continue;

        const distance = Math.sqrt(
          Math.pow(
            args.pointerCoordinates.x -
              current.rect.left -
              current.rect.width / 2,
            2,
          ) +
            Math.pow(
              args.pointerCoordinates.y -
                current.rect.top -
                current.rect.height / 2,
              2,
            ),
        );

        if (distance < (closest?.distance ?? Infinity)) {
          closest = { ...current, distance };
        }
      }

      if (closest) return [closest];
    }

    // Fall back to default collision detection
    return closestCenter(args);
  }, []);

  // Handle drag start
  const handleDragStart = useCallback((event: DragStartEvent) => {
    setActiveId(event.active.id as string);
  }, []);

  // Handle drag end
  const handleDragEnd = useCallback(
    (event: DragEndEvent) => {
      const { active, over } = event;
      setActiveId(null);

      if (!over) return;

      const activeId = active.id.toString();
      const overId = over.id.toString();

      // Handle drop from palette
      if (typeof activeId === "string" && activeId.startsWith("palette-")) {
        const blockType = activeId.replace("palette-", "");

        // Dropping into control block
        if (overId.startsWith("control-")) {
          const controlId = overId.replace("control-", "");
          const newBlock = registry.createBlock(blockType, 0);

          setDesign((prev) => ({
            ...prev,
            blocks: prev.blocks.map((block) =>
              block.id === controlId
                ? {
                    ...block,
                    children: [...(block.children ?? []), newBlock],
                  }
                : block,
            ),
          }));

          setHasUnsavedChanges(true);
          toast.success(`Added ${newBlock.displayName} to control block`);
          return;
        }

        // Dropping in main area or main canvas
        if (overId === "main-canvas" || !overId.includes("-")) {
          const newBlock = registry.createBlock(
            blockType,
            design.blocks.length,
          );
          setDesign((prev) => ({
            ...prev,
            blocks: [...prev.blocks, newBlock],
          }));

          setHasUnsavedChanges(true);
          toast.success(`Added ${newBlock.displayName} block`);
          return;
        }
      }

      // Handle dragging blocks out of control structures to main canvas
      if (
        overId === "main-canvas" &&
        !activeId.toString().startsWith("palette-")
      ) {
        const draggedBlock = findBlockById(activeId, design.blocks);
        if (!draggedBlock) return;

        setDesign((prev) => {
          // Remove from any control structure
          const newBlocks = removeBlockFromStructure(activeId, prev.blocks);

          // Add to main blocks if not already there
          if (!newBlocks.some((b) => b.id === activeId)) {
            newBlocks.push(draggedBlock);
          }

          return { ...prev, blocks: newBlocks };
        });

        setHasUnsavedChanges(true);
        toast.success("Block moved to main flow");
        return;
      }

      // Handle reordering existing blocks or moving into control structures
      if (typeof overId === "string" && overId.startsWith("control-")) {
        const controlId = overId.replace("control-", "");
        const draggedBlock = findBlockById(activeId, design.blocks);

        if (!draggedBlock) return;

        setDesign((prev) => {
          // Remove from current location
          const newBlocks = removeBlockFromStructure(activeId, prev.blocks);

          // Add to control block
          const updatedBlocks = newBlocks.map((block) =>
            block.id === controlId
              ? {
                  ...block,
                  children: [...(block.children ?? []), draggedBlock],
                }
              : block,
          );

          return { ...prev, blocks: updatedBlocks };
        });

        setHasUnsavedChanges(true);
        toast.success("Block moved to control structure");
        return;
      }

      // Normal reordering within main blocks
      if (activeId !== overId && !overId.includes("-")) {
        setDesign((prev) => {
          const activeIndex = prev.blocks.findIndex(
            (block) => block.id === activeId,
          );
          const overIndex = prev.blocks.findIndex(
            (block) => block.id === overId,
          );

          if (activeIndex !== -1 && overIndex !== -1) {
            const newBlocks = arrayMove(prev.blocks, activeIndex, overIndex);
            newBlocks.forEach((block, index) => {
              block.order = index;
            });

            return { ...prev, blocks: newBlocks };
          }
          return prev;
        });
        setHasUnsavedChanges(true);
      }
    },
    [design.blocks, registry, findBlockById, removeBlockFromStructure],
  );

  // Handle block selection
  const handleBlockSelect = useCallback((blockId: string) => {
    setSelectedBlockId((prev) => (prev === blockId ? null : blockId));
  }, []);

  // Handle block deletion
  const handleBlockDelete = useCallback(
    (blockId: string) => {
      setDesign((prev) => ({
        ...prev,
        blocks: prev.blocks.filter((block) => block.id !== blockId),
      }));

      if (selectedBlockId === blockId) {
        setSelectedBlockId(null);
      }

      setHasUnsavedChanges(true);
      toast.success("Block deleted");
    },
    [selectedBlockId],
  );

  // Handle removal from control structure
  const handleRemoveFromControl = useCallback(
    (parentId: string, childId: string) => {
      setDesign((prev) => ({
        ...prev,
        blocks: prev.blocks.map((block) => {
          if (block.id === parentId && block.children) {
            return {
              ...block,
              children: block.children.filter((child) => child.id !== childId),
            };
          }
          return block;
        }),
      }));

      setHasUnsavedChanges(true);
      toast.success("Block removed from control structure");
    },
    [],
  );

  // Handle parameter changes
  const handleParameterChange = useCallback(
    (
      blockId: string,
      parameterId: string,
      value: string | number | boolean,
    ) => {
      setDesign((prev) => ({
        ...prev,
        blocks: prev.blocks.map((block) => {
          if (block.id === blockId) {
            return {
              ...block,
              parameters: block.parameters.map((param) =>
                param.id === parameterId ? { ...param, value } : param,
              ),
            };
          }
          // Also check children in control blocks
          if (block.children) {
            return {
              ...block,
              children: block.children.map((child) =>
                child.id === blockId
                  ? {
                      ...child,
                      parameters: child.parameters.map((param) =>
                        param.id === parameterId ? { ...param, value } : param,
                      ),
                    }
                  : child,
              ),
            };
          }
          return block;
        }),
      }));
      setHasUnsavedChanges(true);
    },
    [],
  );

  // Save design
  const handleSave = useCallback(() => {
    console.log("Saving design:", design);
    const visualDesign = {
      blocks: design.blocks,
      version: design.version,
      lastSaved: new Date().toISOString(),
    };

    updateExperiment.mutate({
      id: experimentId,
      visualDesign,
    });

    if (onSave) {
      const updatedDesign = { ...design, lastSaved: new Date() };
      setDesign(updatedDesign);
      onSave(updatedDesign);
    }
  }, [design, experimentId, onSave, updateExperiment]);

  // Find selected block (including in children)
  const selectedBlock = useMemo(() => {
    if (!selectedBlockId) return null;

    for (const block of design.blocks) {
      if (block.id === selectedBlockId) return block;
      if (block.children) {
        const childBlock = block.children.find((c) => c.id === selectedBlockId);
        if (childBlock) return childBlock;
      }
    }
    return null;
  }, [selectedBlockId, design.blocks]);

  return (
    <DndContext
      sensors={sensors}
      collisionDetection={customCollisionDetection}
      onDragStart={handleDragStart}
      onDragEnd={handleDragEnd}
    >
      <div className="space-y-6">
        {/* Page Header */}
        <PageHeader
          title={design.name}
          description="Design your experiment protocol using visual blocks"
          icon={Palette}
          actions={
            <div className="flex items-center gap-2">
              {hasUnsavedChanges && (
                <Badge
                  variant="outline"
                  className="border-orange-200 text-orange-600"
                >
                  Unsaved Changes
                </Badge>
              )}
              <Badge variant="secondary" className="text-xs">
                {design.blocks.length} blocks
              </Badge>
              <ActionButton
                onClick={handleSave}
                disabled={!hasUnsavedChanges || updateExperiment.isPending}
              >
                <Save className="mr-2 h-4 w-4" />
                {updateExperiment.isPending ? "Saving..." : "Save"}
              </ActionButton>
              <ActionButton variant="outline">
                <Download className="mr-2 h-4 w-4" />
                Export
              </ActionButton>
            </div>
          }
        />

        {/* Main Designer */}
        <div className="grid grid-cols-12 gap-6">
          {/* Block Palette */}
          <div className="col-span-3">
            <Card>
              <CardHeader className="pb-3">
                <CardTitle className="flex items-center gap-2 text-sm">
                  <Palette className="h-4 w-4" />
                  Block Library
                </CardTitle>
              </CardHeader>
              <CardContent className="p-0">
                <BlockPalette showParameterPreview={true} />
              </CardContent>
            </Card>
          </div>

          {/* Block Canvas */}
          <div className="col-span-6">
            <Card className="h-[calc(100vh-12rem)]">
              <CardHeader className="pb-3">
                <CardTitle className="flex items-center gap-2 text-sm">
                  <Play className="h-4 w-4" />
                  Experiment Flow
                </CardTitle>
                <p className="text-muted-foreground text-xs">
                  Drag blocks from the palette • Click to select • Drag to
                  reorder
                </p>
              </CardHeader>
              <CardContent className="flex-1 overflow-hidden p-0">
                <ScrollArea className="h-full">
                  <div className="p-6">
                    <DroppableContainer
                      id="main-canvas"
                      isEmpty={design.blocks.length === 0}
                      isMainCanvas={true}
                    >
                      {design.blocks.length > 0 && (
                        <SortableContext
                          items={design.blocks.map((b) => b.id)}
                          strategy={verticalListSortingStrategy}
                        >
                          <div className="space-y-3">
                            {design.blocks.map((block) => (
                              <SortableBlock
                                key={block.id}
                                block={block}
                                isSelected={selectedBlockId === block.id}
                                selectedBlockId={selectedBlockId}
                                onSelect={() => handleBlockSelect(block.id)}
                                onDelete={() => handleBlockDelete(block.id)}
                                onRemoveFromControl={handleRemoveFromControl}
                              />
                            ))}
                          </div>
                        </SortableContext>
                      )}
                    </DroppableContainer>
                  </div>
                </ScrollArea>
              </CardContent>
            </Card>
          </div>

          {/* Properties Panel */}
          <div className="col-span-3">
            <Card>
              <CardHeader className="pb-3">
                <CardTitle className="flex items-center gap-2 text-sm">
                  <Settings className="h-4 w-4" />
                  Properties
                </CardTitle>
              </CardHeader>
              <CardContent>
                {selectedBlock ? (
                  <div className="space-y-4">
                    <div>
                      <div className="mb-3 flex items-center gap-3">
                        <div
                          className="flex h-8 w-8 items-center justify-center rounded-lg"
                          style={{ backgroundColor: selectedBlock.color }}
                        >
                          {IconComponents[selectedBlock.icon] &&
                            React.createElement(
                              IconComponents[selectedBlock.icon] ?? Bot,
                              {
                                className: "h-4 w-4 text-white",
                              },
                            )}
                        </div>
                        <div>
                          <div className="text-sm font-semibold">
                            {selectedBlock.displayName}
                          </div>
                          <div className="text-muted-foreground text-xs capitalize">
                            {selectedBlock.category} • {selectedBlock.shape}
                          </div>
                        </div>
                      </div>
                      <p className="text-muted-foreground text-sm">
                        {selectedBlock.description}
                      </p>
                    </div>

                    {selectedBlock.parameters.length > 0 && (
                      <>
                        <Separator />
                        <div className="space-y-3">
                          <Label className="text-sm font-medium">
                            Parameters
                          </Label>
                          {selectedBlock.parameters.map((param) => (
                            <div key={param.id} className="space-y-1">
                              <Label className="text-xs font-medium">
                                {param.name}
                              </Label>

                              {param.type === "text" && (
                                <Input
                                  value={
                                    typeof param.value === "string"
                                      ? param.value
                                      : ""
                                  }
                                  placeholder={param.placeholder}
                                  onChange={(e) =>
                                    handleParameterChange(
                                      selectedBlock.id,
                                      param.id,
                                      e.target.value,
                                    )
                                  }
                                  className="h-8"
                                />
                              )}

                              {param.type === "number" && (
                                <Input
                                  type="number"
                                  value={
                                    typeof param.value === "number"
                                      ? param.value
                                      : 0
                                  }
                                  min={param.min}
                                  max={param.max}
                                  step={param.step}
                                  onChange={(e) =>
                                    handleParameterChange(
                                      selectedBlock.id,
                                      param.id,
                                      parseFloat(e.target.value) || 0,
                                    )
                                  }
                                  className="h-8"
                                />
                              )}

                              {param.type === "select" && (
                                <Select
                                  value={
                                    typeof param.value === "string"
                                      ? param.value
                                      : ""
                                  }
                                  onValueChange={(value) =>
                                    handleParameterChange(
                                      selectedBlock.id,
                                      param.id,
                                      value,
                                    )
                                  }
                                >
                                  <SelectTrigger className="h-8">
                                    <SelectValue />
                                  </SelectTrigger>
                                  <SelectContent>
                                    {param.options?.map((option) => (
                                      <SelectItem key={option} value={option}>
                                        {option}
                                      </SelectItem>
                                    ))}
                                  </SelectContent>
                                </Select>
                              )}
                            </div>
                          ))}
                        </div>
                      </>
                    )}
                  </div>
                ) : (
                  <div className="flex h-32 items-center justify-center text-center">
                    <div>
                      <Settings className="text-muted-foreground/50 mx-auto mb-2 h-8 w-8" />
                      <h3 className="mb-1 text-sm font-medium">
                        Select a Block
                      </h3>
                      <p className="text-muted-foreground text-xs">
                        Click on a block to edit its properties
                      </p>
                    </div>
                  </div>
                )}
              </CardContent>
            </Card>
          </div>
        </div>

        {/* Drag Overlay */}
        <DragOverlay>
          {activeId ? (
            <div className="pointer-events-none opacity-80">
              {typeof activeId === "string" &&
              activeId.startsWith("palette-") ? (
                <div className="bg-popover rounded-lg border px-3 py-2 text-sm font-medium shadow-lg">
                  {
                    registry.getBlock(activeId.replace("palette-", ""))
                      ?.displayName
                  }
                </div>
              ) : (
                (() => {
                  const draggedBlock = findBlockById(activeId, design.blocks);
                  return draggedBlock ? (
                    <div
                      className="rounded-lg border px-3 py-2 text-sm font-medium text-white shadow-lg"
                      style={{ backgroundColor: draggedBlock.color }}
                    >
                      {draggedBlock.displayName}
                    </div>
                  ) : null;
                })()
              )}
            </div>
          ) : null}
        </DragOverlay>
      </div>
    </DndContext>
  );
}
