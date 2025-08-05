"use client";

import React, { useState, useCallback } from "react";
import {
  DndContext,
  DragEndEvent,
  DragStartEvent,
  useSensors,
  useSensor,
  PointerSensor,
  KeyboardSensor,
  DragOverlay,
  closestCenter,
} from "@dnd-kit/core";
import {
  SortableContext,
  verticalListSortingStrategy,
  arrayMove,
} from "@dnd-kit/sortable";
import { useDroppable } from "@dnd-kit/core";
import { useSortable } from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import {
  MessageSquare,
  Bot,
  Users,
  ArrowRight,
  Eye,
  Clock,
  Play,
  GitBranch,
  Repeat,
  Plus,
  Save,
  Download,
  Upload,
  Trash2,
  Settings,
  GripVertical,
  Hash,
  Hand,
  Volume2,
  Activity,
  Zap,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Separator } from "~/components/ui/separator";
import {
  ResizablePanelGroup,
  ResizablePanel,
  ResizableHandle,
} from "~/components/ui/resizable";
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

// Types
export type BlockShape =
  | "action"
  | "control"
  | "value"
  | "boolean"
  | "hat"
  | "cap";
export type BlockCategory =
  | "wizard"
  | "robot"
  | "control"
  | "sensor"
  | "logic"
  | "event";

export interface BlockParameter {
  id: string;
  name: string;
  type: "text" | "number" | "select" | "boolean";
  value: any;
  options?: string[];
  placeholder?: string;
  required?: boolean;
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
  description?: string;
  blocks: ExperimentBlock[];
  version: number;
  lastSaved: Date;
}

export interface PluginBlockDefinition {
  type: string;
  shape: BlockShape;
  category: BlockCategory;
  displayName: string;
  description: string;
  icon: string;
  color: string;
  parameters: Omit<BlockParameter, "id" | "value">[];
  nestable?: boolean;
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

  private initializeCoreBlocks(): void {
    // Wizard Actions
    this.registerBlock("wizard_speak", {
      type: "wizard_speak",
      shape: "action",
      category: "wizard",
      displayName: "say",
      description: "Wizard speaks to participant",
      icon: "MessageSquare",
      color: "#9966FF",
      parameters: [
        {
          name: "message",
          type: "text",
          placeholder: "Hello!",
          required: true,
        },
      ],
    });

    this.registerBlock("wizard_gesture", {
      type: "wizard_gesture",
      shape: "action",
      category: "wizard",
      displayName: "gesture",
      description: "Wizard performs gesture",
      icon: "Hand",
      color: "#9966FF",
      parameters: [
        {
          name: "type",
          type: "select",
          options: ["wave", "point", "nod", "thumbs_up"],
          required: true,
        },
      ],
    });

    // Robot Actions
    this.registerBlock("robot_speak", {
      type: "robot_speak",
      shape: "action",
      category: "robot",
      displayName: "say",
      description: "Robot speaks using TTS",
      icon: "Volume2",
      color: "#4C97FF",
      parameters: [
        {
          name: "text",
          type: "text",
          placeholder: "Hello, I'm ready!",
          required: true,
        },
      ],
    });

    this.registerBlock("robot_move", {
      type: "robot_move",
      shape: "action",
      category: "robot",
      displayName: "move",
      description: "Robot moves in direction",
      icon: "ArrowRight",
      color: "#4C97FF",
      parameters: [
        {
          name: "direction",
          type: "select",
          options: ["forward", "backward", "left", "right"],
          required: true,
        },
        {
          name: "distance",
          type: "number",
          min: 0.1,
          max: 5.0,
          step: 0.1,
          required: true,
        },
      ],
    });

    this.registerBlock("robot_look", {
      type: "robot_look",
      shape: "action",
      category: "robot",
      displayName: "look at",
      description: "Robot looks at target",
      icon: "Eye",
      color: "#4C97FF",
      parameters: [
        {
          name: "target",
          type: "select",
          options: ["participant", "object", "door"],
          required: true,
        },
      ],
    });

    // Control Flow
    this.registerBlock("wait", {
      type: "wait",
      shape: "action",
      category: "control",
      displayName: "wait",
      description: "Pause for seconds",
      icon: "Clock",
      color: "#FFAB19",
      parameters: [
        {
          name: "seconds",
          type: "number",
          min: 0.1,
          max: 60,
          step: 0.1,
          required: true,
        },
      ],
    });

    this.registerBlock("repeat", {
      type: "repeat",
      shape: "control",
      category: "control",
      displayName: "repeat",
      description: "Repeat actions multiple times",
      icon: "Repeat",
      color: "#FFAB19",
      parameters: [
        {
          name: "times",
          type: "number",
          min: 1,
          max: 20,
          required: true,
        },
      ],
      nestable: true,
    });

    this.registerBlock("if", {
      type: "if",
      shape: "control",
      category: "control",
      displayName: "if",
      description: "Conditional execution",
      icon: "GitBranch",
      color: "#FFAB19",
      parameters: [
        {
          name: "condition",
          type: "select",
          options: ["participant speaks", "object detected", "timer expired"],
          required: true,
        },
      ],
      nestable: true,
    });

    // Events
    this.registerBlock("trial_start", {
      type: "trial_start",
      shape: "hat",
      category: "event",
      displayName: "when trial starts",
      description: "Trial beginning trigger",
      icon: "Play",
      color: "#59C059",
      parameters: [],
    });

    // Sensors
    this.registerBlock("observe", {
      type: "observe",
      shape: "action",
      category: "sensor",
      displayName: "observe",
      description: "Record observation",
      icon: "Activity",
      color: "#59C059",
      parameters: [
        {
          name: "what",
          type: "text",
          placeholder: "participant behavior",
          required: true,
        },
        {
          name: "duration",
          type: "number",
          min: 1,
          max: 60,
          required: true,
        },
      ],
    });
  }

  registerBlock(id: string, definition: PluginBlockDefinition): void {
    this.blocks.set(id, definition);
  }

  getBlock(type: string): PluginBlockDefinition | undefined {
    return this.blocks.get(type);
  }

  getBlocksByCategory(category: BlockCategory): PluginBlockDefinition[] {
    return Array.from(this.blocks.values()).filter(
      (block) => block.category === category,
    );
  }

  createBlock(type: string, order: number): ExperimentBlock {
    const definition = this.getBlock(type);
    if (!definition) throw new Error(`Unknown block type: ${type}`);

    const parameters: BlockParameter[] = definition.parameters.map(
      (param, index) => ({
        id: `param_${index}`,
        ...param,
        value:
          param.type === "number"
            ? param.min || 1
            : param.type === "boolean"
              ? false
              : param.type === "select" && param.options
                ? param.options[0]
                : param.placeholder || "",
      }),
    );

    return {
      id: `block_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      type,
      category: definition.category,
      shape: definition.shape,
      displayName: definition.displayName,
      description: definition.description,
      icon: definition.icon,
      color: definition.color,
      parameters,
      children: definition.nestable ? [] : undefined,
      nestable: definition.nestable,
      order,
    };
  }
}

// Icon mapping
const IconComponents: Record<string, React.ComponentType<any>> = {
  MessageSquare,
  Bot,
  Users,
  ArrowRight,
  Eye,
  Clock,
  Play,
  GitBranch,
  Repeat,
  Hand,
  Volume2,
  Activity,
  Zap,
  Hash,
};

// Droppable Container for Control Blocks
interface DroppableContainerProps {
  id: string;
  children: React.ReactNode;
  isEmpty: boolean;
}

function DroppableContainer({
  id,
  children,
  isEmpty,
}: DroppableContainerProps) {
  const { setNodeRef, isOver } = useDroppable({
    id,
  });

  return (
    <div
      ref={setNodeRef}
      className={cn(
        "ml-6 min-h-[40px] rounded border-2 border-dashed p-2 transition-colors",
        isOver ? "border-blue-400 bg-blue-50" : "border-gray-300",
        isEmpty && "flex items-center justify-center text-sm text-gray-500",
      )}
    >
      {isEmpty ? "Drop blocks here" : children}
    </div>
  );
}

// Sortable Block Item
interface SortableBlockProps {
  block: ExperimentBlock;
  isSelected: boolean;
  selectedBlockId?: string;
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

  const IconComponent = IconComponents[block.icon] || Bot;

  const renderParameterPreview = () => {
    if (block.parameters.length === 0) return null;

    return block.parameters.map((param) => (
      <span
        key={param.id}
        className="inline-flex items-center rounded bg-white/20 px-1.5 py-0.5 text-xs"
      >
        {param.type === "text" && `"${param.value}"`}
        {param.type === "number" && param.value}
        {param.type === "select" && param.value}
        {param.type === "boolean" && (param.value ? "✓" : "✗")}
      </span>
    ));
  };

  const renderBlock = () => {
    const baseClasses = cn(
      "group relative flex items-center gap-2 rounded-lg border px-3 py-2 text-sm font-medium transition-all",
      "hover:shadow-md cursor-pointer",
      isSelected && "ring-2 ring-blue-500",
      isDragging && "opacity-50",
    );

    const contentClasses = "flex items-center gap-2 flex-1 min-w-0";

    // Different rendering based on shape
    switch (block.shape) {
      case "action":
        return (
          <div
            className={cn(baseClasses, "text-white")}
            style={{ backgroundColor: block.color }}
          >
            <div {...listeners} className="cursor-grab active:cursor-grabbing">
              <GripVertical className="h-4 w-4 opacity-50" />
            </div>
            <div className={contentClasses}>
              <IconComponent className="h-4 w-4 flex-shrink-0" />
              <span className="truncate">{block.displayName}</span>
              <div className="flex items-center gap-1">
                {renderParameterPreview()}
              </div>
            </div>
            <Button
              variant="ghost"
              size="sm"
              className="h-6 w-6 p-0 opacity-0 group-hover:opacity-100 hover:bg-white/20"
              onClick={(e) => {
                e.stopPropagation();
                onDelete();
              }}
            >
              <Trash2 className="h-3 w-3" />
            </Button>
          </div>
        );

      case "control":
        return (
          <div className="space-y-1">
            <div
              className={cn(baseClasses, "text-white")}
              style={{ backgroundColor: block.color }}
            >
              <div
                {...listeners}
                className="cursor-grab active:cursor-grabbing"
              >
                <GripVertical className="h-4 w-4 opacity-50" />
              </div>
              <div className={contentClasses}>
                <IconComponent className="h-4 w-4 flex-shrink-0" />
                <span className="truncate">{block.displayName}</span>
                <div className="flex items-center gap-1">
                  {renderParameterPreview()}
                </div>
              </div>
              <Button
                variant="ghost"
                size="sm"
                className="h-6 w-6 p-0 opacity-0 group-hover:opacity-100 hover:bg-white/20"
                onClick={(e) => {
                  e.stopPropagation();
                  onDelete();
                }}
              >
                <Trash2 className="h-3 w-3" />
              </Button>
            </div>
            {block.nestable && (
              <DroppableContainer
                id={`control-${block.id}`}
                isEmpty={!block.children || block.children.length === 0}
              >
                {block.children && block.children.length > 0 && (
                  <div className="space-y-1">
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
                )}
              </DroppableContainer>
            )}
          </div>
        );

      case "hat":
        return (
          <div className="relative">
            <div
              className="absolute -top-2 right-4 left-4 h-2 rounded-t-lg"
              style={{ backgroundColor: block.color }}
            />
            <div
              className={cn(baseClasses, "pt-3 text-white")}
              style={{ backgroundColor: block.color }}
            >
              <div
                {...listeners}
                className="cursor-grab active:cursor-grabbing"
              >
                <GripVertical className="h-4 w-4 opacity-50" />
              </div>
              <div className={contentClasses}>
                <IconComponent className="h-4 w-4 flex-shrink-0" />
                <span className="truncate">{block.displayName}</span>
              </div>
            </div>
          </div>
        );

      default:
        return (
          <div
            className={cn(baseClasses, "text-white")}
            style={{ backgroundColor: block.color }}
          >
            <div {...listeners} className="cursor-grab active:cursor-grabbing">
              <GripVertical className="h-4 w-4 opacity-50" />
            </div>
            <div className={contentClasses}>
              <IconComponent className="h-4 w-4 flex-shrink-0" />
              <span className="truncate">{block.displayName}</span>
            </div>
          </div>
        );
    }
  };

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...attributes}
      onClick={onSelect}
      className={cn(level > 0 && "ml-4")}
    >
      {renderBlock()}
    </div>
  );
}

// Block Palette
interface BlockPaletteProps {
  onBlockAdd: (blockType: string) => void;
  showParameterPreview?: boolean;
}

function BlockPalette({
  onBlockAdd,
  showParameterPreview = false,
}: BlockPaletteProps) {
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
    <div className="flex h-full flex-col border-r bg-slate-50/50">
      <div className="border-b bg-white p-4 shadow-sm">
        <h3 className="mb-4 text-sm font-semibold text-slate-900">
          Block Library
        </h3>
        <div className="grid grid-cols-1 gap-1.5">
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
                  "h-9 justify-start text-xs font-medium transition-all",
                  isActive
                    ? "bg-blue-600 text-white shadow-sm hover:bg-blue-700"
                    : "text-slate-600 hover:bg-slate-100 hover:text-slate-900",
                )}
                onClick={() => setActiveCategory(category)}
              >
                <IconComponent className="mr-2.5 h-3.5 w-3.5" />
                {config.label}
              </Button>
            );
          })}
        </div>
      </div>

      <ScrollArea className="flex-1">
        <div className="space-y-2 p-4">
          {registry.getBlocksByCategory(activeCategory).map((blockDef) => {
            const IconComponent = IconComponents[blockDef.icon] || Bot;
            return (
              <div
                key={blockDef.type}
                className="group cursor-pointer rounded-lg border border-slate-200 bg-white p-3 shadow-sm transition-all hover:border-slate-300 hover:shadow-md"
                onClick={() => onBlockAdd(blockDef.type)}
              >
                <div className="flex items-center gap-3">
                  <div
                    className="flex h-8 w-8 items-center justify-center rounded-lg shadow-sm"
                    style={{ backgroundColor: blockDef.color }}
                  >
                    <IconComponent className="h-4 w-4 text-white" />
                  </div>
                  <div className="min-w-0 flex-1">
                    <div className="truncate text-sm font-semibold text-slate-900 group-hover:text-blue-600">
                      {blockDef.displayName}
                    </div>
                    <div className="mt-0.5 truncate text-xs text-slate-500">
                      {blockDef.description}
                    </div>
                    {showParameterPreview && blockDef.parameters.length > 0 && (
                      <div className="mt-1 flex flex-wrap gap-1">
                        {blockDef.parameters.slice(0, 2).map((param, idx) => (
                          <Badge
                            key={idx}
                            variant="outline"
                            className="text-xs"
                          >
                            {param.name}
                          </Badge>
                        ))}
                        {blockDef.parameters.length > 2 && (
                          <span className="text-xs text-slate-400">
                            +{blockDef.parameters.length - 2}
                          </span>
                        )}
                      </div>
                    )}
                  </div>
                  <Plus className="h-4 w-4 text-slate-400 opacity-0 transition-opacity group-hover:opacity-100" />
                </div>
              </div>
            );
          })}
        </div>
      </ScrollArea>
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
  const [design, setDesign] = useState<BlockDesign>(
    initialDesign || {
      id: experimentId,
      name: "New Experiment",
      description: "",
      blocks: [],
      version: 1,
      lastSaved: new Date(),
    },
  );

  const [selectedBlockId, setSelectedBlockId] = useState<string | null>(null);
  const [activeId, setActiveId] = useState<string | null>(null);
  const [hasUnsavedChanges, setHasUnsavedChanges] = useState(false);

  const registry = BlockRegistry.getInstance();

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Experiments", href: "/experiments" },
    { label: design.name, href: `/experiments/${experimentId}` },
    { label: "Designer" },
  ]);

  // DnD sensors
  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 8,
      },
    }),
    useSensor(KeyboardSensor),
  );

  // Add new block
  const handleBlockAdd = useCallback(
    (blockType: string) => {
      const newBlock = registry.createBlock(blockType, design.blocks.length);

      setDesign((prev) => ({
        ...prev,
        blocks: [...prev.blocks, newBlock],
      }));

      setHasUnsavedChanges(true);
      toast.success(`Added ${newBlock.displayName} block`);
    },
    [registry, design.blocks.length],
  );

  // Remove block from control structure
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

  // Handle drag start
  const handleDragStart = useCallback((event: DragStartEvent) => {
    setActiveId(event.active.id as string);
  }, []);

  // Handle drag end
  const handleDragEnd = useCallback((event: DragEndEvent) => {
    const { active, over } = event;
    setActiveId(null);

    if (!over) return;

    // Check if dropping into a control block
    if (over.id.toString().startsWith("control-")) {
      const controlId = over.id.toString().replace("control-", "");
      const draggedBlockId = active.id.toString();

      setDesign((prev) => {
        // Remove from main blocks array
        const draggedBlock = prev.blocks.find((b) => b.id === draggedBlockId);
        if (!draggedBlock) return prev;

        const newBlocks = prev.blocks.filter((b) => b.id !== draggedBlockId);

        // Add to control block's children
        const updatedBlocks = newBlocks.map((block) => {
          if (block.id === controlId) {
            return {
              ...block,
              children: [...(block.children || []), draggedBlock],
            };
          }
          return block;
        });

        return {
          ...prev,
          blocks: updatedBlocks,
        };
      });

      setHasUnsavedChanges(true);
      toast.success("Block added to control structure");
      return;
    }

    // Normal reordering
    if (active.id !== over?.id) {
      setDesign((prev) => {
        const activeIndex = prev.blocks.findIndex(
          (block) => block.id === active.id,
        );
        const overIndex = prev.blocks.findIndex(
          (block) => block.id === over?.id,
        );

        if (activeIndex !== -1 && overIndex !== -1) {
          const newBlocks = arrayMove(prev.blocks, activeIndex, overIndex);
          // Update order
          newBlocks.forEach((block, index) => {
            block.order = index;
          });

          return {
            ...prev,
            blocks: newBlocks,
          };
        }
        return prev;
      });
      setHasUnsavedChanges(true);
    }
  }, []);

  // Handle block selection
  const handleBlockSelect = useCallback((blockId: string) => {
    setSelectedBlockId(blockId);
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

  // Handle parameter changes
  const handleParameterChange = useCallback(
    (blockId: string, parameterId: string, value: any) => {
      setDesign((prev) => ({
        ...prev,
        blocks: prev.blocks.map((block) =>
          block.id === blockId
            ? {
                ...block,
                parameters: block.parameters.map((param) =>
                  param.id === parameterId ? { ...param, value } : param,
                ),
              }
            : block,
        ),
      }));
      setHasUnsavedChanges(true);
    },
    [],
  );

  // Save design
  const handleSave = useCallback(() => {
    const updatedDesign = {
      ...design,
      lastSaved: new Date(),
    };

    setDesign(updatedDesign);
    setHasUnsavedChanges(false);

    if (onSave) {
      onSave(updatedDesign);
    }

    toast.success("Design saved successfully");
  }, [design, onSave]);

  const selectedBlock = selectedBlockId
    ? design.blocks.find((b) => b.id === selectedBlockId)
    : null;

  return (
    <DndContext
      sensors={sensors}
      collisionDetection={closestCenter}
      onDragStart={handleDragStart}
      onDragEnd={handleDragEnd}
    >
      <div className="flex h-full flex-col">
        {/* Toolbar */}
        <div className="flex items-center justify-between border-b p-3">
          <div className="flex items-center gap-4">
            <h1 className="text-lg font-semibold">{design.name}</h1>
            {hasUnsavedChanges && (
              <Badge variant="outline" className="text-xs">
                Unsaved
              </Badge>
            )}
            <Badge variant="secondary" className="text-xs">
              {design.blocks.length} blocks
            </Badge>
          </div>

          <div className="flex items-center gap-2">
            <Button variant="outline" size="sm" onClick={handleSave}>
              <Save className="mr-2 h-4 w-4" />
              Save
            </Button>
            <Button variant="outline" size="sm">
              <Download className="mr-2 h-4 w-4" />
              Export
            </Button>
          </div>
        </div>

        {/* Main content */}
        <div className="flex flex-1 overflow-hidden">
          <ResizablePanelGroup direction="horizontal">
            {/* Block Palette */}
            <ResizablePanel defaultSize={25} minSize={20} maxSize={35}>
              <BlockPalette
                onBlockAdd={handleBlockAdd}
                showParameterPreview={true}
              />
            </ResizablePanel>

            <ResizableHandle />

            {/* Block List */}
            <ResizablePanel defaultSize={50}>
              <div className="flex h-full flex-col">
                <div className="border-b p-3">
                  <h2 className="text-sm font-semibold">Experiment Flow</h2>
                  <p className="text-muted-foreground text-xs">
                    Drag blocks to reorder • Click to edit • Control blocks can
                    contain other blocks
                  </p>
                </div>

                <ScrollArea className="flex-1">
                  <div className="p-3">
                    {design.blocks.length === 0 ? (
                      <div className="flex h-64 items-center justify-center text-center">
                        <div>
                          <div className="text-muted-foreground mb-4 flex justify-center">
                            <Plus className="h-16 w-16" />
                          </div>
                          <p className="text-muted-foreground text-sm">
                            No blocks yet. Add some from the palette!
                          </p>
                        </div>
                      </div>
                    ) : (
                      <SortableContext
                        items={design.blocks.map((b) => b.id)}
                        strategy={verticalListSortingStrategy}
                      >
                        <div className="space-y-2">
                          {design.blocks.map((block) => (
                            <SortableBlock
                              key={block.id}
                              block={block}
                              isSelected={selectedBlockId === block.id}
                              selectedBlockId={selectedBlockId}
                              onSelect={() => handleBlockSelect(block.id)}
                              onDelete={() => handleBlockDelete(block.id)}
                              onAddToControl={(parentId, childId) => {
                                setHasUnsavedChanges(true);
                              }}
                              onRemoveFromControl={handleRemoveFromControl}
                            />
                          ))}
                        </div>
                      </SortableContext>
                    )}
                  </div>
                </ScrollArea>
              </div>
            </ResizablePanel>

            <ResizableHandle />

            {/* Properties Panel */}
            <ResizablePanel defaultSize={25} minSize={20} maxSize={35}>
              <div className="flex h-full flex-col border-l">
                <div className="border-b p-3">
                  <h3 className="text-sm font-semibold">Properties</h3>
                </div>

                <ScrollArea className="flex-1">
                  {selectedBlock ? (
                    <div className="space-y-4 p-3">
                      <div>
                        <div className="mb-2 flex items-center gap-2">
                          <div
                            className="flex h-5 w-5 items-center justify-center rounded"
                            style={{ backgroundColor: selectedBlock.color }}
                          >
                            {IconComponents[selectedBlock.icon] &&
                              React.createElement(
                                IconComponents[selectedBlock.icon],
                                {
                                  className: "h-3 w-3 text-white",
                                },
                              )}
                          </div>
                          <span className="text-sm font-medium">
                            {selectedBlock.displayName}
                          </span>
                        </div>
                        <p className="text-muted-foreground text-xs">
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
                                <Label className="text-xs">{param.name}</Label>

                                {param.type === "text" && (
                                  <Input
                                    size="sm"
                                    value={param.value || ""}
                                    placeholder={param.placeholder}
                                    onChange={(e) =>
                                      handleParameterChange(
                                        selectedBlock.id,
                                        param.id,
                                        e.target.value,
                                      )
                                    }
                                  />
                                )}

                                {param.type === "number" && (
                                  <Input
                                    size="sm"
                                    type="number"
                                    value={param.value || 0}
                                    min={param.min}
                                    max={param.max}
                                    step={param.step}
                                    onChange={(e) =>
                                      handleParameterChange(
                                        selectedBlock.id,
                                        param.id,
                                        parseFloat(e.target.value),
                                      )
                                    }
                                  />
                                )}

                                {param.type === "select" && (
                                  <Select
                                    value={param.value}
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
                    <div className="flex h-full items-center justify-center p-4 text-center">
                      <div>
                        <Settings className="mx-auto mb-2 h-8 w-8 opacity-50" />
                        <p className="text-muted-foreground text-sm">
                          Select a block to edit
                        </p>
                      </div>
                    </div>
                  )}
                </ScrollArea>
              </div>
            </ResizablePanel>
          </ResizablePanelGroup>
        </div>

        {/* Drag overlay */}
        <DragOverlay>
          {activeId ? (
            <div className="opacity-80">
              <div
                className="rounded-lg border px-3 py-2 text-sm font-medium shadow-lg"
                style={{
                  backgroundColor: design.blocks.find((b) => b.id === activeId)
                    ?.color,
                  color: "white",
                }}
              >
                {design.blocks.find((b) => b.id === activeId)?.displayName}
              </div>
            </div>
          ) : null}
        </DragOverlay>
      </div>
    </DndContext>
  );
}

export default EnhancedBlockDesigner;
