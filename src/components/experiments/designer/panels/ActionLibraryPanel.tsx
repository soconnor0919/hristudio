"use client";

import React, {
  useCallback,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import { useDraggable } from "@dnd-kit/core";
import {
  Star,
  StarOff,
  Search,
  Filter,
  Sparkles,
  SlidersHorizontal,
  FolderPlus,
  User,
  Bot,
  GitBranch,
  Eye,
  X,
  Layers,
  PanelLeftClose,
} from "lucide-react";
import { Input } from "~/components/ui/input";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";

import { cn } from "~/lib/utils";
import { useActionRegistry } from "../ActionRegistry";
import type { ActionDefinition } from "~/lib/experiment-designer/types";

export type ActionCategory = ActionDefinition["category"];

interface FavoritesState {
  favorites: Set<string>;
}

const FAVORITES_STORAGE_KEY = "hristudio-action-favorites-v1";

interface DraggableActionProps {
  action: ActionDefinition;
  compact: boolean;
  isFavorite: boolean;
  onToggleFavorite: (id: string) => void;
  highlight?: string;
}

const iconMap: Record<string, React.ComponentType<{ className?: string }>> = {
  User,
  Bot,
  GitBranch,
  Eye,
  Sparkles,
  Layers,
};

function highlightMatch(text: string, query: string): ReactNode {
  if (!query.trim()) return text;
  const idx = text.toLowerCase().indexOf(query.toLowerCase());
  if (idx === -1) return text;
  return (
    <>
      {text.slice(0, idx)}
      <span className="bg-yellow-200/60 dark:bg-yellow-500/30">
        {text.slice(idx, idx + query.length)}
      </span>
      {text.slice(idx + query.length)}
    </>
  );
}

function DraggableAction({
  action,
  compact,
  isFavorite,
  onToggleFavorite,
  highlight,
}: DraggableActionProps) {
  const { attributes, listeners, setNodeRef, transform, isDragging } =
    useDraggable({
      id: `action-${action.id}`,
      data: { action },
    });

  const style: React.CSSProperties = transform
    ? {
      transform: `translate3d(${transform.x}px, ${transform.y}px, 0)`,
    }
    : {};

  const IconComponent = iconMap[action.icon] ?? Sparkles;

  const categoryColors: Record<ActionCategory, string> = {
    wizard: "bg-blue-500",
    robot: "bg-emerald-600",
    control: "bg-amber-500",
    observation: "bg-purple-600",
  };

  return (
    <div
      ref={setNodeRef}
      {...attributes}
      {...listeners}
      style={style}
      className={cn(
        "group bg-background/60 hover:bg-accent/50 relative flex w-full cursor-grab touch-none flex-col gap-1 rounded-lg border px-2 text-left transition-colors select-none",
        compact ? "py-1.5 text-[11px]" : "py-2 text-[12px]",
        isDragging && "opacity-50",
      )}
      draggable={false}
      title={action.description ?? ""}
    >
      <button
        type="button"
        aria-label={isFavorite ? "Unfavorite action" : "Favorite action"}
        onClick={(e) => {
          e.stopPropagation();
          onToggleFavorite(action.id);
        }}
        className="text-muted-foreground/60 hover:text-foreground absolute top-1 right-1 rounded p-1 transition-colors"
      >
        {isFavorite ? (
          <Star className="h-3 w-3 fill-current" />
        ) : (
          <StarOff className="h-3 w-3" />
        )}
      </button>

      <div className="flex min-w-0 items-start gap-2 select-none">
        <div
          className={cn(
            "flex h-5 w-5 flex-shrink-0 items-center justify-center rounded text-white",
            categoryColors[action.category],
          )}
        >
          <IconComponent className="h-3 w-3" />
        </div>
        <div className="min-w-0 flex-1">
          <div className="flex items-center gap-1 leading-snug font-medium">
            {action.source.kind === "plugin" ? (
              <span className="inline-flex h-3 w-3 items-center justify-center rounded-full bg-emerald-700 text-[8px] font-bold text-white">
                P
              </span>
            ) : (
              <span className="inline-flex h-3 w-3 items-center justify-center rounded-full bg-slate-500 text-[8px] font-bold text-white">
                C
              </span>
            )}
            <span className="break-words whitespace-normal">
              {highlight ? highlightMatch(action.name, highlight) : action.name}
            </span>
          </div>
          {action.description && !compact && (
            <div className="text-muted-foreground mt-1 line-clamp-3 text-[10.5px] leading-snug break-words whitespace-normal">
              {highlight
                ? highlightMatch(action.description, highlight)
                : action.description}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export interface ActionLibraryPanelProps {
  collapsed?: boolean;
  onCollapse?: (collapsed: boolean) => void;
}

export function ActionLibraryPanel({ collapsed, onCollapse }: ActionLibraryPanelProps = {}) {
  const registry = useActionRegistry();

  const [search, setSearch] = useState("");
  const [selectedCategories, setSelectedCategories] = useState<
    Set<ActionCategory>
  >(new Set<ActionCategory>(["wizard", "robot", "control", "observation"]));
  const [favorites, setFavorites] = useState<FavoritesState>({
    favorites: new Set<string>(),
  });
  const [showOnlyFavorites, setShowOnlyFavorites] = useState(false);
  const [density, setDensity] = useState<"comfortable" | "compact">(
    "comfortable",
  );

  const allActions = registry.getAllActions();

  useEffect(() => {
    try {
      const raw = localStorage.getItem(FAVORITES_STORAGE_KEY);
      if (raw) {
        const parsed = JSON.parse(raw) as { favorites?: string[] };
        if (Array.isArray(parsed.favorites)) {
          setFavorites({ favorites: new Set(parsed.favorites) });
        }
      }
    } catch {
      /* noop */
    }
  }, []);

  const persistFavorites = useCallback((next: Set<string>) => {
    try {
      localStorage.setItem(
        FAVORITES_STORAGE_KEY,
        JSON.stringify({ favorites: Array.from(next) }),
      );
    } catch {
      /* noop */
    }
  }, []);

  const toggleFavorite = useCallback(
    (id: string) => {
      setFavorites((prev) => {
        const copy = new Set(prev.favorites);
        if (copy.has(id)) copy.delete(id);
        else copy.add(id);
        persistFavorites(copy);
        return { favorites: copy };
      });
    },
    [persistFavorites],
  );

  const categories = useMemo(
    () =>
      [
        { key: "wizard", label: "Wizard", icon: User, color: "bg-blue-500" },
        { key: "robot", label: "Robot", icon: Bot, color: "bg-emerald-600" },
        {
          key: "control",
          label: "Control",
          icon: GitBranch,
          color: "bg-amber-500",
        },
        {
          key: "observation",
          label: "Observe",
          icon: Eye,
          color: "bg-purple-600",
        },
      ] as const,
    [],
  );

  /**
   * Enforce invariant:
   * - Either ALL categories selected
   * - Or EXACTLY ONE selected
   *
   * Behaviors:
   * - From ALL -> clicking a category selects ONLY that category
   * - From single selected -> clicking same category returns to ALL
   * - From single selected -> clicking different category switches to that single
   * - Any multi-subset attempt collapses to the clicked category (prevents ambiguous subset)
   */
  const toggleCategory = useCallback(
    (c: ActionCategory) => {
      setSelectedCategories((prev) => {
        const allKeys = categories.map((k) => k.key) as ActionCategory[];
        const fullSize = allKeys.length;
        const isFull = prev.size === fullSize;
        const isSingle = prev.size === 1;
        const has = prev.has(c);

        // Case: full set -> reduce to single clicked
        if (isFull) {
          return new Set<ActionCategory>([c]);
        }

        // Case: single selection
        if (isSingle) {
          // Clicking the same => expand to all
          if (has) {
            return new Set<ActionCategory>(allKeys);
          }
          // Clicking different => switch single
          return new Set<ActionCategory>([c]);
        }

        // (Should not normally reach: ambiguous multi-subset)
        // Collapse to single clicked to restore invariant
        return new Set<ActionCategory>([c]);
      });
    },
    [categories],
  );

  const clearFilters = useCallback(() => {
    setSelectedCategories(new Set(categories.map((c) => c.key)));
    setSearch("");
    setShowOnlyFavorites(false);
  }, [categories]);



  const filtered = useMemo(() => {
    const activeCats = selectedCategories;
    const q = search.trim().toLowerCase();

    return allActions.filter((a) => {
      if (!activeCats.has(a.category)) return false;
      if (showOnlyFavorites && !favorites.favorites.has(a.id)) return false;
      if (!q) return true;
      return (
        a.name.toLowerCase().includes(q) ||
        (a.description?.toLowerCase().includes(q) ?? false) ||
        a.id.toLowerCase().includes(q)
      );
    });
  }, [
    allActions,
    selectedCategories,
    search,
    showOnlyFavorites,
    favorites.favorites,
  ]);

  const countsByCategory = useMemo(() => {
    const map: Record<ActionCategory, number> = {
      wizard: 0,
      robot: 0,
      control: 0,
      observation: 0,
    };
    for (const a of allActions) map[a.category] += 1;
    return map;
  }, [allActions]);

  const visibleFavoritesCount = Array.from(favorites.favorites).filter((id) =>
    filtered.some((a) => a.id === id),
  ).length;

  return (
    <div className="flex h-full flex-col overflow-hidden" id="tour-designer-blocks">
      <div className="bg-background/60 flex-shrink-0 border-b p-2">
        <div className="relative mb-2">
          <Search className="text-muted-foreground absolute top-1/2 left-2 h-3.5 w-3.5 -translate-y-1/2" />
          <Input
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            placeholder="Search"
            className="h-8 w-full pl-7 text-xs"
            aria-label="Search actions"
          />
        </div>

        <div className="mb-2 grid grid-cols-2 gap-1">
          {categories.map((cat) => {
            const active = selectedCategories.has(cat.key);
            const Icon = cat.icon;
            return (
              <Button
                key={cat.key}
                variant={active ? "default" : "ghost"}
                size="sm"
                className={cn(
                  "h-7 justify-start gap-1 text-[11px]",
                  active && `${cat.color} text-white hover:opacity-90`,
                )}
                onClick={() => toggleCategory(cat.key)}
                aria-pressed={active}
                aria-label={cat.label}
              >
                <Icon className="h-3 w-3" />
                <span className="hidden md:inline">{cat.label}</span>
                <span className="ml-auto hidden text-[10px] font-normal opacity-80 lg:inline">
                  {countsByCategory[cat.key]}
                </span>
              </Button>
            );
          })}
        </div>

        <div className="flex flex-wrap gap-1">
          <Button
            variant={showOnlyFavorites ? "default" : "outline"}
            size="sm"
            className="h-7 flex-1"
            onClick={() => setShowOnlyFavorites((s) => !s)}
            aria-pressed={showOnlyFavorites}
            aria-label="Toggle favorites filter"
          >
            <Star className="h-3 w-3" />
            <span className="ml-1 hidden sm:inline">Fav</span>
            {showOnlyFavorites && (
              <Badge
                variant="secondary"
                className="ml-1 hidden h-4 px-1 text-[10px] sm:inline"
                title="Visible favorites"
              >
                {visibleFavoritesCount}
              </Badge>
            )}
          </Button>
          <Button
            variant="outline"
            size="sm"
            className="h-7 flex-1"
            onClick={() =>
              setDensity((d) =>
                d === "comfortable" ? "compact" : "comfortable",
              )
            }
            aria-label="Toggle density"
          >
            <SlidersHorizontal className="h-3 w-3" />
            <span className="ml-1 hidden sm:inline">
              {density === "comfortable" ? "Dense" : "Relax"}
            </span>
          </Button>
          <Button
            variant="ghost"
            size="sm"
            className="h-7 flex-1"
            onClick={clearFilters}
            aria-label="Clear filters"
          >
            <X className="h-3 w-3" />
            <span className="ml-1 hidden sm:inline">Clear</span>
          </Button>
        </div>

        <div className="text-muted-foreground mt-2 flex items-center justify-between text-[10px]">
          <div>
            {filtered.length} / {allActions.length}
          </div>
          <div className="flex items-center gap-1">
            <FolderPlus className="h-3 w-3" />
            <span>
              {registry.getDebugInfo().pluginActionsLoaded
                ? "Plugins ✓"
                : "Plugins …"}
            </span>
          </div>
        </div>
      </div>

      <ScrollArea className="flex-1 overflow-hidden">
        <div className="grid grid-cols-1 gap-2 p-2">
          {filtered.length === 0 ? (
            <div className="text-muted-foreground/70 flex flex-col items-center gap-2 py-10 text-center text-xs">
              <Filter className="h-6 w-6" />
              <div>No actions</div>
            </div>
          ) : (
            filtered.map((action) => (
              <DraggableAction
                key={action.id}
                action={action}
                compact={density === "compact"}
                isFavorite={favorites.favorites.has(action.id)}
                onToggleFavorite={toggleFavorite}
                highlight={search}
              />
            ))
          )}
        </div>
      </ScrollArea>

      <div className="bg-background/60 flex-shrink-0 border-t p-2">
        <div className="flex items-center justify-between text-[10px]">
          <div className="flex items-center gap-2">
            <Badge variant="secondary" className="h-4 px-1 text-[10px]">
              {allActions.length} total
            </Badge>
            {showOnlyFavorites && (
              <Badge variant="outline" className="h-4 px-1 text-[10px]">
                {visibleFavoritesCount} fav
              </Badge>
            )}
          </div>
          <div className="text-muted-foreground flex items-center gap-1">
            <Sparkles className="h-3 w-3" />
            Core: {registry.getDebugInfo().coreActionsLoaded ? "✓" : "…"}
          </div>
        </div>
        <Separator className="my-1" />
        <p className="text-muted-foreground hidden text-[9px] leading-relaxed md:block">
          Drag actions into the flow. Star frequent actions.
        </p>
      </div>
    </div>
  );
}

// Wrap in React.memo to prevent unnecessary re-renders causing flashing in categories
export default React.memo(ActionLibraryPanel);

