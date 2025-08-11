"use client";

import React, {
  useCallback,
  useEffect,
  useMemo,
  useRef,
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
} from "lucide-react";
import { Input } from "~/components/ui/input";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { ScrollArea } from "~/components/ui/scroll-area";
import { cn } from "~/lib/utils";
import { useActionRegistry } from "../ActionRegistry";
import type { ActionDefinition } from "~/lib/experiment-designer/types";

/**
 * ActionLibraryPanel
 *
 * Enhanced wrapper panel for the experiment designer left side:
 *  - Fuzzy-ish search (case-insensitive substring) over name, description, id
 *  - Multi-category filtering (toggle chips)
 *  - Favorites (local persisted)
 *  - Density toggle (comfortable / compact)
 *  - Star / unstar actions inline
 *  - Drag support (DndKit) identical to legacy ActionLibrary
 *
 * Does NOT own persistence of actions themselves—delegates to action registry.
 */

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
        transform: `translate3d(${transform.x}px, ${transform.y}px,0)`,
      }
    : {};

  const IconComponent =
    iconMap[action.icon] ??
    // fallback icon (Sparkles)
    Sparkles;

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
        "group bg-background/60 hover:bg-accent/50 relative flex w-full cursor-grab flex-col items-start gap-1 rounded border px-2 transition-colors",
        compact ? "py-2 text-[11px]" : "py-3 text-[12px]",
        isDragging && "opacity-50",
      )}
      draggable={false}
      title={action.description ?? ""}
    >
      <div
        className={cn(
          "flex h-5 w-5 flex-shrink-0 items-center justify-center rounded text-white",
          categoryColors[action.category],
        )}
      >
        <IconComponent className="h-3 w-3" />
      </div>
      <div className="min-w-0 flex-1">
        <div className="flex items-center gap-1 truncate font-medium">
          {action.source.kind === "plugin" ? (
            <span className="inline-flex h-3 w-3 items-center justify-center rounded-full bg-emerald-700 text-[8px] font-bold text-white">
              P
            </span>
          ) : (
            <span className="inline-flex h-3 w-3 items-center justify-center rounded-full bg-slate-500 text-[8px] font-bold text-white">
              C
            </span>
          )}
          <span className="truncate">
            {highlight ? highlightMatch(action.name, highlight) : action.name}
          </span>
        </div>
        {action.description && !compact && (
          <div className="text-muted-foreground truncate">
            {highlight
              ? highlightMatch(action.description, highlight)
              : action.description}
          </div>
        )}
      </div>
      <button
        type="button"
        aria-label={isFavorite ? "Unfavorite action" : "Favorite action"}
        onClick={(e) => {
          e.stopPropagation();
          onToggleFavorite(action.id);
        }}
        className="text-muted-foreground/60 hover:text-foreground rounded p-1 transition-colors"
      >
        {isFavorite ? (
          <Star className="h-3 w-3 fill-current" />
        ) : (
          <StarOff className="h-3 w-3" />
        )}
      </button>
    </div>
  );
}

/* -------------------------------------------------------------------------- */
/* Panel Component                                                            */
/* -------------------------------------------------------------------------- */

export function ActionLibraryPanel() {
  const registry = useActionRegistry();

  const [search, setSearch] = useState("");
  const [selectedCategories, setSelectedCategories] = useState<
    Set<ActionCategory>
  >(new Set<ActionCategory>(["wizard"]));
  const [favorites, setFavorites] = useState<FavoritesState>({
    favorites: new Set<string>(),
  });
  const [showOnlyFavorites, setShowOnlyFavorites] = useState(false);
  const [density, setDensity] = useState<"comfortable" | "compact">(
    "comfortable",
  );

  const allActions = registry.getAllActions();

  /* ------------------------------- Favorites -------------------------------- */
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

  /* ----------------------------- Category List ------------------------------ */
  const categories: Array<{
    key: ActionCategory;
    label: string;
    icon: React.ComponentType<{ className?: string }>;
    color: string;
  }> = [
    { key: "wizard", label: "Wizard", icon: User, color: "bg-blue-500" },
    { key: "robot", label: "Robot", icon: Bot, color: "bg-emerald-600" },
    {
      key: "control",
      label: "Control",
      icon: GitBranch,
      color: "bg-amber-500",
    },
    { key: "observation", label: "Observe", icon: Eye, color: "bg-purple-600" },
  ];

  const toggleCategory = useCallback((c: ActionCategory) => {
    setSelectedCategories((prev) => {
      const next = new Set(prev);
      if (next.has(c)) {
        next.delete(c);
      } else {
        next.add(c);
      }
      if (next.size === 0) {
        // Keep at least one category selected
        next.add(c);
      }
      return next;
    });
  }, []);

  const clearFilters = useCallback(() => {
    setSelectedCategories(new Set(categories.map((c) => c.key)));
    setSearch("");
    setShowOnlyFavorites(false);
  }, [categories]);

  useEffect(() => {
    // On mount select all categories for richer initial view
    setSelectedCategories(new Set(categories.map((c) => c.key)));
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  /* ------------------------------- Filtering -------------------------------- */
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
    for (const a of allActions) {
      map[a.category] += 1;
    }
    return map;
  }, [allActions]);

  const visibleFavoritesCount = Array.from(favorites.favorites).filter((id) =>
    filtered.some((a) => a.id === id),
  ).length;

  /* ------------------------------- Rendering -------------------------------- */
  return (
    <div className="flex h-full flex-col">
      {/* Toolbar */}
      <div className="bg-background/60 border-b p-2">
        <div className="mb-2 flex gap-2">
          <div className="relative flex-1">
            <Search className="text-muted-foreground absolute top-1/2 left-2 h-3.5 w-3.5 -translate-y-1/2" />
            <Input
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              placeholder="Search actions"
              className="h-8 pl-7 text-xs"
              aria-label="Search actions"
            />
          </div>
          <Button
            variant={showOnlyFavorites ? "default" : "outline"}
            size="sm"
            className="h-8"
            onClick={() => setShowOnlyFavorites((s) => !s)}
            aria-pressed={showOnlyFavorites}
            aria-label="Toggle favorites filter"
          >
            <Star className="mr-1 h-3 w-3" />
            Fav
            {showOnlyFavorites && (
              <Badge
                variant="secondary"
                className="ml-1 h-4 px-1 text-[10px]"
                title="Visible favorites"
              >
                {visibleFavoritesCount}
              </Badge>
            )}
          </Button>
          <Button
            variant="outline"
            size="sm"
            className="h-8"
            onClick={() =>
              setDensity((d) =>
                d === "comfortable" ? "compact" : "comfortable",
              )
            }
            aria-label="Toggle density"
          >
            <SlidersHorizontal className="mr-1 h-3 w-3" />
            {density === "comfortable" ? "Compact" : "Comfort"}
          </Button>
          <Button
            variant="ghost"
            size="sm"
            className="h-8"
            onClick={clearFilters}
            aria-label="Clear filters"
          >
            <X className="h-3 w-3" />
          </Button>
        </div>

        {/* Category Filters */}
        <div className="grid grid-cols-4 gap-1">
          {categories.map((cat) => {
            const active = selectedCategories.has(cat.key);
            const Icon = cat.icon;
            return (
              <Button
                key={cat.key}
                variant={active ? "default" : "ghost"}
                size="sm"
                className={cn(
                  "h-7 justify-start gap-1 truncate text-[11px]",
                  active && `${cat.color} text-white hover:opacity-90`,
                )}
                onClick={() => toggleCategory(cat.key)}
                aria-pressed={active}
              >
                <Icon className="h-3 w-3" />
                {cat.label}
                <span className="ml-auto text-[10px] font-normal opacity-80">
                  {countsByCategory[cat.key]}
                </span>
              </Button>
            );
          })}
        </div>

        <div className="text-muted-foreground mt-2 flex items-center justify-between text-[10px]">
          <div>
            {filtered.length} shown / {allActions.length} total
          </div>
          <div className="flex items-center gap-1">
            <FolderPlus className="h-3 w-3" />
            <span>
              Plugins: {registry.getDebugInfo().pluginActionsLoaded ? "✓" : "…"}
            </span>
          </div>
        </div>
      </div>

      {/* Actions List */}
      <ScrollArea className="flex-1">
        <div className="grid grid-cols-1 gap-2 p-2 sm:grid-cols-2">
          {filtered.length === 0 ? (
            <div className="text-muted-foreground/70 flex flex-col items-center gap-2 py-10 text-center text-xs">
              <Filter className="h-6 w-6" />
              <div>No actions match filters</div>
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

      {/* Footer Summary */}
      <div className="bg-background/60 border-t p-2">
        <div className="flex items-center justify-between text-[10px]">
          <div className="flex items-center gap-2">
            <Badge variant="secondary" className="h-4 px-1 text-[10px]">
              {allActions.length} total
            </Badge>
            {showOnlyFavorites && (
              <Badge variant="outline" className="h-4 px-1 text-[10px]">
                {visibleFavoritesCount} favorites
              </Badge>
            )}
          </div>
          <div className="text-muted-foreground flex items-center gap-1">
            <Sparkles className="h-3 w-3" />
            Core: {registry.getDebugInfo().coreActionsLoaded ? "✓" : "…"}
          </div>
        </div>
        <Separator className="my-1" />
        <p className="text-muted-foreground text-[9px] leading-relaxed">
          Drag actions into the flow. Use search / category filters to narrow
          results. Star actions you use frequently.
        </p>
      </div>
    </div>
  );
}

export default ActionLibraryPanel;
