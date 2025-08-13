"use client";

import React, {
  useCallback,
  useEffect,
  useLayoutEffect,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { cn } from "~/lib/utils";

/**
 * PanelsContainer
 *
 * Structural layout component for the Experiment Designer refactor.
 * Provides:
 *  - Optional left + right side panels (resizable + collapsible)
 *  - Central workspace (always present)
 *  - Persistent panel widths (localStorage)
 *  - Keyboard-accessible resize handles
 *  - Minimal DOM repaint during drag (inline styles)
 *
 * NOT responsible for:
 *  - Business logic or data fetching
 *  - Panel content semantics (passed via props)
 *
 * Accessibility:
 *  - Resize handles are <button> elements with aria-label
 *  - Keyboard: ArrowLeft / ArrowRight adjusts width by step
 */

const STORAGE_KEY = "hristudio-designer-panels-v1";

interface PersistedLayout {
  left: number;
  right: number;
  leftCollapsed: boolean;
  rightCollapsed: boolean;
}

export interface PanelsContainerProps {
  left?: ReactNode;
  center: ReactNode;
  right?: ReactNode;

  /**
   * Initial (non-collapsed) widths in pixels.
   * If panels are omitted, their widths are ignored.
   */
  initialLeftWidth?: number;
  initialRightWidth?: number;

  /**
   * Minimum / maximum constraints to avoid unusable panels.
   */
  minLeftWidth?: number;
  minRightWidth?: number;
  maxLeftWidth?: number;
  maxRightWidth?: number;

  /**
   * Whether persistence to localStorage should be skipped (e.g. SSR preview)
   */
  disablePersistence?: boolean;

  /**
   * ClassName pass-through for root container
   */
  className?: string;
}

interface DragState {
  edge: "left" | "right";
  startX: number;
  startWidth: number;
}

export function PanelsContainer({
  left,
  center,
  right,
  initialLeftWidth = 280,
  initialRightWidth = 340,
  minLeftWidth = 200,
  minRightWidth = 260,
  maxLeftWidth = 520,
  maxRightWidth = 560,
  disablePersistence = false,
  className,
}: PanelsContainerProps) {
  const hasLeft = Boolean(left);
  const hasRight = Boolean(right);

  /* ------------------------------------------------------------------------ */
  /* State                                                                    */
  /* ------------------------------------------------------------------------ */

  const [leftWidth, setLeftWidth] = useState(initialLeftWidth);
  const [rightWidth, setRightWidth] = useState(initialRightWidth);
  const [leftCollapsed, setLeftCollapsed] = useState(false);
  const [rightCollapsed, setRightCollapsed] = useState(false);

  const dragRef = useRef<DragState | null>(null);
  const frameReq = useRef<number | null>(null);

  /* ------------------------------------------------------------------------ */
  /* Persistence                                                              */
  /* ------------------------------------------------------------------------ */

  useLayoutEffect(() => {
    if (disablePersistence) return;
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return;
      const parsed = JSON.parse(raw) as PersistedLayout;
      if (typeof parsed.left === "number") setLeftWidth(parsed.left);
      if (typeof parsed.right === "number")
        setRightWidth(Math.max(parsed.right, minRightWidth));
      if (typeof parsed.leftCollapsed === "boolean") {
        setLeftCollapsed(parsed.leftCollapsed);
      }
      // Always start with right panel visible to avoid hidden inspector state
      setRightCollapsed(false);
    } catch {
      /* noop */
    }
  }, [disablePersistence, minRightWidth]);

  const persist = useCallback(
    (next?: Partial<PersistedLayout>) => {
      if (disablePersistence) return;
      const snapshot: PersistedLayout = {
        left: leftWidth,
        right: rightWidth,
        leftCollapsed,
        rightCollapsed,
        ...next,
      };
      try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(snapshot));
      } catch {
        /* noop */
      }
    },
    [disablePersistence, leftWidth, rightWidth, leftCollapsed, rightCollapsed],
  );

  useEffect(() => {
    persist();
  }, [leftWidth, rightWidth, leftCollapsed, rightCollapsed, persist]);

  /* ------------------------------------------------------------------------ */
  /* Drag Handlers                                                            */
  /* ------------------------------------------------------------------------ */

  const onPointerMove = useCallback(
    (e: PointerEvent) => {
      if (!dragRef.current) return;
      const { edge, startX, startWidth } = dragRef.current;
      const delta = e.clientX - startX;

      if (edge === "left") {
        let next = startWidth + delta;
        next = Math.max(minLeftWidth, Math.min(maxLeftWidth, next));
        if (next !== leftWidth) {
          if (frameReq.current) cancelAnimationFrame(frameReq.current);
          frameReq.current = requestAnimationFrame(() => setLeftWidth(next));
        }
      } else if (edge === "right") {
        let next = startWidth - delta;
        next = Math.max(minRightWidth, Math.min(maxRightWidth, next));
        if (next !== rightWidth) {
          if (frameReq.current) cancelAnimationFrame(frameReq.current);
          frameReq.current = requestAnimationFrame(() => setRightWidth(next));
        }
      }
    },
    [
      leftWidth,
      rightWidth,
      minLeftWidth,
      maxLeftWidth,
      minRightWidth,
      maxRightWidth,
    ],
  );

  const endDrag = useCallback(() => {
    dragRef.current = null;
    window.removeEventListener("pointermove", onPointerMove);
    window.removeEventListener("pointerup", endDrag);
  }, [onPointerMove]);

  const startDrag = useCallback(
    (edge: "left" | "right", e: React.PointerEvent<HTMLButtonElement>) => {
      e.preventDefault();
      if (edge === "left" && leftCollapsed) return;
      if (edge === "right" && rightCollapsed) return;
      dragRef.current = {
        edge,
        startX: e.clientX,
        startWidth: edge === "left" ? leftWidth : rightWidth,
      };
      window.addEventListener("pointermove", onPointerMove);
      window.addEventListener("pointerup", endDrag);
    },
    [
      leftWidth,
      rightWidth,
      leftCollapsed,
      rightCollapsed,
      onPointerMove,
      endDrag,
    ],
  );

  /* ------------------------------------------------------------------------ */
  /* Collapse / Expand                                                         */
  /* ------------------------------------------------------------------------ */

  const toggleLeft = useCallback(() => {
    if (!hasLeft) return;
    setLeftCollapsed((c) => {
      const next = !c;
      if (next === false && leftWidth < minLeftWidth) {
        setLeftWidth(initialLeftWidth);
      }
      return next;
    });
  }, [hasLeft, leftWidth, minLeftWidth, initialLeftWidth]);

  const toggleRight = useCallback(() => {
    if (!hasRight) return;
    setRightCollapsed((c) => {
      const next = !c;
      if (next === false && rightWidth < minRightWidth) {
        setRightWidth(initialRightWidth);
      }
      return next;
    });
  }, [hasRight, rightWidth, minRightWidth, initialRightWidth]);

  /* Keyboard resizing (focused handle) */
  const handleKeyResize = useCallback(
    (edge: "left" | "right", e: React.KeyboardEvent<HTMLButtonElement>) => {
      const step = e.shiftKey ? 24 : 12;
      if (e.key === "ArrowLeft" || e.key === "ArrowRight") {
        e.preventDefault();
        if (edge === "left" && !leftCollapsed) {
          setLeftWidth((w) => {
            const delta = e.key === "ArrowLeft" ? -step : step;
            return Math.max(minLeftWidth, Math.min(maxLeftWidth, w + delta));
          });
        } else if (edge === "right" && !rightCollapsed) {
          setRightWidth((w) => {
            const delta = e.key === "ArrowLeft" ? -step : step;
            return Math.max(minRightWidth, Math.min(maxRightWidth, w + delta));
          });
        }
      } else if (e.key === "Enter" || e.key === " ") {
        if (edge === "left") toggleLeft();
        else toggleRight();
      }
    },
    [
      leftCollapsed,
      rightCollapsed,
      minLeftWidth,
      maxLeftWidth,
      minRightWidth,
      maxRightWidth,
      toggleLeft,
      toggleRight,
    ],
  );

  /* ------------------------------------------------------------------------ */
  /* Render                                                                    */
  /* ------------------------------------------------------------------------ */

  return (
    <div
      className={cn(
        "flex h-full w-full overflow-hidden select-none",
        className,
      )}
      aria-label="Designer panel layout"
    >
      {/* Left Panel */}
      {hasLeft && (
        <div
          className={cn(
            "bg-background/50 relative flex h-full flex-shrink-0 flex-col border-r transition-[width] duration-150",
            leftCollapsed ? "w-0 border-r-0" : "w-[var(--panel-left-width)]",
          )}
          style={
            leftCollapsed
              ? undefined
              : ({
                  ["--panel-left-width" as string]: `${leftWidth}px`,
                } as React.CSSProperties)
          }
        >
          {!leftCollapsed && (
            <div className="flex-1 overflow-hidden">{left}</div>
          )}
        </div>
      )}

      {/* Left Resize Handle */}
      {hasLeft && !leftCollapsed && (
        <button
          type="button"
          aria-label="Resize left panel (Enter to toggle collapse)"
          onPointerDown={(e) => startDrag("left", e)}
          onDoubleClick={toggleLeft}
          onKeyDown={(e) => handleKeyResize("left", e)}
          className="hover:bg-accent/40 focus-visible:ring-ring relative z-10 h-full w-0 cursor-col-resize px-1 outline-none focus-visible:ring-2"
        />
      )}

      {/* Left collapse toggle removed to prevent breadcrumb overlap */}

      {/* Center (Workspace) */}
      <div className="relative flex min-w-0 flex-1 flex-col overflow-hidden">
        <div className="flex-1 overflow-hidden">{center}</div>
      </div>

      {/* Right Resize Handle */}
      {hasRight && !rightCollapsed && (
        <button
          type="button"
          aria-label="Resize right panel (Enter to toggle collapse)"
          onPointerDown={(e) => startDrag("right", e)}
          onDoubleClick={toggleRight}
          onKeyDown={(e) => handleKeyResize("right", e)}
          className="hover:bg-accent/40 focus-visible:ring-ring relative z-10 h-full w-1 cursor-col-resize outline-none focus-visible:ring-2"
        />
      )}

      {/* Right Panel */}
      {hasRight && (
        <div
          className={cn(
            "bg-background/50 relative flex h-full flex-shrink-0 flex-col transition-[width] duration-150",
            rightCollapsed ? "w-0" : "w-[var(--panel-right-width)]",
          )}
          style={
            rightCollapsed
              ? undefined
              : ({
                  ["--panel-right-width" as string]: `${rightWidth}px`,
                } as React.CSSProperties)
          }
        >
          {!rightCollapsed && (
            <div className="min-w-0 flex-1 overflow-hidden">{right}</div>
          )}
        </div>
      )}

      {/* Minimal Right Toggle (top-right), non-intrusive like VSCode */}
      {hasRight && (
        <button
          type="button"
          aria-label={
            rightCollapsed ? "Expand inspector" : "Collapse inspector"
          }
          onClick={toggleRight}
          className={cn(
            "text-muted-foreground hover:text-foreground absolute top-1 z-20 p-1 text-[10px]",
            rightCollapsed ? "right-1" : "right-1",
          )}
          title={rightCollapsed ? "Show inspector" : "Hide inspector"}
        >
          {rightCollapsed ? "◀" : "▶"}
        </button>
      )}
    </div>
  );
}

export default PanelsContainer;
