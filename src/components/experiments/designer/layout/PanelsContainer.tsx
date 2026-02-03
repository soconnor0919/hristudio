"use client";

import * as React from "react";
import { cn } from "~/lib/utils";
import { Sheet, SheetContent, SheetTrigger } from "~/components/ui/sheet";
import { Button } from "~/components/ui/button";
import { PanelLeft, Settings2 } from "lucide-react";
type Edge = "left" | "right";

export interface PanelsContainerProps {
  left?: React.ReactNode;
  center: React.ReactNode;
  right?: React.ReactNode;

  /**
   * Draw dividers between panels (applied to center only to avoid double borders).
   * Defaults to true.
   */
  showDividers?: boolean;

  /** Class applied to the root container */
  className?: string;

  /** Class applied to each panel wrapper (left/center/right) */
  panelClassName?: string;

  /** Class applied to each panel's internal scroll container */
  contentClassName?: string;

  /** Accessible label for the overall layout */
  "aria-label"?: string;

  /** Min/Max fractional widths for left and right panels (0..1), clamped during drag */
  minLeftPct?: number;
  maxLeftPct?: number;
  minRightPct?: number;
  maxRightPct?: number;

  /** Keyboard resize step (fractional) per arrow press; Shift increases by 2x */
  keyboardStepPct?: number;

  /**
   * Controlled collapse state
   */
  leftCollapsed?: boolean;
  rightCollapsed?: boolean;
  onLeftCollapseChange?: (collapsed: boolean) => void;
  onRightCollapseChange?: (collapsed: boolean) => void;
}

/**
 * PanelsContainer
 *
 * Tailwind-first, grid-based panel layout with:
 * - Drag-resizable left/right panels (no persistence)
 * - Collapsible side panels
 * - Strict overflow containment (no page-level x-scroll)
 * - Internal y-scroll for each panel
 * - Optional visual dividers on the center panel only (prevents double borders)
 *
 * Implementation details:
 * - Uses CSS variables for column fractions and an explicit grid template:
 *   [minmax(0,var(--col-left)) minmax(0,var(--col-center)) minmax(0,var(--col-right))]
 * - Resize handles are absolutely positioned over the grid at the left and right boundaries.
 * - Fractions are clamped with configurable min/max so panels remain usable at all sizes.
 */
const Panel: React.FC<React.PropsWithChildren<{
  className?: string;
  panelClassName?: string;
  contentClassName?: string;
}>> = ({
  className: panelCls,
  panelClassName,
  contentClassName,
  children,
}) => (
    <section
      className={cn("min-w-0 overflow-hidden transition-[width,opacity] duration-300 ease-in-out", panelCls, panelClassName)}
    >
      <div
        className={cn(
          "h-full min-h-0 w-full overflow-x-hidden overflow-y-auto",
          contentClassName,
        )}
      >
        {children}
      </div>
    </section>
  );

export function PanelsContainer({
  left,
  center,
  right,
  showDividers = true,
  className,
  panelClassName,
  contentClassName,
  "aria-label": ariaLabel = "Designer panel layout",
  minLeftPct = 0.12,
  maxLeftPct = 0.33,
  minRightPct = 0.12,
  maxRightPct = 0.33,
  keyboardStepPct = 0.02,
  leftCollapsed = false,
  rightCollapsed = false,
  onLeftCollapseChange,
  onRightCollapseChange,
}: PanelsContainerProps) {
  const hasLeft = Boolean(left);
  const hasRight = Boolean(right);
  const hasCenter = Boolean(center);

  // Fractions for side panels (center is derived as 1 - (left + right))
  const [leftPct, setLeftPct] = React.useState<number>(hasLeft ? 0.2 : 0);
  const [rightPct, setRightPct] = React.useState<number>(hasRight ? 0.24 : 0);

  const rootRef = React.useRef<HTMLDivElement | null>(null);
  const dragRef = React.useRef<{
    edge: Edge;
    startX: number;
    startLeft: number;
    startRight: number;
    containerWidth: number;
  } | null>(null);

  const clamp = (v: number, lo: number, hi: number): number =>
    Math.max(lo, Math.min(hi, v));

  const recompute = React.useCallback(
    (lp: number, rp: number) => {
      if (!hasCenter) return { l: 0, c: 0, r: 0 };

      // Effective widths (0 if collapsed)
      const effectiveL = leftCollapsed ? 0 : lp;
      const effectiveR = rightCollapsed ? 0 : rp;

      // When logic runs, we must clamp the *underlying* percentages (lp, rp)
      // but return 0 for the CSS vars if collapsed.

      // Actually, if collapsed, we just want the CSS var to be 0.
      // But we maintain the state `leftPct` so it restores correctly.

      if (hasLeft && hasRight) {
        // Standard clamp (on the state values)
        const lState = clamp(lp, minLeftPct, maxLeftPct);
        const rState = clamp(rp, minRightPct, maxRightPct);

        // Effective output
        const l = leftCollapsed ? 0 : lState;
        const r = rightCollapsed ? 0 : rState;

        // Center takes remainder
        const c = 1 - (l + r);
        return { l, c, r };
      }
      if (hasLeft && !hasRight) {
        const lState = clamp(lp, minLeftPct, maxLeftPct);
        const l = leftCollapsed ? 0 : lState;
        const c = 1 - l;
        return { l, c, r: 0 };
      }
      if (!hasLeft && hasRight) {
        const rState = clamp(rp, minRightPct, maxRightPct);
        const r = rightCollapsed ? 0 : rState;
        const c = 1 - r;
        return { l: 0, c, r };
      }
      // Center only
      return { l: 0, c: 1, r: 0 };
    },
    [
      hasCenter,
      hasLeft,
      hasRight,
      minLeftPct,
      maxLeftPct,
      minRightPct,
      maxRightPct,
      leftCollapsed,
      rightCollapsed
    ],
  );

  const { l, c, r } = recompute(leftPct, rightPct);

  // Attach/detach global pointer handlers safely
  const onPointerMove = React.useCallback(
    (e: PointerEvent) => {
      const d = dragRef.current;
      if (!d || d.containerWidth <= 0) return;

      const deltaPx = e.clientX - d.startX;
      const deltaPct = deltaPx / d.containerWidth;

      if (d.edge === "left" && hasLeft && !leftCollapsed) {
        const nextLeft = clamp(d.startLeft + deltaPct, minLeftPct, maxLeftPct);
        setLeftPct(nextLeft);
      } else if (d.edge === "right" && hasRight && !rightCollapsed) {
        // Dragging the right edge moves leftwards as delta increases
        const nextRight = clamp(
          d.startRight - deltaPct,
          minRightPct,
          maxRightPct,
        );
        setRightPct(nextRight);
      }
    },
    [hasLeft, hasRight, minLeftPct, maxLeftPct, minRightPct, maxRightPct, leftCollapsed, rightCollapsed],
  );

  const endDrag = React.useCallback(() => {
    dragRef.current = null;
    window.removeEventListener("pointermove", onPointerMove);
    window.removeEventListener("pointerup", endDrag);
  }, [onPointerMove]);

  const startDrag =
    (edge: Edge) => (e: React.PointerEvent<HTMLButtonElement>) => {
      if (!rootRef.current) return;
      e.preventDefault();

      const rect = rootRef.current.getBoundingClientRect();
      dragRef.current = {
        edge,
        startX: e.clientX,
        startLeft: leftPct,
        startRight: rightPct,
        containerWidth: rect.width,
      };

      window.addEventListener("pointermove", onPointerMove);
      window.addEventListener("pointerup", endDrag);
    };

  React.useEffect(() => {
    return () => {
      // Cleanup if unmounted mid-drag
      window.removeEventListener("pointermove", onPointerMove);
      window.removeEventListener("pointerup", endDrag);
    };
  }, [onPointerMove, endDrag]);

  // Keyboard resize for handles
  const onKeyResize =
    (edge: Edge) => (e: React.KeyboardEvent<HTMLButtonElement>) => {
      if (e.key !== "ArrowLeft" && e.key !== "ArrowRight") return;
      e.preventDefault();

      const step = (e.shiftKey ? 2 : 1) * keyboardStepPct;

      if (edge === "left" && hasLeft && !leftCollapsed) {
        const next = clamp(
          leftPct + (e.key === "ArrowRight" ? step : -step),
          minLeftPct,
          maxLeftPct,
        );
        setLeftPct(next);
      } else if (edge === "right" && hasRight && !rightCollapsed) {
        const next = clamp(
          rightPct + (e.key === "ArrowLeft" ? step : -step),
          minRightPct,
          maxRightPct,
        );
        setRightPct(next);
      }
    };

  // CSS variables for the grid fractions
  // We use FR units instead of % to let the browser handle exact pixel fitting without rounding errors causing overflow
  const styleVars: React.CSSProperties & Record<string, string> = hasCenter
    ? {
      "--col-left": `${hasLeft ? l : 0}fr`,
      "--col-center": `${c}fr`,
      "--col-right": `${hasRight ? r : 0}fr`,
    }
    : {};

  // Explicit grid template depending on which side panels exist
  const gridAreas =
    hasLeft && hasRight
      ? '"left center right"'
      : hasLeft && !hasRight
        ? '"left center"'
        : !hasLeft && hasRight
          ? '"center right"'
          : '"center"';

  const gridCols =
    hasLeft && hasRight
      ? "[grid-template-columns:var(--col-left)_var(--col-center)_var(--col-right)]"
      : hasLeft && !hasRight
        ? "[grid-template-columns:var(--col-left)_var(--col-center)]"
        : !hasLeft && hasRight
          ? "[grid-template-columns:var(--col-center)_var(--col-right)]"
          : "[grid-template-columns:1fr]";

  // Dividers on the center panel only (prevents double borders if children have their own borders)
  const centerDividers =
    showDividers && hasCenter
      ? cn({
        "border-l": hasLeft,
        "border-r": hasRight,
      })
      : undefined;



  return (
    <>
      {/* Mobile Layout (Flex + Sheets) */}
      <div className={cn("flex flex-col h-full w-full md:hidden", className)}>
        {/* Mobile Header/Toolbar for access to panels */}
        <div className="flex items-center justify-between border-b px-4 py-2 bg-background">
          <div className="flex items-center gap-2">
            {hasLeft && (
              <Sheet>
                <SheetTrigger asChild>
                  <Button variant="outline" size="icon" className="h-8 w-8">
                    <PanelLeft className="h-4 w-4" />
                  </Button>
                </SheetTrigger>
                <SheetContent side="left" className="w-[85vw] p-0 sm:max-w-md">
                  <div className="h-full overflow-hidden">
                    {left}
                  </div>
                </SheetContent>
              </Sheet>
            )}
            <span className="text-sm font-medium">Designer</span>
          </div>

          {hasRight && (
            <Sheet>
              <SheetTrigger asChild>
                <Button variant="outline" size="icon" className="h-8 w-8">
                  <Settings2 className="h-4 w-4" />
                </Button>
              </SheetTrigger>
              <SheetContent side="right" className="w-[85vw] p-0 sm:max-w-md">
                <div className="h-full overflow-hidden">
                  {right}
                </div>
              </SheetContent>
            </Sheet>
          )}
        </div>

        {/* Main Content (Center) */}
        <div className="flex-1 min-h-0 min-w-0 overflow-hidden relative">
          {center}
        </div>
      </div>

      {/* Desktop Layout (Grid) */}
      <div
        ref={rootRef}
        aria-label={ariaLabel}
        className={cn(
          "relative hidden md:grid h-full min-h-0 w-full max-w-full overflow-hidden select-none",
          // 2-3-2 ratio for left-center-right panels when all visible
          hasLeft && hasRight && !leftCollapsed && !rightCollapsed && "grid-cols-[2fr_3fr_2fr]",
          // Left collapsed: center + right (3:2 ratio)
          hasLeft && hasRight && leftCollapsed && !rightCollapsed && "grid-cols-[3fr_2fr]",
          // Right collapsed: left + center (2:3 ratio)
          hasLeft && hasRight && !leftCollapsed && rightCollapsed && "grid-cols-[2fr_3fr]",
          // Both collapsed: center only
          hasLeft && hasRight && leftCollapsed && rightCollapsed && "grid-cols-1",
          // Only left and center
          hasLeft && !hasRight && !leftCollapsed && "grid-cols-[2fr_3fr]",
          hasLeft && !hasRight && leftCollapsed && "grid-cols-1",
          // Only center and right
          !hasLeft && hasRight && !rightCollapsed && "grid-cols-[3fr_2fr]",
          !hasLeft && hasRight && rightCollapsed && "grid-cols-1",
          // Only center
          !hasLeft && !hasRight && "grid-cols-1",
          className,
        )}
      >
        {hasLeft && !leftCollapsed && (
          <Panel
            panelClassName={panelClassName}
            contentClassName={contentClassName}
          >
            {left}
          </Panel>
        )}

        {hasCenter && (
          <Panel
            className={centerDividers}
            panelClassName={panelClassName}
            contentClassName={contentClassName}
          >
            {center}
          </Panel>
        )}

        {hasRight && !rightCollapsed && (
          <Panel
            panelClassName={panelClassName}
            contentClassName={contentClassName}
          >
            {right}
          </Panel>
        )}

        {/* Resize Handles */}
        {hasLeft && !leftCollapsed && (
          <button
            type="button"
            className="absolute top-0 bottom-0 w-1.5 -ml-0.75 z-50 cursor-col-resize hover:bg-blue-400/50 transition-colors focus:outline-none"
            style={{ left: "var(--col-left)" }}
            onPointerDown={startDrag("left")}
            onKeyDown={onKeyResize("left")}
            aria-label="Resize left panel"
          />
        )}
        {hasRight && !rightCollapsed && (
          <button
            type="button"
            className="absolute top-0 bottom-0 w-1.5 -mr-0.75 z-50 cursor-col-resize hover:bg-blue-400/50 transition-colors focus:outline-none"
            style={{ right: "var(--col-right)" }}
            onPointerDown={startDrag("right")}
            onKeyDown={onKeyResize("right")}
            aria-label="Resize right panel"
          />
        )}
      </div>
    </>
  );
}

export default PanelsContainer;
