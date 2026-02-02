"use client";

import * as React from "react";
import { cn } from "~/lib/utils";

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
}

/**
 * PanelsContainer
 *
 * Tailwind-first, grid-based panel layout with:
 * - Drag-resizable left/right panels (no persistence)
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
      className={cn("min-w-0 overflow-hidden", panelCls, panelClassName)}
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

      if (hasLeft && hasRight) {
        const l = clamp(lp, minLeftPct, maxLeftPct);
        const r = clamp(rp, minRightPct, maxRightPct);
        const c = Math.max(0.1, 1 - (l + r)); // always preserve some center space
        return { l, c, r };
      }
      if (hasLeft && !hasRight) {
        const l = clamp(lp, minLeftPct, maxLeftPct);
        const c = Math.max(0.2, 1 - l);
        return { l, c, r: 0 };
      }
      if (!hasLeft && hasRight) {
        const r = clamp(rp, minRightPct, maxRightPct);
        const c = Math.max(0.2, 1 - r);
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

      if (d.edge === "left" && hasLeft) {
        const nextLeft = clamp(d.startLeft + deltaPct, minLeftPct, maxLeftPct);
        setLeftPct(nextLeft);
      } else if (d.edge === "right" && hasRight) {
        // Dragging the right edge moves leftwards as delta increases
        const nextRight = clamp(
          d.startRight - deltaPct,
          minRightPct,
          maxRightPct,
        );
        setRightPct(nextRight);
      }
    },
    [hasLeft, hasRight, minLeftPct, maxLeftPct, minRightPct, maxRightPct],
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

      if (edge === "left" && hasLeft) {
        const next = clamp(
          leftPct + (e.key === "ArrowRight" ? step : -step),
          minLeftPct,
          maxLeftPct,
        );
        setLeftPct(next);
      } else if (edge === "right" && hasRight) {
        const next = clamp(
          rightPct + (e.key === "ArrowLeft" ? step : -step),
          minRightPct,
          maxRightPct,
        );
        setRightPct(next);
      }
    };

  // CSS variables for the grid fractions
  const styleVars: React.CSSProperties & Record<string, string> = hasCenter
    ? {
      "--col-left": `${(hasLeft ? l : 0) * 100}%`,
      "--col-center": `${c * 100}%`,
      "--col-right": `${(hasRight ? r : 0) * 100}%`,
    }
    : {};

  // Explicit grid template depending on which side panels exist
  const gridCols =
    hasLeft && hasRight
      ? "[grid-template-columns:minmax(0,var(--col-left))_minmax(0,var(--col-center))_minmax(0,var(--col-right))]"
      : hasLeft && !hasRight
        ? "[grid-template-columns:minmax(0,var(--col-left))_minmax(0,var(--col-center))]"
        : !hasLeft && hasRight
          ? "[grid-template-columns:minmax(0,var(--col-center))_minmax(0,var(--col-right))]"
          : "[grid-template-columns:minmax(0,1fr)]";

  // Dividers on the center panel only (prevents double borders if children have their own borders)
  const centerDividers =
    showDividers && hasCenter
      ? cn({
        "border-l": hasLeft,
        "border-r": hasRight,
      })
      : undefined;



  return (
    <div
      ref={rootRef}
      aria-label={ariaLabel}
      style={styleVars}
      className={cn(
        "relative grid h-full min-h-0 w-full overflow-hidden select-none",
        gridCols,
        className,
      )}
    >
      {hasLeft && (
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

      {hasRight && (
        <Panel
          panelClassName={panelClassName}
          contentClassName={contentClassName}
        >
          {right}
        </Panel>
      )}

      {/* Resize handles (only render where applicable) */}
      {hasCenter && hasLeft && (
        <button
          type="button"
          role="separator"
          aria-label="Resize left panel"
          aria-orientation="vertical"
          onPointerDown={startDrag("left")}
          onKeyDown={onKeyResize("left")}
          className={cn(
            "absolute inset-y-0 z-10 w-1 cursor-col-resize outline-none",
            "focus-visible:ring-ring focus-visible:ring-2",
          )}
          // Position at the boundary between left and center
          style={{ left: "var(--col-left)", transform: "translateX(-0.5px)" }}
          tabIndex={0}
        />
      )}

      {hasCenter && hasRight && (
        <button
          type="button"
          role="separator"
          aria-label="Resize right panel"
          aria-orientation="vertical"
          onPointerDown={startDrag("right")}
          onKeyDown={onKeyResize("right")}
          className={cn(
            "absolute inset-y-0 z-10 w-1 cursor-col-resize outline-none",
            "focus-visible:ring-ring focus-visible:ring-2",
          )}
          // Position at the boundary between center and right (offset from the right)
          style={{ right: "var(--col-right)", transform: "translateX(0.5px)" }}
          tabIndex={0}
        />
      )}
    </div>
  );
}

export default PanelsContainer;
