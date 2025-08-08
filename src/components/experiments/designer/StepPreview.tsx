import { memo } from "react";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";

/**
 * Lightweight, dependency‑minimal placeholder for step preview rendering in the
 * experiment designer. This was added to satisfy references expecting the file
 * to exist (diagnostics previously reported it missing).
 *
 * Replace / extend this component when richer preview logic (block graphs,
 * parameter summaries, validation states, drift indicators) is implemented.
 *
 * Design Goals:
 * - Zero external (designer-internal) imports to avoid circular dependencies
 * - Strict typing without leaking un-finalized internal step model types
 * - Safe rendering even with partial or incomplete data
 * - Pure presentational; no side-effects or client hooks required
 */

export interface StepPreviewAction {
  id?: string;
  name: string;
  description?: string | null;
  type?: string | null;
  pluginId?: string | null;
  pluginVersion?: string | null;
  category?: string | null;
}

export interface StepPreviewProps {
  id?: string;
  name: string;
  description?: string | null;
  type?: string | null;
  orderIndex?: number;
  required?: boolean;
  durationEstimateSeconds?: number;
  actions?: StepPreviewAction[];
  conditions?: unknown;
  validationIssues?: readonly string[];
  integrityHashFragment?: string | null;
  /**
   * When true, shows a subtle placeholder treatment (e.g. while constructing
   * from a transient visual design mutation).
   */
  transient?: boolean;
}

/**
 * Stateless pure component – safe to use in server or client trees.
 */
export const StepPreview = memo(function StepPreview({
  name,
  description,
  type,
  orderIndex,
  required,
  durationEstimateSeconds,
  actions = [],
  conditions,
  validationIssues,
  integrityHashFragment,
  transient,
}: StepPreviewProps) {
  const hasIssues = (validationIssues?.length ?? 0) > 0;

  return (
    <Card
      data-transient={transient ? "true" : "false"}
      className={[
        "relative overflow-hidden border",
        transient ? "opacity-70" : "",
        hasIssues ? "border-red-300 dark:border-red-500" : "",
      ]
        .filter(Boolean)
        .join(" ")}
    >
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between gap-2">
          <div>
            <CardTitle className="text-sm font-semibold">
              {orderIndex !== undefined && (
                <span className="text-muted-foreground mr-2 text-xs">
                  #{orderIndex + 1}
                </span>
              )}
              {name || "(Untitled Step)"}
            </CardTitle>
            {description && (
              <p className="text-muted-foreground mt-1 line-clamp-2 text-xs">
                {description}
              </p>
            )}
          </div>
          <div className="flex shrink-0 flex-col items-end gap-1">
            {type && (
              <Badge variant="outline" className="text-[10px] uppercase">
                {type}
              </Badge>
            )}
            {required && (
              <Badge
                variant="secondary"
                className="border border-blue-200 bg-blue-50 text-[10px] font-medium text-blue-700 dark:border-blue-400/40 dark:bg-blue-400/10 dark:text-blue-300"
              >
                Required
              </Badge>
            )}
            {hasIssues && (
              <Badge
                variant="destructive"
                className="text-[10px] font-medium tracking-wide"
              >
                {validationIssues?.length} Issue
                {validationIssues && validationIssues.length > 1 ? "s" : ""}
              </Badge>
            )}
          </div>
        </div>
      </CardHeader>
      <CardContent className="pt-0">
        <div className="flex flex-col gap-3">
          <div className="text-muted-foreground flex flex-wrap items-center gap-2 text-xs">
            {durationEstimateSeconds !== undefined && (
              <span>≈ {Math.max(1, Math.round(durationEstimateSeconds))}s</span>
            )}
            {actions.length > 0 && (
              <span>
                {actions.length} action{actions.length > 1 ? "s" : ""}
              </span>
            )}
            {conditions !== undefined && conditions !== null && (
              <span>Conditional</span>
            )}
            {integrityHashFragment && (
              <span className="truncate font-mono text-[10px] opacity-70">
                hash:{integrityHashFragment.slice(0, 8)}
              </span>
            )}
            {transient && <span className="italic">transient</span>}
          </div>

          {/* Action summary */}
          {actions.length > 0 && (
            <ol className="space-y-1">
              {actions.slice(0, 5).map((a, idx) => (
                <li
                  key={a.id ?? `${a.name}-${idx}`}
                  className="bg-muted/30 flex items-center gap-2 rounded border px-2 py-1 text-xs"
                >
                  <span className="font-medium">{a.name}</span>
                  {a.type && (
                    <span className="bg-background text-muted-foreground rounded px-1 py-0.5 text-[10px] uppercase">
                      {a.type}
                    </span>
                  )}
                  {a.pluginId && (
                    <span className="text-muted-foreground truncate text-[10px]">
                      {a.pluginId}
                      {a.pluginVersion && (
                        <span className="opacity-60">@{a.pluginVersion}</span>
                      )}
                    </span>
                  )}
                </li>
              ))}
              {actions.length > 5 && (
                <li className="text-muted-foreground text-[10px] italic">
                  + {actions.length - 5} more…
                </li>
              )}
            </ol>
          )}

          {hasIssues && validationIssues && (
            <ul className="space-y-1 rounded border border-red-300/50 bg-red-50/60 p-2 text-[11px] text-red-700 dark:border-red-500/40 dark:bg-red-950/30 dark:text-red-300">
              {validationIssues.slice(0, 3).map((issue, i) => (
                <li key={i} className="leading-snug">
                  • {issue}
                </li>
              ))}
              {validationIssues.length > 3 && (
                <li className="opacity-70">
                  + {validationIssues.length - 3} more…
                </li>
              )}
            </ul>
          )}
        </div>
      </CardContent>
    </Card>
  );
});

export default StepPreview;
