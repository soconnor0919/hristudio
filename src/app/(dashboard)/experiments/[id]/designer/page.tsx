import { notFound } from "next/navigation";
import type {
  ExperimentStep,
  ExperimentAction,
  StepType,
  ActionCategory,
  ExecutionDescriptor,
} from "~/lib/experiment-designer/types";
import { api } from "~/trpc/server";
import { DesignerPageClient } from "./DesignerPageClient";

interface ExperimentDesignerPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default async function ExperimentDesignerPage({
  params,
}: ExperimentDesignerPageProps) {
  try {
    const resolvedParams = await params;
    const experiment = await api.experiments.get({ id: resolvedParams.id });

    if (!experiment) {
      notFound();
    }

    // Parse existing visual design if available
    const existingDesign = experiment.visualDesign as {
      steps?: unknown[];
      version?: number;
      lastSaved?: string;
    } | null;

    // Only pass initialDesign if there's existing visual design data
    let initialDesign:
      | {
          id: string;
          name: string;
          description: string;
          steps: ExperimentStep[];
          version: number;
          lastSaved: Date;
        }
      | undefined;

    if (existingDesign?.steps && existingDesign.steps.length > 0) {
      initialDesign = {
        id: experiment.id,
        name: experiment.name,
        description: experiment.description ?? "",
        steps: existingDesign.steps as ExperimentStep[],
        version: existingDesign.version ?? 1,
        lastSaved:
          typeof existingDesign.lastSaved === "string"
            ? new Date(existingDesign.lastSaved)
            : new Date(),
      };
    } else {
      // Fallback: hydrate from DB steps/actions if visualDesign is empty

      const exec = await api.experiments.getExecutionData({
        experimentId: experiment.id,
      });
      if (exec.steps.length > 0) {
        type InstalledStudyPlugin = {
          plugin: {
            id: string;
            name: string;
            version: string | null;
            actionDefinitions: Array<{ id: string }> | null;
          };
        };
        const rawInstalledPluginsUnknown: unknown =
          await api.robots.plugins.getStudyPlugins({
            studyId: experiment.study.id,
          });

        function asRecord(v: unknown): Record<string, unknown> | null {
          return v && typeof v === "object"
            ? (v as Record<string, unknown>)
            : null;
        }

        function narrowActionDefs(v: unknown): Array<{ id: string }> | null {
          if (!Array.isArray(v)) return null;
          const out: Array<{ id: string }> = [];
          for (const item of v) {
            const rec = asRecord(item);
            const id = rec && typeof rec.id === "string" ? rec.id : null;
            if (id) out.push({ id });
          }
          return out.length ? out : null;
        }

        const installedPlugins: InstalledStudyPlugin[] = (
          Array.isArray(rawInstalledPluginsUnknown)
            ? (rawInstalledPluginsUnknown as unknown[])
            : []
        ).map((entry) => {
          const rec = asRecord(entry);
          const pluginRec = rec ? asRecord(rec.plugin) : null;

          const id =
            pluginRec && typeof pluginRec.id === "string" ? pluginRec.id : "";
          const name =
            pluginRec && typeof pluginRec.name === "string"
              ? pluginRec.name
              : "";
          const version =
            pluginRec && typeof pluginRec.version === "string"
              ? pluginRec.version
              : null;
          const actionDefinitions = narrowActionDefs(
            pluginRec ? pluginRec.actionDefinitions : undefined,
          );

          return {
            plugin: { id, name, version, actionDefinitions },
          };
        });
        const mapped: ExperimentStep[] = exec.steps.map((s, idx) => {
          const actions: ExperimentAction[] = s.actions.map((a) => {
            // Normalize legacy plugin action ids and provenance
            const rawType = a.type ?? "";

            // Try to resolve alias-style legacy ids using installed study plugins
            const dynamicLegacy = (() => {
              if (rawType.includes(".")) {
                const [alias, base] = rawType.split(".", 2);
                if (alias && base) {
                  const baseMap: Record<string, string> = {
                    speak: "say_text",
                    say: "say_text",
                    walk: "walk_to_position",
                    animation: "play_animation",
                    led: "set_led_color",
                    leds: "set_led_color",
                    sit: "sit_down",
                    stand: "stand_up",
                    head: "turn_head",
                    turn_head: "turn_head",
                  };
                  const mappedBase = baseMap[base] ?? base;
                  const candidate =
                    installedPlugins.find(
                      (p) =>
                        p.plugin.id.startsWith(alias) ||
                        p.plugin.name
                          .toLowerCase()
                          .includes(alias.toLowerCase()),
                    ) ?? null;
                  if (
                    candidate &&
                    Array.isArray(candidate.plugin.actionDefinitions) &&
                    candidate.plugin.actionDefinitions.some(
                      (ad) => ad.id === mappedBase,
                    )
                  ) {
                    return {
                      pluginId: candidate.plugin.id,
                      baseId: mappedBase,
                      pluginVersion: candidate.plugin.version ?? undefined,
                    };
                  }
                }
              }
              return null;
            })();

            const legacy = dynamicLegacy;

            const isPluginType = Boolean(legacy) || rawType.includes(".");
            const typeOut = legacy
              ? `${legacy.pluginId}.${legacy.baseId}`
              : rawType;

            const execution: ExecutionDescriptor = { transport: "internal" };

            const categoryOut: ActionCategory = isPluginType
              ? "robot"
              : "wizard";

            const sourceKind: "core" | "plugin" = isPluginType
              ? "plugin"
              : "core";
            const pluginId = legacy?.pluginId;
            const pluginVersion = legacy?.pluginVersion;

            return {
              id: a.id,
              type: typeOut,
              name: a.name,
              parameters: (a.parameters ?? {}) as Record<string, unknown>,
              category: categoryOut,
              source: {
                kind: sourceKind,
                pluginId,
                pluginVersion,
                robotId: null,
                baseActionId: legacy?.baseId,
              },
              execution,
            };
          });
          return {
            id: s.id,
            name: s.name,
            description: s.description ?? "",
            type: ((): StepType => {
              const raw = (s.type as string) ?? "sequential";
              if (raw === "wizard") return "sequential";
              const allowed = [
                "sequential",
                "parallel",
                "conditional",
                "loop",
              ] as const;
              return (allowed as readonly string[]).includes(raw)
                ? (raw as StepType)
                : "sequential";
            })(),
            order: s.orderIndex ?? idx,
            trigger: { type: "trial_start", conditions: {} },
            actions,
            expanded: true,
          };
        });
        initialDesign = {
          id: experiment.id,
          name: experiment.name,
          description: experiment.description ?? "",
          steps: mapped,
          version: experiment.version ?? 1,
          lastSaved: new Date(),
        };
      }
    }

    return (
      <DesignerPageClient
        experiment={experiment}
        initialDesign={initialDesign}
      />
    );
  } catch (error) {
    console.error("Error loading experiment:", error);
    notFound();
  }
}

export async function generateMetadata({
  params,
}: ExperimentDesignerPageProps): Promise<{
  title: string;
  description: string;
}> {
  try {
    const resolvedParams = await params;
    const experiment = await api.experiments.get({ id: resolvedParams.id });

    return {
      title: `${experiment?.name} - Designer | HRIStudio`,
      description: `Design experiment protocol for ${experiment?.name} using step-based editor`,
    };
  } catch {
    return {
      title: "Experiment Designer | HRIStudio",
      description: "Step-based experiment protocol designer",
    };
  }
}
