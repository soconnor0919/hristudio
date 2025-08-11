import { notFound } from "next/navigation";
import { DesignerRoot } from "~/components/experiments/designer/DesignerRoot";
import type { ExperimentStep } from "~/lib/experiment-designer/types";
import { api } from "~/trpc/server";

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
    const initialDesign =
      existingDesign?.steps && existingDesign.steps.length > 0
        ? {
            id: experiment.id,
            name: experiment.name,
            description: experiment.description ?? "",
            steps: existingDesign.steps as ExperimentStep[],
            version: existingDesign.version ?? 1,
            lastSaved:
              typeof existingDesign.lastSaved === "string"
                ? new Date(existingDesign.lastSaved)
                : new Date(),
          }
        : undefined;

    return (
      <DesignerRoot
        experimentId={experiment.id}
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
