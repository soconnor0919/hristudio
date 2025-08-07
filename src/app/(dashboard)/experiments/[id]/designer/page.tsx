import { notFound } from "next/navigation";
import { EnhancedBlockDesigner } from "~/components/experiments/designer/EnhancedBlockDesigner";
import type { ExperimentBlock } from "~/components/experiments/designer/EnhancedBlockDesigner";
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
      blocks?: unknown[];
      version?: number;
      lastSaved?: string;
    } | null;

    // Only pass initialDesign if there's existing visual design data
    const initialDesign =
      existingDesign?.blocks && existingDesign.blocks.length > 0
        ? {
            id: experiment.id,
            name: experiment.name,
            description: experiment.description ?? "",
            blocks: existingDesign.blocks as ExperimentBlock[],
            version: existingDesign.version ?? 1,
            lastSaved:
              typeof existingDesign.lastSaved === "string"
                ? new Date(existingDesign.lastSaved)
                : new Date(),
          }
        : undefined;

    return (
      <EnhancedBlockDesigner
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
      title: `${experiment?.name} - Flow Designer | HRIStudio`,
      description: `Design experiment protocol for ${experiment?.name} using visual flow editor`,
    };
  } catch {
    return {
      title: "Experiment Flow Designer | HRIStudio",
      description: "Immersive visual experiment protocol designer",
    };
  }
}
