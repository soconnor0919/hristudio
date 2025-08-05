import { notFound } from "next/navigation";
import { EnhancedBlockDesigner } from "~/components/experiments/designer/EnhancedBlockDesigner";
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

    return (
      <EnhancedBlockDesigner
        experimentId={experiment.id}
        initialDesign={{
          id: experiment.id,
          name: experiment.name,
          description: experiment.description ?? "",
          blocks: [],
          version: 1,
          lastSaved: new Date(),
        }}
      />
    );
  } catch (error) {
    console.error("Error loading experiment:", error);
    notFound();
  }
}

export async function generateMetadata({
  params,
}: ExperimentDesignerPageProps) {
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
