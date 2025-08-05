import { notFound } from "next/navigation";
import { ExperimentDesignerClient } from "~/components/experiments/designer/ExperimentDesignerClient";
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
      <div className="fixed inset-0 z-50">
        <ExperimentDesignerClient
          experiment={{
            ...experiment,
            description: experiment.description ?? "",
          }}
        />
      </div>
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
