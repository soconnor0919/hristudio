import { notFound } from "next/navigation";
import { ExperimentDesignerClient } from "~/components/experiments/designer/ExperimentDesignerClient";
import { api } from "~/trpc/server";

interface ExperimentDesignerPageProps {
  params: {
    id: string;
  };
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

    return <ExperimentDesignerClient experiment={experiment} />;
  } catch (error) {
    console.error("Error loading experiment:", error);
    notFound();
  }
}
