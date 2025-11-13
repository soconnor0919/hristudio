import { ExperimentForm } from "~/components/experiments/ExperimentForm";

interface EditExperimentPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default async function EditExperimentPage({
  params,
}: EditExperimentPageProps) {
  const { id } = await params;

  return <ExperimentForm mode="edit" experimentId={id} />;
}
