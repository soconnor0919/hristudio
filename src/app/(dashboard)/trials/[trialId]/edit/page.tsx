import { TrialForm } from "~/components/trials/TrialForm";

interface EditTrialPageProps {
  params: Promise<{
    trialId: string;
  }>;
}

export default async function EditTrialPage({ params }: EditTrialPageProps) {
  const { trialId } = await params;

  return <TrialForm mode="edit" trialId={trialId} />;
}
