import { TrialForm } from "~/components/trials/TrialForm";

interface NewStudyTrialPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default async function NewStudyTrialPage({
  params,
}: NewStudyTrialPageProps) {
  const { id } = await params;

  return <TrialForm mode="create" studyId={id} />;
}
