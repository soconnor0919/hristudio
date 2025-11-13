import { ParticipantForm } from "~/components/participants/ParticipantForm";

interface NewStudyParticipantPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default async function NewStudyParticipantPage({
  params,
}: NewStudyParticipantPageProps) {
  const { id } = await params;

  return <ParticipantForm mode="create" studyId={id} />;
}
