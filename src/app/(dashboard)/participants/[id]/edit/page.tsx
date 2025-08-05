import { ParticipantForm } from "~/components/participants/ParticipantForm";

interface EditParticipantPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default async function EditParticipantPage({
  params,
}: EditParticipantPageProps) {
  const { id } = await params;

  return <ParticipantForm mode="edit" participantId={id} />;
}
