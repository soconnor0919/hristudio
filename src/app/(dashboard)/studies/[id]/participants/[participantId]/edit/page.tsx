import { ParticipantForm } from "~/components/participants/ParticipantForm";
import { api } from "~/trpc/server";
import { notFound } from "next/navigation";

interface EditParticipantPageProps {
    params: Promise<{
        id: string;
        participantId: string;
    }>;
}

export default async function EditParticipantPage({
    params,
}: EditParticipantPageProps) {
    const { id: studyId, participantId } = await params;

    const participant = await api.participants.get({ id: participantId });

    if (!participant || participant.studyId !== studyId) {
        notFound();
    }

    // Transform data to match form expectations if needed, or pass directly
    return (
        <ParticipantForm
            mode="edit"
            studyId={studyId}
            participantId={participantId}
        />
    );
}
