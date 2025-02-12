"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { ParticipantForm, type ParticipantFormValues } from "~/components/participants/participant-form";
import { use } from "react";
import { useToast } from "~/hooks/use-toast";
import { ROLES } from "~/lib/permissions/constants";
import { CardSkeleton } from "~/components/ui/skeleton";

export default function EditParticipantPage({
  params,
}: {
  params: Promise<{ id: string; participantId: string }>;
}) {
  const router = useRouter();
  const { toast } = useToast();
  const resolvedParams = use(params);
  const studyId = Number(resolvedParams.id);
  const participantId = Number(resolvedParams.participantId);

  const { data: study } = api.study.getById.useQuery({ id: studyId });
  const { data: participant, isLoading } = api.participant.getById.useQuery({ id: participantId });

  const { mutate: updateParticipant, isPending: isUpdating } = api.participant.update.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Participant updated successfully",
      });
      router.push(`/dashboard/studies/${studyId}/participants/${participantId}`);
      router.refresh();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  function onSubmit(data: ParticipantFormValues) {
    updateParticipant({
      id: participantId,
      ...data,
    });
  }

  if (isLoading) {
    return (
      <>
        <PageHeader
          title="Edit Participant"
          description="Loading participant information..."
        />
        <PageContent>
          <Card>
            <CardHeader>
              <CardTitle>Participant Details</CardTitle>
              <CardDescription>
                Please wait while we load the participant information.
              </CardDescription>
            </CardHeader>
            <CardContent>
              <CardSkeleton />
            </CardContent>
          </Card>
        </PageContent>
      </>
    );
  }

  if (!study || !participant) {
    return <div>Not found</div>;
  }

  // Check if user has permission to edit participants
  const canManageParticipants = [ROLES.OWNER, ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR]
    .map(r => r.toLowerCase())
    .includes(study.role.toLowerCase());

  if (!canManageParticipants) {
    return <div>You do not have permission to edit participants in this study.</div>;
  }

  return (
    <>
      <PageHeader
        title="Edit Participant"
        description={`Update participant details for ${study.title}`}
      />
      <PageContent>
        <Card>
          <CardHeader>
            <CardTitle>Participant Details</CardTitle>
            <CardDescription>
              Update the participant's information. Fields marked with * are required.
            </CardDescription>
          </CardHeader>
          <CardContent>
            <ParticipantForm
              defaultValues={{
                identifier: participant.identifier ?? "",
                email: participant.email ?? "",
                firstName: participant.firstName ?? "",
                lastName: participant.lastName ?? "",
                notes: participant.notes ?? "",
                status: participant.status,
              }}
              onSubmit={onSubmit}
              isSubmitting={isUpdating}
              submitLabel="Save Changes"
            />
          </CardContent>
        </Card>
      </PageContent>
    </>
  );
} 