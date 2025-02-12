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

function generateIdentifier(studyId: number, count: number) {
  // Format: P001, P002, etc. with study prefix
  const paddedCount = String(count + 1).padStart(3, '0');
  return `P${paddedCount}`;
}

export default function NewParticipantPage({ params }: { params: Promise<{ id: string }> }) {
  const router = useRouter();
  const { toast } = useToast();
  const resolvedParams = use(params);
  const studyId = Number(resolvedParams.id);

  const { data: study } = api.study.getById.useQuery({ id: studyId });
  const { data: participantCount = 0 } = api.participant.getCount.useQuery(
    { studyId },
    { enabled: !!study }
  );

  const { mutate: createParticipant, isPending: isCreating } = api.participant.create.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Participant added successfully",
      });
      router.push(`/dashboard/studies/${studyId}/participants`);
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
    createParticipant({
      studyId,
      ...data,
    });
  }

  if (!study) {
    return <div>Study not found</div>;
  }

  // Check if user has permission to add participants
  const canAddParticipants = [ROLES.OWNER, ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR]
    .map(r => r.toLowerCase())
    .includes(study.role.toLowerCase());

  if (!canAddParticipants) {
    return <div>You do not have permission to add participants to this study.</div>;
  }

  return (
    <>
      <PageHeader
        title="Add Participant"
        description={`Add a new participant to ${study.title}`}
      />
      <PageContent>
        <Card>
          <CardHeader>
            <CardTitle>Participant Details</CardTitle>
            <CardDescription>
              Enter the participant's information. Fields marked with * are required.
            </CardDescription>
          </CardHeader>
          <CardContent>
            <ParticipantForm
              defaultValues={{
                identifier: generateIdentifier(studyId, participantCount),
                email: "",
                firstName: "",
                lastName: "",
                notes: "",
                status: "active",
              }}
              onSubmit={onSubmit}
              isSubmitting={isCreating}
              submitLabel="Add Participant"
            />
          </CardContent>
        </Card>
      </PageContent>
    </>
  );
} 