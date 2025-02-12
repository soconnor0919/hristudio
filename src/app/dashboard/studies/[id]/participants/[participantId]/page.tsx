"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Pencil as PencilIcon, Trash as TrashIcon } from "lucide-react";
import { use } from "react";
import { Badge } from "~/components/ui/badge";
import { useToast } from "~/hooks/use-toast";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "~/components/ui/alert-dialog";
import { ROLES } from "~/lib/permissions/constants";

export default function ParticipantDetailsPage({
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

  const { mutate: deleteParticipant, isPending: isDeleting } = api.participant.delete.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Participant deleted successfully",
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

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!study || !participant) {
    return <div>Not found</div>;
  }

  const canViewIdentifiableInfo = [ROLES.OWNER, ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR]
    .map(r => r.toLowerCase())
    .includes(study.role.toLowerCase());
  const canManageParticipants = [ROLES.OWNER, ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR]
    .map(r => r.toLowerCase())
    .includes(study.role.toLowerCase());

  return (
    <>
      <PageHeader
        title="Participant Details"
        description={`View participant details for ${study.title}`}
      >
        {canManageParticipants && (
          <div className="flex gap-2">
            <Button
              variant="outline"
              size="sm"
              onClick={() =>
                router.push(`/dashboard/studies/${studyId}/participants/${participantId}/edit`)
              }
            >
              <PencilIcon className="h-4 w-4 mr-2" />
              Edit
            </Button>
            <AlertDialog>
              <AlertDialogTrigger asChild>
                <Button variant="destructive" size="sm" disabled={isDeleting}>
                  <TrashIcon className="h-4 w-4 mr-2" />
                  Delete
                </Button>
              </AlertDialogTrigger>
              <AlertDialogContent>
                <AlertDialogHeader>
                  <AlertDialogTitle>Are you sure?</AlertDialogTitle>
                  <AlertDialogDescription>
                    This action cannot be undone. This will permanently delete the participant and all
                    associated data.
                  </AlertDialogDescription>
                </AlertDialogHeader>
                <AlertDialogFooter>
                  <AlertDialogCancel>Cancel</AlertDialogCancel>
                  <AlertDialogAction
                    onClick={() => deleteParticipant({ id: participantId })}
                    disabled={isDeleting}
                  >
                    {isDeleting ? "Deleting..." : "Delete"}
                  </AlertDialogAction>
                </AlertDialogFooter>
              </AlertDialogContent>
            </AlertDialog>
          </div>
        )}
      </PageHeader>
      <PageContent>
        <div className="grid gap-6">
          <Card>
            <CardHeader>
              <CardTitle>Basic Information</CardTitle>
              {!canViewIdentifiableInfo && (
                <CardDescription className="text-yellow-600">
                  Some information is redacted based on your role.
                </CardDescription>
              )}
            </CardHeader>
            <CardContent>
              <dl className="grid gap-4 sm:grid-cols-2">
                <div>
                  <dt className="text-sm font-medium text-muted-foreground">Identifier</dt>
                  <dd className="mt-1 text-sm">
                    {canViewIdentifiableInfo ? participant.identifier || "—" : "REDACTED"}
                  </dd>
                </div>
                <div>
                  <dt className="text-sm font-medium text-muted-foreground">Status</dt>
                  <dd className="mt-1">
                    <Badge variant="secondary">
                      {participant.status}
                    </Badge>
                  </dd>
                </div>
                <div>
                  <dt className="text-sm font-medium text-muted-foreground">Name</dt>
                  <dd className="mt-1 text-sm">
                    {canViewIdentifiableInfo
                      ? participant.firstName && participant.lastName
                        ? `${participant.firstName} ${participant.lastName}`
                        : "—"
                      : "REDACTED"}
                  </dd>
                </div>
                <div>
                  <dt className="text-sm font-medium text-muted-foreground">Email</dt>
                  <dd className="mt-1 text-sm">
                    {canViewIdentifiableInfo ? participant.email || "—" : "REDACTED"}
                  </dd>
                </div>
              </dl>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>Notes</CardTitle>
              <CardDescription>Additional information about this participant</CardDescription>
            </CardHeader>
            <CardContent>
              <p className="text-sm whitespace-pre-wrap">
                {participant.notes || "No notes available."}
              </p>
            </CardContent>
          </Card>
        </div>
      </PageContent>
    </>
  );
} 