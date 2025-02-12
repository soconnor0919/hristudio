"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { StudyForm, type StudyFormValues } from "~/components/studies/study-form";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { use } from "react";
import { CardSkeleton } from "~/components/ui/skeleton";

export default function EditStudyPage({ params }: { params: Promise<{ id: string }> }) {
  const router = useRouter();
  const resolvedParams = use(params);
  const id = Number(resolvedParams.id);

  const { data: study, isLoading: isLoadingStudy } = api.study.getById.useQuery(
    { id }
  );

  const { mutate: updateStudy, isPending: isUpdating } = api.study.update.useMutation({
    onSuccess: () => {
      router.push(`/dashboard/studies/${id}`);
      router.refresh();
    },
  });

  function onSubmit(data: StudyFormValues) {
    updateStudy({ id, ...data });
  }

  if (isLoadingStudy) {
    return (
      <>
        <PageHeader
          title="Edit Study"
          description="Loading study details..."
        />
        <PageContent className="max-w-2xl">
          <Card>
            <CardHeader>
              <CardTitle>Study Details</CardTitle>
              <CardDescription>
                Please wait while we load the study information.
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

  if (!study) {
    return <div>Study not found</div>;
  }

  return (
    <>
      <PageHeader
        title="Edit Study"
        description="Update study details"
      />
      <PageContent className="max-w-2xl">
        <Card>
          <CardHeader>
            <CardTitle>Study Details</CardTitle>
            <CardDescription>
              Update the information for your study.
            </CardDescription>
          </CardHeader>
          <CardContent>
            <StudyForm
              defaultValues={{ title: study.title, description: study.description ?? "" }}
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