"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { StudyForm, type StudyFormValues } from "~/components/studies/study-form";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";

export default function NewStudyPage() {
  const router = useRouter();

  const { mutate: createStudy, isPending: isCreating } = api.study.create.useMutation({
    onSuccess: (data) => {
      router.push(`/dashboard/studies/${data.id}`);
      router.refresh();
    },
  });

  function onSubmit(data: StudyFormValues) {
    createStudy(data);
  }

  return (
    <>
      <PageHeader
        title="New Study"
        description="Create a new study"
      />
      <PageContent>
        <Card>
          <CardHeader>
            <CardTitle>Study Details</CardTitle>
            <CardDescription>
              Enter the information for your new study.
            </CardDescription>
          </CardHeader>
          <CardContent>
            <StudyForm
              defaultValues={{ title: "", description: "" }}
              onSubmit={onSubmit}
              isSubmitting={isCreating}
              submitLabel="Create Study"
            />
          </CardContent>
        </Card>
      </PageContent>
    </>
  );
} 