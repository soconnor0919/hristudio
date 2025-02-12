"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { use } from "react";
import { StudyParticipants } from "~/components/studies/study-participants";

export default function ParticipantsPage({ params }: { params: Promise<{ id: string }> }) {
  const router = useRouter();
  const resolvedParams = use(params);
  const studyId = Number(resolvedParams.id);

  const { data: study, isLoading } = api.study.getById.useQuery({ id: studyId });

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!study) {
    return <div>Study not found</div>;
  }

  return (
    <>
      <PageHeader
        title="Participants"
        description={`Manage participants for ${study.title}`}
      />
      <PageContent>
        <StudyParticipants studyId={studyId} role={study.role} />
      </PageContent>
    </>
  );
} 