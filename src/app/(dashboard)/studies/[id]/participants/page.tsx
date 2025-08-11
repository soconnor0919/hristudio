"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { ParticipantsTable } from "~/components/participants/ParticipantsTable";
import { ManagementPageLayout } from "~/components/ui/page-layout";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

export default function StudyParticipantsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Sync selected study (unified study-context)
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  return (
    <ManagementPageLayout
      title="Participants"
      description="Manage participant registration, consent, and trial assignments for this study"
      breadcrumb={[
        { label: "Dashboard", href: "/dashboard" },
        { label: "Studies", href: "/studies" },
        { label: study?.name ?? "Study", href: `/studies/${studyId}` },
        { label: "Participants" },
      ]}
      createButton={{
        label: "Add Participant",
        href: `/studies/${studyId}/participants/new`,
      }}
    >
      <Suspense fallback={<div>Loading participants...</div>}>
        <ParticipantsTable studyId={studyId} />
      </Suspense>
    </ManagementPageLayout>
  );
}
