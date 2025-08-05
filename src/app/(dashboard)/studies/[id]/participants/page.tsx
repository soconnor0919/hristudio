"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { ParticipantsTable } from "~/components/participants/ParticipantsTable";
import { ManagementPageLayout } from "~/components/ui/page-layout";
import { useActiveStudy } from "~/hooks/useActiveStudy";

export default function StudyParticipantsPage() {
  const params = useParams();
  const studyId = typeof params.id === "string" ? params.id : "";
  const { setActiveStudy, activeStudy } = useActiveStudy();

  // Set the active study if it doesn't match the current route
  useEffect(() => {
    if (studyId && activeStudy?.id !== studyId) {
      setActiveStudy(studyId);
    }
  }, [studyId, activeStudy?.id, setActiveStudy]);

  return (
    <ManagementPageLayout
      title="Participants"
      description="Manage participant registration, consent, and trial assignments for this study"
      breadcrumb={[
        { label: "Dashboard", href: "/dashboard" },
        { label: "Studies", href: "/studies" },
        { label: activeStudy?.title ?? "Study", href: `/studies/${studyId}` },
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
