"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { TrialsTable } from "~/components/trials/TrialsTable";
import { ManagementPageLayout } from "~/components/ui/page-layout";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

export default function StudyTrialsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Set the active study if it doesn't match the current route
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  return (
    <ManagementPageLayout
      title="Trials"
      description="Schedule, execute, and monitor HRI experiment trials with real-time wizard control for this study"
      breadcrumb={[
        { label: "Dashboard", href: "/dashboard" },
        { label: "Studies", href: "/studies" },
        { label: study?.name ?? "Study", href: `/studies/${studyId}` },
        { label: "Trials" },
      ]}
      createButton={{
        label: "Schedule Trial",
        href: `/studies/${studyId}/trials/new`,
      }}
    >
      <Suspense fallback={<div>Loading trials...</div>}>
        <TrialsTable studyId={studyId} />
      </Suspense>
    </ManagementPageLayout>
  );
}
