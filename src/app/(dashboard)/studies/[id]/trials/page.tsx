"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { TrialsTable } from "~/components/trials/TrialsTable";
import { ManagementPageLayout } from "~/components/ui/page-layout";
import { useActiveStudy } from "~/hooks/useActiveStudy";

export default function StudyTrialsPage() {
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
      title="Trials"
      description="Schedule, execute, and monitor HRI experiment trials with real-time wizard control for this study"
      breadcrumb={[
        { label: "Dashboard", href: "/dashboard" },
        { label: "Studies", href: "/studies" },
        { label: activeStudy?.title ?? "Study", href: `/studies/${studyId}` },
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
