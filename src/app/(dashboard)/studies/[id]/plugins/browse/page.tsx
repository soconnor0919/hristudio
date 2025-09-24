"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { PluginStoreBrowse } from "~/components/plugins/plugin-store-browse";
import { ManagementPageLayout } from "~/components/ui/page-layout";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

export default function StudyPluginBrowsePage() {
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
      title="Plugin Store"
      description="Browse and install robot plugins for this study"
      breadcrumb={[
        { label: "Dashboard", href: "/dashboard" },
        { label: "Studies", href: "/studies" },
        { label: study?.name ?? "Study", href: `/studies/${studyId}` },
        { label: "Plugins", href: `/studies/${studyId}/plugins` },
        { label: "Browse" },
      ]}
    >
      <Suspense fallback={<div>Loading plugin store...</div>}>
        <PluginStoreBrowse />
      </Suspense>
    </ManagementPageLayout>
  );
}
