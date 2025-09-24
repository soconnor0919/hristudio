"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { Users, Plus } from "lucide-react";
import { ParticipantsTable } from "~/components/participants/ParticipantsTable";
import { PageHeader } from "~/components/ui/page-header";
import { Button } from "~/components/ui/button";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

export default function StudyParticipantsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Participants" },
  ]);

  // Sync selected study (unified study-context)
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  return (
    <div className="space-y-6">
      <PageHeader
        title="Participants"
        description="Manage participant registration, consent, and trial assignments for this study"
        icon={Users}
        actions={
          <Button asChild>
            <a href={`/studies/${studyId}/participants/new`}>
              <Plus className="mr-2 h-4 w-4" />
              Add Participant
            </a>
          </Button>
        }
      />

      <Suspense fallback={<div>Loading participants...</div>}>
        <ParticipantsTable studyId={studyId} />
      </Suspense>
    </div>
  );
}
