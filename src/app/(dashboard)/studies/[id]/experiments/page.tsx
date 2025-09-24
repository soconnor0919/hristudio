"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { FlaskConical, Plus } from "lucide-react";
import { ExperimentsDataTable } from "~/components/experiments/experiments-data-table";
import { PageHeader } from "~/components/ui/page-header";
import { Button } from "~/components/ui/button";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

export default function StudyExperimentsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Experiments" },
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
        title="Experiments"
        description="Design and manage experiment protocols for this study"
        icon={FlaskConical}
        actions={
          <Button asChild>
            <a href={`/studies/${studyId}/experiments/new`}>
              <Plus className="mr-2 h-4 w-4" />
              Create Experiment
            </a>
          </Button>
        }
      />

      <Suspense fallback={<div>Loading experiments...</div>}>
        <ExperimentsDataTable />
      </Suspense>
    </div>
  );
}
