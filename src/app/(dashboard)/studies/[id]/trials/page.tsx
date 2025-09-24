"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import Link from "next/link";
import { TestTube, Plus } from "lucide-react";
import { TrialsTable } from "~/components/trials/TrialsTable";
import { PageHeader } from "~/components/ui/page-header";
import { Button } from "~/components/ui/button";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

export default function StudyTrialsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Trials" },
  ]);

  // Set the active study if it doesn't match the current route
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  return (
    <div className="space-y-6">
      <PageHeader
        title="Trials"
        description="Manage trial execution, scheduling, and data collection for this study"
        icon={TestTube}
        actions={
          <Button asChild>
            <Link href={`/studies/${studyId}/trials/new`}>
              <Plus className="mr-2 h-4 w-4" />
              Schedule Trial
            </Link>
          </Button>
        }
      />

      <Suspense fallback={<div>Loading trials...</div>}>
        <TrialsTable studyId={studyId} />
      </Suspense>
    </div>
  );
}
