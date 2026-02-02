"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { Puzzle, Plus } from "lucide-react";
import { PluginsDataTable } from "~/components/plugins/plugins-data-table";
import { PageHeader } from "~/components/ui/page-header";
import { Button } from "~/components/ui/button";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

export default function StudyPluginsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Plugins" },
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
        title="Plugins"
        description="Manage robot plugins and capabilities for this study"
        icon={Puzzle}
        actions={
          <Button asChild>
            <a href={`/studies/${studyId}/plugins/browse`}>
              <Plus className="mr-2 h-4 w-4" />
              Browse Plugin Store
            </a>
          </Button>
        }
      />

      <Suspense fallback={<div>Loading plugins...</div>}>
        <PluginsDataTable />
      </Suspense>
    </div>
  );
}
