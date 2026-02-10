"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import { BarChart3 } from "lucide-react";

import { PageHeader } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";
import { api } from "~/trpc/react";
import { StudyAnalyticsDataTable } from "~/components/analytics/study-analytics-data-table";

export default function StudyAnalyticsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Fetch list of trials
  const { data: trialsList, isLoading } = api.trials.list.useQuery(
    { studyId, limit: 100 },
    { enabled: !!studyId }
  );

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Analytics" },
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
        title="Analysis"
        description="View and analyze session data across all trials"
        icon={BarChart3}
      />

      <div className="bg-transparent">
        <Suspense fallback={<div>Loading analytics...</div>}>
          {isLoading ? (
            <div className="flex items-center justify-center h-64">
              <div className="flex flex-col items-center gap-2 animate-pulse">
                <div className="h-8 w-8 rounded-full border-2 border-primary border-t-transparent animate-spin" />
                <span className="text-muted-foreground text-sm">Loading session data...</span>
              </div>
            </div>
          ) : (
            <StudyAnalyticsDataTable data={(trialsList ?? []).map(t => ({
              ...t,
              startedAt: t.startedAt ? new Date(t.startedAt) : null,
              completedAt: t.completedAt ? new Date(t.completedAt) : null,
              createdAt: new Date(t.createdAt),
            }))} />
          )}
        </Suspense>
      </div>
    </div>
  );
}
