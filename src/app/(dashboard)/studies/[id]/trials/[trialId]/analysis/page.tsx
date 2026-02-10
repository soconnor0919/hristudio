"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import Link from "next/link";
import { LineChart, ArrowLeft } from "lucide-react";
import { PageHeader } from "~/components/ui/page-header";
import { Button } from "~/components/ui/button";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";
import { TrialAnalysisView } from "~/components/trials/views/TrialAnalysisView";
import { api } from "~/trpc/react";

function AnalysisPageContent() {
    const params = useParams();
    const studyId: string = typeof params.id === "string" ? params.id : "";
    const trialId: string =
        typeof params.trialId === "string" ? params.trialId : "";

    const { setSelectedStudyId, selectedStudyId } = useStudyContext();
    const { study } = useSelectedStudyDetails();

    // Get trial data
    const {
        data: trial,
        isLoading,
        error,
    } = api.trials.get.useQuery({ id: trialId }, { enabled: !!trialId });

    // Set breadcrumbs
    useBreadcrumbsEffect([
        { label: "Dashboard", href: "/dashboard" },
        { label: "Studies", href: "/studies" },
        { label: study?.name ?? "Study", href: `/studies/${studyId}` },
        { label: "Trials", href: `/studies/${studyId}/trials` },
        {
            label: trial?.experiment.name ?? "Trial",
            href: `/studies/${studyId}/trials`,
        },
        { label: "Analysis" },
    ]);

    // Sync selected study (unified study-context)
    useEffect(() => {
        if (studyId && selectedStudyId !== studyId) {
            setSelectedStudyId(studyId);
        }
    }, [studyId, selectedStudyId, setSelectedStudyId]);

    if (isLoading) {
        return (
            <div className="flex h-96 items-center justify-center">
                <div className="text-muted-foreground">Loading analysis...</div>
            </div>
        );
    }

    if (error || !trial) {
        return (
            <div className="space-y-6">
                <PageHeader
                    title="Trial Analysis"
                    description="Analyze trial results"
                    icon={LineChart}
                    actions={
                        <Button asChild variant="outline">
                            <Link href={`/studies/${studyId}/trials`}>
                                <ArrowLeft className="mr-2 h-4 w-4" />
                                Back to Trials
                            </Link>
                        </Button>
                    }
                />
                <div className="flex h-96 items-center justify-center">
                    <div className="text-center">
                        <h3 className="text-destructive mb-2 text-lg font-semibold">
                            {error ? "Error Loading Trial" : "Trial Not Found"}
                        </h3>
                        <p className="text-muted-foreground">
                            {error?.message || "The requested trial could not be found."}
                        </p>
                    </div>
                </div>
            </div>
        );
    }

    const trialData = {
        ...trial,
        startedAt: trial.startedAt ? new Date(trial.startedAt) : null,
        completedAt: trial.completedAt ? new Date(trial.completedAt) : null,
        eventCount: (trial as any).eventCount,
        mediaCount: (trial as any).mediaCount,
    };

    return (
        <TrialAnalysisView
            trial={trialData}
            backHref={`/studies/${studyId}/trials/${trialId}`}
        />
    );
}

export default function TrialAnalysisPage() {
    return (
        <Suspense
            fallback={
                <div className="flex h-96 items-center justify-center">
                    <div className="text-muted-foreground">Loading...</div>
                </div>
            }
        >
            <AnalysisPageContent />
        </Suspense>
    );
}
